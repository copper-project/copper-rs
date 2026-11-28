//! Dirty host pages across all mappings of a log. A producer can dirty a claimed
//! page again while msync runs. Only the service caller waits for other flushes.
use super::MappedFile;
use cu29_traits::CuResult;
use std::sync::{Arc, Mutex};

use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

const TARGET_BYTES: usize = 1024 * 1024;
const MAX_AGE: Duration = Duration::from_millis(100);

#[doc(hidden)]
pub struct WritebackDriver(Arc<Writeback>);
impl WritebackDriver {
    pub fn service(&self) -> CuResult<Duration> {
        self.0.service_due(Instant::now())
    }
    pub fn drain(&self) -> CuResult<()> {
        self.0.drain()
    }
}
impl Drop for WritebackDriver {
    fn drop(&mut self) {
        self.0.drivers.fetch_sub(1, Ordering::Release);
    }
}

struct Slab {
    map: Arc<MappedFile>,
    dirty: Vec<u64>,
    complete: Vec<u64>,
    queued: bool,
}

#[derive(Default)]
struct Pending {
    oldest: Option<Instant>,
    complete_pages: usize,
    urgent: bool,
    slabs: Vec<Slab>,
    queue: Vec<usize>,
}

pub(super) struct Writeback {
    drivers: AtomicUsize,
    pub page_size: usize,
    pending: Mutex<Pending>,
    service: Mutex<Scratch>,
}

#[derive(Default)]
struct Scratch {
    queue: Vec<usize>,
    dirty: Vec<u64>,
}

impl Writeback {
    pub fn new() -> Self {
        Self {
            drivers: AtomicUsize::default(),
            page_size: page_size::get(),
            pending: Mutex::default(),
            service: Mutex::default(),
        }
    }

    pub fn add(&self, map: Arc<MappedFile>) {
        let words = map.length.div_ceil(self.page_size).div_ceil(64);
        let mut pending = self.pending.lock().unwrap();
        pending.slabs.push(Slab {
            map,
            dirty: vec![0; words],
            complete: vec![0; words],
            queued: false,
        });
        let spare = pending.slabs.len() - pending.queue.len();
        pending.queue.reserve(spare);
    }

    pub fn mark(&self, index: usize, offset: usize, length: usize, payload: bool) {
        let mut pending = self.pending.lock().unwrap();
        let slab = &mut pending.slabs[index];
        let mut new_complete = 0;
        let first = offset / self.page_size;
        let last = (offset + length - 1) / self.page_size;
        let first_word = first / 64;
        for (index, bits) in slab.dirty[first_word..=last / 64].iter_mut().enumerate() {
            let word = first_word + index;
            let low = first.saturating_sub(word * 64);
            let high = (last - word * 64).min(63);
            let mask = (u64::MAX << low) & (u64::MAX >> (63 - high));
            *bits |= mask;
            if payload {
                let partial = word == last / 64
                    && !(offset + length).is_multiple_of(self.page_size)
                    && offset + length != slab.map.length;
                let full = if partial { mask & !(1 << high) } else { mask };
                new_complete += (full & !slab.complete[word]).count_ones() as usize;
                slab.complete[word] |= full;
            }
        }
        if !slab.queued {
            slab.queued = true;
            pending.queue.push(index);
        }
        pending.complete_pages += new_complete;
        pending.oldest.get_or_insert_with(Instant::now);
    }

    pub fn driver(self: &Arc<Self>) -> WritebackDriver {
        self.drivers.fetch_add(1, Ordering::AcqRel);
        WritebackDriver(self.clone())
    }

    pub fn finish_section(&self) {
        self.pending.lock().unwrap().urgent = true;
    }

    pub fn inline(&self) -> CuResult<()> {
        if self.drivers.load(Ordering::Acquire) == 0 {
            self.service_due(Instant::now())?;
        }
        Ok(())
    }

    fn service_due(&self, now: Instant) -> CuResult<Duration> {
        let all = {
            let pending = self.pending.lock().unwrap();
            let Some(oldest) = pending.oldest else {
                return Ok(MAX_AGE);
            };
            let left = MAX_AGE.saturating_sub(now.saturating_duration_since(oldest));
            let all = pending.urgent || left.is_zero();
            if !all && pending.complete_pages < TARGET_BYTES.div_ceil(self.page_size) {
                return Ok(left);
            }
            all
        };
        self.submit(all)?;
        let pending = self.pending.lock().unwrap();
        Ok(pending
            .oldest
            .map_or(MAX_AGE, |old| MAX_AGE.saturating_sub(old.elapsed())))
    }

    pub fn drain(&self) -> CuResult<()> {
        self.submit(true)
    }

    fn submit(&self, all: bool) -> CuResult<()> {
        self.submit_with(all, |map, dirty| self.flush_pages(map, dirty))
    }

    fn submit_with(
        &self,
        all: bool,
        mut flush: impl FnMut(&MappedFile, &[u64]) -> CuResult<()>,
    ) -> CuResult<()> {
        let mut scratch = self.service.lock().unwrap();
        let Scratch { queue, dirty } = &mut *scratch;
        {
            let mut pending = self.pending.lock().unwrap();
            if all {
                pending.oldest = None;
                pending.urgent = false;
            }
            queue.clear();
            queue.reserve(pending.slabs.len());
            std::mem::swap(queue, &mut pending.queue);
        }
        // Keep failed claims queued for retry, including sync_all failures.
        let mut failure = Ok(());
        for index in queue.drain(..) {
            let map = {
                let mut pending = self.pending.lock().unwrap();
                let slab = &mut pending.slabs[index];
                dirty.resize(slab.dirty.len(), 0);
                dirty.fill(0);
                let complete = slab
                    .complete
                    .iter()
                    .map(|w| w.count_ones() as usize)
                    .sum::<usize>();
                if all {
                    std::mem::swap(dirty, &mut slab.dirty);
                    slab.complete.fill(0);
                } else {
                    std::mem::swap(dirty, &mut slab.complete);
                    for (bits, claim) in slab.dirty.iter_mut().zip(dirty.iter()) {
                        *bits &= !claim;
                    }
                }
                slab.queued = !all && slab.dirty.iter().any(|&w| w != 0);
                let queued = slab.queued;
                let map = slab.map.clone();
                if queued {
                    pending.queue.push(index);
                }
                pending.complete_pages -= complete;
                map
            };
            if let Err(error) = flush(&map, dirty) {
                let mut pending = self.pending.lock().unwrap();
                let slab = &mut pending.slabs[index];
                for (pending, claimed) in slab.dirty.iter_mut().zip(dirty.iter()) {
                    *pending |= claimed;
                }
                if !slab.queued {
                    slab.queued = true;
                    pending.queue.push(index);
                }
                pending.urgent = true;
                pending.oldest.get_or_insert_with(Instant::now);
                failure = Err(error);
            }
        }
        let mut pending = self.pending.lock().unwrap();
        if pending.queue.is_empty() {
            pending.oldest = None;
        }
        failure
    }

    fn flush_pages(&self, map: &MappedFile, dirty: &[u64]) -> CuResult<()> {
        let mut run = None;
        for (index, &word) in dirty.iter().chain(std::iter::once(&0)).enumerate() {
            if word == 0 && run.is_none() {
                continue;
            }
            if word == u64::MAX && run.is_some() {
                continue;
            }
            for bit in 0..64 {
                let page = index * 64 + bit;
                if word & (1 << bit) != 0 {
                    run.get_or_insert(page);
                } else if let Some(start) = run.take() {
                    let offset = start * self.page_size;
                    let end = (page * self.page_size).min(map.length);
                    map.flush_range(offset, end - offset)?;
                }
            }
        }
        if dirty.iter().any(|&word| word != 0) {
            map.sync()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn writeback(bytes: usize) -> Arc<Writeback> {
        let file = tempfile::tempfile().unwrap();
        file.set_len(bytes as u64).unwrap();
        let service = Arc::new(Writeback::new());
        service.add(Arc::new(MappedFile::new(file, true).unwrap()));
        service
    }

    #[test]
    fn deadline_includes_partial_tail_and_threshold_keeps_it_pending() {
        let service = writeback(TARGET_BYTES * 2);
        service.mark(0, 0, TARGET_BYTES + 1, true);
        let oldest = service.pending.lock().unwrap().oldest.unwrap();
        service.service_due(oldest).unwrap();
        {
            let pending = service.pending.lock().unwrap();
            assert_eq!(pending.complete_pages, 0);
            assert_eq!(pending.oldest, Some(oldest));
            assert_eq!(
                pending.slabs[0]
                    .dirty
                    .iter()
                    .map(|w| w.count_ones())
                    .sum::<u32>(),
                1
            );
        }
        service.service_due(oldest + MAX_AGE).unwrap();
        assert!(service.pending.lock().unwrap().queue.is_empty());
    }

    #[test]
    fn writes_during_io_survive_completion_and_failure() {
        let service = writeback(16384);
        service.mark(0, 0, 20, false);
        let (started, wait) = std::sync::mpsc::sync_channel(0);
        let (resume, held) = std::sync::mpsc::sync_channel(0);
        std::thread::scope(|scope| {
            let writer = service.clone();
            scope.spawn(move || {
                wait.recv().unwrap();
                writer.mark(0, 0, 20, false);
                resume.send(()).unwrap();
            });
            service
                .submit_with(true, |_, pages| {
                    assert_eq!(pages[0], 1);
                    started.send(()).unwrap();
                    held.recv().unwrap();
                    Ok(())
                })
                .unwrap();
        });
        assert!(
            service
                .submit_with(true, |_, pages| {
                    assert_eq!(pages[0], 1);
                    Err("injected flush failure".into())
                })
                .is_err()
        );
        let mut seen = 0;
        service
            .submit_with(true, |_, pages| {
                seen += pages[0];
                Ok(())
            })
            .unwrap();
        assert_eq!(seen, 1);
    }
}
