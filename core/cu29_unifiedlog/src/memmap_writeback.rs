//! Dirty host pages across all mappings of a log. A producer can dirty a claimed
//! page again while msync runs. Only the service caller waits for other flushes.
use super::MappedFile;
use cu29_traits::CuResult;
use std::sync::{Arc, Mutex};

struct Slab {
    map: Arc<MappedFile>,
    dirty: Vec<u64>,
    queued: bool,
}

#[derive(Default)]
struct Pending {
    slabs: Vec<Slab>,
    queue: Vec<usize>,
}

pub(super) struct Writeback {
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
            queued: false,
        });
        let spare = pending.slabs.len() - pending.queue.len();
        pending.queue.reserve(spare);
    }

    pub fn mark(&self, index: usize, offset: usize, length: usize) {
        let mut pending = self.pending.lock().unwrap();
        let slab = &mut pending.slabs[index];
        let first = offset / self.page_size;
        let last = (offset + length - 1) / self.page_size;
        let first_word = first / 64;
        for (index, bits) in slab.dirty[first_word..=last / 64].iter_mut().enumerate() {
            let word = first_word + index;
            let low = first.saturating_sub(word * 64);
            let high = (last - word * 64).min(63);
            *bits |= (u64::MAX << low) & (u64::MAX >> (63 - high));
        }
        if !slab.queued {
            slab.queued = true;
            pending.queue.push(index);
        }
    }

    pub fn drain(&self) -> CuResult<()> {
        let mut scratch = self.service.lock().unwrap();
        let Scratch { queue, dirty } = &mut *scratch;
        {
            let mut pending = self.pending.lock().unwrap();
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
                std::mem::swap(dirty, &mut slab.dirty);
                slab.queued = false;
                slab.map.clone()
            };
            if let Err(error) = self.flush_pages(&map, dirty) {
                let mut pending = self.pending.lock().unwrap();
                let slab = &mut pending.slabs[index];
                for (pending, claimed) in slab.dirty.iter_mut().zip(dirty.iter()) {
                    *pending |= claimed;
                }
                if !slab.queued {
                    slab.queued = true;
                    pending.queue.push(index);
                }
                failure = Err(error);
            }
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
        map.sync()
    }
}
