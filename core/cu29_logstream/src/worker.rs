//! Host driver for the autonomous sender. Only semantic output workers call these sinks.

use crate::{CuStreamTx, LogStreamSenderConfig, OneWay, SenderCore, SenderStats};
use cu29_clock::{CuDuration, CuTime, RobotClock};
#[allow(unused_imports)]
use cu29_log::{ANONYMOUS, CuLogEntry, CuLogLevel};
#[allow(unused_imports)]
use cu29_log_runtime::log;
#[cfg(debug_assertions)]
#[allow(unused_imports)]
use cu29_log_runtime::log_debug_mode;
use cu29_runtime::{copperlist::CopperList, curuntime::KeyFrame};
use cu29_traits::{CopperListTuple, CuError, CuResult, WriteStream};
#[allow(unused_imports)]
use cu29_value::to_value;
use std::{
    fmt::{Debug, Formatter},
    marker::PhantomData,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender, TryRecvError},
    },
    thread::{JoinHandle, Thread},
};

const CL_BUFFERS: usize = 4;
const KF_BUFFERS: usize = 2;

struct Record {
    bytes: Box<[u8]>,
    len: usize,
    keyframe: bool,
    queued_at: CuTime,
}

/// Final counters remain readable after both output sinks have been dropped.
#[derive(Clone, Debug)]
pub struct SenderMonitor(Arc<Shared>);

#[derive(Debug, Default)]
struct Shared {
    stop: AtomicBool,
    failed: AtomicBool,
    inbox_drops: AtomicU64,
    final_stats: Mutex<Option<SenderStats>>,
}

impl SenderMonitor {
    pub fn inbox_drops(&self) -> u64 {
        self.0.inbox_drops.load(Ordering::Relaxed)
    }
    pub fn failed(&self) -> bool {
        self.0.failed.load(Ordering::Acquire)
    }
    pub fn final_stats(&self) -> Option<SenderStats> {
        *self.0.final_stats.lock().expect("sender stats poisoned")
    }
}

#[derive(Debug)]
struct Worker {
    shared: Arc<Shared>,
    thread: Thread,
    handle: Option<JoinHandle<()>>,
}

impl Drop for Worker {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Release);
        self.thread.unpark();
        if self
            .handle
            .take()
            .is_some_and(|handle| handle.join().is_err())
        {
            self.shared.failed.store(true, Ordering::Release);
        }
    }
}

#[derive(Debug)]
struct Inbox {
    clock: RobotClock,
    worker: Arc<Worker>,
    pending: SyncSender<Record>,
    // Mutex supplies Sync for WriteStream; exclusive log() uses get_mut, never lock().
    free: Mutex<Receiver<Box<[u8]>>>,
    recycled: SyncSender<Box<[u8]>>,
}

impl Inbox {
    fn submit(
        &mut self,
        keyframe: bool,
        encode: impl FnOnce(&mut [u8]) -> CuResult<usize>,
    ) -> CuResult<()> {
        if self.worker.shared.failed.load(Ordering::Acquire) {
            return Err(CuError::from("Logstream sender worker failed"));
        }
        let mut bytes = match self.free.get_mut().expect("exclusive inbox").try_recv() {
            Ok(bytes) => bytes,
            Err(TryRecvError::Empty) => {
                self.worker
                    .shared
                    .inbox_drops
                    .fetch_add(1, Ordering::Relaxed);
                return Ok(());
            }
            Err(TryRecvError::Disconnected) => {
                return Err(CuError::from("Logstream sender worker stopped"));
            }
        };
        let queued_at = self.clock.now();
        let len = match encode(&mut bytes) {
            Ok(len) => len,
            Err(error) => {
                let _ = self.recycled.try_send(bytes);
                return Err(error);
            }
        };
        // Capacity equals the total buffer count, so an owned free buffer always
        // has a corresponding inbox slot. Neither this nor pool exhaustion waits.
        self.pending
            .try_send(Record {
                bytes,
                len,
                keyframe,
                queued_at,
            })
            .map_err(|_| CuError::from("Logstream sender inbox disconnected"))?;
        self.worker.thread.unpark();
        Ok(())
    }
}

/// Encodes directly into a sender-owned buffer on the existing CL output worker.
pub struct ScheduledCopperListSink<P> {
    inbox: Inbox,
    _payload: PhantomData<fn() -> P>,
}
impl<P> Debug for ScheduledCopperListSink<P> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ScheduledCopperListSink")
            .finish_non_exhaustive()
    }
}
impl<P: CopperListTuple + Send + Sync> WriteStream<CopperList<P>> for ScheduledCopperListSink<P> {
    fn log(&mut self, record: &CopperList<P>) -> CuResult<()> {
        self.inbox.submit(false, |bytes| {
            crate::encode_copperlist_record_into(record, bytes)
                .map_err(|error| CuError::from(error.to_string()))
        })
    }
}

/// Bounded keyframe encoding; capture objects are released before pacing begins.
#[derive(Debug)]
pub struct ScheduledKeyFrameSink {
    inbox: Inbox,
    interval: u32,
}
impl WriteStream<KeyFrame> for ScheduledKeyFrameSink {
    fn log(&mut self, keyframe: &KeyFrame) -> CuResult<()> {
        if !keyframe.culistid.is_multiple_of(u64::from(self.interval)) {
            return Ok(());
        }
        self.inbox.submit(true, |bytes| {
            if bytes.len() < crate::record::RECORD_HEADER_LEN {
                return Err(CuError::from("Keyframe buffer too small"));
            }
            let (header, payload) = bytes.split_at_mut(crate::record::RECORD_HEADER_LEN);
            let len = bincode::encode_into_slice(keyframe, payload, bincode::config::standard())
                .map_err(|error| CuError::from(error.to_string()))?;
            crate::record::encode_record_header(
                crate::RecordKind::KeyFrame,
                keyframe.culistid,
                &payload[..len],
                header,
            )
            .map_err(|error| CuError::from(error.to_string()))?;
            Ok(header.len() + len)
        })
    }
}

/// Start one autonomous worker with an explicitly selected local RobotClock.
/// A real carrier must use a running clock even when application time is mocked.
/// Drops of both sinks stop repetition, then drain at most max_latency in local
/// clock time. No application object, serialization, or wait is added to the RT path.
pub fn scheduled_sinks<P, T>(
    transport: T,
    config: LogStreamSenderConfig,
    clock: RobotClock,
) -> CuResult<(
    ScheduledCopperListSink<P>,
    ScheduledKeyFrameSink,
    SenderMonitor,
)>
where
    P: CopperListTuple + Send + Sync,
    T: CuStreamTx + 'static,
{
    let cl_bytes = config.continuous.max_record_bytes;
    let kf_bytes = usize::try_from(config.recovery.finite.max_object_bytes)
        .map_err(|_| CuError::from("Keyframe limit exceeds usize"))?;
    let reserved = cl_bytes
        .checked_mul(CL_BUFFERS)
        .and_then(|v| {
            kf_bytes
                .checked_mul(KF_BUFFERS)
                .and_then(|k| v.checked_add(k))
        })
        .ok_or_else(|| CuError::from("Sender pool size overflow"))?;
    let shutdown_clock = if clock.is_mock() {
        RobotClock::new()
    } else {
        clock.clone()
    };
    let interval = config.recovery.anchor_interval;
    let drain_time = config.pacing.max_latency;
    let mut core = SenderCore::new(config, clock.now(), reserved)
        .map_err(|error| CuError::from(error.to_string()))?;
    let (pending, records) = mpsc::sync_channel::<Record>(CL_BUFFERS + KF_BUFFERS);
    let (cl_return, cl_free) = mpsc::sync_channel(CL_BUFFERS);
    let (kf_return, kf_free) = mpsc::sync_channel(KF_BUFFERS);
    for _ in 0..CL_BUFFERS {
        cl_return
            .try_send(vec![0; cl_bytes].into_boxed_slice())
            .expect("new pool");
    }
    for _ in 0..KF_BUFFERS {
        kf_return
            .try_send(vec![0; kf_bytes].into_boxed_slice())
            .expect("new pool");
    }
    let cl_clock = clock.clone();
    let kf_clock = clock.clone();
    let shared = Arc::new(Shared::default());
    let status = shared.clone();
    let cl_recycled = cl_return.clone();
    let kf_recycled = kf_return.clone();
    let handle = std::thread::Builder::new()
        .name("cu-logstream".into())
        .spawn(move || {
            let mut transport = OneWay::new(transport);
            let mut stop_at = None;
            // Teardown remains finite even for deliberately frozen scheduler test clocks.
            let result = (|| -> crate::Result<()> {
                loop {
                    if status.stop.load(Ordering::Acquire) && stop_at.is_none() {
                        core.begin_shutdown();
                        stop_at = Some(shutdown_clock.now() + drain_time);
                    }
                    let mut received = 0;
                    for _ in 0..CL_BUFFERS + KF_BUFFERS {
                        let Ok(record) = records.try_recv() else {
                            break;
                        };
                        let result =
                            core.accept_record(&record.bytes[..record.len], record.queued_at);
                        let returned = if record.keyframe {
                            &kf_return
                        } else {
                            &cl_return
                        };
                        let _ = returned.try_send(record.bytes);
                        result?;
                        received += 1;
                    }
                    let next = core.poll(clock.now(), &mut transport)?;
                    if stop_at.is_some_and(|deadline| shutdown_clock.now() >= deadline)
                        || (stop_at.is_some() && received == 0 && core.is_idle())
                    {
                        break;
                    }
                    if received > 0 {
                        continue;
                    }
                    if let Some(deadline) = next {
                        let now = clock.now();
                        if deadline > now {
                            let duration = (deadline - now).min(CuDuration::from_millis(50));
                            std::thread::park_timeout(std::time::Duration::from_nanos(
                                duration.as_nanos(),
                            ));
                        }
                    } else {
                        std::thread::park_timeout(std::time::Duration::from_millis(50));
                    }
                }
                Ok(())
            })();
            core.discard_pending();
            let stats = core.stats();
            *status.final_stats.lock().expect("sender stats") = Some(stats);
            cu29_log_derive::info!("Logstream sender stopped: sent={} bytes={} queue_drops={} expired={} transport_drops={} inbox_drops={} shutdown_drops={}",
                stats.packets_sent, stats.bytes_sent, stats.queue_drops, stats.expired_packets, stats.transport_drops,
                status.inbox_drops.load(Ordering::Relaxed), stats.shutdown_drops);
            if let Err(error) = result {
                status.failed.store(true, Ordering::Release);
                cu29_log_derive::error!("Logstream sender failed: {}", error.to_string());
            }
        })
        .map_err(|error| CuError::new_with_cause("Start logstream sender", error))?;
    let worker = Arc::new(Worker {
        shared: shared.clone(),
        thread: handle.thread().clone(),
        handle: Some(handle),
    });
    Ok((
        ScheduledCopperListSink {
            inbox: Inbox {
                clock: cl_clock,
                worker: worker.clone(),
                pending: pending.clone(),
                free: Mutex::new(cl_free),
                recycled: cl_recycled,
            },
            _payload: PhantomData,
        },
        ScheduledKeyFrameSink {
            inbox: Inbox {
                clock: kf_clock,
                worker,
                pending,
                free: Mutex::new(kf_free),
                recycled: kf_recycled,
            },
            interval,
        },
        SenderMonitor(shared),
    ))
}
