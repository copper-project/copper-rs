//! Pull-based delivery after native archival, independent of UI processing.
//!
//! One publisher owns reception; one reader owns consumption. Unread frames are
//! overwritten oldest first. Status is coalesced separately, so a paused reader
//! can still inspect recording health. This host-only exchange adds no robot
//! task-path work. Publish only after a successful `NativeArchive::accept`.
//!
//! Storage is allocated at construction: a ring, a status slot, and one reader
//! frame. Frames move without cloning. Payload-owned allocations and user-owned
//! projections are outside this bound. Queue operations use Crossbeam atomics;
//! they do not wait for capacity or for user processing, but are not an RT
//! wait-free primitive. Frame destructors and registered wakers must not block.

use atomic_waker::AtomicWaker;
use crossbeam_queue::ArrayQueue;
use std::{
    future::poll_fn,
    num::NonZeroUsize,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    task::{Poll, Wake, Waker},
    thread::{self, Thread, ThreadId},
    time::{Duration, Instant},
};

struct Sequenced<T> {
    sequence: u64,
    frame: T,
}

struct Shared<T, S> {
    frames: ArrayQueue<Sequenced<T>>,
    status: ArrayQueue<S>,
    waker: AtomicWaker,
    closed: AtomicBool,
    connected: AtomicBool,
    overwritten: AtomicU64,
}

/// The sole producer of archived frames and current receiver status.
pub struct TelemetryPublisher<T, S> {
    shared: Arc<Shared<T, S>>,
    next_sequence: u64,
    status: S,
}

/// A single consumer cursor. User code chooses when to read and what to retain.
pub struct TelemetryReader<T, S> {
    shared: Arc<Shared<T, S>>,
    next_sequence: u64,
    current: Option<Sequenced<T>>,
    status: S,
    thread_waker: Option<(ThreadId, Waker)>,
}

/// A stable borrow of one frame, valid until the next mutable reader operation.
///
/// Holding this borrow never pins a ring slot or prevents overwrite. `missed`
/// counts locally overwritten unread frames, not missing source CopperLists.
pub struct TelemetryRead<'a, T> {
    pub frame: &'a T,
    pub missed: u64,
}

/// Creates a preallocated circular delivery buffer and independent status slot.
///
/// The publisher and reader are deliberately not cloneable. Use `T` for your
/// typed frame (including its session identity) and a small `Copy` type for
/// receiver status. An initial status notification is available immediately.
pub fn telemetry_channel<T, S: Copy + PartialEq>(
    capacity: NonZeroUsize,
    status: S,
) -> (TelemetryPublisher<T, S>, TelemetryReader<T, S>) {
    let shared = Arc::new(Shared {
        frames: ArrayQueue::new(capacity.get()),
        status: ArrayQueue::new(1),
        waker: AtomicWaker::new(),
        closed: AtomicBool::new(false),
        connected: AtomicBool::new(true),
        overwritten: AtomicU64::new(0),
    });
    shared.status.force_push(status);
    (
        TelemetryPublisher {
            shared: shared.clone(),
            next_sequence: 0,
            status,
        },
        TelemetryReader {
            shared,
            next_sequence: 0,
            current: None,
            status,
            thread_waker: None,
        },
    )
}

impl<T, S: Copy + PartialEq> TelemetryPublisher<T, S> {
    /// Moves an already archived frame into the ring. A full ring replaces its
    /// oldest unread frame. A disconnected reader causes immediate discard.
    pub fn publish(&mut self, frame: T) {
        if !self.is_connected() {
            return;
        }
        let entry = Sequenced {
            sequence: self.next_sequence,
            frame,
        };
        self.next_sequence = self.next_sequence.wrapping_add(1);
        if self.shared.frames.force_push(entry).is_some() {
            self.shared.overwritten.fetch_add(1, Ordering::Relaxed);
        }
        self.shared.waker.wake();
    }

    /// Replaces current status and notifies even when no frame is arriving.
    pub fn set_status(&mut self, status: S) {
        if self.status != status {
            self.status = status;
            self.shared.status.force_push(status);
            self.shared.waker.wake();
        }
    }

    pub fn status(&self) -> S {
        self.status
    }

    pub fn is_connected(&self) -> bool {
        self.shared.connected.load(Ordering::Acquire)
    }

    /// Cumulative presentation loss; independent of source/archive gaps.
    pub fn overwritten(&self) -> u64 {
        self.shared.overwritten.load(Ordering::Relaxed)
    }
}

impl<T, S> Drop for TelemetryPublisher<T, S> {
    fn drop(&mut self) {
        self.shared.closed.store(true, Ordering::Release);
        self.shared.waker.wake();
    }
}

impl<T, S: Copy> TelemetryReader<T, S> {
    /// Pulls the oldest retained frame without waiting. At most one borrowed
    /// frame can be held through this reader. Frame sequence wraps modulo u64;
    /// fewer than 2^64 publications may separate consecutive reads.
    pub fn try_read(&mut self) -> Option<TelemetryRead<'_, T>> {
        let entry = self.shared.frames.pop()?;
        let missed = entry.sequence.wrapping_sub(self.next_sequence);
        self.next_sequence = entry.sequence.wrapping_add(1);
        self.current = Some(entry);
        Some(TelemetryRead {
            frame: &self.current.as_ref().unwrap().frame,
            missed,
        })
    }

    /// Observes/acknowledges coalesced current status independently of frames.
    /// A buffered frame may belong to an older session than this status.
    pub fn status(&mut self) -> S {
        if let Some(status) = self.shared.status.pop() {
            self.status = status;
        }
        self.status
    }

    /// True after publisher shutdown; any retained frames can still be read.
    pub fn is_closed(&self) -> bool {
        self.shared.closed.load(Ordering::Acquire)
    }

    pub fn overwritten(&self) -> u64 {
        self.shared.overwritten.load(Ordering::Relaxed)
    }

    fn is_ready(&self) -> bool {
        !self.shared.frames.is_empty() || !self.shared.status.is_empty() || self.is_closed()
    }

    /// Registers a one-shot payload-free wake. Re-arm before checking data and
    /// status. The waker must only schedule/unpark, never process user data.
    /// The post-registration readiness check prevents lost wakeups.
    pub fn register_waker(&self, waker: &Waker) {
        self.shared.waker.register(waker);
        if self.is_ready() {
            self.shared.waker.wake();
        }
    }

    /// Waits asynchronously for unread data, changed status, or closure. This
    /// is level-triggered: drain data and observe status before waiting again.
    pub async fn ready(&mut self) {
        poll_fn(|cx| {
            self.register_waker(cx.waker());
            if self.is_ready() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }

    /// Synchronous counterpart of `ready`, with a timeout for UI/input timers.
    /// Returns false only on timeout. The first call on a thread allocates its
    /// reusable thread waker; publication itself never allocates a notification.
    pub fn wait_timeout(&mut self, timeout: Duration) -> bool {
        let current = thread::current();
        if self.thread_waker.as_ref().map(|(id, _)| *id) != Some(current.id()) {
            self.thread_waker = Some((current.id(), Waker::from(Arc::new(ThreadWake(current)))));
        }
        let waker = &self.thread_waker.as_ref().unwrap().1;
        let start = Instant::now();
        loop {
            self.register_waker(waker);
            if self.is_ready() {
                return true;
            }
            let remaining = timeout.saturating_sub(start.elapsed());
            if remaining.is_zero() {
                return false;
            }
            thread::park_timeout(remaining);
        }
    }
}

impl<T, S> Drop for TelemetryReader<T, S> {
    fn drop(&mut self) {
        self.shared.connected.store(false, Ordering::Release);
        self.shared.waker.take();
    }
}

struct ThreadWake(Thread);

impl Wake for ThreadWake {
    fn wake(self: Arc<Self>) {
        self.0.unpark();
    }

    fn wake_by_ref(self: &Arc<Self>) {
        self.0.unpark();
    }
}
