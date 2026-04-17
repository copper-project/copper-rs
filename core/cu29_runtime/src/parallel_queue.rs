//! Scratch SPSC handoff queue used by the parallel runtime stage pipeline.
//!
//! The generated stage graph is single-producer/single-consumer between
//! adjacent stages, so a bounded SPSC ring is a closer match than a general
//! MPSC channel on that path. To avoid the regression from pure spinning, this
//! wrapper parks the lone producer/consumer thread while the ring is full or
//! empty and wakes the opposite side when progress becomes possible.

use core::fmt::{Display, Formatter, Result as FmtResult};
use core::sync::atomic::{AtomicBool, Ordering};
use rtrb::{Consumer, PopError, Producer, PushError, RingBuffer};
use std::sync::Arc;
use std::sync::OnceLock;
use std::thread::{self, Thread};

#[derive(Debug)]
pub struct StageSendError<T>(pub T);

#[derive(Clone, Copy, Debug, Default)]
pub struct StageRecvError;

impl<T> Display for StageSendError<T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        f.write_str("parallel stage queue receiver disconnected")
    }
}

impl Display for StageRecvError {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        f.write_str("parallel stage queue sender disconnected")
    }
}

#[derive(Debug)]
pub struct StageSender<T> {
    inner: Producer<T>,
    shared: Arc<StageQueueShared>,
}

#[derive(Debug)]
pub struct StageReceiver<T> {
    inner: Consumer<T>,
    shared: Arc<StageQueueShared>,
}

#[derive(Debug, Default)]
struct StageQueueShared {
    sender_thread: OnceLock<Thread>,
    receiver_thread: OnceLock<Thread>,
    sender_waiting: AtomicBool,
    receiver_waiting: AtomicBool,
    sender_alive: AtomicBool,
    receiver_alive: AtomicBool,
}

impl StageQueueShared {
    #[inline]
    fn register_sender(&self) {
        if let Err(thread) = self.sender_thread.set(thread::current()) {
            debug_assert_eq!(
                self.sender_thread
                    .get()
                    .expect("sender thread should be registered")
                    .id(),
                thread.id(),
                "stage sender used from multiple threads"
            );
        }
    }

    #[inline]
    fn register_receiver(&self) {
        if let Err(thread) = self.receiver_thread.set(thread::current()) {
            debug_assert_eq!(
                self.receiver_thread
                    .get()
                    .expect("receiver thread should be registered")
                    .id(),
                thread.id(),
                "stage receiver used from multiple threads"
            );
        }
    }

    #[inline]
    fn wake_sender(&self) {
        if self.sender_waiting.swap(false, Ordering::AcqRel)
            && let Some(thread) = self.sender_thread.get()
        {
            thread.unpark();
        }
    }

    #[inline]
    fn wake_receiver(&self) {
        if self.receiver_waiting.swap(false, Ordering::AcqRel)
            && let Some(thread) = self.receiver_thread.get()
        {
            thread.unpark();
        }
    }
}

#[inline]
pub fn stage_queue<T>(capacity: usize) -> (StageSender<T>, StageReceiver<T>) {
    let (producer, consumer) = RingBuffer::new(capacity.max(1));
    let shared = Arc::new(StageQueueShared {
        sender_thread: OnceLock::new(),
        receiver_thread: OnceLock::new(),
        sender_waiting: AtomicBool::new(false),
        receiver_waiting: AtomicBool::new(false),
        sender_alive: AtomicBool::new(true),
        receiver_alive: AtomicBool::new(true),
    });
    (
        StageSender {
            inner: producer,
            shared: Arc::clone(&shared),
        },
        StageReceiver {
            inner: consumer,
            shared,
        },
    )
}

impl<T> StageSender<T> {
    #[inline]
    pub fn send(&mut self, mut value: T) -> Result<(), StageSendError<T>> {
        self.shared.register_sender();
        loop {
            if !self.shared.receiver_alive.load(Ordering::Acquire) {
                return Err(StageSendError(value));
            }
            match self.inner.push(value) {
                Ok(()) => {
                    self.shared.wake_receiver();
                    return Ok(());
                }
                Err(PushError::Full(returned)) => {
                    value = returned;
                }
            }

            self.shared.sender_waiting.store(true, Ordering::Release);

            if !self.shared.receiver_alive.load(Ordering::Acquire) {
                self.shared.sender_waiting.store(false, Ordering::Release);
                return Err(StageSendError(value));
            }

            match self.inner.push(value) {
                Ok(()) => {
                    self.shared.sender_waiting.store(false, Ordering::Release);
                    self.shared.wake_receiver();
                    return Ok(());
                }
                Err(PushError::Full(returned)) => {
                    value = returned;
                    thread::park();
                    self.shared.sender_waiting.store(false, Ordering::Release);
                    if !self.shared.receiver_alive.load(Ordering::Acquire) {
                        return Err(StageSendError(value));
                    }
                }
            }
        }
    }
}

impl<T> StageReceiver<T> {
    #[inline]
    pub fn recv(&mut self) -> Result<T, StageRecvError> {
        self.shared.register_receiver();
        loop {
            match self.inner.pop() {
                Ok(value) => {
                    self.shared.wake_sender();
                    return Ok(value);
                }
                Err(PopError::Empty) => {
                    self.shared.receiver_waiting.store(true, Ordering::Release);

                    match self.inner.pop() {
                        Ok(value) => {
                            self.shared.receiver_waiting.store(false, Ordering::Release);
                            self.shared.wake_sender();
                            return Ok(value);
                        }
                        Err(PopError::Empty) => {}
                    }

                    if !self.shared.sender_alive.load(Ordering::Acquire) {
                        self.shared.receiver_waiting.store(false, Ordering::Release);
                        return Err(StageRecvError);
                    }

                    thread::park();
                    self.shared.receiver_waiting.store(false, Ordering::Release);
                }
            }
        }
    }
}

impl<T> Drop for StageSender<T> {
    fn drop(&mut self) {
        self.shared.sender_alive.store(false, Ordering::Release);
        self.shared.wake_receiver();
    }
}

impl<T> Drop for StageReceiver<T> {
    fn drop(&mut self) {
        self.shared.receiver_alive.store(false, Ordering::Release);
        self.shared.wake_sender();
    }
}
