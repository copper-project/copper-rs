//! Some basic internal monitoring tooling Copper uses to monitor itself and the tasks it is running.
//!

use crate::config::ComponentConfig;
use crate::cutask::CuMsgMetadata;
use cu29_clock::RobotClock;
use cu29_traits::{CuError, CuResult};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

/// The state of a task.
#[derive(Debug)]
pub enum CuTaskState {
    Start,
    Preprocess,
    Process,
    Postprocess,
    Stop,
}

/// Monitor decision to be taken when a task errored out.
#[derive(Debug)]
pub enum Decision {
    SkipCopperlist,
    ContinueWithNoOuput,
    Shutdown,
}

/// Trait to implement a monitoring task.
pub trait CuMonitor {
    fn new(config: Option<&ComponentConfig>, taskids: &'static [&'static str]) -> CuResult<Self>
    where
        Self: Sized;

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Callback that will be trigger at the end of every copperlist (before, on or after the serialization).
    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()>;

    /// Callbacked when a Task errored out. The runtime requires an immediate decision.
    fn process_error(&self, taskid: usize, step: CuTaskState, error: CuError) -> Decision;

    /// Callbacked when copper is stopping.
    fn stop(&mut self) -> CuResult<()> {
        Ok(())
    }
}

#[global_allocator]
pub static GLOBAL: CountingAllocator = CountingAllocator::new();

/// A simple allocator that counts the number of bytes allocated and deallocated.
pub struct CountingAllocator {
    allocated: AtomicUsize,
    deallocated: AtomicUsize,
}

impl CountingAllocator {
    pub const fn new() -> Self {
        CountingAllocator {
            allocated: AtomicUsize::new(0),
            deallocated: AtomicUsize::new(0),
        }
    }

    pub fn get_allocated(&self) -> usize {
        self.allocated.load(Ordering::SeqCst)
    }

    pub fn get_deallocated(&self) -> usize {
        self.deallocated.load(Ordering::SeqCst)
    }

    pub fn reset(&self) {
        self.allocated.store(0, Ordering::SeqCst);
        self.deallocated.store(0, Ordering::SeqCst);
    }
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = System.alloc(layout);
        if !ptr.is_null() {
            self.allocated.fetch_add(layout.size(), Ordering::SeqCst);
        }
        ptr
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout);
        self.deallocated.fetch_add(layout.size(), Ordering::SeqCst);
    }
}

/// A simple struct that counts the number of bytes allocated and deallocated in a scope.
pub struct ScopedAllocCounter {
    bf_allocated: usize,
    bf_deallocated: usize,
}

impl ScopedAllocCounter {
    pub fn new() -> Self {
        ScopedAllocCounter {
            bf_allocated: GLOBAL.get_allocated(),
            bf_deallocated: GLOBAL.get_deallocated(),
        }
    }
}

/// Build a difference between the number of bytes allocated and deallocated in the scope at drop time.
impl Drop for ScopedAllocCounter {
    fn drop(&mut self) {
        let _allocated = GLOBAL.get_allocated() - self.bf_allocated;
        let _deallocated = GLOBAL.get_deallocated() - self.bf_deallocated;
        // TODO(gbin): Fix this when the logger is ready.
        // debug!(
        //     "Allocations: +{}B -{}B",
        //     allocated = allocated,
        //     deallocated = deallocated,
        // );
    }
}
