use crate::config::NodeInstanceConfig;
use crate::cutask::{CuMsg, CuSrcTask, CuTaskLifecycle};
use crate::CuResult;

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};
use copper_clock::RobotClock;

use copper_log::debug;

#[global_allocator]
pub static GLOBAL: CountingAllocator = CountingAllocator::new();

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

impl Drop for ScopedAllocCounter {
    fn drop(&mut self) {
        let allocated = GLOBAL.get_allocated() - self.bf_allocated;
        let deallocated = GLOBAL.get_deallocated() - self.bf_deallocated;
        debug!(
            "Allocations: +{}B -{}B",
            allocated = allocated,
            deallocated = deallocated,
        );
    }
}

pub struct MonitoringTask {}

impl CuTaskLifecycle for MonitoringTask {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let allocated = GLOBAL.get_allocated();
        let deallocated = GLOBAL.get_deallocated();
        println!("New: Allocated: {} bytes", allocated);
        println!("New: Deallocated: {} bytes", deallocated);
        Ok(Self {})
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        println!("Start: Reset counting");
        GLOBAL.reset();
        Ok(())
    }
}

impl CuSrcTask for MonitoringTask {
    type Output = ();

    fn process(&mut self, _clock: &RobotClock, _empty_msg: &mut CuMsg<Self::Output>) -> CuResult<()> {
        Ok(())
    }
}
