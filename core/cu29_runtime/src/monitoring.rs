//! Some basic internal monitoring tooling Copper uses to monitor itself and the tasks it is running.
//!

use crate::config::CuConfig;
use crate::cutask::CuMsgMetadata;
use crate::log::*;
use cu29_clock::{CuDuration, RobotClock};
use cu29_traits::{CuError, CuResult};
use hdrhistogram::Histogram;
use serde_derive::{Deserialize, Serialize};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

/// The state of a task.
#[derive(Debug, Serialize, Deserialize)]
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
    Abort,    // for a step (stop, start) or a copperlist, just stop trying to process it.
    Ignore, // Ignore this error and try to continue, ie calling the other tasks steps, setting a None return value and continue a copperlist.
    Shutdown, // This is a fatal error, shutdown the copper as cleanly as possible.
}

/// Trait to implement a monitoring task.
pub trait CuMonitor: Sized {
    fn new(config: &CuConfig, taskids: &'static [&'static str]) -> CuResult<Self>
    where
        Self: Sized;

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Callback that will be trigger at the end of every copperlist (before, on or after the serialization).
    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()>;

    /// Callbacked when a Task errored out. The runtime requires an immediate decision.
    fn process_error(&self, taskid: usize, step: CuTaskState, error: &CuError) -> Decision;

    /// Callbacked when copper is stopping.
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}

/// A do nothing monitor if no monitor is provided.
/// This is basically defining the default behavior of Copper in case of error.
pub struct NoMonitor {}
impl CuMonitor for NoMonitor {
    fn new(_config: &CuConfig, _taskids: &'static [&'static str]) -> CuResult<Self> {
        Ok(NoMonitor {})
    }

    fn process_copperlist(&self, _msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        // By default, do nothing.
        Ok(())
    }

    fn process_error(&self, _taskid: usize, _step: CuTaskState, _error: &CuError) -> Decision {
        // By default, just try to continue.
        Decision::Ignore
    }
}

#[global_allocator]
pub static GLOBAL: CountingAllocator = CountingAllocator::new();

/// A simple allocator that counts the number of bytes allocated and deallocated.
pub struct CountingAllocator {
    allocated: AtomicUsize,
    deallocated: AtomicUsize,
}

impl Default for CountingAllocator {
    fn default() -> Self {
        Self::new()
    }
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

impl Default for ScopedAllocCounter {
    fn default() -> Self {
        Self::new()
    }
}

impl ScopedAllocCounter {
    pub fn new() -> Self {
        ScopedAllocCounter {
            bf_allocated: GLOBAL.get_allocated(),
            bf_deallocated: GLOBAL.get_deallocated(),
        }
    }

    /// Returns the total number of bytes allocated in the current scope
    /// since the creation of this `ScopedAllocCounter`.
    pub fn get_allocated(&self) -> usize {
        GLOBAL.get_allocated() - self.bf_allocated
    }

    /// Returns the total number of bytes deallocated in the current scope
    /// since the creation of this `ScopedAllocCounter`.
    pub fn get_deallocated(&self) -> usize {
        GLOBAL.get_deallocated() - self.bf_deallocated
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

/// Accumulative stat object that can give your some real time statistics.
#[derive(Debug, Clone)]
pub struct LiveStatistics {
    stats: Histogram<u64>, // u64 is the Counter type.
}

impl LiveStatistics {
    pub fn new_unbounded() -> Self {
        LiveStatistics {
            stats: Histogram::<u64>::new(3).unwrap(),
        }
    }

    pub fn new_with_max(max: u64) -> Self {
        LiveStatistics {
            stats: Histogram::<u64>::new_with_bounds(1, max, 3).unwrap(),
        }
    }

    #[inline]
    pub fn min(&self) -> u64 {
        self.stats.min()
    }

    #[inline]
    pub fn max(&self) -> u64 {
        self.stats.max()
    }

    #[inline]
    pub fn mean(&self) -> f64 {
        self.stats.mean()
    }

    #[inline]
    pub fn percentile(&self, percentile: f64) -> u64 {
        self.stats.value_at_quantile(percentile)
    }

    /// Adds a value to the statistics.
    #[inline]
    pub fn record(&mut self, value: u64) {
        let maybe_error = self.stats.record(value);
        if let Err(e) = maybe_error {
            debug!("stats.record errored out: {}", e.to_string());
        }
    }

    #[inline]
    pub fn len(&self) -> u64 {
        self.stats.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.stats.len() == 0
    }

    #[inline]
    pub fn reset(&mut self) {
        self.stats.reset();
    }
}

/// A Specialized statistics object for CuDuration.
/// It will also keep track of the jitter between the values.
#[derive(Debug, Clone)]
pub struct CuDurationStatistics {
    bare: LiveStatistics,
    jitter: LiveStatistics,
    last_value: CuDuration,
}

impl CuDurationStatistics {
    pub fn new(max: CuDuration) -> Self {
        CuDurationStatistics {
            bare: LiveStatistics::new_with_max(max.0),
            jitter: LiveStatistics::new_with_max(max.0),
            last_value: CuDuration::default(),
        }
    }

    #[inline]
    pub fn min(&self) -> CuDuration {
        CuDuration(self.bare.min())
    }

    #[inline]
    pub fn max(&self) -> CuDuration {
        CuDuration(self.bare.max())
    }

    #[inline]
    pub fn mean(&self) -> CuDuration {
        CuDuration(self.bare.mean() as u64) // CuDuration is in ns, it is ok.
    }

    #[inline]
    pub fn percentile(&self, percentile: f64) -> CuDuration {
        CuDuration(self.bare.percentile(percentile))
    }

    #[inline]
    pub fn stddev(&self) -> CuDuration {
        CuDuration(self.bare.stats.stdev() as u64)
    }

    #[inline]
    pub fn len(&self) -> u64 {
        self.bare.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.bare.len() == 0
    }

    #[inline]
    pub fn jitter_min(&self) -> CuDuration {
        CuDuration(self.jitter.min())
    }

    #[inline]
    pub fn jitter_max(&self) -> CuDuration {
        CuDuration(self.jitter.max())
    }

    #[inline]
    pub fn jitter_mean(&self) -> CuDuration {
        CuDuration(self.jitter.mean() as u64)
    }

    #[inline]
    pub fn jitter_stddev(&self) -> CuDuration {
        CuDuration(self.jitter.stats.stdev() as u64)
    }

    #[inline]
    pub fn jitter_percentile(&self, percentile: f64) -> CuDuration {
        CuDuration(self.jitter.percentile(percentile))
    }

    #[inline]
    pub fn record(&mut self, value: CuDuration) {
        if self.bare.is_empty() {
            self.bare.stats.record(value.0).unwrap();
            self.last_value = value;
            return;
        }
        self.bare.stats.record(value.0).unwrap();
        self.jitter
            .stats
            .record(value.0.abs_diff(self.last_value.0))
            .unwrap();
        self.last_value = value;
    }

    #[inline]
    pub fn reset(&mut self) {
        self.bare.reset();
        self.jitter.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_live_statistics() {
        let mut stats = LiveStatistics::new_unbounded();
        stats.record(1);
        stats.record(2);
        stats.record(3);
        stats.record(4);
        stats.record(5);
        assert_eq!(stats.min(), 1);
        assert_eq!(stats.max(), 5);
        assert_eq!(stats.mean(), 3.0);
        assert_eq!(stats.percentile(0.5), 3);
        assert_eq!(stats.percentile(0.90), 5);
        assert_eq!(stats.percentile(0.99), 5);
        assert_eq!(stats.len(), 5);
        stats.reset();
        assert_eq!(stats.len(), 0);
    }

    #[test]
    fn test_duration_stats() {
        let mut stats = CuDurationStatistics::new(CuDuration(100));
        stats.record(CuDuration(100));
        stats.record(CuDuration(200));
        stats.record(CuDuration(500));
        stats.record(CuDuration(400));
        assert_eq!(stats.min(), CuDuration(100));
        assert_eq!(stats.max(), CuDuration(500));
        assert_eq!(stats.mean(), CuDuration(300));
        assert_eq!(stats.percentile(0.5), CuDuration(200));
        assert_eq!(stats.percentile(0.90), CuDuration(500));
        assert_eq!(stats.percentile(0.99), CuDuration(500));
        assert_eq!(stats.len(), 4);
        assert_eq!(stats.jitter.len(), 3);
        assert_eq!(stats.jitter_min(), CuDuration(100));
        assert_eq!(stats.jitter_max(), CuDuration(300));
        assert_eq!(stats.jitter_mean(), CuDuration((100 + 300 + 100) / 3));
        assert_eq!(stats.jitter_percentile(0.5), CuDuration(100));
        assert_eq!(stats.jitter_percentile(0.90), CuDuration(300));
        assert_eq!(stats.jitter_percentile(0.99), CuDuration(300));
        stats.reset();
        assert_eq!(stats.len(), 0);
    }
}
