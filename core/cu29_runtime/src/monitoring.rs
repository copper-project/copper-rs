//! Some basic internal monitoring tooling Copper uses to monitor itself and the tasks it is running.
//!

use crate::config::CuConfig;
use crate::config::{BridgeChannelConfigRepresentation, BridgeConfig, Flavor};
use crate::cutask::CuMsgMetadata;
use cu29_clock::{CuDuration, RobotClock};
#[allow(unused_imports)]
use cu29_log::CuLogLevel;
use cu29_traits::{CuError, CuResult};
use petgraph::visit::IntoEdgeReferences;
use serde_derive::{Deserialize, Serialize};

#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(feature = "std")]
use std::{collections::HashMap as Map, format, string::String, string::ToString, vec::Vec};

#[cfg(not(feature = "std"))]
use alloc::{collections::BTreeMap as Map, format, string::String, string::ToString, vec::Vec};

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::alloc::{GlobalAlloc, Layout};
    pub use core::sync::atomic::{AtomicUsize, Ordering};
    pub use libm::sqrt;
}

#[cfg(feature = "std")]
mod imp {
    #[cfg(feature = "memory_monitoring")]
    use super::CountingAlloc;
    #[cfg(feature = "memory_monitoring")]
    pub use std::alloc::System;
    pub use std::alloc::{GlobalAlloc, Layout};
    pub use std::sync::atomic::{AtomicUsize, Ordering};
    #[cfg(feature = "memory_monitoring")]
    #[global_allocator]
    pub static GLOBAL: CountingAlloc<System> = CountingAlloc::new(System);
}

use imp::*;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComponentKind {
    Task,
    Bridge,
}

#[derive(Debug, Clone)]
pub struct MonitorNode {
    pub id: String,
    pub type_name: Option<String>,
    pub kind: ComponentKind,
    /// Ordered list of input port identifiers.
    pub inputs: Vec<String>,
    /// Ordered list of output port identifiers.
    pub outputs: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct MonitorConnection {
    pub src: String,
    pub src_port: Option<String>,
    pub dst: String,
    pub dst_port: Option<String>,
    pub msg: String,
}

#[derive(Debug, Clone, Default)]
pub struct MonitorTopology {
    pub nodes: Vec<MonitorNode>,
    pub connections: Vec<MonitorConnection>,
}

/// Derive a monitor-friendly topology from the runtime configuration.
pub fn build_monitor_topology(
    config: &CuConfig,
    mission: Option<&str>,
) -> CuResult<MonitorTopology> {
    let graph = config.get_graph(mission)?;
    let mut nodes: Map<String, MonitorNode> = Map::new();

    let mut bridge_lookup: Map<&str, &BridgeConfig> = Map::new();
    for bridge in &config.bridges {
        bridge_lookup.insert(bridge.id.as_str(), bridge);
    }

    for (_, node) in graph.get_all_nodes() {
        let kind = match node.get_flavor() {
            Flavor::Bridge => ComponentKind::Bridge,
            _ => ComponentKind::Task,
        };
        let node_id = node.get_id();

        let mut inputs = Vec::new();
        let mut outputs = Vec::new();
        if kind == ComponentKind::Bridge {
            if let Some(bridge) = bridge_lookup.get(node_id.as_str()) {
                for ch in &bridge.channels {
                    match ch {
                        BridgeChannelConfigRepresentation::Rx { id, .. } => inputs.push(id.clone()),
                        BridgeChannelConfigRepresentation::Tx { id, .. } => {
                            outputs.push(id.clone())
                        }
                    }
                }
            }
        }

        nodes.insert(
            node_id.clone(),
            MonitorNode {
                id: node_id,
                type_name: Some(node.get_type().to_string()),
                kind,
                inputs,
                outputs,
            },
        );
    }

    let mut connections = Vec::new();
    for edge in graph.0.edge_references() {
        let cnx = edge.weight();
        let src = cnx.src.clone();
        let dst = cnx.dst.clone();

        let src_port = cnx.src_channel.clone();
        let dst_port = cnx.dst_channel.clone();

        // ensure ports exist for tasks if bridges did not predeclare them.
        if let Some(node) = nodes.get_mut(&src) {
            if node.kind == ComponentKind::Task && src_port.is_none() && node.outputs.is_empty() {
                node.outputs.push("out0".to_string());
            }
        }
        if let Some(node) = nodes.get_mut(&dst) {
            if node.kind == ComponentKind::Task && dst_port.is_none() {
                let next = format!("in{}", node.inputs.len());
                node.inputs.push(next);
            }
        }

        connections.push(MonitorConnection {
            src,
            src_port,
            dst,
            dst_port,
            msg: cnx.msg.clone(),
        });
    }

    Ok(MonitorTopology {
        nodes: nodes.into_values().collect(),
        connections,
    })
}

/// Trait to implement a monitoring task.
pub trait CuMonitor: Sized {
    fn new(config: &CuConfig, taskids: &'static [&'static str]) -> CuResult<Self>
    where
        Self: Sized;

    fn set_topology(&mut self, _topology: MonitorTopology) {}

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

/// A simple allocator that counts the number of bytes allocated and deallocated.
pub struct CountingAlloc<A: GlobalAlloc> {
    inner: A,
    allocated: AtomicUsize,
    deallocated: AtomicUsize,
}

impl<A: GlobalAlloc> CountingAlloc<A> {
    pub const fn new(inner: A) -> Self {
        CountingAlloc {
            inner,
            allocated: AtomicUsize::new(0),
            deallocated: AtomicUsize::new(0),
        }
    }

    pub fn allocated(&self) -> usize {
        self.allocated.load(Ordering::SeqCst)
    }

    pub fn deallocated(&self) -> usize {
        self.deallocated.load(Ordering::SeqCst)
    }

    pub fn reset(&self) {
        self.allocated.store(0, Ordering::SeqCst);
        self.deallocated.store(0, Ordering::SeqCst);
    }
}

unsafe impl<A: GlobalAlloc> GlobalAlloc for CountingAlloc<A> {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = self.inner.alloc(layout);
        if !p.is_null() {
            self.allocated.fetch_add(layout.size(), Ordering::SeqCst);
        }
        p
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        self.inner.dealloc(ptr, layout);
        self.deallocated.fetch_add(layout.size(), Ordering::SeqCst);
    }
}

/// A simple struct that counts the number of bytes allocated and deallocated in a scope.
#[cfg(feature = "memory_monitoring")]
pub struct ScopedAllocCounter {
    bf_allocated: usize,
    bf_deallocated: usize,
}

#[cfg(feature = "memory_monitoring")]
impl Default for ScopedAllocCounter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "memory_monitoring")]
impl ScopedAllocCounter {
    pub fn new() -> Self {
        ScopedAllocCounter {
            bf_allocated: GLOBAL.allocated(),
            bf_deallocated: GLOBAL.deallocated(),
        }
    }

    /// Returns the total number of bytes allocated in the current scope
    /// since the creation of this `ScopedAllocCounter`.
    ///
    /// # Example
    /// ```
    /// use cu29_runtime::monitoring::ScopedAllocCounter;
    ///
    /// let counter = ScopedAllocCounter::new();
    /// let _vec = vec![0u8; 1024];
    /// println!("Bytes allocated: {}", counter.get_allocated());
    /// ```
    pub fn allocated(&self) -> usize {
        GLOBAL.allocated() - self.bf_allocated
    }

    /// Returns the total number of bytes deallocated in the current scope
    /// since the creation of this `ScopedAllocCounter`.
    ///
    /// # Example
    /// ```
    /// use cu29_runtime::monitoring::ScopedAllocCounter;
    ///
    /// let counter = ScopedAllocCounter::new();
    /// let _vec = vec![0u8; 1024];
    /// drop(_vec);
    /// println!("Bytes deallocated: {}", counter.get_deallocated());
    /// ```
    pub fn deallocated(&self) -> usize {
        GLOBAL.deallocated() - self.bf_deallocated
    }
}

/// Build a difference between the number of bytes allocated and deallocated in the scope at drop time.
#[cfg(feature = "memory_monitoring")]
impl Drop for ScopedAllocCounter {
    fn drop(&mut self) {
        let _allocated = GLOBAL.allocated() - self.bf_allocated;
        let _deallocated = GLOBAL.deallocated() - self.bf_deallocated;
        // TODO(gbin): Fix this when the logger is ready.
        // debug!(
        //     "Allocations: +{}B -{}B",
        //     allocated = allocated,
        //     deallocated = deallocated,
        // );
    }
}

const BUCKET_COUNT: usize = 1024;

/// Accumulative stat object that can give your some real time statistics.
/// Uses a fixed-size bucketed histogram for accurate percentile calculations.
#[derive(Debug, Clone)]
pub struct LiveStatistics {
    buckets: [u64; BUCKET_COUNT],
    min_val: u64,
    max_val: u64,
    sum: u64,
    sum_sq: u64,
    count: u64,
    max_value: u64,
}

impl LiveStatistics {
    /// Creates a new `LiveStatistics` instance with a specified maximum value.
    ///
    /// This function initializes a `LiveStatistics` structure with default values
    /// for tracking statistical data, while setting an upper limit for the data
    /// points that the structure tracks.
    ///
    /// # Parameters
    /// - `max_value` (`u64`): The maximum value that can be recorded or tracked.
    ///
    /// # Returns
    /// A new instance of `LiveStatistics` with:
    /// - `buckets`: An array pre-filled with zeros to categorize data points.
    /// - `min_val`: Initialized to the maximum possible `u64` value to track the minimum correctly.
    /// - `max_val`: Initialized to zero.
    /// - `sum`: The sum of all data points, initialized to zero.
    /// - `sum_sq`: The sum of squares of all data points, initialized to zero.
    /// - `count`: The total number of data points, initialized to zero.
    /// - `max_value`: The maximum allowable value for data points, set to the provided `max_value`.
    ///
    pub fn new_with_max(max_value: u64) -> Self {
        LiveStatistics {
            buckets: [0; BUCKET_COUNT],
            min_val: u64::MAX,
            max_val: 0,
            sum: 0,
            sum_sq: 0,
            count: 0,
            max_value,
        }
    }

    #[inline]
    fn value_to_bucket(&self, value: u64) -> usize {
        if value >= self.max_value {
            BUCKET_COUNT - 1
        } else {
            ((value as u128 * BUCKET_COUNT as u128) / self.max_value as u128) as usize
        }
    }

    #[inline]
    pub fn min(&self) -> u64 {
        if self.count == 0 {
            0
        } else {
            self.min_val
        }
    }

    #[inline]
    pub fn max(&self) -> u64 {
        self.max_val
    }

    #[inline]
    pub fn mean(&self) -> f64 {
        if self.count == 0 {
            0.0
        } else {
            self.sum as f64 / self.count as f64
        }
    }

    #[inline]
    pub fn stdev(&self) -> f64 {
        if self.count == 0 {
            return 0.0;
        }
        let mean = self.mean();
        let variance = (self.sum_sq as f64 / self.count as f64) - (mean * mean);
        if variance < 0.0 {
            return 0.0;
        }
        #[cfg(feature = "std")]
        return variance.sqrt();
        #[cfg(not(feature = "std"))]
        return sqrt(variance);
    }

    #[inline]
    pub fn percentile(&self, percentile: f64) -> u64 {
        if self.count == 0 {
            return 0;
        }

        let target_count = (self.count as f64 * percentile) as u64;
        let mut accumulated = 0u64;

        for (bucket_idx, &bucket_count) in self.buckets.iter().enumerate() {
            accumulated += bucket_count;
            if accumulated >= target_count {
                // Linear interpolation within the bucket
                let bucket_start = (bucket_idx as u64 * self.max_value) / BUCKET_COUNT as u64;
                let bucket_end = ((bucket_idx + 1) as u64 * self.max_value) / BUCKET_COUNT as u64;
                let bucket_fraction = if bucket_count > 0 {
                    (target_count - (accumulated - bucket_count)) as f64 / bucket_count as f64
                } else {
                    0.5
                };
                return bucket_start
                    + ((bucket_end - bucket_start) as f64 * bucket_fraction) as u64;
            }
        }

        self.max_val
    }

    /// Adds a value to the statistics.
    #[inline]
    pub fn record(&mut self, value: u64) {
        if value < self.min_val {
            self.min_val = value;
        }
        if value > self.max_val {
            self.max_val = value;
        }
        self.sum += value;
        self.sum_sq += value * value;
        self.count += 1;

        let bucket = self.value_to_bucket(value);
        self.buckets[bucket] += 1;
    }

    #[inline]
    pub fn len(&self) -> u64 {
        self.count
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    #[inline]
    pub fn reset(&mut self) {
        self.buckets.fill(0);
        self.min_val = u64::MAX;
        self.max_val = 0;
        self.sum = 0;
        self.sum_sq = 0;
        self.count = 0;
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
        let CuDuration(max) = max;
        CuDurationStatistics {
            bare: LiveStatistics::new_with_max(max),
            jitter: LiveStatistics::new_with_max(max),
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
        CuDuration(self.bare.stdev() as u64)
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
        CuDuration(self.jitter.stdev() as u64)
    }

    #[inline]
    pub fn jitter_percentile(&self, percentile: f64) -> CuDuration {
        CuDuration(self.jitter.percentile(percentile))
    }

    #[inline]
    pub fn record(&mut self, value: CuDuration) {
        let CuDuration(nanos) = value;
        if self.bare.is_empty() {
            self.bare.record(nanos);
            self.last_value = value;
            return;
        }
        self.bare.record(nanos);
        let CuDuration(last_nanos) = self.last_value;
        self.jitter.record(nanos.abs_diff(last_nanos));
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
    fn test_live_statistics_percentiles() {
        let mut stats = LiveStatistics::new_with_max(1000);

        // Record 100 values from 0 to 99
        for i in 0..100 {
            stats.record(i);
        }

        assert_eq!(stats.len(), 100);
        assert_eq!(stats.min(), 0);
        assert_eq!(stats.max(), 99);
        assert_eq!(stats.mean() as u64, 49); // Average of 0..99

        // Test percentiles - should be approximately correct
        let p50 = stats.percentile(0.5);
        let p90 = stats.percentile(0.90);
        let p95 = stats.percentile(0.95);
        let p99 = stats.percentile(0.99);

        // With 100 samples from 0-99, percentiles should be close to their index
        assert!((p50 as i64 - 49).abs() < 5, "p50={} expected ~49", p50);
        assert!((p90 as i64 - 89).abs() < 5, "p90={} expected ~89", p90);
        assert!((p95 as i64 - 94).abs() < 5, "p95={} expected ~94", p95);
        assert!((p99 as i64 - 98).abs() < 5, "p99={} expected ~98", p99);
    }

    #[test]
    fn test_duration_stats() {
        let mut stats = CuDurationStatistics::new(CuDuration(1000));
        stats.record(CuDuration(100));
        stats.record(CuDuration(200));
        stats.record(CuDuration(500));
        stats.record(CuDuration(400));
        assert_eq!(stats.min(), CuDuration(100));
        assert_eq!(stats.max(), CuDuration(500));
        assert_eq!(stats.mean(), CuDuration(300));
        assert_eq!(stats.len(), 4);
        assert_eq!(stats.jitter.len(), 3);
        assert_eq!(stats.jitter_min(), CuDuration(100));
        assert_eq!(stats.jitter_max(), CuDuration(300));
        assert_eq!(stats.jitter_mean(), CuDuration((100 + 300 + 100) / 3));
        stats.reset();
        assert_eq!(stats.len(), 0);
    }
}
