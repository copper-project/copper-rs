#[cfg(feature = "log_pane")]
use crate::logpane::StyledLine;
use compact_str::{CompactString, ToCompactString};
use cu29::clock::CuDuration;
use cu29::cutask::CuMsgMetadata;
use cu29::monitoring::{
    ComponentId, CopperListInfo, CopperListIoStats, CopperListView, CuDurationStatistics,
    CuMonitoringMetadata, MonitorComponentMetadata, MonitorTopology,
};
use cu29::prelude::{CuCompactString, CuTime, pool};
#[cfg(feature = "log_pane")]
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use web_time::{Duration, Instant};

const COPPERLIST_RATE_WINDOW: Duration = Duration::from_secs(1);
#[cfg(feature = "log_pane")]
const DEFAULT_LOG_CAPACITY: usize = 1_024;

#[derive(Clone)]
pub struct MonitorModel {
    pub(crate) inner: Arc<MonitorModelInner>,
}

pub(crate) struct MonitorModelInner {
    pub(crate) components: &'static [MonitorComponentMetadata],
    pub(crate) topology: MonitorTopology,
    pub(crate) component_stats: Mutex<ComponentStats>,
    pub(crate) component_statuses: Mutex<Vec<ComponentStatus>>,
    pub(crate) pool_stats: Mutex<Vec<PoolStats>>,
    pub(crate) copperlist_stats: Mutex<CopperListStats>,
    #[cfg(feature = "log_pane")]
    pub(crate) log_lines: Mutex<VecDeque<StyledLine>>,
}

impl MonitorModel {
    pub fn from_metadata(metadata: &CuMonitoringMetadata) -> Self {
        Self::from_parts(
            metadata.components(),
            metadata.copperlist_info(),
            metadata.topology().clone(),
        )
    }

    pub fn from_parts(
        components: &'static [MonitorComponentMetadata],
        copperlist_info: CopperListInfo,
        topology: MonitorTopology,
    ) -> Self {
        let component_count = components.len();
        let mut copperlist_stats = CopperListStats::new();
        copperlist_stats.set_info(copperlist_info);

        Self {
            inner: Arc::new(MonitorModelInner {
                components,
                topology,
                component_stats: Mutex::new(ComponentStats::new(
                    component_count,
                    CuDuration::from(Duration::from_secs(5)),
                )),
                component_statuses: Mutex::new(vec![ComponentStatus::default(); component_count]),
                pool_stats: Mutex::new(Vec::new()),
                copperlist_stats: Mutex::new(copperlist_stats),
                #[cfg(feature = "log_pane")]
                log_lines: Mutex::new(VecDeque::with_capacity(DEFAULT_LOG_CAPACITY)),
            }),
        }
    }

    pub fn components(&self) -> &'static [MonitorComponentMetadata] {
        self.inner.components
    }

    pub fn topology(&self) -> &MonitorTopology {
        &self.inner.topology
    }

    pub fn component_count(&self) -> usize {
        self.inner.components.len()
    }

    pub fn set_copperlist_info(&self, info: CopperListInfo) {
        self.inner.copperlist_stats.lock().unwrap().set_info(info);
    }

    pub fn last_seen_copperlist_id(&self) -> Option<u64> {
        self.inner.copperlist_stats.lock().unwrap().last_seen_clid
    }

    pub fn record_component_latency(&self, component_id: ComponentId, duration: CuDuration) {
        let mut component_stats = self.inner.component_stats.lock().unwrap();
        if let Some(stat) = component_stats.stats.get_mut(component_id.index()) {
            stat.record(duration);
        }
    }

    pub fn record_end_to_end_latency(&self, duration: CuDuration) {
        self.inner
            .component_stats
            .lock()
            .unwrap()
            .end2end
            .record(duration);
    }

    pub fn reset_latency(&self) {
        self.inner.component_stats.lock().unwrap().reset();
    }

    pub fn set_component_status(
        &self,
        component_id: ComponentId,
        status_txt: impl ToCompactString,
    ) {
        if let Some(status) = self
            .inner
            .component_statuses
            .lock()
            .unwrap()
            .get_mut(component_id.index())
        {
            status.status_txt = status_txt.to_compact_string();
        }
    }

    pub fn set_component_error(&self, component_id: ComponentId, error_txt: impl ToCompactString) {
        if let Some(status) = self
            .inner
            .component_statuses
            .lock()
            .unwrap()
            .get_mut(component_id.index())
        {
            status.is_error = true;
            status.error = error_txt.to_compact_string();
        }
    }

    pub fn clear_component_error(&self, component_id: ComponentId) {
        if let Some(status) = self
            .inner
            .component_statuses
            .lock()
            .unwrap()
            .get_mut(component_id.index())
        {
            status.is_error = false;
            status.error.clear();
        }
    }

    pub fn observe_copperlist_io(&self, stats: CopperListIoStats) {
        self.inner.copperlist_stats.lock().unwrap().update_io(stats);
    }

    pub fn update_copperlist_rate(&self, clid: u64) {
        self.inner
            .copperlist_stats
            .lock()
            .unwrap()
            .update_rate(clid);
    }

    pub fn upsert_pool_stat(
        &self,
        id: impl ToCompactString,
        space_left: usize,
        total_size: usize,
        buffer_size: usize,
    ) {
        let id = id.to_compact_string();
        let mut pool_stats = self.inner.pool_stats.lock().unwrap();
        if let Some(existing) = pool_stats.iter_mut().find(|pool| pool.id == id) {
            existing.buffer_size = buffer_size;
            existing.update(space_left, total_size);
        } else {
            pool_stats.push(PoolStats::new(id, space_left, total_size, buffer_size));
        }
    }

    pub fn refresh_pool_stats_from_runtime(&self) {
        let pool_stats_data = pool::pools_statistics();
        for (id, space_left, total_size, buffer_size) in pool_stats_data {
            self.upsert_pool_stat(id.to_string(), space_left, total_size, buffer_size);
        }
    }

    #[cfg(feature = "log_pane")]
    pub fn push_log_line(&self, line: StyledLine) {
        if line.text.is_empty() {
            return;
        }

        let mut log_lines = self.inner.log_lines.lock().unwrap();
        log_lines.push_back(line);
        while log_lines.len() > DEFAULT_LOG_CAPACITY {
            log_lines.pop_front();
        }
    }

    #[cfg(feature = "log_pane")]
    pub fn log_lines(&self) -> Vec<StyledLine> {
        self.inner
            .log_lines
            .lock()
            .unwrap()
            .iter()
            .cloned()
            .collect()
    }

    #[cfg(feature = "log_pane")]
    pub fn log_line_count(&self) -> usize {
        self.inner.log_lines.lock().unwrap().len()
    }

    pub fn process_copperlist(&self, copperlist_id: u64, view: CopperListView<'_>) {
        self.inner.component_stats.lock().unwrap().update(view);
        self.update_copperlist_rate(copperlist_id);

        let mut component_statuses = self.inner.component_statuses.lock().unwrap();
        for entry in view.entries() {
            let component_index = entry.component_id.index();
            assert!(
                component_index < component_statuses.len(),
                "cu_tuimon: mapped component index {} out of component_statuses bounds {}",
                component_index,
                component_statuses.len()
            );
            let CuCompactString(status_txt) = &entry.msg.status_txt;
            component_statuses[component_index].status_txt = status_txt.clone();
        }
        drop(component_statuses);

        self.refresh_pool_stats_from_runtime();
    }
}

pub(crate) struct ComponentStats {
    pub(crate) stats: Vec<CuDurationStatistics>,
    pub(crate) end2end: CuDurationStatistics,
}

impl ComponentStats {
    fn new(component_count: usize, max_duration: CuDuration) -> Self {
        let stats = vec![CuDurationStatistics::new(max_duration); component_count];
        Self {
            stats,
            end2end: CuDurationStatistics::new(max_duration),
        }
    }

    fn update(&mut self, view: CopperListView<'_>) {
        for entry in view.entries() {
            let component_index = entry.component_id.index();
            assert!(
                component_index < self.stats.len(),
                "cu_tuimon: mapped component index {} out of stats bounds {}",
                component_index,
                self.stats.len()
            );
            let msg = entry.msg;
            let before = Option::<CuTime>::from(msg.process_time.start);
            let after = Option::<CuTime>::from(msg.process_time.end);
            if let (Some(before), Some(after)) = (before, after)
                && after >= before
            {
                self.stats[component_index].record(after - before);
            }
        }
        self.end2end.record(compute_end_to_end_latency(view.msgs()));
    }

    fn reset(&mut self) {
        for stat in &mut self.stats {
            stat.reset();
        }
        self.end2end.reset();
    }
}

#[derive(Default, Clone)]
pub(crate) struct ComponentStatus {
    pub(crate) is_error: bool,
    pub(crate) status_txt: CompactString,
    pub(crate) error: CompactString,
}

pub(crate) struct PoolStats {
    pub(crate) id: CompactString,
    pub(crate) space_left: usize,
    pub(crate) total_size: usize,
    pub(crate) buffer_size: usize,
    pub(crate) handles_in_use: usize,
    pub(crate) handles_per_second: usize,
    last_update: Instant,
}

impl PoolStats {
    fn new(
        id: impl ToCompactString,
        space_left: usize,
        total_size: usize,
        buffer_size: usize,
    ) -> Self {
        Self {
            id: id.to_compact_string(),
            space_left,
            total_size,
            buffer_size,
            handles_in_use: total_size.saturating_sub(space_left),
            handles_per_second: 0,
            last_update: Instant::now(),
        }
    }

    fn update(&mut self, space_left: usize, total_size: usize) {
        let now = Instant::now();
        let handles_in_use = total_size.saturating_sub(space_left);
        let elapsed = now.duration_since(self.last_update).as_secs_f32();

        if elapsed >= 1.0 {
            self.handles_per_second =
                ((handles_in_use.abs_diff(self.handles_in_use)) as f32 / elapsed) as usize;
            self.last_update = now;
        }

        self.handles_in_use = handles_in_use;
        self.space_left = space_left;
        self.total_size = total_size;
    }
}

pub(crate) struct CopperListStats {
    pub(crate) size_bytes: usize,
    pub(crate) raw_culist_bytes: u64,
    pub(crate) handle_bytes: u64,
    pub(crate) encoded_bytes: u64,
    pub(crate) keyframe_bytes: u64,
    pub(crate) structured_total_bytes: u64,
    pub(crate) structured_bytes_per_cl: u64,
    pub(crate) total_copperlists: u64,
    pub(crate) window_copperlists: u64,
    pub(crate) last_seen_clid: Option<u64>,
    last_rate_at: Instant,
    pub(crate) rate_hz: f64,
}

impl CopperListStats {
    fn new() -> Self {
        Self {
            size_bytes: 0,
            raw_culist_bytes: 0,
            handle_bytes: 0,
            encoded_bytes: 0,
            keyframe_bytes: 0,
            structured_total_bytes: 0,
            structured_bytes_per_cl: 0,
            total_copperlists: 0,
            window_copperlists: 0,
            last_seen_clid: None,
            last_rate_at: Instant::now(),
            rate_hz: 0.0,
        }
    }

    fn set_info(&mut self, info: CopperListInfo) {
        self.size_bytes = info.size_bytes;
    }

    fn update_io(&mut self, stats: CopperListIoStats) {
        self.raw_culist_bytes = stats.raw_culist_bytes;
        self.handle_bytes = stats.handle_bytes;
        self.encoded_bytes = stats.encoded_culist_bytes;
        self.keyframe_bytes = stats.keyframe_bytes;
        let total = stats.structured_log_bytes_total;
        self.structured_bytes_per_cl = total.saturating_sub(self.structured_total_bytes);
        self.structured_total_bytes = total;
    }

    fn update_rate(&mut self, clid: u64) {
        let newly_seen = self
            .last_seen_clid
            .map_or(1, |prev| clid.wrapping_sub(prev));
        self.last_seen_clid = Some(clid);
        self.total_copperlists = self.total_copperlists.saturating_add(newly_seen);
        self.window_copperlists = self.window_copperlists.saturating_add(newly_seen);

        let now = Instant::now();
        let elapsed = now.duration_since(self.last_rate_at);
        if elapsed >= COPPERLIST_RATE_WINDOW {
            let elapsed_secs = elapsed.as_secs_f64();
            self.rate_hz = if elapsed_secs > 0.0 {
                self.window_copperlists as f64 / elapsed_secs
            } else {
                0.0
            };
            self.window_copperlists = 0;
            self.last_rate_at = now;
        }
    }
}

fn compute_end_to_end_latency(msgs: &[&CuMsgMetadata]) -> CuDuration {
    let start = msgs.first().map(|msg| msg.process_time.start);
    let end = msgs.last().map(|msg| msg.process_time.end);

    if let (Some(start), Some(end)) = (start, end)
        && let (Some(start), Some(end)) =
            (Option::<CuTime>::from(start), Option::<CuTime>::from(end))
        && end >= start
    {
        end - start
    } else {
        CuDuration::MIN
    }
}
