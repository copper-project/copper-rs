//! CuDebug: lightweight time-travel debugger helpers on top of Copper logs.
//!
//! Design goals:
//! - Do **not** load entire copperlists into memory (logs can be huge).
//! - Build a compact section index in one streaming pass (copperlists + keyframes).
//! - Keep keyframes in memory (much smaller) and lazily page copperlist sections
//!   with a tiny LRU cache for snappy stepping.
//! - Reuse the public `CuSimApplication` API and user-provided sim callbacks.

use crate::app::CuSimApplication;
use crate::curuntime::KeyFrame;
use crate::simulation::SimOverride;
use bincode::config::standard;
use bincode::decode_from_std_read;
use bincode::error::DecodeError;
use cu29_clock::{CuTime, RobotClock, RobotClockMock};
use cu29_traits::{CopperListTuple, CuError, CuResult, UnifiedLogType};
use cu29_unifiedlog::{
    LogPosition, SectionHeader, SectionStorage, UnifiedLogRead, UnifiedLogWrite, UnifiedLogger,
    UnifiedLoggerBuilder, UnifiedLoggerRead,
};
use std::collections::{HashMap, VecDeque};
use std::io;
use std::marker::PhantomData;
use std::path::Path;
use std::sync::Arc;

/// Result of a jump/step, useful for benchmarking cache effectiveness.
#[derive(Debug, Clone)]
pub struct JumpOutcome {
    /// Copperlist id we landed on
    pub culistid: u32,
    /// Keyframe used to rewind (if any)
    pub keyframe_culistid: Option<u32>,
    /// Number of copperlists replayed after the keyframe
    pub replayed: usize,
}

/// Metadata for one copperlist section (no payload kept).
#[derive(Debug, Clone)]
pub(crate) struct SectionIndexEntry {
    pos: LogPosition,
    start_idx: usize,
    len: usize,
    first_id: u32,
    last_id: u32,
    first_ts: Option<CuTime>,
    last_ts: Option<CuTime>,
}

/// Cached copperlists for one section.
#[derive(Debug, Clone)]
struct CachedSection<P: CopperListTuple> {
    entries: Vec<Arc<crate::copperlist::CopperList<P>>>,
    timestamps: Vec<Option<CuTime>>,
}

/// A reusable debugging session that can time-travel within a recorded log.
///
/// `CB` builds a simulation callback for a specific copperlist entry. This keeps the
/// API generic: the caller can replay recorded outputs, poke clocks, or inject extra
/// assertions inside the callback. `TF` extracts a timestamp from a copperlist to
/// support time-based seeking.
const DEFAULT_SECTION_CACHE_CAP: usize = 8;
pub struct CuDebugSession<App, P, CB, TF, S, L>
where
    P: CopperListTuple,
    S: SectionStorage,
    L: UnifiedLogWrite<S> + 'static,
{
    app: App,
    robot_clock: RobotClock,
    clock_mock: RobotClockMock,
    log_reader: UnifiedLoggerRead,
    sections: Vec<SectionIndexEntry>,
    total_entries: usize,
    keyframes: Vec<KeyFrame>,
    started: bool,
    current_idx: Option<usize>,
    last_keyframe: Option<u32>,
    build_callback: CB,
    time_of: TF,
    // Tiny LRU cache of decoded sections
    cache: HashMap<usize, CachedSection<P>>,
    cache_order: VecDeque<usize>,
    cache_cap: usize,
    phantom: PhantomData<(S, L)>,
}

impl<App, P, CB, TF, S, L> CuDebugSession<App, P, CB, TF, S, L>
where
    App: CuSimApplication<S, L>,
    L: UnifiedLogWrite<S> + 'static,
    S: SectionStorage,
    P: CopperListTuple,
    CB: for<'a> Fn(
        &'a crate::copperlist::CopperList<P>,
        RobotClock,
    ) -> Box<dyn for<'z> FnMut(App::Step<'z>) -> SimOverride + 'a>,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime> + Clone,
{
    /// Build a session directly from a unified log on disk (streaming index, no bulk load).
    pub fn from_log(
        log_base: &Path,
        app: App,
        robot_clock: RobotClock,
        clock_mock: RobotClockMock,
        build_callback: CB,
        time_of: TF,
    ) -> CuResult<Self> {
        let (sections, keyframes, total_entries) = index_log::<P, _>(log_base, &time_of)?;
        let log_reader = build_read_logger(log_base)?;
        Ok(Self::new(
            log_reader,
            app,
            robot_clock,
            clock_mock,
            sections,
            total_entries,
            keyframes,
            build_callback,
            time_of,
        ))
    }

    /// Build a session directly from a log, with an explicit cache size.
    pub fn from_log_with_cache_cap(
        log_base: &Path,
        app: App,
        robot_clock: RobotClock,
        clock_mock: RobotClockMock,
        build_callback: CB,
        time_of: TF,
        cache_cap: usize,
    ) -> CuResult<Self> {
        let (sections, keyframes, total_entries) = index_log::<P, _>(log_base, &time_of)?;
        let log_reader = build_read_logger(log_base)?;
        Ok(Self::new_with_cache_cap(
            log_reader,
            app,
            robot_clock,
            clock_mock,
            sections,
            total_entries,
            keyframes,
            build_callback,
            time_of,
            cache_cap,
        ))
    }

    /// Create a new session from prebuilt indices.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        log_reader: UnifiedLoggerRead,
        app: App,
        robot_clock: RobotClock,
        clock_mock: RobotClockMock,
        sections: Vec<SectionIndexEntry>,
        total_entries: usize,
        keyframes: Vec<KeyFrame>,
        build_callback: CB,
        time_of: TF,
    ) -> Self {
        Self::new_with_cache_cap(
            log_reader,
            app,
            robot_clock,
            clock_mock,
            sections,
            total_entries,
            keyframes,
            build_callback,
            time_of,
            DEFAULT_SECTION_CACHE_CAP,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new_with_cache_cap(
        log_reader: UnifiedLoggerRead,
        app: App,
        robot_clock: RobotClock,
        clock_mock: RobotClockMock,
        sections: Vec<SectionIndexEntry>,
        total_entries: usize,
        keyframes: Vec<KeyFrame>,
        build_callback: CB,
        time_of: TF,
        cache_cap: usize,
    ) -> Self {
        Self {
            app,
            robot_clock,
            clock_mock,
            log_reader,
            sections,
            total_entries,
            keyframes,
            started: false,
            current_idx: None,
            last_keyframe: None,
            build_callback,
            time_of,
            cache: HashMap::new(),
            cache_order: VecDeque::new(),
            cache_cap: cache_cap.max(1),
            phantom: PhantomData,
        }
    }

    fn ensure_started(&mut self) -> CuResult<()> {
        if self.started {
            return Ok(());
        }
        let mut noop = |_step: App::Step<'_>| SimOverride::ExecuteByRuntime;
        self.app.start_all_tasks(&mut noop)?;
        self.started = true;
        Ok(())
    }

    fn nearest_keyframe(&self, target_culistid: u32) -> Option<KeyFrame> {
        self.keyframes
            .iter()
            .filter(|kf| kf.culistid <= target_culistid)
            .max_by_key(|kf| kf.culistid)
            .cloned()
    }

    fn restore_keyframe(&mut self, kf: &KeyFrame) -> CuResult<()> {
        self.app.restore_keyframe(kf)?;
        self.clock_mock.set_value(kf.timestamp.as_nanos());
        self.last_keyframe = Some(kf.culistid);
        Ok(())
    }

    fn find_section_for_index(&self, idx: usize) -> Option<usize> {
        self.sections
            .binary_search_by(|s| {
                if idx < s.start_idx {
                    std::cmp::Ordering::Greater
                } else if idx >= s.start_idx + s.len {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Equal
                }
            })
            .ok()
    }

    fn find_section_for_culistid(&self, culistid: u32) -> Option<usize> {
        self.sections
            .binary_search_by(|s| {
                if culistid < s.first_id {
                    std::cmp::Ordering::Greater
                } else if culistid > s.last_id {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Equal
                }
            })
            .ok()
    }

    /// Lower-bound lookup: return the first section whose `first_ts >= ts`.
    /// If `ts` is earlier than the first section, return the first section.
    /// Return `None` only when `ts` is beyond the last section's range.
    fn find_section_for_time(&self, ts: CuTime) -> Option<usize> {
        if self.sections.is_empty() {
            return None;
        }

        // Fast path when all sections carry timestamps.
        if self.sections.iter().all(|s| s.first_ts.is_some()) {
            let idx = match self.sections.binary_search_by(|s| {
                let a = s.first_ts.unwrap();
                if a < ts {
                    std::cmp::Ordering::Less
                } else if a > ts {
                    std::cmp::Ordering::Greater
                } else {
                    std::cmp::Ordering::Equal
                }
            }) {
                Ok(i) => i,
                Err(i) => i, // insertion point = first first_ts >= ts
            };

            if idx < self.sections.len() {
                return Some(idx);
            }

            // ts is after the last section start; allow selecting the last section
            // if the timestamp still lies inside its recorded range.
            let last = self.sections.last().unwrap();
            if let Some(last_ts) = last.last_ts
                && ts <= last_ts
            {
                return Some(self.sections.len() - 1);
            }
            return None;
        }

        // Fallback for sections missing timestamps: choose first window that contains ts;
        // if ts is earlier than the first timestamped section, pick that section; otherwise
        // only return None when ts is past the last known range.
        if let Some(first_ts) = self.sections.first().and_then(|s| s.first_ts)
            && ts <= first_ts
        {
            return Some(0);
        }

        if let Some(idx) = self
            .sections
            .iter()
            .position(|s| match (s.first_ts, s.last_ts) {
                (Some(a), Some(b)) => a <= ts && ts <= b,
                (Some(a), None) => a <= ts,
                _ => false,
            })
        {
            return Some(idx);
        }

        let last = self.sections.last().unwrap();
        match last.last_ts {
            Some(b) if ts <= b => Some(self.sections.len() - 1),
            _ => None,
        }
    }

    fn touch_cache(&mut self, key: usize) {
        if let Some(pos) = self.cache_order.iter().position(|k| *k == key) {
            self.cache_order.remove(pos);
        }
        self.cache_order.push_back(key);
        while self.cache_order.len() > self.cache_cap {
            if let Some(old) = self.cache_order.pop_front() {
                self.cache.remove(&old);
            }
        }
    }

    fn load_section(&mut self, section_idx: usize) -> CuResult<&CachedSection<P>> {
        if self.cache.contains_key(&section_idx) {
            self.touch_cache(section_idx);
            // SAFETY: key exists, unwrap ok.
            return Ok(self.cache.get(&section_idx).unwrap());
        }

        let entry = &self.sections[section_idx];
        let (header, data) = read_section_at(&mut self.log_reader, entry.pos)?;
        if header.entry_type != UnifiedLogType::CopperList {
            return Err(CuError::from(
                "Section type mismatch while loading copperlists",
            ));
        }

        let (entries, timestamps) = decode_copperlists::<P, _>(&data, &self.time_of)?;
        let cached = CachedSection {
            entries,
            timestamps,
        };
        self.cache.insert(section_idx, cached);
        self.touch_cache(section_idx);
        Ok(self.cache.get(&section_idx).unwrap())
    }

    fn copperlist_at(
        &mut self,
        idx: usize,
    ) -> CuResult<(Arc<crate::copperlist::CopperList<P>>, Option<CuTime>)> {
        let section_idx = self
            .find_section_for_index(idx)
            .ok_or_else(|| CuError::from("Index outside copperlist log"))?;
        let start_idx = self.sections[section_idx].start_idx;
        let section = self.load_section(section_idx)?;
        let local = idx - start_idx;
        let cl = section
            .entries
            .get(local)
            .ok_or_else(|| CuError::from("Corrupt section index vs cache"))?
            .clone();
        let ts = section.timestamps.get(local).copied().unwrap_or(None);
        Ok((cl, ts))
    }

    fn index_for_culistid(&mut self, culistid: u32) -> CuResult<usize> {
        let section_idx = self
            .find_section_for_culistid(culistid)
            .ok_or_else(|| CuError::from("Requested culistid not present in log"))?;
        let start_idx = self.sections[section_idx].start_idx;
        let section = self.load_section(section_idx)?;
        let mut idx = start_idx;
        for cl in &section.entries {
            if cl.id == culistid {
                return Ok(idx);
            }
            idx += 1;
        }
        Err(CuError::from("culistid not found inside indexed section"))
    }

    fn index_for_time(&mut self, ts: CuTime) -> CuResult<usize> {
        let section_idx = self
            .find_section_for_time(ts)
            .ok_or_else(|| CuError::from("No copperlist at or after requested timestamp"))?;
        let start_idx = self.sections[section_idx].start_idx;
        let section = self.load_section(section_idx)?;
        let idx = start_idx;
        for (i, maybe) in section.timestamps.iter().enumerate() {
            if matches!(maybe, Some(t) if *t >= ts) {
                return Ok(idx + i);
            }
        }
        Err(CuError::from("Timestamp not found within section"))
    }

    fn replay_range(&mut self, start: usize, end: usize) -> CuResult<usize> {
        let mut replayed = 0usize;
        for idx in start..=end {
            let (entry, ts) = self.copperlist_at(idx)?;
            if let Some(ts) = ts {
                self.clock_mock.set_value(ts.as_nanos());
            }
            let clock_for_cb = self.robot_clock.clone();
            let mut cb = (self.build_callback)(entry.as_ref(), clock_for_cb);
            self.app.run_one_iteration(&mut cb)?;
            replayed += 1;
            self.current_idx = Some(idx);
        }
        Ok(replayed)
    }

    fn goto_index(&mut self, target_idx: usize) -> CuResult<JumpOutcome> {
        self.ensure_started()?;
        if target_idx >= self.total_entries {
            return Err(CuError::from("Target index outside log"));
        }
        let (target_cl, _) = self.copperlist_at(target_idx)?;
        let target_culistid = target_cl.id;

        let keyframe_used: Option<u32>;
        let replay_start: usize;

        // Fast path: forward stepping from current state.
        if let Some(current) = self.current_idx {
            if target_idx >= current {
                replay_start = current + 1;
                keyframe_used = self.last_keyframe;
            } else {
                // Need to rewind to nearest keyframe
                let Some(kf) = self.nearest_keyframe(target_culistid) else {
                    return Err(CuError::from("No keyframe available to rewind"));
                };
                self.restore_keyframe(&kf)?;
                keyframe_used = Some(kf.culistid);
                replay_start = self.index_for_culistid(kf.culistid)?;
            }
        } else {
            // First jump: align to nearest keyframe
            let Some(kf) = self.nearest_keyframe(target_culistid) else {
                return Err(CuError::from("No keyframe found in log"));
            };
            self.restore_keyframe(&kf)?;
            keyframe_used = Some(kf.culistid);
            replay_start = self.index_for_culistid(kf.culistid)?;
        }

        if replay_start > target_idx {
            return Err(CuError::from(
                "Replay start past target index; log ordering issue",
            ));
        }

        let replayed = self.replay_range(replay_start, target_idx)?;

        Ok(JumpOutcome {
            culistid: target_culistid,
            keyframe_culistid: keyframe_used,
            replayed,
        })
    }

    /// Jump to a copperlist by id.
    pub fn goto_cl(&mut self, culistid: u32) -> CuResult<JumpOutcome> {
        let idx = self.index_for_culistid(culistid)?;
        self.goto_index(idx)
    }

    /// Jump to the first copperlist at or after a timestamp.
    pub fn goto_time(&mut self, ts: CuTime) -> CuResult<JumpOutcome> {
        let idx = self.index_for_time(ts)?;
        self.goto_index(idx)
    }

    /// Step relative to the current cursor. Negative values rewind via keyframe.
    pub fn step(&mut self, delta: i32) -> CuResult<JumpOutcome> {
        let current =
            self.current_idx
                .ok_or_else(|| CuError::from("Cannot step before any jump"))? as i32;
        let target = current + delta;
        if target < 0 || target as usize >= self.total_entries {
            return Err(CuError::from("Step would move outside log bounds"));
        }
        self.goto_index(target as usize)
    }

    /// Access the copperlist at the current cursor, if any (cloned).
    pub fn current_cl(&mut self) -> CuResult<Option<Arc<crate::copperlist::CopperList<P>>>> {
        match self.current_idx {
            Some(idx) => Ok(Some(self.copperlist_at(idx)?.0)),
            None => Ok(None),
        }
    }

    /// Access a copperlist by absolute index in the log (cloned).
    pub fn cl_at(&mut self, idx: usize) -> CuResult<Option<Arc<crate::copperlist::CopperList<P>>>> {
        if idx >= self.total_entries {
            return Ok(None);
        }
        Ok(Some(self.copperlist_at(idx)?.0))
    }

    /// Borrow the underlying application for inspection (e.g., task state asserts).
    pub fn with_app<R>(&mut self, f: impl FnOnce(&mut App) -> R) -> R {
        f(&mut self.app)
    }
}

/// Decode all copperlists contained in a single unified-log section.
#[allow(clippy::type_complexity)]
fn decode_copperlists<
    P: CopperListTuple,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime>,
>(
    section: &[u8],
    time_of: &TF,
) -> CuResult<(
    Vec<Arc<crate::copperlist::CopperList<P>>>,
    Vec<Option<CuTime>>,
)> {
    let mut cursor = std::io::Cursor::new(section);
    let mut entries = Vec::new();
    let mut timestamps = Vec::new();
    loop {
        match decode_from_std_read::<crate::copperlist::CopperList<P>, _, _>(
            &mut cursor,
            standard(),
        ) {
            Ok(cl) => {
                timestamps.push(time_of(&cl));
                entries.push(Arc::new(cl));
            }
            Err(DecodeError::UnexpectedEnd { .. }) => break,
            Err(DecodeError::Io { inner, .. }) if inner.kind() == io::ErrorKind::UnexpectedEof => {
                break;
            }
            Err(e) => {
                return Err(CuError::new_with_cause(
                    "Failed to decode CopperList section",
                    e,
                ));
            }
        }
    }
    Ok((entries, timestamps))
}

/// Scan a copperlist section for metadata only.
#[allow(clippy::type_complexity)]
fn scan_copperlist_section<
    P: CopperListTuple,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime>,
>(
    section: &[u8],
    time_of: &TF,
) -> CuResult<(usize, u32, u32, Option<CuTime>, Option<CuTime>)> {
    let mut cursor = std::io::Cursor::new(section);
    let mut count = 0usize;
    let mut first_id = None;
    let mut last_id = None;
    let mut first_ts = None;
    let mut last_ts = None;
    loop {
        match decode_from_std_read::<crate::copperlist::CopperList<P>, _, _>(
            &mut cursor,
            standard(),
        ) {
            Ok(cl) => {
                let ts = time_of(&cl);
                if ts.is_none() {
                    #[cfg(feature = "std")]
                    eprintln!(
                        "CuDebug index warning: missing timestamp on culistid {}; time-based seek may be less accurate",
                        cl.id
                    );
                }
                if first_id.is_none() {
                    first_id = Some(cl.id);
                    first_ts = ts;
                }
                // Recover first_ts if the first entry lacked a timestamp but a later one has it.
                if first_ts.is_none() {
                    first_ts = ts;
                }
                last_id = Some(cl.id);
                last_ts = ts.or(last_ts);
                count += 1;
            }
            Err(DecodeError::UnexpectedEnd { .. }) => break,
            Err(DecodeError::Io { inner, .. }) if inner.kind() == io::ErrorKind::UnexpectedEof => {
                break;
            }
            Err(e) => {
                return Err(CuError::new_with_cause(
                    "Failed to scan copperlist section",
                    e,
                ));
            }
        }
    }
    let first_id = first_id.ok_or_else(|| CuError::from("Empty copperlist section"))?;
    let last_id = last_id.unwrap_or(first_id);
    Ok((count, first_id, last_id, first_ts, last_ts))
}

/// Build a reusable read-only unified logger for this session.
fn build_read_logger(log_base: &Path) -> CuResult<UnifiedLoggerRead> {
    let logger = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|e| CuError::new_with_cause("Failed to open unified log", e))?;
    let UnifiedLogger::Read(dl) = logger else {
        return Err(CuError::from("Expected read-only unified logger"));
    };
    Ok(dl)
}

/// Read a specific section at a given position from disk using an existing handle.
fn read_section_at(
    log_reader: &mut UnifiedLoggerRead,
    pos: LogPosition,
) -> CuResult<(SectionHeader, Vec<u8>)> {
    log_reader.seek(pos)?;
    log_reader.raw_read_section()
}

/// Build a section-level index in one pass (copperlists + keyframes).
fn index_log<P, TF>(
    log_base: &Path,
    time_of: &TF,
) -> CuResult<(Vec<SectionIndexEntry>, Vec<KeyFrame>, usize)>
where
    P: CopperListTuple,
    TF: Fn(&crate::copperlist::CopperList<P>) -> Option<CuTime>,
{
    let logger = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|e| CuError::new_with_cause("Failed to open unified log", e))?;
    let UnifiedLogger::Read(mut dl) = logger else {
        return Err(CuError::from("Expected read-only unified logger"));
    };

    let mut sections = Vec::new();
    let mut keyframes = Vec::new();
    let mut total_entries = 0usize;

    loop {
        let pos = dl.position();
        let (header, data) = dl.raw_read_section()?;
        if header.entry_type == UnifiedLogType::LastEntry {
            break;
        }

        match header.entry_type {
            UnifiedLogType::CopperList => {
                let (len, first_id, last_id, first_ts, last_ts) =
                    scan_copperlist_section::<P, _>(&data, time_of)?;
                if len == 0 {
                    continue;
                }
                sections.push(SectionIndexEntry {
                    pos,
                    start_idx: total_entries,
                    len,
                    first_id,
                    last_id,
                    first_ts,
                    last_ts,
                });
                total_entries += len;
            }
            UnifiedLogType::FrozenTasks => {
                // Read all keyframes in this section
                let mut cursor = std::io::Cursor::new(&data);
                loop {
                    match decode_from_std_read::<KeyFrame, _, _>(&mut cursor, standard()) {
                        Ok(kf) => keyframes.push(kf),
                        Err(DecodeError::UnexpectedEnd { .. }) => break,
                        Err(DecodeError::Io { inner, .. })
                            if inner.kind() == io::ErrorKind::UnexpectedEof =>
                        {
                            break;
                        }
                        Err(e) => {
                            return Err(CuError::new_with_cause(
                                "Failed to decode keyframe section",
                                e,
                            ));
                        }
                    }
                }
            }
            _ => {
                // ignore other sections
            }
        }
    }

    Ok((sections, keyframes, total_entries))
}
