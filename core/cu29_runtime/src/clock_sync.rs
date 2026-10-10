//! Experimental clock-reference lifecycle and portable correction records.

use bincode::{Decode, Encode};
use cu29_clock::sync::ClockSnapshot;

/// Experimental recorded correction applied before the named CopperList.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub struct ClockSyncRecord {
    /// First CopperList using this published curve.
    pub culistid: u64,
    /// Reference mapping and uncertainty at publication.
    pub snapshot: ClockSnapshot,
}

#[cfg(feature = "clock-sync")]
mod maintenance;
#[cfg(feature = "clock-sync")]
pub use maintenance::{ClockMaintenance, ClockReference, ClockReferenceBundle, MaintenanceConfig};

/// Reads corrections from a selected run's RuntimeLifecycle stream, offline.
#[cfg(all(feature = "std", feature = "clock-sync"))]
pub fn read_clock_sync_records(
    mut source: impl std::io::Read,
) -> cu29_traits::CuResult<alloc::vec::Vec<ClockSyncRecord>> {
    use crate::curuntime::{RuntimeLifecycleEvent, RuntimeLifecycleRecord};
    use cu29_traits::CuError;
    use std::io::Read;
    let mut records = alloc::vec::Vec::new();
    loop {
        // Probe a single byte so EOF is accepted only between records. An
        // incomplete record must remain an error, even if its last field is one byte.
        let mut first = [0u8; 1];
        match source.read_exact(&mut first) {
            Ok(()) => {}
            Err(error) if error.kind() == std::io::ErrorKind::UnexpectedEof => break,
            Err(error) => {
                return Err(CuError::new_with_cause(
                    "Clock correction read failed",
                    error,
                ));
            }
        }
        let mut reader = std::io::Cursor::new(first).chain(&mut source);
        let record: RuntimeLifecycleRecord =
            bincode::decode_from_std_read(&mut reader, bincode::config::standard()).map_err(
                |error| CuError::new_with_cause("Clock correction decode failed", error),
            )?;
        if let RuntimeLifecycleEvent::ClockSync(record) = record.event {
            if records
                .last()
                .is_some_and(|last: &ClockSyncRecord| last.culistid > record.culistid)
            {
                return Err(CuError::from(
                    "Clock corrections are out of CopperList order; select one recorded run",
                ));
            }
            records.push(record);
        }
    }
    Ok(records)
}

#[cfg(all(feature = "std", feature = "clock-sync"))]
pub(crate) fn load_clock_sync_records(
    path: &std::path::Path,
) -> cu29_traits::CuResult<alloc::vec::Vec<ClockSyncRecord>> {
    let logger = crate::debug::build_read_logger(path)?;
    read_clock_sync_records(cu29_unifiedlog::UnifiedLoggerIOReader::new(
        logger,
        cu29_traits::UnifiedLogType::RuntimeLifecycle,
    ))
}

/// Offline clock restoration used by debugger callbacks with a recorded counter.
#[cfg(all(feature = "std", feature = "clock-sync"))]
pub(crate) struct ClockReplay {
    records: alloc::vec::Vec<ClockSyncRecord>,
    controller: Option<cu29_clock::sync::ClockSync>,
}

#[cfg(all(feature = "std", feature = "clock-sync"))]
impl ClockReplay {
    pub(crate) fn new(records: alloc::vec::Vec<ClockSyncRecord>) -> Self {
        Self {
            records,
            controller: None,
        }
    }
    pub(crate) fn apply(
        &mut self,
        clock: &cu29_clock::RobotClock,
        mock: &cu29_clock::RobotClockMock,
        id: u64,
        time: cu29_clock::CuTime,
    ) -> cu29_traits::CuResult<()> {
        use cu29_clock::sync::ClockSync;
        use cu29_traits::CuError;
        let index = self.records.partition_point(|record| record.culistid <= id);
        if index == 0 {
            mock.set_value(time.0);
            return Ok(());
        }
        let snapshot = self.records[index - 1].snapshot;
        if let Some(controller) = &mut self.controller {
            controller
                .restore(snapshot)
                .map_err(|error| CuError::new_with_cause("Replay clock restore failed", error))?;
        } else {
            self.controller =
                Some(ClockSync::from_snapshot(clock, snapshot).map_err(|error| {
                    CuError::new_with_cause("Replay clock attach failed", error)
                })?);
        }
        if let Some(controller) = &mut self.controller {
            controller
                .set_replay_time(mock, time)
                .map_err(|error| CuError::new_with_cause("Replay clock time failed", error))?;
        }
        Ok(())
    }
}
