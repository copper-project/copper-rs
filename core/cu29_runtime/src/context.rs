//! User-facing execution context passed to task and bridge process callbacks.

use core::ops::Deref;
use cu29_clock::RobotClock;

/// Context available inside each `process`/`send`/`receive` callback.
///
/// `CuContext` gives callback code access to runtime metadata for the current
/// execution step without exposing the full copper list internals:
/// - `clock` (or `context.now()` via `Deref`) for time access
/// - `cl_id()` for the current copper list identifier
/// - `task_id()` / `task_index()` for the currently running task (when applicable)
///
/// A new context is created by the runtime for each copper-list iteration and
/// reused across all callbacks executed during that iteration.
#[derive(Clone, Debug)]
pub struct CuContext {
    /// Runtime clock. Kept as a field for direct access (`context.clock.now()`).
    pub clock: RobotClock,
    cl_id: u32,
    task_ids: &'static [&'static str],
    current_task_index: Option<usize>,
}

impl CuContext {
    /// Creates a context for one copper-list execution iteration.
    pub fn new(clock: RobotClock, clid: u32, task_ids: &'static [&'static str]) -> Self {
        Self {
            clock,
            cl_id: clid,
            task_ids,
            current_task_index: None,
        }
    }

    /// Sets the currently executing task index.
    pub fn set_current_task(&mut self, task_index: usize) {
        self.current_task_index = Some(task_index);
    }

    /// Clears the currently executing task.
    pub fn clear_current_task(&mut self) {
        self.current_task_index = None;
    }

    /// Returns the current copper list id.
    pub fn cl_id(&self) -> u32 {
        self.cl_id
    }

    /// Returns the current task index, if any.
    pub fn task_index(&self) -> Option<usize> {
        self.current_task_index
    }

    /// Returns the current task id, if any.
    pub fn task_id(&self) -> Option<&'static str> {
        self.current_task_index
            .and_then(|idx| self.task_ids.get(idx).copied())
    }
}

impl Deref for CuContext {
    type Target = RobotClock;

    fn deref(&self) -> &Self::Target {
        &self.clock
    }
}
