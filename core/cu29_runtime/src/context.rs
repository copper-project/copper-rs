//! User-facing execution context passed to task and bridge process callbacks.

use core::ops::Deref;
use cu29_clock::CuTime;
use cu29_clock::{RobotClock, RobotClockMock};

/// Execution context passed to task and bridge callbacks.
///
/// `CuContext` provides callback code with:
/// - time access through `clock` and `Deref<Target = RobotClock>`
/// - current execution sequence id via `cl_id()`
/// - current task metadata via `task_id()` / `task_index()`
///
/// The execution sequence id matches the copper-list id of the iteration being
/// processed. It is also available in other lifecycle callbacks
/// (`start`/`preprocess`/`postprocess`/`stop`) for continuity, but outside
/// `process` callbacks it must not be treated as a live copper-list handle.
///
/// The runtime creates one context per execution loop and updates transient
/// fields such as the currently executing task before each callback.
#[derive(Clone, Debug)]
pub struct CuContext {
    /// Runtime clock. Kept as a field for direct access (`context.clock.now()`).
    pub clock: RobotClock,
    cl_id: u64,
    task_ids: &'static [&'static str],
    current_task_index: Option<usize>,
}

impl CuContext {
    /// Starts a context builder from a clock.
    pub fn builder(clock: RobotClock) -> CuContextBuilder {
        CuContextBuilder {
            clock,
            cl_id: 0,
            task_ids: &[],
        }
    }

    /// Creates a context from an existing clock with default metadata.
    ///
    /// Defaults:
    /// - `cl_id = 0`
    /// - no task id table
    pub fn from_clock(clock: RobotClock) -> Self {
        Self::builder(clock).build()
    }

    /// Creates a context backed by a real robot clock.
    ///
    /// Defaults:
    /// - `cl_id = 0`
    /// - no task id table
    #[cfg(feature = "std")]
    pub fn new_with_clock() -> Self {
        Self::from_clock(RobotClock::new())
    }

    /// Creates a context backed by a mock clock.
    ///
    /// Returns both the context and its [`RobotClockMock`] control handle.
    pub fn new_mock_clock() -> (Self, RobotClockMock) {
        let (clock, mock) = RobotClock::mock();
        (Self::from_clock(clock), mock)
    }

    /// Internal constructor used by runtime internals and code generation.
    pub(crate) fn new(clock: RobotClock, clid: u64, task_ids: &'static [&'static str]) -> Self {
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

    /// Returns the current execution sequence id.
    ///
    /// In `process` callbacks, this value is the id of the copper-list being
    /// processed. In other lifecycle callbacks, this value is still meaningful
    /// for sequencing but does not imply that a copper-list instance is alive.
    pub fn cl_id(&self) -> u64 {
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

    pub(crate) fn clone_with_fixed_time(&self, cl_id: u64, now: CuTime) -> Self {
        let (clock, mock) = RobotClock::mock();
        mock.set_value(now.as_nanos());
        Self {
            clock,
            cl_id,
            task_ids: self.task_ids,
            current_task_index: self.current_task_index,
        }
    }
}

/// Builder for [`CuContext`].
#[derive(Clone, Debug)]
pub struct CuContextBuilder {
    clock: RobotClock,
    cl_id: u64,
    task_ids: &'static [&'static str],
}

impl CuContextBuilder {
    /// Sets the copper-list id for the context.
    pub fn cl_id(mut self, cl_id: u64) -> Self {
        self.cl_id = cl_id;
        self
    }

    /// Sets the static task id table for task metadata access.
    pub fn task_ids(mut self, task_ids: &'static [&'static str]) -> Self {
        self.task_ids = task_ids;
        self
    }

    /// Builds a context value.
    pub fn build(self) -> CuContext {
        CuContext::new(self.clock, self.cl_id, self.task_ids)
    }
}

impl Deref for CuContext {
    type Target = RobotClock;

    fn deref(&self) -> &Self::Target {
        &self.clock
    }
}
