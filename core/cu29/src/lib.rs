//! # Copper Runtime & SDK
//!
//! Think of Copper as a robotics game engine: define a task graph, compile once,
//! and get deterministic execution, unified logging, and sub-microsecond
//! latency from Linux workstations all the way down to bare-metal MPU builds.
//!
//! ## Quick start
//!
//! ```bash
//! cargo install cargo-cunew
//! cargo cunew /path/to/my_robot
//! cd /path/to/my_robot
//! cargo run
//! ```
//!
//! It will generate a minimal Copper robot project at `/path/to/my_robot` using the latest
//! stable Copper crates from crates.io by default.
//!
//! ## Feature flags
//!
//! - `default` = `["std", "signal-handler", "textlogs", "units"]`
//! - `units`: exposes `cu29::units` (re-export of `cu29-units`)
//! - `std`: host/runtime support that is also safe to compile for browser targets
//! - `signal-handler`: desktop Ctrl-C integration for generated `run()` loops
//! - `reflect`: reflection support for runtime and units types
//! - `textlogs`: text logging derive support
//! - `remote-debug`: remote debug transport support
//! - `sysclock-perf`: use a host/system clock for runtime perf timing while keeping robot time for `tov` and `rate_target_hz`
//! - `high-precision-limiter`: std-only hybrid sleep/spin loop limiter for tighter `rate_target_hz` cadence
//! - `async-cl-io`: offload CopperList serialization/logging to a dedicated std thread
//! - `parallel-rt`: prepare the runtime for a future multi-threaded deterministic executor
//! - `safety-ids`: std-only safety-case metadata collection and JSON export helpers
//!
//! ## Concepts behind Copper
//!
//! Check out the [Copper Wiki](https://github.com/copper-project/copper-rs/wiki) to understand the
//! deployments concepts, task lifecycle, available components, etc ...
//!
//! ## More examples to get you started
//!
//! - `examples/cu_caterpillar`: a minimal running example passing around booleans.
//! - `examples/cu_rp_balancebot`: a more complete example try Copper without hardware via
//!   `cargo install cu-rp-balancebot` + `balancebot-sim` (Bevy + Avian3d).
//!
//! ## Key traits and structs to check out
//!
//! - `cu29_runtime::app::CuApp`: the main trait the copper runtime will expose to run your application. (when run() etc .. is coming from)
//! - `cu29_runtime::config::CuConfig`: the configuration of your runtime
//! - `cu29_runtime::cutask::CuTask`: the core trait and helpers to implement your own tasks.
//! - `cu29_runtime::cubridge::CuBridge`: the trait to implement bridges to hardware or other software.
//! - `cu29_runtime::curuntime::CuRuntime`: the runtime that manages task execution.
//! - `cu29_runtime::simulation`: This will explain how to hook up your tasks to a simulation environment.
//!
//! ## V1 API status
//!
//! The V1 public contract is defined in `doc/v1-api-surface.md`. The prelude is the
//! canonical application import surface; lower-level modules remain addressable by
//! module path when needed, but are not implicitly part of the prelude contract.
//!
//! Need help or want to show what you're building? Join
//! [Discord](https://discord.gg/VkCG7Sb9Kw) and hop into the #general channel.
//!

#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(all(feature = "parallel-rt", not(feature = "std")))]
compile_error!("feature `parallel-rt` requires `std`");
#[cfg(not(feature = "std"))]
extern crate alloc;
extern crate self as cu29;

pub use cu29_derive::{bundle_resources, resources, safety_case};
pub use cu29_runtime::app;
pub use cu29_runtime::config;
pub use cu29_runtime::context;
pub use cu29_runtime::copperlist;
#[cfg(feature = "std")]
pub use cu29_runtime::cuasynctask;
pub use cu29_runtime::cubridge;
pub use cu29_runtime::curuntime;
pub use cu29_runtime::cutask;
pub use cu29_runtime::cutask_anytime;
// Re-exported at the crate root so RON can name `cu29::AnytimeTask<...>` directly.
pub use cu29_runtime::cutask_anytime::{Anytime, AnytimeOutput, AnytimeTask, Progress, Step};
#[cfg(feature = "std")]
pub use cu29_runtime::debug;
#[cfg(feature = "std")]
pub use cu29_runtime::distributed_replay;
pub use cu29_runtime::input_msg;
pub use cu29_runtime::logcodec;
pub use cu29_runtime::monitoring;
pub use cu29_runtime::output_msg;
#[cfg(all(feature = "std", feature = "parallel-rt"))]
pub use cu29_runtime::parallel_queue;
#[cfg(all(feature = "std", feature = "parallel-rt"))]
pub use cu29_runtime::parallel_rt;
pub use cu29_runtime::payload;
#[cfg(feature = "std")]
pub use cu29_runtime::pool;
pub use cu29_runtime::reflect;
pub use cu29_runtime::reflect as bevy_reflect;
#[cfg(feature = "remote-debug")]
pub use cu29_runtime::remote_debug;
#[cfg(feature = "std")]
pub use cu29_runtime::replay;
pub use cu29_runtime::resource;
pub use cu29_runtime::rx_channels;
#[cfg(feature = "std")]
pub use cu29_runtime::simulation;
#[cfg(feature = "std")]
pub use cu29_runtime::thread_pool;
pub use cu29_runtime::tx_channels;
#[cfg(feature = "safety-ids")]
pub mod safety;
#[cfg(all(feature = "std", any(test, feature = "safety-ids")))]
mod safety_runtime_cases;

#[cfg(feature = "safety-ids")]
#[doc(hidden)]
pub fn link_safety_ids() {
    safety_runtime_cases::link_safety_ids();
}

#[cfg(feature = "rtsan")]
pub mod rtsan {
    pub use rtsan_standalone::*;
}

#[cfg(not(feature = "rtsan"))]
pub mod rtsan {
    use core::ffi::CStr;

    #[derive(Default)]
    pub struct ScopedSanitizeRealtime;

    #[derive(Default)]
    pub struct ScopedDisabler;

    #[inline]
    pub fn realtime_enter() {}

    #[inline]
    pub fn realtime_exit() {}

    #[inline]
    pub fn disable() {}

    #[inline]
    pub fn enable() {}

    #[inline]
    pub fn ensure_initialized() {}

    #[allow(unused_variables)]
    pub fn notify_blocking_call(_function_name: &'static CStr) {}
}

pub use bincode;
pub use cu29_clock as clock;
#[cfg(feature = "units")]
pub use cu29_units as units;
#[doc(hidden)]
pub use serde;
#[cfg(feature = "defmt")]
pub mod defmt {
    pub use defmt::{debug, error, info, warn};
}
#[cfg(feature = "std")]
pub use cu29_runtime::config::read_configuration;
#[cfg(feature = "std")]
pub use cu29_runtime::config::read_multi_configuration;
pub use cu29_traits::*;

#[cfg(feature = "std")]
pub use rayon;

#[doc(hidden)]
pub mod __private {
    #[doc(hidden)]
    pub mod sync {
        #[cfg(not(feature = "std"))]
        pub use alloc::sync::Arc;
        #[cfg(not(feature = "std"))]
        pub use spin::Mutex;
        #[cfg(feature = "std")]
        pub use std::sync::{Arc, Mutex};
    }
}

// defmt shims re-exported for proc-macro call sites
#[cfg(all(feature = "defmt", not(feature = "std")))]
#[macro_export]
macro_rules! defmt_debug {
    ($fmt:literal $(, $arg:expr)* $(,)?) => {
        $crate::defmt::debug!($fmt $(, $arg)*);
    }
}
#[cfg(not(all(feature = "defmt", not(feature = "std"))))]
#[macro_export]
macro_rules! defmt_debug {
    ($($tt:tt)*) => {{}};
}

#[cfg(all(feature = "defmt", not(feature = "std")))]
#[macro_export]
macro_rules! defmt_info {
    ($fmt:literal $(, $arg:expr)* $(,)?) => {
        $crate::defmt::info!($fmt $(, $arg)*);
    }
}
#[cfg(not(all(feature = "defmt", not(feature = "std"))))]
#[macro_export]
macro_rules! defmt_info {
    ($($tt:tt)*) => {{}};
}

#[cfg(all(feature = "defmt", not(feature = "std")))]
#[macro_export]
macro_rules! defmt_warn {
    ($fmt:literal $(, $arg:expr)* $(,)?) => {
        $crate::defmt::warn!($fmt $(, $arg)*);
    }
}
#[cfg(not(all(feature = "defmt", not(feature = "std"))))]
#[macro_export]
macro_rules! defmt_warn {
    ($($tt:tt)*) => {{}};
}

#[cfg(all(feature = "defmt", not(feature = "std")))]
#[macro_export]
macro_rules! defmt_error {
    ($fmt:literal $(, $arg:expr)* $(,)?) => {
        $crate::defmt::error!($fmt $(, $arg)*);
    }
}
#[cfg(not(all(feature = "defmt", not(feature = "std"))))]
#[macro_export]
macro_rules! defmt_error {
    ($($tt:tt)*) => {{}};
}

#[macro_export]
macro_rules! safety_check {
    ($check_id:literal, $requirement_id:literal, $condition:expr $(,)?) => {
        assert!(
            $condition,
            "safety check {} for requirement {} failed",
            $check_id, $requirement_id
        );
    };
}

#[macro_export]
macro_rules! safety_check_eq {
    ($check_id:literal, $requirement_id:literal, $left:expr, $right:expr $(,)?) => {
        assert_eq!(
            $left, $right,
            "safety check {} for requirement {} failed",
            $check_id, $requirement_id
        );
    };
}

/// Canonical imports for Copper applications.
///
/// This module intentionally re-exports each stable application-facing group once.
/// Runtime internals, remote-debug plumbing, and experimental executor APIs should
/// be imported from their explicit module paths instead of from the prelude.
pub mod prelude {
    pub use crate::bevy_reflect;
    #[cfg(feature = "units")]
    pub use crate::units;
    pub use crate::{defmt_debug, defmt_error, defmt_info, defmt_warn};
    pub use crate::{safety_case, safety_check, safety_check_eq};
    #[cfg(feature = "reflect")]
    pub use bevy_reflect_derive::Reflect;
    #[cfg(feature = "signal-handler")]
    pub use ctrlc;
    pub use cu29_clock::*;
    pub use cu29_derive::*;
    pub use cu29_log::*;
    pub use cu29_log_derive::*;
    pub use cu29_log_runtime::*;
    #[cfg(not(feature = "reflect"))]
    pub use cu29_reflect_derive::Reflect;
    pub use cu29_runtime::app;
    pub use cu29_runtime::app::*;
    pub use cu29_runtime::config::*;
    pub use cu29_runtime::context::*;
    pub use cu29_runtime::copperlist::*;
    pub use cu29_runtime::cubridge::*;
    pub use cu29_runtime::curuntime::{
        CuRuntime, KeyFrame, RuntimeLifecycleConfigSource, RuntimeLifecycleEvent,
        RuntimeLifecycleRecord, RuntimeLifecycleStackInfo,
    };
    pub use cu29_runtime::cutask::*;
    pub use cu29_runtime::cutask_anytime::*;
    #[cfg(feature = "std")]
    pub use cu29_runtime::debug::*;
    pub use cu29_runtime::input_msg;
    pub use cu29_runtime::monitoring::*;
    pub use cu29_runtime::output_msg;
    pub use cu29_runtime::payload::*;
    #[cfg(feature = "std")]
    pub use cu29_runtime::pool::*;
    #[cfg(feature = "reflect")]
    pub use cu29_runtime::reflect::serde as reflect_serde;
    #[cfg(feature = "reflect")]
    pub use cu29_runtime::reflect::serde::{
        ReflectSerializer, SerializationData, TypedReflectSerializer,
    };
    pub use cu29_runtime::reflect::{
        GetTypeRegistration, ReflectTaskIntrospection, ReflectTypePath, TypeInfo, TypePath,
        TypeRegistry, dump_type_registry_schema,
    };
    pub use cu29_runtime::resource::*;
    pub use cu29_runtime::rx_channels;
    #[cfg(feature = "std")]
    pub use cu29_runtime::simulation::*;
    pub use cu29_runtime::tx_channels;
    pub use cu29_traits::{
        COMPACT_STRING_CAPACITY, CopperListTuple, CuCompactString, CuError, CuMsgMetadataTrait,
        CuMsgOrigin, CuPayloadRawBytes, CuResult, DebugFieldDescriptor, DebugFieldKind,
        DebugFieldSemantics, DebugScalarKind, DebugScalarRegistration, DebugScalarType,
        ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks, Metadata, ObservedWriter,
        PayloadSchemas, TaskOutputSpec, UnifiedLogType, WriteStream, abort_observed_encode,
        begin_observed_encode, finish_observed_encode, observed_encode_bytes,
        record_observed_encode_bytes, with_cause,
    };
    #[cfg(feature = "std")]
    pub use cu29_unifiedlog::memmap;
    pub use cu29_unifiedlog::*;
    pub use cu29_value::Value;
    pub use cu29_value::to_value;
    pub use serde_derive::{Deserialize, Serialize};
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::prelude::*;
    use std::sync::{Arc, Mutex, OnceLock};

    #[derive(Debug)]
    struct CaptureStream;

    impl WriteStream<CuLogEntry> for CaptureStream {
        fn log(&mut self, _obj: &CuLogEntry) -> CuResult<()> {
            Ok(())
        }
    }

    fn logger_test_lock() -> std::sync::MutexGuard<'static, ()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
            .lock()
            .unwrap_or_else(|poison| poison.into_inner())
    }

    fn capture_one_log<F>(emit: F) -> CuLogEntry
    where
        F: FnOnce(),
    {
        let _guard = logger_test_lock();
        let runtime = LoggerRuntime::init(RobotClock::default(), CaptureStream, None::<NullLog>);
        let captured = Arc::new(Mutex::new(Vec::new()));
        let sink = captured.clone();
        let _listener = scoped_live_log_listener(move |entry, _, _| {
            sink.lock()
                .unwrap_or_else(|poison| poison.into_inner())
                .push(entry.clone());
        });

        emit();

        drop(runtime);

        let entries = captured.lock().unwrap_or_else(|poison| poison.into_inner());
        assert_eq!(entries.len(), 1, "expected exactly one captured log entry");
        entries[0].clone()
    }

    #[test]
    fn explicit_context_logs_capture_task_origin() {
        let mut ctx = CuContext::builder(RobotClock::default())
            .cl_id(77)
            .task_ids(&["task-0"])
            .build();
        ctx.set_current_task(0);

        let entry = capture_one_log(|| {
            debug!(ctx, "task log {}", 7);
        });

        assert_eq!(entry.origin.culistid, Some(77));
        assert_eq!(entry.origin.component_id, Some(0));
        assert_eq!(entry.origin.task_index, Some(0));
    }

    #[test]
    fn explicit_context_logs_capture_bridge_component_origin() {
        let mut ctx = CuContext::builder(RobotClock::default()).cl_id(88).build();
        ctx.set_current_component(5);
        ctx.clear_current_task();

        let entry = capture_one_log(|| {
            info!(ctx, "bridge log {}", 3);
        });

        assert_eq!(entry.origin.culistid, Some(88));
        assert_eq!(entry.origin.component_id, Some(5));
        assert_eq!(entry.origin.task_index, None);
    }

    #[test]
    fn context_free_logs_leave_origin_empty() {
        let entry = capture_one_log(|| {
            warning!("context free {}", 1);
        });

        assert_eq!(entry.origin, CuLogOrigin::default());
    }
}
