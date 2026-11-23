//! internal prelude imports for logging from core itself

#[allow(unused_imports)]
pub(crate) use cu29_value::to_value;

#[allow(unused_imports)]
pub(crate) use cu29_log_runtime::log;

#[allow(unused_imports)]
#[cfg(debug_assertions)]
pub(crate) use cu29_log_runtime::log_debug_mode;

#[allow(unused_imports)]
pub(crate) use cu29_log_derive::{critical, debug, error, info, warning};

#[allow(unused_imports)]
pub(crate) use cu29_log::{CuLogEntry, CuLogLevel};

#[allow(unused_imports)]
pub(crate) use cu29_log::ANONYMOUS;
