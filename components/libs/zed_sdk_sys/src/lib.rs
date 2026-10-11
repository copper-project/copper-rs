#![no_std]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

#[cfg(not(target_os = "linux"))]
compile_error!("zed-sdk-sys currently supports Linux targets only");

mod bindings;
mod local_shim;

pub use bindings::*;
pub use local_shim::*;

pub const HAS_NATIVE_ZED_WRAPPER: bool = cfg!(zed_sdk_sys_has_native);
