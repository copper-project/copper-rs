use core::ffi::c_void;

use crate::{SL_MAT_TYPE, SL_MEM};

unsafe extern "C" {
    pub fn sl_mat_is_memory_owner(ptr: *mut c_void) -> bool;

    pub fn sl_mat_create_alias(
        width: i32,
        height: i32,
        type_: SL_MAT_TYPE,
        mem: SL_MEM,
        data: *mut c_void,
        step_bytes: usize,
    ) -> *mut c_void;
}
