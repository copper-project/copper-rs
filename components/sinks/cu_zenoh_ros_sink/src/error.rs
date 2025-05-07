use cu29::CuError;
use zenoh::Error as ZenohError;

pub fn cu_error(msg: &str, error: ZenohError) -> CuError {
    CuError::new_with_cause(msg, error.as_ref())
}

pub fn cu_error_map(msg: &str) -> impl FnOnce(ZenohError) -> CuError + '_ {
    |e| cu_error(msg, e)
}
