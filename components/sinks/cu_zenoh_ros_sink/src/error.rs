use cu29::CuError;
use zenoh::Error as ZenohError;

pub fn cu_error(msg: &str, error: ZenohError) -> CuError {
    CuError::from(msg).add_cause(&error.to_string())
}

pub fn cu_error_map(msg: &str) -> impl FnOnce(ZenohError) -> CuError + '_ {
    |e| cu_error(msg, e)
}
