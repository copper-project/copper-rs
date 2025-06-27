use cu29::CuError;
use zenoh::Error as ZenohError;

pub fn cu_error_map(msg: &'static str) -> impl FnOnce(ZenohError) -> CuError {
    move |e| CuError::new_with_cause(msg, e)
}
