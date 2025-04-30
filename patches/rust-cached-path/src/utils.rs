use sha2::{Digest, Sha256};
use std::time::SystemTime;

pub(crate) fn hash_str(s: &str) -> String {
    format!("{:x}", Sha256::digest(s.as_bytes()))
}

pub(crate) fn now() -> f64 {
    // Safe to unwrap unless the system time is seriously screwed up.
    SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_secs_f64()
}
