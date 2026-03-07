use std::sync::OnceLock;

static START: OnceLock<web_time::Instant> = OnceLock::new();

#[inline(always)]
fn start_instant() -> web_time::Instant {
    *START.get_or_init(web_time::Instant::now)
}

pub fn initialize() {
    let _ = start_instant();
}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    let elapsed = web_time::Instant::now().duration_since(start_instant());
    u64::try_from(elapsed.as_nanos()).unwrap_or(u64::MAX)
}
