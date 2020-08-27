use spinlock::Spinlock;
use std::time::Instant;

pub(crate) struct Cu {
    pub(crate) tov: Instant,
    pub(crate) lock: Spinlock<Instant>,
    pub(crate) value: i32,
}

