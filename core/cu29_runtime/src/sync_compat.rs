#[cfg(feature = "std")]
pub(crate) use std::sync::{Mutex, MutexGuard, OnceLock};

#[cfg(not(feature = "std"))]
pub(crate) use spin::once::Once as OnceLock;
#[cfg(not(feature = "std"))]
pub(crate) use spin::{Mutex, MutexGuard};

#[inline]
pub(crate) fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    #[cfg(feature = "std")]
    {
        mutex.lock().unwrap_or_else(|poison| poison.into_inner())
    }

    #[cfg(not(feature = "std"))]
    {
        mutex.lock()
    }
}

#[inline]
pub(crate) fn once_get_or_init<T>(cell: &OnceLock<T>, init: impl FnOnce() -> T) -> &T {
    #[cfg(feature = "std")]
    {
        cell.get_or_init(init)
    }

    #[cfg(not(feature = "std"))]
    {
        cell.call_once(init)
    }
}
