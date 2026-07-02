#[cfg(feature = "std")]
pub(crate) use std::sync::{Mutex, MutexGuard, OnceLock};

#[cfg(not(feature = "std"))]
pub(crate) use spin::Mutex;
#[cfg(not(feature = "std"))]
pub(crate) use spin::once::Once as OnceLock;
#[cfg(not(feature = "std"))]
pub(crate) type MutexGuard<'a, T> = spin::MutexGuard<'a, T, spin::relax::Spin>;

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
pub(crate) fn init_once<T>(cell: &OnceLock<T>, value: T) {
    #[cfg(feature = "std")]
    {
        assert!(cell.set(value).is_ok(), "OnceLock already initialized");
    }

    #[cfg(not(feature = "std"))]
    {
        assert!(cell.get().is_none(), "OnceLock already initialized");
        let _ = cell.call_once(|| value);
    }
}
