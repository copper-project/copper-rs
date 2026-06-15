//! Construction of configured worker thread pools (std-only).
//!
//! A pool is built from a [`ThreadPoolConfig`] into a rayon [`ThreadPool`] whose
//! worker threads optionally get:
//! - a fixed CPU affinity, applied **Spread**: worker `i` is pinned to
//!   `affinity[i % affinity.len()]`, so `threads == affinity.len()` yields one
//!   worker pinned per dedicated core; and
//! - a scheduler policy/priority (normal/niceness or `SCHED_FIFO`/`SCHED_RR`).
//!
//! Affinity and scheduling are applied **once per worker thread at startup**
//! (never on the per-CopperList hot path), and only when the `rt-scheduling`
//! feature is enabled on a supported platform (Linux for real-time policies; CPU
//! affinity is cross-platform). When the feature is off, the requested affinity
//! and scheduler are ignored and a warning is emitted.
//!
//! Per-pool [`OnError`] controls what happens when a request cannot be applied
//! (for example, setting a real-time priority without `CAP_SYS_NICE`):
//! [`OnError::Warn`] logs and continues with default scheduling, while
//! [`OnError::Strict`] fails the build.

#[cfg(feature = "rt-scheduling")]
use crate::config::OnError;
use crate::config::{Scheduler, ThreadPoolConfig};
#[allow(unused_imports)] // pulls the `warning!` macro and its support symbols into scope
use crate::log::*;
use cu29_traits::{CuError, CuResult};
use rayon::ThreadPool;

/// Builds a rayon thread pool from a declarative [`ThreadPoolConfig`], applying
/// the configured CPU affinity and scheduler policy/priority to each worker.
pub fn build_pool(spec: &ThreadPoolConfig) -> CuResult<ThreadPool> {
    let id = spec.id.clone();
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(spec.threads)
        .thread_name({
            let id = id.clone();
            move |i| format!("cu-pool-{id}-{i}")
        })
        .build()
        .map_err(|e| CuError::from(format!("Failed to build thread pool '{id}': {e}")))?;

    apply_scheduling(&pool, spec)?;
    Ok(pool)
}

#[cfg(feature = "rt-scheduling")]
fn apply_scheduling(pool: &ThreadPool, spec: &ThreadPoolConfig) -> CuResult<()> {
    // Nothing to pin or reschedule: leave workers on the OS default.
    if spec.affinity.is_none() && spec.scheduler == Scheduler::Other {
        return Ok(());
    }

    let affinity = spec.affinity.as_deref();
    let scheduler = spec.scheduler;

    // `broadcast` runs the closure on every worker thread and waits, so each
    // setting is applied from within the thread it targets, with deterministic,
    // ordered results. The pool has no jobs in flight yet at this point.
    let results: Vec<CuResult<()>> =
        pool.broadcast(|ctx| apply_to_current_thread(affinity, scheduler, ctx.index()));

    for (worker, result) in results.into_iter().enumerate() {
        if let Err(e) = result {
            match spec.on_error {
                OnError::Strict => {
                    return Err(CuError::from(format!(
                        "Thread pool '{}' worker {worker} could not apply scheduling: {e}",
                        spec.id
                    )));
                }
                OnError::Warn => {
                    let pool_id = spec.id.as_str();
                    let reason = e.to_string();
                    warning!(
                        "Thread pool {} worker {} could not apply scheduling ({}); using default scheduling",
                        pool_id,
                        worker,
                        reason.as_str()
                    );
                }
            }
        }
    }

    Ok(())
}

#[cfg(not(feature = "rt-scheduling"))]
fn apply_scheduling(_pool: &ThreadPool, spec: &ThreadPoolConfig) -> CuResult<()> {
    if spec.affinity.is_some() || spec.scheduler != Scheduler::Other {
        let pool_id = spec.id.as_str();
        warning!(
            "Thread pool {} requests CPU affinity/scheduling but the 'rt-scheduling' feature is disabled; using default scheduling",
            pool_id
        );
    }
    Ok(())
}

#[cfg(feature = "rt-scheduling")]
fn apply_to_current_thread(
    affinity: Option<&[usize]>,
    scheduler: Scheduler,
    index: usize,
) -> CuResult<()> {
    if let Some(cores) = affinity
        && !cores.is_empty()
    {
        let core = cores[index % cores.len()];
        set_affinity(core)?;
    }
    set_scheduler(scheduler)
}

#[cfg(feature = "rt-scheduling")]
fn set_affinity(core: usize) -> CuResult<()> {
    if core_affinity::set_for_current(core_affinity::CoreId { id: core }) {
        Ok(())
    } else {
        Err(CuError::from(format!("failed to pin worker to CPU core {core}")))
    }
}

#[cfg(all(feature = "rt-scheduling", target_os = "linux"))]
fn set_scheduler(scheduler: Scheduler) -> CuResult<()> {
    match scheduler {
        Scheduler::Other => Ok(()),
        Scheduler::Nice(nice) => set_nice(nice),
        Scheduler::Fifo { priority } => set_rt_policy(libc::SCHED_FIFO, priority),
        Scheduler::RoundRobin { priority } => set_rt_policy(libc::SCHED_RR, priority),
    }
}

#[cfg(all(feature = "rt-scheduling", not(target_os = "linux")))]
fn set_scheduler(scheduler: Scheduler) -> CuResult<()> {
    match scheduler {
        Scheduler::Other => Ok(()),
        _ => Err(CuError::from(
            "real-time scheduler policies are only supported on Linux",
        )),
    }
}

#[cfg(all(feature = "rt-scheduling", target_os = "linux"))]
fn set_rt_policy(policy: libc::c_int, priority: u8) -> CuResult<()> {
    let param = libc::sched_param {
        sched_priority: libc::c_int::from(priority),
    };
    // SAFETY: `pthread_self()` always returns a valid handle for the current
    // thread, and `param` is a fully initialized `sched_param`.
    let ret = unsafe { libc::pthread_setschedparam(libc::pthread_self(), policy, &param) };
    if ret != 0 {
        // `pthread_setschedparam` returns the error number directly.
        return Err(CuError::from(format!(
            "pthread_setschedparam failed ({}); setting a real-time priority typically requires CAP_SYS_NICE",
            std::io::Error::from_raw_os_error(ret)
        )));
    }
    Ok(())
}

#[cfg(all(feature = "rt-scheduling", target_os = "linux"))]
fn set_nice(nice: i8) -> CuResult<()> {
    // On Linux niceness is per-thread; address this thread by its tid.
    // SAFETY: `gettid` takes no arguments and cannot fail.
    let tid = unsafe { libc::syscall(libc::SYS_gettid) } as libc::id_t;

    // `setpriority` returns -1 both on error and for the legitimate nice value
    // -1, so disambiguate via errno.
    // SAFETY: `__errno_location` returns a valid per-thread pointer.
    unsafe { *libc::__errno_location() = 0 };
    // SAFETY: simple syscall wrapper with scalar arguments.
    let ret = unsafe { libc::setpriority(libc::PRIO_PROCESS, tid, libc::c_int::from(nice)) };
    if ret == -1 {
        let err = std::io::Error::last_os_error();
        if err.raw_os_error().unwrap_or(0) != 0 {
            return Err(CuError::from(format!("setpriority failed: {err}")));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{OnError, Scheduler, ThreadPoolConfig};

    fn spec(id: &str, threads: usize) -> ThreadPoolConfig {
        ThreadPoolConfig {
            id: id.to_string(),
            threads,
            affinity: None,
            scheduler: Scheduler::Other,
            on_error: OnError::Warn,
        }
    }

    #[test]
    fn builds_plain_pool_with_requested_thread_count() {
        let pool = build_pool(&spec("plain", 3)).unwrap();
        assert_eq!(pool.current_num_threads(), 3);
    }

    #[test]
    fn warn_mode_tolerates_unappliable_request() {
        // A real-time priority request that may or may not succeed depending on
        // platform/privilege; Warn mode must build the pool regardless.
        let mut s = spec("warn", 2);
        s.scheduler = Scheduler::Fifo { priority: 50 };
        let pool = build_pool(&s).unwrap();
        assert_eq!(pool.current_num_threads(), 2);
    }

    // Pinning to a non-existent (but in-range) CPU reliably fails regardless of
    // privilege, so Strict mode must surface the error.
    #[cfg(all(feature = "rt-scheduling", target_os = "linux"))]
    #[test]
    fn strict_mode_fails_on_invalid_affinity() {
        let mut s = spec("strict", 1);
        s.affinity = Some(vec![1000]); // below CPU_SETSIZE, above any real core count
        s.on_error = OnError::Strict;
        assert!(build_pool(&s).is_err());
    }

    // When affinity targets valid cores, pinning succeeds.
    #[cfg(feature = "rt-scheduling")]
    #[test]
    fn affinity_to_valid_cores_succeeds() {
        let Some(cores) = core_affinity::get_core_ids() else {
            return; // affinity unsupported on this platform; nothing to assert
        };
        if cores.is_empty() {
            return;
        }
        let mut s = spec("affinity", cores.len());
        s.affinity = Some(cores.iter().map(|c| c.id).collect());
        s.on_error = OnError::Strict;
        let pool = build_pool(&s).unwrap();
        assert_eq!(pool.current_num_threads(), cores.len());
    }
}
