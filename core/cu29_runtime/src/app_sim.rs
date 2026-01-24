use crate::curuntime::{CuRuntime, KeyFrame};
use cu29_clock::CuTime;
use cu29_traits::CopperListTuple;

/// Helper methods injected for simulation-only builds.
impl<CT, CB, P, M, const NBCL: usize> CuRuntime<CT, CB, P, M, NBCL>
where
    P: CopperListTuple + crate::copperlist::CuListZeroedInit + Default + 'static,
    M: crate::monitoring::CuMonitor,
{
    #[cfg(feature = "std")]
    pub fn set_forced_keyframe_timestamp(&mut self, ts: CuTime) {
        self.keyframes_manager.set_forced_timestamp(ts);
    }

    /// Reuse a recorded keyframe verbatim (used during deterministic replay).
    #[cfg(feature = "std")]
    pub fn lock_keyframe(&mut self, keyframe: &KeyFrame) {
        self.keyframes_manager.lock_keyframe(keyframe);
    }
}
