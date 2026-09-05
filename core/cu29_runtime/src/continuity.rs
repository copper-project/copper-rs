//! Native received-log provenance and continuity, independent of packet transports.

use alloc::vec::Vec;
use bincode::{Decode, Encode};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub enum SourceGapReason {
    RlcWindowExpired,
    SessionEnded,
    LateJoin,
    AnchorRecovery,
}

/// Stored in `UnifiedLogType::StreamContinuity`. Ranges are inclusive.
/// Gaps remain missing history even when a later keyframe permits state replay.
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode)]
pub enum StreamContinuityRecord {
    /// Canonical, versioned semantic manifest bytes bind identity, plan and schema.
    Manifest { record: Vec<u8> },
    Gap {
        first_id: u64,
        last_id: u64,
        reason: SourceGapReason,
    },
    /// Verified keyframe/manifest references, retained as canonical anchor bytes.
    Anchor { copperlist_id: u64, record: Vec<u8> },
    /// Explicit receiver finalization, not a claim about an unobserved sender tail.
    Finished { next_copperlist_id: u64 },
    /// Versioned capture proof for an intentionally partial native CopperList.
    /// Retains original presence and reconstruction digests for offline replay.
    Capture { copperlist_id: u64, proof: Vec<u8> },
}

/// Rejects replay across missing history unless state is restored at this boundary.
/// Call before executing any task or changing the replay clock.
pub fn validate_replay_continuity(
    expected: u64,
    actual: u64,
    keyframe: Option<u64>,
) -> cu29_traits::CuResult<()> {
    if keyframe.is_some_and(|boundary| boundary != actual) {
        return Err("Replay keyframe does not match the CopperList boundary".into());
    }
    if actual != expected && keyframe != Some(actual) {
        return Err("Replay cannot cross an unhealed CopperList gap without a keyframe".into());
    }
    Ok(())
}
