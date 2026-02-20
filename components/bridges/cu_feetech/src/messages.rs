//! Message types for the Feetech bridge.
//!
//! Positions are published in the unit configured on the bridge
//! (`"units"` key in `copperconfig.ron`):
//!
//! - `"raw"` (default) — raw 16-bit register values (0–65535).
//! - `"deg"` — degrees relative to calibration center (0 = center).
//! - `"rad"` — radians relative to calibration center (0 = center).
//! - `"normalize"` — [-1, 1] over calibrated min..max (same scale for leader/follower).

use cu29::prelude::*;

/// Maximum number of servos supported on a single bus.
///
/// Feetech IDs go up to 253, but real arms (SO-100 / SO-101) have 6
/// joints.  We cap at 8 to keep the payload small and stack-allocated.
pub const MAX_SERVOS: usize = 8;

/// Joint positions for up to [`MAX_SERVOS`] Feetech bus servos.
///
/// Values are `f32` so they can carry raw ticks, degrees, or radians
/// depending on the bridge configuration.
pub type JointPositions = CuArray<f32, MAX_SERVOS>;
