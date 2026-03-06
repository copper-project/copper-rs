//! Message types for the Feetech bridge.
//!
//! Positions are published in the unit configured on the bridge
//! (`"units"` key in `copperconfig.ron`):
//!
//! - `"raw"` (default) — raw 16-bit register values (0–65535).
//! - `"deg"` — degrees relative to calibration center (0 = center).
//! - `"rad"` — radians relative to calibration center (0 = center).
//! - `"normalize"` — [-1, 1] over calibrated min..max (same scale for leader/follower).

/// Maximum number of servos supported on a single bus.
///
/// Feetech IDs go up to 253, but real arms (SO-100 / SO-101) have 6
/// joints.  We cap at 8 to keep the payload small and stack-allocated.
pub const MAX_SERVOS: usize = 8;

/// Joint state for a Feetech servo arm.
///
/// A type alias for [`cu_sensor_payloads::JointState`] with capacity
/// [`MAX_SERVOS`].  The `positions` field carries the servo values in
/// whichever unit is configured on the bridge; `velocities` and `efforts`
/// are not populated by the hardware read path and remain empty (len = 0).
pub type JointPositions = cu_sensor_payloads::JointState<MAX_SERVOS>;
