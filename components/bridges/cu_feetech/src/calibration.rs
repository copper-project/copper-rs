//! Calibration data and unit conversions for Feetech servos.
//!
//! Each servo has a recorded min and max raw position. The center is
//! `(min + max) / 2` and is used as the zero reference when converting
//! to degrees or radians.
//!
//! Run the `feetech-calibrate` binary to generate a `calibration.json`.

use cu29::units::si::angle::{degree, radian};
use cu29::units::si::f32::Angle;
use serde::{Deserialize, Serialize};
use std::path::Path;
use std::str::FromStr;

/// Output unit for published positions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Units {
    /// Raw 16-bit register values (0–65535).  No calibration needed.
    #[default]
    Raw,
    /// Degrees relative to the calibration center (0 = center).
    Deg,
    /// Radians relative to the calibration center (0 = center).
    Rad,
    /// Normalized range [-1, 1]: min → -1, center → 0, max → 1. Same scale for leader/follower.
    Normalize,
}

impl FromStr for Units {
    type Err = ();

    /// Parse from a config string.  Returns `Err` for unrecognised values.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "raw" => Ok(Self::Raw),
            "deg" => Ok(Self::Deg),
            "rad" => Ok(Self::Rad),
            "normalize" | "norm" => Ok(Self::Normalize),
            _ => Err(()),
        }
    }
}

// =========================================================================
// Conversion helpers
// =========================================================================

/// Default ticks per revolution when not specified (e.g. STS3215 often uses 4096).
/// Actual value is model-dependent; set via bridge config `ticks_per_rev`.
pub const DEFAULT_TICKS_PER_REV: u32 = 4096;

impl Units {
    /// Convert a raw 16-bit tick to the output unit.
    ///
    /// For `Raw`: `param` is ignored.
    /// For `Deg`/`Rad`: `param` is `ticks_per_rev`.
    /// For `Normalize`: `param` is half_range `(max - min) / 2`; result is in [-1, 1].
    #[inline]
    pub fn from_raw(self, raw: u16, center: f32, param: f32) -> f32 {
        match self {
            Self::Raw => raw as f32,
            Self::Deg => {
                let deg = (raw as f32 - center) * 360.0 / param;
                Angle::new::<degree>(deg).get::<degree>()
            }
            Self::Rad => {
                let rad = (raw as f32 - center) * core::f32::consts::TAU / param;
                Angle::new::<radian>(rad).get::<radian>()
            }
            Self::Normalize => {
                if param <= 0.0 {
                    0.0
                } else {
                    ((raw as f32 - center) / param).clamp(-1.0, 1.0)
                }
            }
        }
    }

    /// Convert an output-unit value back to a raw 16-bit tick.
    ///
    /// For `Normalize`, `param` is half_range; value must be in [-1, 1].
    /// Result is clamped to `0..=65535`.
    #[inline]
    pub fn to_raw(self, value: f32, center: f32, param: f32) -> u16 {
        let raw = match self {
            Self::Raw => value,
            Self::Deg => {
                let deg = Angle::new::<degree>(value).get::<degree>();
                deg * param / 360.0 + center
            }
            Self::Rad => {
                let rad = Angle::new::<radian>(value).get::<radian>();
                rad * param / core::f32::consts::TAU + center
            }
            Self::Normalize => center + value.clamp(-1.0, 1.0) * param,
        };
        raw.round().clamp(0.0, 65535.0) as u16
    }
}

// =========================================================================
// Per-servo calibration
// =========================================================================

/// Calibration for a single servo.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServoCalibration {
    pub id: u8,
    pub min: u16,
    pub max: u16,
}

impl ServoCalibration {
    /// Midpoint between min and max — the "zero" position.
    pub fn center(&self) -> f32 {
        (self.min as f32 + self.max as f32) / 2.0
    }

    /// Total usable range in raw ticks.
    pub fn range(&self) -> u16 {
        self.max.saturating_sub(self.min)
    }
}

/// Calibration data for all servos on a bus.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CalibrationData {
    pub servos: Vec<ServoCalibration>,
}

impl CalibrationData {
    pub fn load(path: &Path) -> std::io::Result<Self> {
        let contents = std::fs::read_to_string(path)?;
        serde_json::from_str(&contents)
            .map_err(|e| std::io::Error::other(format!("bad calibration JSON: {e}")))
    }

    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(path, json)
    }

    /// Look up the center (midpoint) for a servo by bus ID.
    ///
    /// Returns `None` if no calibration entry exists for that ID.
    pub fn center_for(&self, id: u8) -> Option<f32> {
        self.servos.iter().find(|s| s.id == id).map(|s| s.center())
    }

    /// Look up half the range `(max - min) / 2` for a servo by bus ID.
    /// Used for the `normalize` unit ([-1, 1] over the calibrated range).
    pub fn half_range_for(&self, id: u8) -> Option<f32> {
        self.servos
            .iter()
            .find(|s| s.id == id)
            .map(|s| (s.max as f32 - s.min as f32) / 2.0)
    }
}
