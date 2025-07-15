pub mod broadcaster;
pub mod error;
pub mod frames;
pub mod interpolation;
pub mod transform;
pub mod transform_payload;
pub mod tree;
pub mod velocity;
mod velocity_cache;

#[cfg(test)]
mod test_utils;

use arrayvec::ArrayString;

/// Type alias for frame identifier strings
/// Uses a stack-allocated string with a maximum of 64 characters
/// This avoids heap allocations and is more deterministic
pub type FrameIdString = ArrayString<64>;

pub use broadcaster::{ModernTransformBroadcaster, ModernTransformListener, TransformBroadcast};
pub use error::{TransformError, TransformResult};
pub use frames::{
    BaseFrame, CameraFrame, FrameId, FramePair, ImuFrame, LidarFrame, RobotFrame, WorldFrame,
};
pub use frames::{BaseToRobot, RobotToCamera, RobotToImu, RobotToLidar, WorldToBase, WorldToRobot};
pub use interpolation::interpolate_transforms;
pub use transform::{ConstTransformBuffer, StampedTransform, TransformBuffer, TransformStore};
pub use transform_payload::{TransformPayload, TypedTransformBuffer, TypedTransformMsg};
pub use tree::TransformTree;
pub use velocity::VelocityTransform;

pub use cu29::prelude::CuMsg;
pub use cu_spatial_payloads::Transform3D;
