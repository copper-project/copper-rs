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

/// Frame identifier strings
pub type FrameIdString = ArrayString<64>;

pub use error::{TransformError, TransformResult};
pub use frames::{
    BaseFrame, CameraFrame, FrameId, FramePair, ImuFrame, LidarFrame, RobotFrame, WorldFrame,
};
pub use frames::{BaseToRobot, RobotToCamera, RobotToImu, RobotToLidar, WorldToBase, WorldToRobot};
pub use interpolation::interpolate_transforms;
pub use transform::{ConstTransformBuffer, StampedTransform, TransformBuffer, TransformStore};
pub use transform_payload::{FrameTransform, TypedTransform, TypedTransformBuffer};
pub use tree::TransformTree;
pub use velocity::VelocityTransform;

pub use cu29::prelude::CuMsg;
pub use cu_spatial_payloads::Transform3D;
