//! Standalone Candle inference for the ViTFly ViT+LSTM policy.
//!
//! The low-level [`VitFly::forward`] contract mirrors the trained PyTorch
//! model: depth `[1, 1, 60, 90]`, desired velocity `[1, 1]`, attitude
//! quaternion `[1, 4]`, and an optional recurrent state.

mod model;
mod task;

pub use model::{
    ATTITUDE_SHAPE, DEPTH_SHAPE, DESIRED_VELOCITY_SHAPE, ForwardTrace, INPUT_HEIGHT, INPUT_WIDTH,
    OUTPUT_SHAPE, RECURRENT_SHAPE, VitFly, VitFlyState,
};
pub use task::{VitFlyTask, VitFlyVelocity};
