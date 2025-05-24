//! Frame identification system for compile-time frame safety
//! This module provides a way to identify coordinate frames at compile time
//! to avoid dynamic string allocations and improve type safety.

use std::fmt::Debug;

/// Trait for compile-time frame identification
/// Each frame type must implement this to provide a unique identifier
pub trait FrameId: Debug + Clone + Copy + PartialEq + Eq + 'static {
    /// Unique identifier for this frame
    const ID: u32;
    
    /// Human-readable name for debugging
    const NAME: &'static str;
}

/// A transform relationship between two specific frame types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FramePair<Parent: FrameId, Child: FrameId> {
    /// Parent frame type
    pub parent: std::marker::PhantomData<Parent>,
    /// Child frame type  
    pub child: std::marker::PhantomData<Child>,
}

impl<Parent: FrameId, Child: FrameId> FramePair<Parent, Child> {
    pub fn new() -> Self {
        Self {
            parent: std::marker::PhantomData,
            child: std::marker::PhantomData,
        }
    }
    
    pub fn parent_id() -> u32 {
        Parent::ID
    }
    
    pub fn child_id() -> u32 {
        Child::ID
    }
    
    pub fn parent_name() -> &'static str {
        Parent::NAME
    }
    
    pub fn child_name() -> &'static str {
        Child::NAME
    }
}

impl<Parent: FrameId, Child: FrameId> Default for FramePair<Parent, Child> {
    fn default() -> Self {
        Self::new()
    }
}

/// Common frame definitions for robotics applications
/// Users can extend this by defining their own frame types
///
/// Base/world coordinate frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WorldFrame;

impl FrameId for WorldFrame {
    const ID: u32 = 0;
    const NAME: &'static str = "world";
}

/// Robot base frame  
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BaseFrame;

impl FrameId for BaseFrame {
    const ID: u32 = 1;
    const NAME: &'static str = "base";
}

/// Robot frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RobotFrame;

impl FrameId for RobotFrame {
    const ID: u32 = 2;
    const NAME: &'static str = "robot";
}

/// Camera frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CameraFrame;

impl FrameId for CameraFrame {
    const ID: u32 = 3;
    const NAME: &'static str = "camera";
}

/// Lidar frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LidarFrame;

impl FrameId for LidarFrame {
    const ID: u32 = 4;
    const NAME: &'static str = "lidar";
}

/// IMU frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImuFrame;

impl FrameId for ImuFrame {
    const ID: u32 = 5;
    const NAME: &'static str = "imu";
}

/// Convenience type aliases for common transform relationships
pub type WorldToBase = FramePair<WorldFrame, BaseFrame>;
pub type WorldToRobot = FramePair<WorldFrame, RobotFrame>;
pub type BaseToRobot = FramePair<BaseFrame, RobotFrame>;
pub type RobotToCamera = FramePair<RobotFrame, CameraFrame>;
pub type RobotToLidar = FramePair<RobotFrame, LidarFrame>;
pub type RobotToImu = FramePair<RobotFrame, ImuFrame>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_ids() {
        assert_eq!(WorldFrame::ID, 0);
        assert_eq!(WorldFrame::NAME, "world");
        
        assert_eq!(RobotFrame::ID, 2);
        assert_eq!(RobotFrame::NAME, "robot");
    }

    #[test]
    fn test_frame_pair() {
        let _pair = WorldToRobot::new();
        assert_eq!(WorldToRobot::parent_id(), WorldFrame::ID);
        assert_eq!(WorldToRobot::child_id(), RobotFrame::ID);
        assert_eq!(WorldToRobot::parent_name(), "world");
        assert_eq!(WorldToRobot::child_name(), "robot");
    }

    #[test]
    fn test_frame_equality() {
        let frame1 = WorldFrame;
        let frame2 = WorldFrame;
        assert_eq!(frame1, frame2);
        
        // Test different frame types
        assert_eq!(WorldFrame::ID, 0);
        assert_eq!(RobotFrame::ID, 2);
        assert_ne!(WorldFrame::ID, RobotFrame::ID);
    }
}