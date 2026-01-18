use bincode::{Decode, Encode};
use serde::{Deserialize, Serialize};

/// Maximum number of poses to detect per frame
pub const MAX_POSES: usize = 32;

/// Number of keypoints in COCO pose format (YOLOv8-pose)
pub const NUM_KEYPOINTS: usize = 17;

// COCO keypoint indices for reference:
// 0: nose, 1: left_eye, 2: right_eye, 3: left_ear, 4: right_ear,
// 5: left_shoulder, 6: right_shoulder, 7: left_elbow, 8: right_elbow,
// 9: left_wrist, 10: right_wrist, 11: left_hip, 12: right_hip,
// 13: left_knee, 14: right_knee, 15: left_ankle, 16: right_ankle

// Skeleton connections for visualization (pairs of keypoint indices)
pub const SKELETON_CONNECTIONS: [(usize, usize); 19] = [
    (0, 1),   // nose -> left_eye
    (0, 2),   // nose -> right_eye
    (1, 3),   // left_eye -> left_ear
    (2, 4),   // right_eye -> right_ear
    (5, 6),   // left_shoulder -> right_shoulder
    (5, 7),   // left_shoulder -> left_elbow
    (7, 9),   // left_elbow -> left_wrist
    (6, 8),   // right_shoulder -> right_elbow
    (8, 10),  // right_elbow -> right_wrist
    (5, 11),  // left_shoulder -> left_hip
    (6, 12),  // right_shoulder -> right_hip
    (11, 12), // left_hip -> right_hip
    (11, 13), // left_hip -> left_knee
    (13, 15), // left_knee -> left_ankle
    (12, 14), // right_hip -> right_knee
    (14, 16), // right_knee -> right_ankle
    (0, 5),   // nose -> left_shoulder (neck proxy)
    (0, 6),   // nose -> right_shoulder (neck proxy)
    (5, 6),   // left_shoulder -> right_shoulder (duplicate for torso)
];

/// A single keypoint with x, y coordinates and confidence score
#[derive(Debug, Clone, Copy, Default, Encode, Decode, Serialize, Deserialize)]
pub struct Keypoint {
    pub x: f32,
    pub y: f32,
    pub confidence: f32,
}

impl Keypoint {
    pub fn new(x: f32, y: f32, confidence: f32) -> Self {
        Self { x, y, confidence }
    }

    /// Check if keypoint is visible (confidence above threshold)
    pub fn is_visible(&self, threshold: f32) -> bool {
        self.confidence >= threshold
    }
}

/// A single detected pose with bounding box and keypoints
#[derive(Debug, Clone, Default, Encode, Decode, Serialize, Deserialize)]
pub struct Pose {
    /// Bounding box in format [x_center, y_center, width, height]
    pub bbox: [f32; 4],
    /// Detection confidence score
    pub confidence: f32,
    /// 17 COCO keypoints
    pub keypoints: [Keypoint; NUM_KEYPOINTS],
}

impl Pose {
    /// Get bounding box corners as (x1, y1, x2, y2)
    pub fn bbox_corners(&self) -> (f32, f32, f32, f32) {
        let [cx, cy, w, h] = self.bbox;
        let x1 = cx - w / 2.0;
        let y1 = cy - h / 2.0;
        let x2 = cx + w / 2.0;
        let y2 = cy + h / 2.0;
        (x1, y1, x2, y2)
    }
}

/// Collection of detected poses in a frame
#[derive(Debug, Clone, Default, Encode, Decode, Serialize, Deserialize)]
pub struct CuPoses {
    pub poses: Vec<Pose>,
}

impl CuPoses {
    pub fn new() -> Self {
        Self { poses: Vec::new() }
    }

    #[allow(clippy::result_large_err)]
    pub fn push(&mut self, pose: Pose) -> Result<(), Pose> {
        if self.poses.len() >= MAX_POSES {
            return Err(pose);
        }
        self.poses.push(pose);
        Ok(())
    }

    pub fn len(&self) -> usize {
        self.poses.len()
    }

    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Pose> {
        self.poses.iter()
    }
}
