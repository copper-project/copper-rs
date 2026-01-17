//! Postprocessing for YOLOv8 pose predictions
//!
//! Handles decoding model output, non-maximum suppression, and
//! conversion to CuPoses payload format.

use crate::payloads::{CuPoses, Keypoint, NUM_KEYPOINTS, Pose};
use candle_core::{Device, IndexOp, Tensor};

/// Process raw model predictions into structured pose data
///
/// # Arguments
/// * `predictions` - Raw model output tensor of shape (pred_size, num_predictions)
/// * `img_width` - Original image width (for coordinate scaling)
/// * `img_height` - Original image height (for coordinate scaling)
/// * `model_width` - Model input width used during preprocessing
/// * `model_height` - Model input height used during preprocessing
/// * `conf_threshold` - Minimum confidence to keep a detection
/// * `iou_threshold` - IoU threshold for NMS
pub fn process_predictions(
    predictions: &Tensor,
    img_width: u32,
    img_height: u32,
    model_width: usize,
    model_height: usize,
    conf_threshold: f32,
    iou_threshold: f32,
) -> CuPoses {
    let mut poses = CuPoses::new();

    // Move to CPU for processing
    let predictions = match predictions.to_device(&Device::Cpu) {
        Ok(p) => p,
        Err(_) => return poses,
    };

    // Expected shape: (56, num_preds) where 56 = 4 (bbox) + 1 (conf) + 51 (17*3 keypoints)
    let (pred_size, num_preds) = match predictions.dims2() {
        Ok(dims) => dims,
        Err(_) => return poses,
    };

    // Validate prediction size: 4 + 1 + 17*3 = 56
    if pred_size != 4 + 1 + NUM_KEYPOINTS * 3 {
        return poses;
    }

    let w_ratio = img_width as f32 / model_width as f32;
    let h_ratio = img_height as f32 / model_height as f32;

    // Collect candidate detections
    let mut candidates: Vec<PoseCandidate> = Vec::new();

    for idx in 0..num_preds {
        let pred = match predictions.i((.., idx)) {
            Ok(p) => p,
            Err(_) => continue,
        };

        let pred_vec: Vec<f32> = match pred.to_vec1() {
            Ok(v) => v,
            Err(_) => continue,
        };

        let confidence = pred_vec[4];
        if confidence < conf_threshold {
            continue;
        }

        // Extract bbox (center x, center y, width, height)
        let cx = pred_vec[0];
        let cy = pred_vec[1];
        let w = pred_vec[2];
        let h = pred_vec[3];

        // Convert to corner format for NMS
        let x1 = cx - w / 2.0;
        let y1 = cy - h / 2.0;
        let x2 = cx + w / 2.0;
        let y2 = cy + h / 2.0;

        // Extract keypoints (17 keypoints, each with x, y, confidence)
        let mut keypoints = [Keypoint::default(); NUM_KEYPOINTS];
        for i in 0..NUM_KEYPOINTS {
            let base = 5 + i * 3;
            keypoints[i] = Keypoint::new(pred_vec[base], pred_vec[base + 1], pred_vec[base + 2]);
        }

        candidates.push(PoseCandidate {
            x1,
            y1,
            x2,
            y2,
            confidence,
            keypoints,
        });
    }

    // Sort by confidence descending
    candidates.sort_by(|a, b| {
        b.confidence
            .partial_cmp(&a.confidence)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Apply NMS
    let mut keep = vec![true; candidates.len()];
    for i in 0..candidates.len() {
        if !keep[i] {
            continue;
        }
        for j in (i + 1)..candidates.len() {
            if !keep[j] {
                continue;
            }
            let iou = compute_iou(&candidates[i], &candidates[j]);
            if iou > iou_threshold {
                keep[j] = false;
            }
        }
    }

    // Convert to CuPoses with coordinate scaling
    for (i, candidate) in candidates.into_iter().enumerate() {
        if !keep[i] {
            continue;
        }

        // Scale bbox back to original image coordinates
        let bbox = [
            (candidate.x1 + candidate.x2) / 2.0 * w_ratio, // cx
            (candidate.y1 + candidate.y2) / 2.0 * h_ratio, // cy
            (candidate.x2 - candidate.x1) * w_ratio,       // w
            (candidate.y2 - candidate.y1) * h_ratio,       // h
        ];

        // Scale keypoints back to original image coordinates
        let mut scaled_keypoints = [Keypoint::default(); NUM_KEYPOINTS];
        for (j, kp) in candidate.keypoints.iter().enumerate() {
            scaled_keypoints[j] = Keypoint::new(kp.x * w_ratio, kp.y * h_ratio, kp.confidence);
        }

        let pose = Pose {
            bbox,
            confidence: candidate.confidence,
            keypoints: scaled_keypoints,
        };

        if poses.push(pose).is_err() {
            // Reached maximum poses capacity
            break;
        }
    }

    poses
}

/// Internal struct for NMS processing
struct PoseCandidate {
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
    confidence: f32,
    keypoints: [Keypoint; NUM_KEYPOINTS],
}

/// Compute Intersection over Union between two bounding boxes
fn compute_iou(a: &PoseCandidate, b: &PoseCandidate) -> f32 {
    let x1 = a.x1.max(b.x1);
    let y1 = a.y1.max(b.y1);
    let x2 = a.x2.min(b.x2);
    let y2 = a.y2.min(b.y2);

    let intersection = (x2 - x1).max(0.0) * (y2 - y1).max(0.0);

    let area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    let area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    let union = area_a + area_b - intersection;

    if union > 0.0 {
        intersection / union
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_iou_same_box() {
        let a = PoseCandidate {
            x1: 0.0,
            y1: 0.0,
            x2: 10.0,
            y2: 10.0,
            confidence: 0.9,
            keypoints: [Keypoint::default(); NUM_KEYPOINTS],
        };
        assert!((compute_iou(&a, &a) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_iou_no_overlap() {
        let a = PoseCandidate {
            x1: 0.0,
            y1: 0.0,
            x2: 10.0,
            y2: 10.0,
            confidence: 0.9,
            keypoints: [Keypoint::default(); NUM_KEYPOINTS],
        };
        let b = PoseCandidate {
            x1: 20.0,
            y1: 20.0,
            x2: 30.0,
            y2: 30.0,
            confidence: 0.8,
            keypoints: [Keypoint::default(); NUM_KEYPOINTS],
        };
        assert!((compute_iou(&a, &b)).abs() < 1e-6);
    }
}
