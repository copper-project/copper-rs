//! Rerun visualization sink for pose estimation results

use crate::payloads::{CuPoses, SKELETON_CONNECTIONS};
use cu_sensor_payloads::CuImage;
use cu29::prelude::*;
use rerun::{
    Boxes2D, ChannelDatatype, ColorModel, Image, LineStrips2D, Points2D, RecordingStream,
    RecordingStreamBuilder, Vec2D,
};
use std::sync::atomic::{AtomicUsize, Ordering};

static IMAGE_LOG_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Rerun visualization sink for pose estimation
///
/// Displays the camera image with overlaid pose skeletons,
/// keypoints, and bounding boxes.
pub struct RerunPoseViz {
    rec: RecordingStream,
}

const CAMERA_ENTITY: &str = "camera/image";

impl Freezable for RerunPoseViz {}

impl CuSinkTask for RerunPoseViz {
    type Resources<'r> = ();
    // Two separate inputs: image from camera, poses from YOLO
    // Use 'm lifetime and separate types for multiple inputs
    type Input<'m> = input_msg!('m, CuImage<Vec<u8>>, CuPoses);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let rec = RecordingStreamBuilder::new("YOLO Pose Demo")
            .spawn()
            .map_err(|e| CuError::new_with_cause("Failed to spawn Rerun stream", e))?;

        Ok(Self { rec })
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        // Input is a reference to a tuple of message references
        let (image_msg, poses_msg) = *input;

        // Log the camera image if available
        if let Some(image) = image_msg.payload() {
            self.log_image(image)?;
        }

        // Log pose visualizations if available
        if let Some(poses) = poses_msg.payload() {
            self.log_poses(poses)?;
        }

        Ok(())
    }
}

impl RerunPoseViz {
    fn log_image(&self, image: &CuImage<Vec<u8>>) -> CuResult<()> {
        let width = image.format.width;
        let height = image.format.height;
        let pixel_format = &image.format.pixel_format;

        // Convert image to RGB for visualization
        let rgb_data: Vec<u8> = image.buffer_handle.with_inner(|data| {
            match pixel_format {
                b"YUYV" => yuyv_to_rgb(data, width as usize, height as usize),
                b"NV12" => nv12_to_rgb(data, width as usize, height as usize),
                b"RGB3" | b"RGB " => data.to_vec(),
                b"BGR3" | b"BGR " => bgr_to_rgb(data),
                _ => {
                    // Attempt to handle as RGB or grayscale
                    if data.len() == (width * height * 3) as usize {
                        data.to_vec()
                    } else if data.len() == (width * height) as usize {
                        data.iter().flat_map(|&g| [g, g, g]).collect()
                    } else {
                        vec![128u8; (width * height * 3) as usize]
                    }
                }
            }
        });

        let log_idx = IMAGE_LOG_COUNT.fetch_add(1, Ordering::Relaxed);
        if log_idx < 5 {
            info!(
                "rerun_viz: width={} height={} pixel_format={} rgb_len={}",
                width,
                height,
                String::from_utf8_lossy(pixel_format),
                rgb_data.len()
            );
        }

        let rerun_image = Image::from_color_model_and_bytes(
            rgb_data,
            [width, height],
            ColorModel::RGB,
            ChannelDatatype::U8,
        );

        self.rec
            .log(CAMERA_ENTITY, &rerun_image)
            .map_err(|e| CuError::new_with_cause("Failed to log image", e))?;

        Ok(())
    }

    fn log_poses(&self, poses: &CuPoses) -> CuResult<()> {
        if poses.is_empty() {
            // Clear previous visualizations when no poses detected
            let _ = self
                .rec
                .log(CAMERA_ENTITY, &Points2D::new(Vec::<Vec2D>::new()));
            let _ = self
                .rec
                .log(CAMERA_ENTITY, &LineStrips2D::new(Vec::<Vec<Vec2D>>::new()));
            let _ = self.rec.log(
                CAMERA_ENTITY,
                &Boxes2D::from_centers_and_sizes(Vec::<Vec2D>::new(), Vec::<Vec2D>::new()),
            );
            return Ok(());
        }

        // Collect all keypoints for visualization
        let mut all_keypoints: Vec<Vec2D> = Vec::new();
        let mut keypoint_colors: Vec<rerun::Color> = Vec::new();

        // Collect skeleton lines
        let mut skeleton_strips: Vec<Vec<Vec2D>> = Vec::new();

        // Collect bounding boxes
        let mut box_centers: Vec<Vec2D> = Vec::new();
        let mut box_sizes: Vec<Vec2D> = Vec::new();

        // Color palette for different poses
        let pose_colors = [
            rerun::Color::from_rgb(255, 0, 0),   // Red
            rerun::Color::from_rgb(0, 255, 0),   // Green
            rerun::Color::from_rgb(0, 0, 255),   // Blue
            rerun::Color::from_rgb(255, 255, 0), // Yellow
            rerun::Color::from_rgb(255, 0, 255), // Magenta
            rerun::Color::from_rgb(0, 255, 255), // Cyan
        ];

        for (pose_idx, pose) in poses.iter().enumerate() {
            let color = pose_colors[pose_idx % pose_colors.len()];

            // Add keypoints
            for kp in &pose.keypoints {
                if kp.confidence > 0.5 {
                    all_keypoints.push(Vec2D::new(kp.x, kp.y));
                    keypoint_colors.push(color);
                }
            }

            // Add skeleton connections
            for &(i, j) in &SKELETON_CONNECTIONS {
                let kp1 = &pose.keypoints[i];
                let kp2 = &pose.keypoints[j];

                if kp1.confidence > 0.5 && kp2.confidence > 0.5 {
                    skeleton_strips.push(vec![Vec2D::new(kp1.x, kp1.y), Vec2D::new(kp2.x, kp2.y)]);
                }
            }

            // Add bounding box
            box_centers.push(Vec2D::new(pose.bbox[0], pose.bbox[1]));
            box_sizes.push(Vec2D::new(pose.bbox[2], pose.bbox[3]));
        }

        // Log keypoints
        if !all_keypoints.is_empty() {
            let radii = vec![rerun::Radius::new_scene_units(5.0); all_keypoints.len()];
            self.rec
                .log(
                    CAMERA_ENTITY,
                    &Points2D::new(all_keypoints)
                        .with_colors(keypoint_colors)
                        .with_radii(radii),
                )
                .map_err(|e| CuError::new_with_cause("Failed to log keypoints", e))?;
        }

        // Log skeleton
        if !skeleton_strips.is_empty() {
            let skeleton_colors = vec![rerun::Color::from_rgb(255, 255, 0); skeleton_strips.len()];
            self.rec
                .log(
                    CAMERA_ENTITY,
                    &LineStrips2D::new(skeleton_strips).with_colors(skeleton_colors),
                )
                .map_err(|e| CuError::new_with_cause("Failed to log skeleton", e))?;
        }

        // Log bounding boxes
        if !box_centers.is_empty() {
            let box_colors = vec![rerun::Color::from_rgb(0, 255, 0); box_centers.len()];
            self.rec
                .log(
                    CAMERA_ENTITY,
                    &Boxes2D::from_centers_and_sizes(box_centers, box_sizes)
                        .with_colors(box_colors),
                )
                .map_err(|e| CuError::new_with_cause("Failed to log boxes", e))?;
        }

        Ok(())
    }
}

// Color conversion helpers (duplicated from preprocess for self-containment)

fn yuyv_to_rgb(data: &[u8], width: usize, height: usize) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(width * height * 3);

    for chunk in data.chunks(4) {
        if chunk.len() < 4 {
            break;
        }
        let y0 = chunk[0] as f32;
        let u = chunk[1] as f32 - 128.0;
        let y1 = chunk[2] as f32;
        let v = chunk[3] as f32 - 128.0;

        let r0 = (y0 + 1.402 * v).clamp(0.0, 255.0) as u8;
        let g0 = (y0 - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8;
        let b0 = (y0 + 1.772 * u).clamp(0.0, 255.0) as u8;
        rgb.extend_from_slice(&[r0, g0, b0]);

        let r1 = (y1 + 1.402 * v).clamp(0.0, 255.0) as u8;
        let g1 = (y1 - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8;
        let b1 = (y1 + 1.772 * u).clamp(0.0, 255.0) as u8;
        rgb.extend_from_slice(&[r1, g1, b1]);
    }

    rgb
}

fn nv12_to_rgb(data: &[u8], width: usize, height: usize) -> Vec<u8> {
    let mut rgb = Vec::with_capacity(width * height * 3);
    let y_plane = &data[..width * height];
    let uv_plane = &data[width * height..];

    for y_idx in 0..height {
        for x_idx in 0..width {
            let y = y_plane[y_idx * width + x_idx] as f32;
            let uv_idx = (y_idx / 2) * width + (x_idx / 2) * 2;
            let u = uv_plane.get(uv_idx).copied().unwrap_or(128) as f32 - 128.0;
            let v = uv_plane.get(uv_idx + 1).copied().unwrap_or(128) as f32 - 128.0;

            let r = (y + 1.402 * v).clamp(0.0, 255.0) as u8;
            let g = (y - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8;
            let b = (y + 1.772 * u).clamp(0.0, 255.0) as u8;
            rgb.extend_from_slice(&[r, g, b]);
        }
    }

    rgb
}

fn bgr_to_rgb(data: &[u8]) -> Vec<u8> {
    data.chunks(3)
        .flat_map(|bgr| {
            if bgr.len() >= 3 {
                [bgr[2], bgr[1], bgr[0]]
            } else {
                [0, 0, 0]
            }
        })
        .collect()
}
