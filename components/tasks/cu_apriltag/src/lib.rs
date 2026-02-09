#[cfg(unix)]
use std::mem::ManuallyDrop;

#[cfg(unix)]
use apriltag::{DetectorBuilder, Family, Image, TagParams};

#[cfg(unix)]
use apriltag_sys::image_u8_t;

use bincode::de::Decoder;
use bincode::error::DecodeError;
use cu_sensor_payloads::CuImage;
use cu_spatial_payloads::Pose as CuPose;
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::ser::SerializeTuple;
use serde::{Deserialize, Deserializer, Serialize};

// the maximum number of detections that can be returned by the detector
const MAX_DETECTIONS: usize = 16;

// Defaults
#[cfg(not(windows))]
const TAG_SIZE: f64 = 0.14;
#[cfg(not(windows))]
const FX: f64 = 2600.0;
#[cfg(not(windows))]
const FY: f64 = 2600.0;
#[cfg(not(windows))]
const CX: f64 = 900.0;
#[cfg(not(windows))]
const CY: f64 = 520.0;
#[cfg(not(windows))]
const FAMILY: &str = "tag16h5";

#[derive(Default, Debug, Clone, Encode, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct AprilTagDetections {
    pub ids: CuArrayVec<usize, MAX_DETECTIONS>,
    pub poses: CuArrayVec<CuPose<f32>, MAX_DETECTIONS>,
    pub decision_margins: CuArrayVec<f32, MAX_DETECTIONS>,
}

impl Decode<()> for AprilTagDetections {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let ids = CuArrayVec::<usize, MAX_DETECTIONS>::decode(decoder)?;
        let poses = CuArrayVec::<CuPose<f32>, MAX_DETECTIONS>::decode(decoder)?;
        let decision_margins = CuArrayVec::<f32, MAX_DETECTIONS>::decode(decoder)?;
        Ok(AprilTagDetections {
            ids,
            poses,
            decision_margins,
        })
    }
}

// implement serde support for AprilTagDetections
// This is so it can be logged with debug!.
impl Serialize for AprilTagDetections {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let CuArrayVec(ids) = &self.ids;
        let CuArrayVec(poses) = &self.poses;
        let CuArrayVec(decision_margins) = &self.decision_margins;
        let mut tup = serializer.serialize_tuple(ids.len())?;

        ids.iter()
            .zip(poses.iter())
            .zip(decision_margins.iter())
            .map(|((id, pose), margin)| (id, pose, margin))
            .for_each(|(id, pose, margin)| {
                tup.serialize_element(&(id, pose, margin)).unwrap();
            });

        tup.end()
    }
}

impl<'de> Deserialize<'de> for AprilTagDetections {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct AprilTagDetectionsVisitor;

        impl<'de> serde::de::Visitor<'de> for AprilTagDetectionsVisitor {
            type Value = AprilTagDetections;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("a tuple of (id, pose, decision_margin)")
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: serde::de::SeqAccess<'de>,
            {
                let mut detections = AprilTagDetections::new();
                while let Some((id, pose, decision_margin)) = seq.next_element()? {
                    let CuArrayVec(ids) = &mut detections.ids;
                    ids.push(id);
                    let CuArrayVec(poses) = &mut detections.poses;
                    poses.push(pose);
                    let CuArrayVec(decision_margins) = &mut detections.decision_margins;
                    decision_margins.push(decision_margin);
                }
                Ok(detections)
            }
        }

        deserializer.deserialize_tuple(MAX_DETECTIONS, AprilTagDetectionsVisitor)
    }
}

impl AprilTagDetections {
    fn new() -> Self {
        Self::default()
    }
    pub fn filtered_by_decision_margin(
        &self,
        threshold: f32,
    ) -> impl Iterator<Item = (usize, &CuPose<f32>, f32)> {
        let CuArrayVec(ids) = &self.ids;
        let CuArrayVec(poses) = &self.poses;
        let CuArrayVec(decision_margins) = &self.decision_margins;

        ids.iter()
            .zip(poses.iter())
            .zip(decision_margins.iter())
            .filter_map(move |((id, pose), margin)| {
                (*margin > threshold).then_some((*id, pose, *margin))
            })
    }
}

#[cfg(unix)]
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct AprilTags {
    family: String,
    bits_corrected: usize,
    tagsize: f64,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
}

#[cfg(not(unix))]
pub struct AprilTags {}

#[cfg(not(windows))]
fn image_from_cuimage<A>(cu_image: &CuImage<A>) -> ManuallyDrop<Image>
where
    A: ArrayLike<Element = u8>,
{
    // SAFETY: We hand AprilTag a view into CuImage's buffer and prevent it from freeing memory.
    unsafe {
        // Try to emulate what the C code is doing on the heap to avoid double free.
        let buffer_ptr = cu_image.buffer_handle.with_inner(|inner| inner.as_ptr());
        let low_level_img = Box::new(image_u8_t {
            buf: buffer_ptr as *mut u8,
            width: cu_image.format.width as i32,
            height: cu_image.format.height as i32,
            stride: cu_image.format.stride as i32,
        });
        let ptr = Box::into_raw(low_level_img);
        ManuallyDrop::new(Image::from_raw(ptr))
    }
}

impl Freezable for AprilTags {}

#[cfg(windows)]
impl CuTask for AprilTags {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuImage<Vec<u8>>);
    type Output<'m> = output_msg!(AprilTagDetections);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        _input: &Self::Input<'_>,
        _output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        Ok(())
    }
}

#[cfg(not(windows))]
impl CuTask for AprilTags {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuImage<Vec<u8>>);
    type Output<'m> = output_msg!(AprilTagDetections);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        if let Some(config) = _config {
            let family_cfg = config
                .get::<String>("family")?
                .unwrap_or(FAMILY.to_string());
            let bits_corrected: u32 = config.get::<u32>("bits_corrected")?.unwrap_or(1);
            let tagsize = config.get::<f64>("tag_size")?.unwrap_or(TAG_SIZE);
            let fx = config.get::<f64>("fx")?.unwrap_or(FX);
            let fy = config.get::<f64>("fy")?.unwrap_or(FY);
            let cx = config.get::<f64>("cx")?.unwrap_or(CX);
            let cy = config.get::<f64>("cy")?.unwrap_or(CY);
            return Ok(Self {
                family: family_cfg,
                bits_corrected: bits_corrected as usize,
                tagsize,
                fx,
                fy,
                cx,
                cy,
            });
        }
        Ok(Self {
            family: FAMILY.to_string(),
            bits_corrected: 1,
            tagsize: TAG_SIZE,
            fx: FX,
            fy: FY,
            cx: CX,
            cy: CY,
        })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let mut result = AprilTagDetections::new();
        if let Some(payload) = input.payload() {
            let family: Family = self
                .family
                .parse()
                .map_err(|_| CuError::from("invalid apriltag family"))?;
            let mut detector = DetectorBuilder::default()
                .add_family_bits(family, self.bits_corrected)
                .build()
                .map_err(|e| CuError::new_with_cause("failed to build apriltag detector", e))?;
            let tag_params = TagParams {
                fx: self.fx,
                fy: self.fy,
                cx: self.cx,
                cy: self.cy,
                tagsize: self.tagsize,
            };
            let image = image_from_cuimage(payload);
            let detections = detector.detect(&image);
            for detection in detections {
                if let Some(aprilpose) = detection.estimate_tag_pose(&tag_params) {
                    let translation = aprilpose.translation();
                    let rotation = aprilpose.rotation();
                    let mut mat: [[f32; 4]; 4] = [[0.0, 0.0, 0.0, 0.0]; 4];
                    mat[0][3] = translation.data()[0] as f32;
                    mat[1][3] = translation.data()[1] as f32;
                    mat[2][3] = translation.data()[2] as f32;
                    mat[0][0] = rotation.data()[0] as f32;
                    mat[0][1] = rotation.data()[3] as f32;
                    mat[0][2] = rotation.data()[2 * 3] as f32;
                    mat[1][0] = rotation.data()[1] as f32;
                    mat[1][1] = rotation.data()[1 + 3] as f32;
                    mat[1][2] = rotation.data()[1 + 2 * 3] as f32;
                    mat[2][0] = rotation.data()[2] as f32;
                    mat[2][1] = rotation.data()[2 + 3] as f32;
                    mat[2][2] = rotation.data()[2 + 2 * 3] as f32;

                    let pose = CuPose::<f32>::from_matrix(mat);
                    let CuArrayVec(detections) = &mut result.poses;
                    detections.push(pose);
                    let CuArrayVec(decision_margin) = &mut result.decision_margins;
                    decision_margin.push(detection.decision_margin());
                    let CuArrayVec(ids) = &mut result.ids;
                    ids.push(detection.id());
                }
            }
        };
        output.tov = input.tov;
        output.set_payload(result);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #[allow(unused_imports)]
    use super::*;
    use anyhow::Context;
    use anyhow::Result;
    use image::{ImageBuffer, ImageReader};
    use image::{Luma, imageops::FilterType, imageops::crop, imageops::resize};

    #[cfg(not(windows))]
    use cu_sensor_payloads::CuImageBufferFormat;

    #[allow(dead_code)]
    fn process_image(path: &str) -> Result<ImageBuffer<Luma<u8>, Vec<u8>>> {
        let reader = ImageReader::open(path).with_context(|| "Failed to open image")?;
        let mut img = reader
            .decode()
            .context("Failed to decode image")?
            .into_luma8();
        let (orig_w, orig_h) = img.dimensions();

        let new_h = (orig_w as f32 * 9.0 / 16.0) as u32;
        let crop_y = (orig_h - new_h) / 2; // Center crop

        let cropped = crop(&mut img, 0, crop_y, orig_w, new_h).to_image();
        Ok(resize(&cropped, 1920, 1080, FilterType::Lanczos3))
    }

    #[test]
    #[cfg(not(windows))]
    fn test_end2end_apriltag() -> Result<()> {
        let img = process_image("tests/data/simple.png")?;
        let format = CuImageBufferFormat {
            width: img.width(),
            height: img.height(),
            stride: img.width(),
            pixel_format: "GRAY".as_bytes().try_into()?,
        };
        let buffer_handle = CuHandle::new_detached(img.into_raw());
        let cuimage = CuImage::new(format, buffer_handle);

        let mut config = ComponentConfig::default();
        config.set("tag_size", 0.14);
        config.set("fx", 2600.0);
        config.set("fy", 2600.0);
        config.set("cx", 900.0);
        config.set("cy", 520.0);
        config.set("family", "tag16h5".to_string());

        let mut task = AprilTags::new(Some(&config), ())?;
        let input = CuMsg::<CuImage<Vec<u8>>>::new(Some(cuimage));
        let mut output = CuMsg::<AprilTagDetections>::default();

        let clock = RobotClock::new();
        let result = task.process(&clock, &input, &mut output);
        assert!(result.is_ok());

        if let Some(detections) = output.payload() {
            let detections = detections
                .filtered_by_decision_margin(150.0)
                .collect::<Vec<_>>();

            assert_eq!(detections.len(), 1);
            assert_eq!(detections[0].0, 4);
            return Ok(());
        }
        Err(anyhow::anyhow!("No output"))
    }
}
