// you need a blank line between the cfg and the use
use apriltag::{Detector, DetectorBuilder, Family, Image, TagParams};
use apriltag_sys::image_u8_t;
use arrayvec::ArrayVec;
use bincode::de::{BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::BorrowDecode;
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu_sensor_payloads::CuImage;
use cu_spatial_payloads::Pose as CuPose;
use serde::ser::SerializeTuple;
use serde::{Deserialize, Deserializer, Serialize};
use std::mem::ManuallyDrop;

// the maximum number of detections that can be returned by the detector
const MAX_DETECTIONS: usize = 16;

// Defaults
const TAG_SIZE: f64 = 0.14;
const FX: f64 = 2600.0;
const FY: f64 = 2600.0;
const CX: f64 = 900.0;
const CY: f64 = 520.0;
const FAMILY: &str = "tag16h5";

/// TODO: Move that to the runtime as it is useful.
#[derive(Debug, Clone)]
pub struct CuArrayVec<T, const N: usize>(ArrayVec<T, N>);

impl<T, const N: usize> Default for CuArrayVec<T, N> {
    fn default() -> Self {
        Self(ArrayVec::new())
    }
}

impl<T, const N: usize> Encode for CuArrayVec<T, N>
where
    T: Encode + 'static,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let CuArrayVec(inner) = self;
        inner.as_slice().encode(encoder)
    }
}

impl<T, const N: usize> Decode<()> for CuArrayVec<T, N>
where
    T: Decode<()> + 'static,
{
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let inner = Vec::<T>::decode(decoder)?;
        let actual_len = inner.len();
        if actual_len > N {
            return Err(DecodeError::ArrayLengthMismatch {
                required: N,
                found: actual_len,
            });
        }

        let mut array_vec = ArrayVec::new();
        for item in inner {
            array_vec.push(item); // Push elements one by one
        }
        Ok(CuArrayVec(array_vec))
    }
}

impl<'de, T, const N: usize> BorrowDecode<'de, ()> for CuArrayVec<T, N>
where
    T: BorrowDecode<'de, ()> + 'static,
{
    fn borrow_decode<D: BorrowDecoder<'de, Context = ()>>(
        decoder: &mut D,
    ) -> Result<Self, DecodeError> {
        let inner = Vec::<T>::borrow_decode(decoder)?;
        let actual_len = inner.len();
        if actual_len > N {
            return Err(DecodeError::ArrayLengthMismatch {
                required: N,
                found: actual_len,
            });
        }

        let mut array_vec = ArrayVec::new();
        for item in inner {
            array_vec.push(item); // Push elements one by one
        }
        Ok(CuArrayVec(array_vec))
    }
}

#[derive(Default, Debug, Clone, Encode)]
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

pub struct AprilTags {
    detector: Detector,
    tag_params: TagParams,
}

// Absolute shitshow of an API. This is to avoid a full image copy.
// Hopefully this is sound.
fn image_from_cuimage<A>(cu_image: &CuImage<A>) -> ManuallyDrop<Image>
where
    A: ArrayLike<Element = u8>,
{
    unsafe {
        // Try to emulate what the C code is doing on the heap to avoid double free
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

impl<'cl> CuTask<'cl> for AprilTags {
    type Input = input_msg!('cl, CuImage<Vec<u8>>);
    type Output = output_msg!('cl, AprilTagDetections);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        if let Some(config) = _config {
            let family_cfg: String = config.get("family").unwrap_or(FAMILY.to_string());
            let family: Family = family_cfg.parse().unwrap();
            let bits_corrected: u32 = config.get("bits_corrected").unwrap_or(1);
            let tagsize = config.get("tag_size").unwrap_or(TAG_SIZE);
            let fx = config.get("fx").unwrap_or(FX);
            let fy = config.get("fy").unwrap_or(FY);
            let cx = config.get("cx").unwrap_or(CX);
            let cy = config.get("cy").unwrap_or(CY);
            let tag_params = TagParams {
                fx,
                fy,
                cx,
                cy,
                tagsize,
            };

            let detector = DetectorBuilder::default()
                .add_family_bits(family, bits_corrected as usize)
                .build()
                .unwrap();
            return Ok(Self {
                detector,
                tag_params,
            });
        }
        Ok(Self {
            detector: DetectorBuilder::default()
                .add_family_bits(FAMILY.parse::<Family>().unwrap(), 1)
                .build()
                .unwrap(),
            tag_params: TagParams {
                fx: FX,
                fy: FY,
                cx: CX,
                cy: CY,
                tagsize: TAG_SIZE,
            },
        })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        let mut result = AprilTagDetections::new();
        if let Some(payload) = input.payload() {
            let image = image_from_cuimage(payload);
            let detections = self.detector.detect(&image);
            for detection in detections {
                if let Some(aprilpose) = detection.estimate_tag_pose(&self.tag_params) {
                    let translation = aprilpose.translation();
                    let rotation = aprilpose.rotation();
                    let mut pose = CuPose::<f32>::default();
                    pose.mat[0][3] = translation.data()[0] as f32;
                    pose.mat[1][3] = translation.data()[1] as f32;
                    pose.mat[2][3] = translation.data()[2] as f32;
                    pose.mat[0][0] = rotation.data()[0] as f32;
                    pose.mat[0][1] = rotation.data()[3] as f32;
                    pose.mat[0][2] = rotation.data()[2 * 3] as f32;
                    pose.mat[1][0] = rotation.data()[1] as f32;
                    pose.mat[1][1] = rotation.data()[1 + 3] as f32;
                    pose.mat[1][2] = rotation.data()[1 + 2 * 3] as f32;
                    pose.mat[2][0] = rotation.data()[2] as f32;
                    pose.mat[2][1] = rotation.data()[2 + 3] as f32;
                    pose.mat[2][2] = rotation.data()[2 + 2 * 3] as f32;
                    let CuArrayVec(detections) = &mut result.poses;
                    detections.push(pose);
                    let CuArrayVec(decision_margin) = &mut result.decision_margins;
                    decision_margin.push(detection.decision_margin());
                    let CuArrayVec(ids) = &mut result.ids;
                    ids.push(detection.id());
                }
            }
        };
        output.set_payload(result);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::Context;
    use anyhow::Result;
    use cu_sensor_payloads::CuImageBufferFormat;
    use image::{imageops::crop, imageops::resize, imageops::FilterType, Luma};
    use image::{ImageBuffer, ImageReader};

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
    fn test_end2end_apriltag() -> Result<()> {
        let img = process_image("tests/data/simple.jpg")?;
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

        let mut task = AprilTags::new(Some(&config))?;
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
