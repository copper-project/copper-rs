use apriltag::{Detector, DetectorBuilder, Family, Image, TagParams};
use apriltag_sys::image_u8;
use arrayvec::ArrayVec;
use bincode::de::{BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::BorrowDecode;
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu_sensor_payloads::CuImage;
use cu_spatial_payloads::Pose as CuPose;

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

impl<T, const N: usize> Decode for CuArrayVec<T, N>
where
    T: Decode + 'static,
{
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
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

impl<'de, T, const N: usize> BorrowDecode<'de> for CuArrayVec<T, N>
where
    T: BorrowDecode<'de> + 'static,
{
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
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

#[derive(Default, Debug, Clone, Encode, Decode)]
pub struct AprilTagDetections {
    pub detections: CuArrayVec<CuPose<f32>, MAX_DETECTIONS>,
    pub thresholds: CuArrayVec<f32, MAX_DETECTIONS>,
}

impl AprilTagDetections {
    fn new() -> Self {
        Self::default()
    }
}

struct AprilTags {
    detector: Detector,
    tag_params: TagParams,
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
            payload.buffer_handle.with_inner(|buffer| {
                let slice: &[u8] = buffer.as_ref();
                let mut low_level_img: image_u8 = image_u8 {
                    buf: slice.as_ptr() as *mut u8,
                    width: payload.format.width as i32,
                    height: payload.format.height as i32,
                    stride: payload.format.width as i32,
                };
                // If those API developers could give us a safe way to inject a buffer
                // without copying megabytes of data in their types this would avoid this kind of
                // shady crap.
                let image = unsafe { Image::from_raw(&mut low_level_img) };

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
                        let CuArrayVec(detections) = &mut result.detections;
                        detections.push(pose);
                        let CuArrayVec(thresholds) = &mut result.thresholds;
                        thresholds.push(detection.decision_margin());
                    }
                }
            })
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
    use image::{imageops::crop, imageops::resize, imageops::FilterType, GenericImageView, Luma};
    use image::{ImageBuffer, ImageReader};

    fn process_image(path: &str) -> Result<ImageBuffer<Luma<u8>, Vec<u8>>> {
        let reader = ImageReader::open(&path).with_context(|| "Failed to open image")?;
        let img = reader.decode().context("Failed to decode image")?;

        println!(
            "Loaded image with dimensions: {} x {}",
            img.width(),
            img.height()
        );

        let mut img = img.into_luma8();
        let (orig_w, orig_h) = img.dimensions();

        let new_h = (orig_w as f32 * 9.0 / 16.0) as u32;
        let crop_y = (orig_h - new_h) / 2; // Center crop

        let cropped = crop(&mut img, 0, crop_y, orig_w, new_h).to_image();
        Ok(resize(&cropped, 1920, 1080, FilterType::Lanczos3))
    }

    #[test]
    fn test_end2end_apriltag() -> Result<()> {
        let img = process_image("tests/data/simple.jpg")?;
        Ok(())
    }
}
