//! AnyNet stereo disparity as a three-stage Copper anytime task.
//!
//! This ports the first three stages of AnyNet (ICRA 2019) to Candle. Stage 1
//! is the mandatory base result and stages 2 and 3 are bounded refinement
//! quanta. The optional SPN stage from the original model is intentionally not
//! included.
//!
//! The architecture is ported from AnyNet (Wang et al., ICRA 2019) under its
//! MIT license.

mod model;

use bincode::de::Decoder;
use bincode::error::DecodeError;
use bincode::{Decode, Encode};
use candle_core::{DType, Device, Tensor};
use candle_nn::{VarBuilder, VarMap};
use cu_sensor_payloads::{CuDepthLength, CuDepthMap, CuDepthMapFormat, CuImage};
use cu29::cutask_anytime::{AnytimeStatus, CuAnytimeTask, Quality, quality_from_f32};
use cu29::prelude::*;
use cu29::units::si::f32::{Length, Ratio};
use cu29::units::si::length::meter;
use serde::{Deserialize, Serialize};

const DEFAULT_MAX_DISPARITY: u32 = 192;
const DEFAULT_QUALITIES: [f32; 3] = [0.49, 0.70, 1.00];

/// Stages this port publishes: the base plus two refinement quanta.
pub const STAGES: usize = 3;

/// One rectified RGB stereo pair.
#[derive(Default, Debug, Clone, Encode, Serialize, Deserialize, Reflect)]
#[reflect(from_reflect = false, no_field_bounds)]
pub struct StereoPair {
    pub left: CuImage<Vec<u8>>,
    pub right: CuImage<Vec<u8>>,
}

impl Decode<()> for StereoPair {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            left: Decode::decode(decoder)?,
            right: Decode::decode(decoder)?,
        })
    }
}

/// Depth exactly as one stage published it, kept for offline comparison.
///
/// The depth buffer is a detached copy: the live result is refined in place, so
/// a shared handle would show every snapshot the final stage's values.
#[derive(Default, Debug, Clone, Encode, Serialize, Deserialize)]
pub struct StageSnapshot {
    pub depth: CuDepthMap<Vec<Length>, CuDepthLength>,
    pub stage: u8,
    pub quality: Ratio,
    /// Time from the start of the job to the moment this stage published.
    pub elapsed: CuDuration,
}

impl Decode<()> for StageSnapshot {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            depth: Decode::decode(decoder)?,
            stage: Decode::decode(decoder)?,
            quality: Decode::decode(decoder)?,
            elapsed: Decode::decode(decoder)?,
        })
    }
}

/// Depth result published after one of AnyNet's three inference stages.
#[derive(Default, Debug, Clone, Encode, Serialize, Deserialize, Reflect)]
#[reflect(from_reflect = false, no_field_bounds)]
pub struct StereoDepth {
    pub depth: CuDepthMap<Vec<Length>, CuDepthLength>,
    pub stage: u8,
    pub quality: Ratio,
    /// Indexed by stage - 1, populated for every stage this job published when
    /// the node sets `stage_snapshots`. All `None` otherwise: a copy per quantum
    /// is a visualization cost, not something a consumer of the result pays for.
    #[reflect(ignore)]
    pub snapshots: [Option<StageSnapshot>; STAGES],
}

impl Decode<()> for StereoDepth {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            depth: Decode::decode(decoder)?,
            stage: Decode::decode(decoder)?,
            quality: Decode::decode(decoder)?,
            snapshots: Decode::decode(decoder)?,
        })
    }
}

/// Inference device selected by the component's `device` config key.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AnyNetDevice {
    #[default]
    Cpu,
    Cuda,
}

/// Input normalization the loaded weights were trained with, selected by the
/// component's `normalization` config key.
///
/// The original AnyNet dataloader feeds `/255` ImageNet-normalized RGB, which
/// is the default here. Some publicly available checkpoints (e.g. the
/// `Stereo-3D-Detection` mirror of the KITTI weights) were trained on raw
/// 0..255 pixel values instead; a mismatch is not an error anywhere — it just
/// produces confidently wrong disparities.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AnyNetNormalization {
    #[default]
    Imagenet,
    Raw,
}

/// Three-stage anytime stereo-depth inference.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct AnyNetStereo {
    #[reflect(ignore)]
    model: model::AnyNet,
    #[reflect(ignore)]
    device: Device,
    focal_pixels: f32,
    baseline_meters: f32,
    qualities: [Quality; 3],
    #[reflect(ignore)]
    normalization: AnyNetNormalization,
    #[reflect(ignore)]
    left_features: Option<[Tensor; 3]>,
    #[reflect(ignore)]
    right_features: Option<[Tensor; 3]>,
    #[reflect(ignore)]
    disparity: Option<Tensor>,
    image_height: usize,
    image_width: usize,
    stage: u8,
    stage_snapshots: bool,
    job_start: CuTime,
}

// Model weights are immutable and all inference tensors are per-job state.
impl Freezable for AnyNetStereo {}

impl CuAnytimeTask for AnyNetStereo {
    type Input<'m> = input_msg!(StereoPair);
    type Output<'m> = output_msg!(StereoDepth);
    type Resources<'r> = ();
    type Quality = Quality;

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let device_config = config
            .map(|config| config.get_value::<AnyNetDevice>("device"))
            .transpose()?
            .flatten()
            .unwrap_or_default();
        let device = select_device(device_config)?;
        let max_disparity = config
            .map(|config| config.get::<u32>("max_disp"))
            .transpose()?
            .flatten()
            .unwrap_or(DEFAULT_MAX_DISPARITY);
        if max_disparity == 0 || !max_disparity.is_multiple_of(16) {
            return Err(CuError::from(
                "cu_anynet: max_disp must be a non-zero multiple of 16",
            ));
        }
        // Required: silently defaulting to one rig's calibration would produce
        // metrically wrong depth on every other camera.
        let focal_pixels = config
            .map(|config| config.get::<f32>("focal_px"))
            .transpose()?
            .flatten()
            .ok_or("cu_anynet: focal_px (camera focal length in pixels) is required")?;
        let baseline_meters = config
            .map(|config| config.get::<f32>("baseline_m"))
            .transpose()?
            .flatten()
            .ok_or("cu_anynet: baseline_m (stereo baseline in meters) is required")?;
        if !focal_pixels.is_finite()
            || focal_pixels <= 0.0
            || !baseline_meters.is_finite()
            || baseline_meters <= 0.0
        {
            return Err(CuError::from(
                "cu_anynet: focal_px and baseline_m must be finite and positive",
            ));
        }
        let mut quality_values = DEFAULT_QUALITIES;
        if let Some(config) = config {
            for (index, value) in quality_values.iter_mut().enumerate() {
                let key = match index {
                    0 => "quality_stage1",
                    1 => "quality_stage2",
                    _ => "quality_stage3",
                };
                if let Some(configured) = config.get::<f32>(key)? {
                    *value = configured;
                }
            }
        }
        validate_qualities(quality_values)?;
        let qualities = quality_values.map(quality_from_f32);

        let stage_snapshots = config
            .map(|config| config.get::<bool>("stage_snapshots"))
            .transpose()?
            .flatten()
            .unwrap_or(false);

        let normalization = config
            .map(|config| config.get_value::<AnyNetNormalization>("normalization"))
            .transpose()?
            .flatten()
            .unwrap_or_default();

        let weights = config
            .map(|config| config.get::<String>("weights"))
            .transpose()?
            .flatten();
        let model = match weights {
            Some(path) => {
                // SAFETY: Candle memory-maps the safetensors file read-only and
                // the model tensors retain ownership of the mapping.
                let builder = unsafe {
                    VarBuilder::from_mmaped_safetensors(&[path], DType::F32, &device).map_err(
                        |error| CuError::new_with_cause("cu_anynet: failed to load weights", error),
                    )?
                };
                model::AnyNet::load(builder, max_disparity as usize).map_err(|error| {
                    CuError::new_with_cause("cu_anynet: failed to build model", error)
                })?
            }
            None => {
                warning!("cu_anynet: no weights configured; using random smoke-mode weights");
                let variables = VarMap::new();
                let builder = VarBuilder::from_varmap(&variables, DType::F32, &device);
                model::AnyNet::load(builder, max_disparity as usize).map_err(|error| {
                    CuError::new_with_cause("cu_anynet: failed to build smoke-mode model", error)
                })?
            }
        };

        Ok(Self {
            model,
            device,
            focal_pixels,
            baseline_meters,
            qualities,
            normalization,
            left_features: None,
            right_features: None,
            disparity: None,
            image_height: 0,
            image_width: 0,
            stage: 0,
            stage_snapshots,
            job_start: CuTime::default(),
        })
    }

    fn base(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        let Some(pair) = input.payload() else {
            output.clear_payload();
            self.clear_job();
            return Ok(AnytimeStatus::Aborted);
        };
        // A failure below must not leave the previous job's stage and cached
        // features behind as if they belonged to this one.
        self.clear_job();
        validate_stereo_pair(pair)?;
        if self.stage_snapshots {
            self.job_start = ctx.now();
        }
        self.image_height = pair.left.format.height as usize;
        self.image_width = pair.left.format.width as usize;

        let left = image_to_tensor(&pair.left, &self.device, self.normalization)?;
        let right = image_to_tensor(&pair.right, &self.device, self.normalization)?;
        let left_features = self.model.features(&left).map_err(model_error)?;
        let right_features = self.model.features(&right).map_err(model_error)?;
        let disparity = self
            .model
            .base(
                &left_features,
                &right_features,
                self.image_height,
                self.image_width,
            )
            .map_err(model_error)?;

        let depth = depth_from_disparity(
            &disparity,
            self.image_width,
            self.image_height,
            self.focal_pixels,
            self.baseline_meters,
        )?;
        let mut payload = StereoDepth {
            depth,
            stage: 1,
            quality: self.qualities[0],
            snapshots: Default::default(),
        };
        if self.stage_snapshots {
            payload.snapshots[0] = Some(StageSnapshot {
                depth: detached_copy(&payload.depth),
                stage: 1,
                quality: payload.quality,
                elapsed: ctx.now() - self.job_start,
            });
        }
        output.set_payload(payload);
        output.tov = input.tov;
        self.left_features = Some(left_features);
        self.right_features = Some(right_features);
        self.disparity = Some(disparity);
        self.stage = 1;
        Ok(AnytimeStatus::Improved(self.qualities[0]))
    }

    fn refine(
        &mut self,
        ctx: &CuContext,
        output: &mut Self::Output<'_>,
    ) -> CuResult<AnytimeStatus<Quality>> {
        if self.stage >= 3 {
            return Ok(AnytimeStatus::Converged(self.qualities[2]));
        }
        let (Some(left_features), Some(right_features), Some(disparity)) =
            (&self.left_features, &self.right_features, &self.disparity)
        else {
            return Err("cu_anynet: refine() called without a base job".into());
        };
        let scale = self.stage as usize;
        let refined = self
            .model
            .refine(
                scale,
                left_features,
                right_features,
                disparity,
                self.image_height,
                self.image_width,
            )
            .map_err(model_error)?;
        self.stage += 1;
        let quality = self.qualities[self.stage as usize - 1];
        let payload = output
            .payload_mut()
            .as_mut()
            .ok_or("cu_anynet: base output disappeared before refinement")?;
        write_depth(
            &refined,
            &mut payload.depth,
            self.focal_pixels,
            self.baseline_meters,
        )?;
        payload.stage = self.stage;
        payload.quality = quality;
        if self.stage_snapshots {
            let snapshot = StageSnapshot {
                depth: detached_copy(&payload.depth),
                stage: self.stage,
                quality,
                elapsed: ctx.now() - self.job_start,
            };
            payload.snapshots[self.stage as usize - 1] = Some(snapshot);
        }
        self.disparity = Some(refined);

        if self.stage == 3 {
            Ok(AnytimeStatus::Converged(quality))
        } else {
            Ok(AnytimeStatus::Improved(quality))
        }
    }
}

impl AnyNetStereo {
    fn clear_job(&mut self) {
        self.left_features = None;
        self.right_features = None;
        self.disparity = None;
        self.image_height = 0;
        self.image_width = 0;
        self.stage = 0;
    }
}

fn select_device(device: AnyNetDevice) -> CuResult<Device> {
    match device {
        AnyNetDevice::Cpu => Ok(Device::Cpu),
        AnyNetDevice::Cuda => {
            #[cfg(feature = "cuda")]
            {
                Device::new_cuda(0).map_err(|error| {
                    CuError::new_with_cause("cu_anynet: failed to initialize CUDA", error)
                })
            }
            #[cfg(not(feature = "cuda"))]
            {
                Err(CuError::from(
                    "cu_anynet: device cuda requires the cu-anynet/cuda feature",
                ))
            }
        }
    }
}

fn validate_qualities(qualities: [f32; 3]) -> CuResult<()> {
    if qualities
        .iter()
        .any(|quality| !quality.is_finite() || !(0.0..=1.0).contains(quality))
        || qualities[0] > qualities[1]
        || qualities[1] > qualities[2]
    {
        return Err(CuError::from(
            "cu_anynet: stage qualities must be finite, monotone, and in 0..=1",
        ));
    }
    Ok(())
}

fn validate_stereo_pair(pair: &StereoPair) -> CuResult<()> {
    let left = pair.left.format;
    let right = pair.right.format;
    if left.width != right.width || left.height != right.height {
        return Err(CuError::from(
            "cu_anynet: left and right image dimensions differ",
        ));
    }
    if left.pixel_format != *b"RGB3" || right.pixel_format != *b"RGB3" {
        return Err(CuError::from("cu_anynet: input images must use RGB3"));
    }
    if left.width == 0
        || left.height == 0
        || !left.width.is_multiple_of(16)
        || !left.height.is_multiple_of(16)
    {
        return Err(CuError::from(
            "cu_anynet: input dimensions must be non-zero multiples of 16",
        ));
    }
    Ok(())
}

fn image_to_tensor(
    image: &CuImage<Vec<u8>>,
    device: &Device,
    normalization: AnyNetNormalization,
) -> CuResult<Tensor> {
    let width = image.format.width as usize;
    let height = image.format.height as usize;
    let values = image.with_plane_bytes(0, |bytes, layout| {
        let mut values = Vec::with_capacity(width * height * 3);
        for row in 0..height {
            let start = row * layout.stride_bytes as usize;
            let end = start + width * 3;
            values.extend(bytes[start..end].iter().map(|value| *value as f32));
        }
        values
    })?;
    let image = Tensor::from_vec(values, (1, height, width, 3), device)
        .and_then(|image| image.permute((0, 3, 1, 2)))
        .map_err(model_error)?;
    if normalization == AnyNetNormalization::Raw {
        return Ok(image);
    }
    let image = (&image / 255.0).map_err(model_error)?;
    let mean = Tensor::new(&[0.485f32, 0.456, 0.406], device)
        .and_then(|mean| mean.reshape((1, 3, 1, 1)))
        .map_err(model_error)?;
    let std = Tensor::new(&[0.229f32, 0.224, 0.225], device)
        .and_then(|std| std.reshape((1, 3, 1, 1)))
        .map_err(model_error)?;
    image
        .broadcast_sub(&mean)
        .and_then(|image| image.broadcast_div(&std))
        .map_err(model_error)
}

fn depth_from_disparity(
    disparity: &Tensor,
    width: usize,
    height: usize,
    focal_pixels: f32,
    baseline_meters: f32,
) -> CuResult<CuDepthMap<Vec<Length>, CuDepthLength>> {
    let format = CuDepthMapFormat {
        width: width as u32,
        height: height as u32,
        stride: width as u32,
    };
    let mut depth = CuDepthMap::new(
        format,
        CuHandle::new_detached(vec![Length::new::<meter>(f32::NAN); width * height]),
    );
    write_depth(disparity, &mut depth, focal_pixels, baseline_meters)?;
    Ok(depth)
}

/// Copies a depth map into freshly detached storage.
///
/// `CuDepthMap::clone` shares the handle, so a cloned snapshot would alias the
/// live buffer that the next refinement quantum overwrites.
fn detached_copy(
    depth: &CuDepthMap<Vec<Length>, CuDepthLength>,
) -> CuDepthMap<Vec<Length>, CuDepthLength> {
    let mut samples = Vec::with_capacity(depth.format.required_elements());
    depth.with_samples(|source, _format| samples.extend_from_slice(source));
    CuDepthMap::new(depth.format, CuHandle::new_detached(samples))
}

fn write_depth(
    disparity: &Tensor,
    depth: &mut CuDepthMap<Vec<Length>, CuDepthLength>,
    focal_pixels: f32,
    baseline_meters: f32,
) -> CuResult<()> {
    let values = disparity
        .flatten_all()
        .and_then(|values| values.to_vec1::<f32>())
        .map_err(model_error)?;
    if values.len() != depth.format.required_elements() {
        return Err(CuError::from(
            "cu_anynet: disparity dimensions do not match the depth output",
        ));
    }
    let scale = focal_pixels * baseline_meters;
    depth.with_samples_mut(|samples, _format| {
        for (sample, disparity) in samples.iter_mut().zip(values) {
            let meters = if disparity > 0.0 {
                scale / disparity
            } else {
                f32::NAN
            };
            *sample = Length::new::<meter>(meters);
        }
    });
    Ok(())
}

fn model_error(error: candle_core::Error) -> CuError {
    CuError::new_with_cause("cu_anynet: inference failed", error)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu_sensor_payloads::CuImageBufferFormat;

    /// Smoke-mode task with the required calibration keys plus `extra` config
    /// entries (`, "key": Value(...)` shaped).
    fn smoke_task(extra: &str) -> AnyNetStereo {
        let config: ComponentConfig = ron::from_str(&format!(
            r#"({{"focal_px": Value(721.5), "baseline_m": Value(0.54){extra}}})"#
        ))
        .unwrap();
        AnyNetStereo::new(Some(&config), ()).unwrap()
    }

    fn rgb_image(width: u32, height: u32, value: u8) -> CuImage<Vec<u8>> {
        CuImage::new(
            CuImageBufferFormat {
                width,
                height,
                stride: width * 3,
                pixel_format: *b"RGB3",
            },
            CuHandle::new_detached(vec![value; width as usize * height as usize * 3]),
        )
    }

    #[test]
    fn normalization_modes_scale_pixels_differently() {
        let image = rgb_image(16, 16, 51);
        let raw = image_to_tensor(&image, &Device::Cpu, AnyNetNormalization::Raw)
            .unwrap()
            .flatten_all()
            .unwrap()
            .to_vec1::<f32>()
            .unwrap();
        assert_eq!(raw[0], 51.0);
        let normalized = image_to_tensor(&image, &Device::Cpu, AnyNetNormalization::Imagenet)
            .unwrap()
            .flatten_all()
            .unwrap()
            .to_vec1::<f32>()
            .unwrap();
        let expected = (51.0 / 255.0 - 0.485) / 0.229;
        assert!((normalized[0] - expected).abs() < 1e-6);
    }

    #[test]
    fn disparity_conversion_marks_non_positive_samples_invalid() {
        let disparity = Tensor::new(&[[[[2f32, 0.0, -1.0, 4.0]]]], &Device::Cpu).unwrap();
        let depth = depth_from_disparity(&disparity, 4, 1, 10.0, 0.5).unwrap();
        assert_eq!(depth.get_meters(0, 0), Some(2.5));
        assert!(depth.get_meters(1, 0).unwrap().is_nan());
        assert!(depth.get_meters(2, 0).unwrap().is_nan());
        assert_eq!(depth.get_meters(3, 0), Some(1.25));
    }

    #[test]
    fn payloads_round_trip_through_bincode() {
        let pair = StereoPair {
            left: rgb_image(16, 16, 10),
            right: rgb_image(16, 16, 20),
        };
        let config = bincode::config::standard();
        let encoded = bincode::encode_to_vec(&pair, config).unwrap();
        let (decoded, consumed): (StereoPair, usize) =
            bincode::decode_from_slice(&encoded, config).unwrap();
        assert_eq!(consumed, encoded.len());
        assert_eq!(decoded.left.format.width, 16);
        decoded
            .right
            .with_plane_bytes(0, |bytes, _layout| assert_eq!(bytes[0], 20))
            .unwrap();

        let disparity = Tensor::ones((1, 1, 16, 16), DType::F32, &Device::Cpu).unwrap();
        let depth = StereoDepth {
            depth: depth_from_disparity(&disparity, 16, 16, 10.0, 0.5).unwrap(),
            stage: 2,
            quality: quality_from_f32(0.7),
            snapshots: Default::default(),
        };
        let encoded = bincode::encode_to_vec(&depth, config).unwrap();
        let (decoded, consumed): (StereoDepth, usize) =
            bincode::decode_from_slice(&encoded, config).unwrap();
        assert_eq!(consumed, encoded.len());
        assert_eq!(decoded.stage, 2);
        assert_eq!(decoded.depth.get_meters(0, 0), Some(5.0));
    }

    #[test]
    fn calibration_keys_are_required() {
        assert!(AnyNetStereo::new(None, ()).is_err());
    }

    #[test]
    fn anytime_task_runs_all_three_smoke_mode_stages() {
        let ctx = CuContext::new_with_clock();
        let mut task = smoke_task("");
        let mut input = CuMsg::new(Some(StereoPair {
            left: rgb_image(128, 64, 10),
            right: rgb_image(128, 64, 20),
        }));
        input.tov = Tov::Time(ctx.now());
        let mut output = CuMsg::new(None);

        assert!(matches!(
            task.base(&ctx, &input, &mut output).unwrap(),
            AnytimeStatus::Improved(_)
        ));
        assert_eq!(output.payload().unwrap().stage, 1);
        assert_eq!(output.tov, input.tov);
        let first_storage = output.payload().unwrap().depth.buffer_handle.storage_id();
        assert!(matches!(
            task.refine(&ctx, &mut output).unwrap(),
            AnytimeStatus::Improved(_)
        ));
        assert_eq!(output.payload().unwrap().stage, 2);
        assert!(matches!(
            task.refine(&ctx, &mut output).unwrap(),
            AnytimeStatus::Converged(_)
        ));
        assert_eq!(output.payload().unwrap().stage, 3);

        task.base(&ctx, &input, &mut output).unwrap();
        assert_ne!(
            output.payload().unwrap().depth.buffer_handle.storage_id(),
            first_storage,
            "each base job must own fresh detached depth storage"
        );
    }

    #[test]
    fn stage_snapshots_hold_each_stage_independently() {
        let ctx = CuContext::new_with_clock();
        let mut task = smoke_task(r#", "stage_snapshots": Value(true)"#);
        let mut input = CuMsg::new(Some(StereoPair {
            left: rgb_image(128, 64, 10),
            right: rgb_image(128, 64, 20),
        }));
        input.tov = Tov::Time(ctx.now());
        let mut output = CuMsg::new(None);

        task.base(&ctx, &input, &mut output).unwrap();
        task.refine(&ctx, &mut output).unwrap();
        task.refine(&ctx, &mut output).unwrap();

        let payload = output.payload().unwrap();
        let stages: Vec<u8> = payload
            .snapshots
            .iter()
            .flatten()
            .map(|snapshot| snapshot.stage)
            .collect();
        assert_eq!(stages, vec![1, 2, 3]);
        // Refinement rewrites the published depth in place, so a snapshot that
        // shared its handle would report the final stage's values.
        for snapshot in payload.snapshots.iter().flatten() {
            assert_ne!(
                snapshot.depth.buffer_handle.storage_id(),
                payload.depth.buffer_handle.storage_id(),
                "stage {} snapshot aliases the live depth buffer",
                snapshot.stage
            );
        }

        // A second job must not carry the previous job's later stages.
        task.base(&ctx, &input, &mut output).unwrap();
        let snapshots = &output.payload().unwrap().snapshots;
        assert!(snapshots[0].is_some());
        assert!(snapshots[1].is_none() && snapshots[2].is_none());
    }

    #[test]
    fn stage_snapshots_stay_empty_by_default() {
        let ctx = CuContext::new_with_clock();
        let mut task = smoke_task("");
        let mut input = CuMsg::new(Some(StereoPair {
            left: rgb_image(128, 64, 10),
            right: rgb_image(128, 64, 20),
        }));
        input.tov = Tov::Time(ctx.now());
        let mut output = CuMsg::new(None);

        task.base(&ctx, &input, &mut output).unwrap();
        task.refine(&ctx, &mut output).unwrap();
        assert!(
            output
                .payload()
                .unwrap()
                .snapshots
                .iter()
                .all(Option::is_none)
        );
    }
}
