use crate::{DEPTH_SHAPE, INPUT_HEIGHT, INPUT_WIDTH, VitFly, VitFlyState};
use candle_core::{Device, Tensor};
use cu_ahrs::AhrsPose;
use cu_zed::ZedDepthMap;
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::angle::radian;
use cu29::units::si::f32::Velocity;
use cu29::units::si::length::meter;
use cu29::units::si::velocity::meter_per_second;

/// Open-loop world-frame velocity predicted by ViTFly, in `[forward, left, up]` order.
pub type VitFlyVelocity = [Velocity; 3];

const DEFAULT_MAX_DEPTH_M: f32 = 12.5;
const DEFAULT_INVALID_DEPTH: f32 = 0.8;
const RECURRENT_VALUES: usize = 3 * 128;

/// Copper task for the pretrained ViTFly depth policy.
///
/// The task consumes the standard ZED depth raster, Copper's AHRS pose, and a
/// unit-safe desired speed. It resizes and normalizes the depth map, maintains
/// the recurrent state, and emits a unit-safe XYZ velocity vector.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct VitFlyTask {
    #[reflect(ignore)]
    model: VitFly,
    #[reflect(ignore)]
    device: Device,
    #[reflect(ignore)]
    recurrent: Option<VitFlyState>,
    #[reflect(ignore)]
    resized_depth: Vec<f32>,
    max_depth_m: f32,
    invalid_depth: f32,
}

impl VitFlyTask {
    fn from_config(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let max_depth_m = config_f32(config, "max_depth_m", DEFAULT_MAX_DEPTH_M)?;
        if !max_depth_m.is_finite() || max_depth_m <= 0.0 {
            return Err(CuError::from(
                "vitfly max_depth_m must be finite and positive",
            ));
        }
        let invalid_depth = config_f32(config, "invalid_depth", DEFAULT_INVALID_DEPTH)?;
        if !invalid_depth.is_finite() {
            return Err(CuError::from("vitfly invalid_depth must be finite"));
        }

        let requested_device = config
            .map(|cfg| cfg.get::<String>("device"))
            .transpose()?
            .flatten();
        let cuda_ordinal = config
            .map(|cfg| cfg.get::<u32>("cuda_ordinal"))
            .transpose()?
            .flatten()
            .unwrap_or(0) as usize;
        let device = select_device(requested_device.as_deref(), cuda_ordinal)?;
        let model = VitFly::load(&device)
            .map_err(|err| CuError::new_with_cause("failed to load ViTFly model", err))?;

        Ok(Self {
            model,
            device,
            recurrent: None,
            resized_depth: vec![0.0; INPUT_HEIGHT * INPUT_WIDTH],
            max_depth_m,
            invalid_depth,
        })
    }

    fn run_model(
        &mut self,
        depth: &ZedDepthMap,
        pose: &AhrsPose,
        desired_speed: Velocity,
    ) -> CuResult<VitFlyVelocity> {
        let format = depth.format;
        if format.width == 0
            || format.height == 0
            || !format.is_valid()
            || format.required_elements() == 0
        {
            return Err(CuError::from(
                "vitfly received an invalid ZED raster format",
            ));
        }

        depth.with_samples(|samples, format| {
            resize_and_normalize_depth(
                samples,
                format.width as usize,
                format.height as usize,
                format.stride as usize,
                self.max_depth_m,
                self.invalid_depth,
                &mut self.resized_depth,
            );
        });

        let speed_mps = desired_speed.get::<meter_per_second>();
        if !speed_mps.is_finite() || speed_mps < 0.0 {
            return Err(CuError::from(
                "vitfly desired velocity must be finite and non-negative",
            ));
        }

        let quaternion = euler_to_scalar_first_quaternion(
            pose.roll.get::<radian>(),
            pose.pitch.get::<radian>(),
            pose.yaw.get::<radian>(),
        );
        let depth = Tensor::from_slice(&self.resized_depth, &DEPTH_SHAPE, &self.device)
            .map_err(|err| CuError::new_with_cause("failed to upload ViTFly depth", err))?;
        let desired_velocity = Tensor::new(&[[speed_mps]], &self.device)
            .map_err(|err| CuError::new_with_cause("failed to upload ViTFly speed", err))?;
        let attitude = Tensor::new(&[quaternion], &self.device)
            .map_err(|err| CuError::new_with_cause("failed to upload ViTFly attitude", err))?;
        let (prediction, recurrent) = self
            .model
            .forward(
                &depth,
                &desired_velocity,
                &attitude,
                self.recurrent.as_ref(),
            )
            .map_err(|err| CuError::new_with_cause("ViTFly inference failed", err))?;
        let prediction = prediction
            .to_vec2::<f32>()
            .map_err(|err| CuError::new_with_cause("failed to download ViTFly output", err))?;
        self.recurrent = Some(recurrent);

        let [forward, left, up]: [f32; 3] = prediction
            .first()
            .and_then(|row| row.as_slice().try_into().ok())
            .ok_or_else(|| CuError::from("ViTFly returned an invalid output shape"))?;
        Ok([forward, left, up]
            .map(|component| Velocity::new::<meter_per_second>(component * speed_mps)))
    }
}

impl Freezable for VitFlyTask {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        let snapshot = self
            .recurrent
            .as_ref()
            .map(|state| {
                let hidden = state.hidden_tensor()?.flatten_all()?.to_vec1::<f32>()?;
                let cell = state.cell_tensor()?.flatten_all()?.to_vec1::<f32>()?;
                candle_core::Result::Ok((hidden, cell))
            })
            .transpose()
            .map_err(|err| cu29::bincode::error::EncodeError::OtherString(err.to_string()))?;
        Encode::encode(&snapshot, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        let snapshot: Option<(Vec<f32>, Vec<f32>)> = Decode::decode(decoder)?;
        self.recurrent = snapshot
            .map(|(hidden, cell)| {
                if hidden.len() != RECURRENT_VALUES || cell.len() != RECURRENT_VALUES {
                    return Err(cu29::bincode::error::DecodeError::Other(
                        "invalid ViTFly recurrent state length",
                    ));
                }
                let hidden = Tensor::from_vec(hidden, (3, 128), &self.device)
                    .map_err(candle_decode_error)?;
                let cell =
                    Tensor::from_vec(cell, (3, 128), &self.device).map_err(candle_decode_error)?;
                VitFlyState::from_tensors(&hidden, &cell).map_err(candle_decode_error)
            })
            .transpose()?;
        Ok(())
    }
}

impl CuTask for VitFlyTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, ZedDepthMap, AhrsPose, Velocity);
    type Output<'m> = output_msg!(VitFlyVelocity);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Self::from_config(config)
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let (depth_msg, pose_msg, speed_msg) = *input;
        output.tov = depth_msg.tov;
        let (Some(depth), Some(pose), Some(speed)) = (
            depth_msg.payload(),
            pose_msg.payload(),
            speed_msg.payload().copied(),
        ) else {
            output.clear_payload();
            output.metadata.set_status("missing input");
            return Ok(());
        };

        output.set_payload(self.run_model(depth, pose, speed)?);
        output.metadata.set_status("ok");
        Ok(())
    }
}

fn select_device(requested: Option<&str>, cuda_ordinal: usize) -> CuResult<Device> {
    let requested = requested.unwrap_or(if cfg!(feature = "cuda") {
        "cuda"
    } else {
        "cpu"
    });
    match requested {
        "cpu" => Ok(Device::Cpu),
        "cuda" => {
            #[cfg(feature = "cuda")]
            {
                Device::new_cuda(cuda_ordinal)
                    .map_err(|err| CuError::new_with_cause("failed to initialize CUDA", err))
            }
            #[cfg(not(feature = "cuda"))]
            {
                let _ = cuda_ordinal;
                Err(CuError::from(
                    "vitfly device=cuda requires the cu-vitfly cuda feature",
                ))
            }
        }
        _ => Err(CuError::from("vitfly device must be either cpu or cuda")),
    }
}

fn config_f32(config: Option<&ComponentConfig>, key: &str, default: f32) -> CuResult<f32> {
    Ok(config
        .map(|cfg| cfg.get::<f32>(key))
        .transpose()?
        .flatten()
        .unwrap_or(default))
}

fn candle_decode_error(err: candle_core::Error) -> cu29::bincode::error::DecodeError {
    cu29::bincode::error::DecodeError::OtherString(err.to_string())
}

fn normalize_depth(sample: u16, max_depth_m: f32, invalid_depth: f32) -> f32 {
    ZedDepthMap::decode_sample(sample)
        .map(|depth| (depth.get::<meter>() / max_depth_m).clamp(0.0, 1.0))
        .unwrap_or(invalid_depth)
}

#[allow(clippy::too_many_arguments)]
fn resize_and_normalize_depth(
    source: &[u16],
    source_width: usize,
    source_height: usize,
    source_stride: usize,
    max_depth_m: f32,
    invalid_depth: f32,
    destination: &mut [f32],
) {
    debug_assert_eq!(destination.len(), INPUT_HEIGHT * INPUT_WIDTH);
    let scale_x = source_width as f32 / INPUT_WIDTH as f32;
    let scale_y = source_height as f32 / INPUT_HEIGHT as f32;

    for output_y in 0..INPUT_HEIGHT {
        let source_y =
            ((output_y as f32 + 0.5) * scale_y - 0.5).clamp(0.0, (source_height - 1) as f32);
        let y0 = source_y.floor() as usize;
        let y1 = (y0 + 1).min(source_height - 1);
        let wy = source_y - y0 as f32;
        for output_x in 0..INPUT_WIDTH {
            let source_x =
                ((output_x as f32 + 0.5) * scale_x - 0.5).clamp(0.0, (source_width - 1) as f32);
            let x0 = source_x.floor() as usize;
            let x1 = (x0 + 1).min(source_width - 1);
            let wx = source_x - x0 as f32;
            let top_left =
                normalize_depth(source[y0 * source_stride + x0], max_depth_m, invalid_depth);
            let top_right =
                normalize_depth(source[y0 * source_stride + x1], max_depth_m, invalid_depth);
            let bottom_left =
                normalize_depth(source[y1 * source_stride + x0], max_depth_m, invalid_depth);
            let bottom_right =
                normalize_depth(source[y1 * source_stride + x1], max_depth_m, invalid_depth);
            let top = top_left * (1.0 - wx) + top_right * wx;
            let bottom = bottom_left * (1.0 - wx) + bottom_right * wx;
            destination[output_y * INPUT_WIDTH + output_x] = top * (1.0 - wy) + bottom * wy;
        }
    }
}

fn euler_to_scalar_first_quaternion(roll: f32, pitch: f32, yaw: f32) -> [f32; 4] {
    let (sr, cr) = (0.5 * roll).sin_cos();
    let (sp, cp) = (0.5 * pitch).sin_cos();
    let (sy, cy) = (0.5 * yaw).sin_cos();
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_pose_maps_to_scalar_first_identity() {
        assert_eq!(
            euler_to_scalar_first_quaternion(0.0, 0.0, 0.0),
            [1.0, 0.0, 0.0, 0.0]
        );
    }

    #[test]
    fn resize_respects_stride_and_normalizes_meters() {
        let mut source = vec![u16::MAX; 6 * 2];
        source[..4].copy_from_slice(&[0, 6_250, 12_500, 25_000]);
        source[6..10].copy_from_slice(&[0, 6_250, 12_500, 25_000]);
        let mut destination = vec![0.0; INPUT_HEIGHT * INPUT_WIDTH];
        resize_and_normalize_depth(&source, 4, 2, 6, 12.5, 0.8, &mut destination);
        assert_eq!(destination[0], 0.8);
        assert!((destination[INPUT_WIDTH - 1] - 1.0).abs() < 1.0e-6);
    }
}
