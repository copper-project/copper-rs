//! Tasks owned by the onboard-compute Copper subsystem.

#[cfg(any(feature = "sim", feature = "end2end"))]
use crate::messages::{AutonomyContext, AutonomyVelocityCommand};
#[cfg(any(feature = "sim", feature = "end2end"))]
use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
#[cfg(any(feature = "sim", feature = "end2end"))]
use cu29::units::si::{f32::Velocity, velocity::meter_per_second};

#[cfg(any(feature = "sim", feature = "end2end"))]
const MIN_FORWARD_SPEED_MPS: f32 = 1.0;
#[cfg(any(feature = "sim", feature = "end2end"))]
const OBSTACLE_SPEED_MIN_MPS: f32 = 2.0;
#[cfg(any(feature = "sim", feature = "end2end"))]
const OBSTACLE_SPEED_MAX_MPS: f32 = 7.0;
#[cfg(any(feature = "sim", feature = "end2end"))]
const OBSTACLE_STANDOFF_M: f32 = 2.5;
#[cfg(any(feature = "sim", feature = "end2end"))]
const COMFORTABLE_BRAKING_MPS2: f32 = 2.5;
#[cfg(any(feature = "sim", feature = "end2end"))]
const SPEED_UP_RATE_MPS2: f32 = 1.5;
#[cfg(any(feature = "sim", feature = "end2end"))]
const SLOW_DOWN_RATE_MPS2: f32 = 3.0;
#[cfg(any(feature = "sim", feature = "end2end"))]
const CLEARANCE_APPROACH_TIME_S: f32 = 0.15;
#[cfg(any(feature = "sim", feature = "end2end"))]
const CLEARANCE_RELEASE_TIME_S: f32 = 0.75;
#[cfg(any(feature = "sim", feature = "end2end"))]
const DEPTH_HISTOGRAM_MAX_M: f32 = 12.5;
#[cfg(any(feature = "sim", feature = "end2end"))]
const DEPTH_HISTOGRAM_BINS: usize = 64;

/// Gives simulated depth its own logged Copper slot without copying the raster.
///
/// The ZED source remains unlogged so its unused stereo and confidence outputs do
/// not inflate the compute log. Cloning `ZedDepthMap` only clones its `CuHandle`.
#[cfg(feature = "sim")]
#[derive(Reflect)]
pub struct VitFlyDepthLog;

#[cfg(feature = "sim")]
impl Freezable for VitFlyDepthLog {}

#[cfg(feature = "sim")]
impl CuTask for VitFlyDepthLog {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(cu_zed::ZedDepthMap<Vec<f32>>);
    type Output<'m> = output_msg!(cu_zed::ZedDepthMap<Vec<f32>>);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        if let Some(depth) = input.payload() {
            output.set_payload(depth.clone());
        } else {
            output.clear_payload();
        }
        Ok(())
    }
}

/// Adapts the MCU context and forward depth corridor to ViTFly's typed inputs.
#[cfg(any(feature = "sim", feature = "end2end"))]
#[derive(Reflect, Default)]
pub struct VitFlyContextAdapter {
    last_context: Option<AutonomyContext>,
    speed_limit_mps: Option<f32>,
    filtered_clearance_m: Option<f32>,
    last_speed_update: Option<CuTime>,
    last_depth_seq: Option<u64>,
    last_active: bool,
}

#[cfg(any(feature = "sim", feature = "end2end"))]
impl Freezable for VitFlyContextAdapter {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        Encode::encode(&self.last_context, encoder)?;
        Encode::encode(&self.speed_limit_mps, encoder)?;
        Encode::encode(&self.filtered_clearance_m, encoder)?;
        Encode::encode(&self.last_speed_update, encoder)?;
        Encode::encode(&self.last_depth_seq, encoder)?;
        Encode::encode(&self.last_active, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.last_context = Decode::decode(decoder)?;
        self.speed_limit_mps = Decode::decode(decoder)?;
        self.filtered_clearance_m = Decode::decode(decoder)?;
        self.last_speed_update = Decode::decode(decoder)?;
        self.last_depth_seq = Decode::decode(decoder)?;
        self.last_active = Decode::decode(decoder)?;
        Ok(())
    }
}

#[cfg(any(feature = "sim", feature = "end2end"))]
impl CuTask for VitFlyContextAdapter {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, AutonomyContext, cu_zed::ZedDepthMap<Vec<f32>>);
    type Output<'m> = output_msg!(cu_ahrs::AhrsPose, Velocity);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(context) = input.0.payload() {
            self.last_context = Some(*context);
        }

        let Some(context) = self.last_context else {
            output.0.clear_payload();
            output.1.clear_payload();
            return Ok(());
        };

        let mission_speed_mps = context.desired_speed.get::<meter_per_second>();
        let now = ctx.now();
        let just_activated = context.active && !self.last_active;
        self.last_active = context.active;
        if just_activated {
            // The preview inference keeps running while AUTO is inactive. Start
            // the actual flight at the low-speed end of the governor instead
            // of inheriting its already-settled open-space speed.
            self.speed_limit_mps = Some(OBSTACLE_SPEED_MIN_MPS.min(mission_speed_mps));
            self.last_speed_update = Some(now);
        }

        if let Some(depth) = input.1.payload()
            && self.last_depth_seq != Some(depth.seq)
        {
            let elapsed_s = self
                .last_speed_update
                .map(|last_update| (now - last_update).as_nanos() as f32 * 1e-9)
                .unwrap_or(0.0);
            let target_speed_mps = match closest_forward_obstacle_m(depth) {
                Some(clearance_m) => {
                    let filtered_clearance_m =
                        filter_clearance_m(self.filtered_clearance_m, clearance_m, elapsed_s);
                    self.filtered_clearance_m = Some(filtered_clearance_m);
                    obstacle_speed_mps(filtered_clearance_m)
                }
                None => OBSTACLE_SPEED_MIN_MPS,
            }
            .min(mission_speed_mps);
            self.speed_limit_mps = Some(match (self.speed_limit_mps, self.last_speed_update) {
                (Some(current_speed_mps), Some(_)) => slew_speed_mps(
                    current_speed_mps.min(mission_speed_mps),
                    target_speed_mps,
                    elapsed_s,
                ),
                _ => target_speed_mps,
            });
            self.last_speed_update = Some(now);
            self.last_depth_seq = Some(depth.seq);
        }
        let desired_speed = Velocity::new::<meter_per_second>(
            self.speed_limit_mps
                .unwrap_or(mission_speed_mps)
                .min(mission_speed_mps),
        );

        // The MCU context is periodic, but ViTFly is a continuous compute task.
        // Keep inference running between context packets and while AUTO is inactive;
        // VitFlyCommandAdapter remains the safety gate for commands sent to the MCU.
        let now = Tov::Time(ctx.now());
        output.0.tov = now;
        output.1.tov = now;
        output.0.set_payload(context.pose);
        output.1.set_payload(desired_speed);
        Ok(())
    }
}

/// Packages ViTFly output for the typed compute -> MCU bridge channel.
#[cfg(any(feature = "sim", feature = "end2end"))]
#[derive(Reflect)]
pub struct VitFlyCommandAdapter;

#[cfg(any(feature = "sim", feature = "end2end"))]
impl Freezable for VitFlyCommandAdapter {}

#[cfg(any(feature = "sim", feature = "end2end"))]
impl CuTask for VitFlyCommandAdapter {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, AutonomyContext, cu_vitfly::VitFlyVelocity, Velocity);
    type Output<'m> = CuMsg<AutonomyVelocityCommand>;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.tov = input.1.tov;
        let (Some(context), Some(velocity), Some(desired_speed)) =
            (input.0.payload(), input.1.payload(), input.2.payload())
        else {
            output.clear_payload();
            return Ok(());
        };
        if !context.active {
            output.clear_payload();
            return Ok(());
        }
        let Some([north, west, up]) = conditioned_world_velocity(
            velocity.map(|axis| axis.get::<meter_per_second>()),
            desired_speed.get::<meter_per_second>(),
        ) else {
            output.clear_payload();
            return Ok(());
        };
        output.set_payload(AutonomyVelocityCommand {
            context_sequence: context.sequence,
            mission_generation: context.mission_generation,
            north: cu29::units::si::f32::Velocity::new::<meter_per_second>(north),
            west: cu29::units::si::f32::Velocity::new::<meter_per_second>(west),
            up: cu29::units::si::f32::Velocity::new::<meter_per_second>(up),
        });
        Ok(())
    }
}

/// A robust closest-obstacle estimate for the camera's immediate travel corridor.
///
/// The fifth percentile reacts when an obstacle occupies a meaningful part of
/// the central 30%-wide, 36%-high corridor without letting one invalid/noisy
/// depth pixel slam the global speed limit. Keeping the side margins out lets
/// ViTFly fly through a clear gap between trees. The fixed histogram keeps this
/// allocation-free.
#[cfg(any(feature = "sim", feature = "end2end"))]
fn closest_forward_obstacle_m(depth: &cu_zed::ZedDepthMap<Vec<f32>>) -> Option<f32> {
    let format = depth.format;
    depth.buffer_handle.with_inner(|samples| {
        robust_corridor_clearance_m(
            samples,
            format.width as usize,
            format.height as usize,
            format.stride as usize,
        )
    })
}

#[cfg(any(feature = "sim", feature = "end2end"))]
fn robust_corridor_clearance_m(
    samples: &[f32],
    width: usize,
    height: usize,
    stride: usize,
) -> Option<f32> {
    if width == 0 || height == 0 || stride < width || samples.len() < stride * height {
        return None;
    }

    let mut histogram = [0_u32; DEPTH_HISTOGRAM_BINS];
    let mut valid_count = 0_u32;
    let x_start = width * 35 / 100;
    let x_end = width - x_start;
    let y_start = height * 32 / 100;
    let y_end = height - y_start;

    for y in y_start..y_end {
        for &depth_m in &samples[y * stride + x_start..y * stride + x_end] {
            if !depth_m.is_finite() || depth_m <= 0.0 {
                continue;
            }
            let bin = ((depth_m.min(DEPTH_HISTOGRAM_MAX_M) / DEPTH_HISTOGRAM_MAX_M)
                * DEPTH_HISTOGRAM_BINS as f32) as usize;
            histogram[bin.min(DEPTH_HISTOGRAM_BINS - 1)] += 1;
            valid_count += 1;
        }
    }

    if valid_count == 0 {
        return None;
    }
    let fifth_percentile_rank = valid_count.div_ceil(20);
    let mut cumulative = 0_u32;
    for (bin, count) in histogram.into_iter().enumerate() {
        cumulative += count;
        if cumulative >= fifth_percentile_rank {
            return Some((bin + 1) as f32 * DEPTH_HISTOGRAM_MAX_M / DEPTH_HISTOGRAM_BINS as f32);
        }
    }
    Some(DEPTH_HISTOGRAM_MAX_M)
}

#[cfg(any(feature = "sim", feature = "end2end"))]
fn obstacle_speed_mps(clearance_m: f32) -> f32 {
    // This is the inverse of stopping distance: the permitted speed is the
    // speed from which a comfortable constant deceleration reaches the minimum
    // speed at the standoff distance. At 7 m/s it begins slowing about 11.5 m
    // before a centered obstacle, rather than waiting for a fixed near cutoff.
    let braking_distance_m = (clearance_m - OBSTACLE_STANDOFF_M).max(0.0);
    libm::sqrtf(
        OBSTACLE_SPEED_MIN_MPS * OBSTACLE_SPEED_MIN_MPS
            + 2.0 * COMFORTABLE_BRAKING_MPS2 * braking_distance_m,
    )
    .min(OBSTACLE_SPEED_MAX_MPS)
}

#[cfg(any(feature = "sim", feature = "end2end"))]
fn filter_clearance_m(current_m: Option<f32>, measured_m: f32, elapsed_s: f32) -> f32 {
    let Some(current_m) = current_m else {
        return measured_m;
    };
    let elapsed_s = elapsed_s.clamp(0.0, 1.0);
    let time_constant_s = if measured_m < current_m {
        CLEARANCE_APPROACH_TIME_S
    } else {
        CLEARANCE_RELEASE_TIME_S
    };
    let blend = elapsed_s / (time_constant_s + elapsed_s);
    current_m + (measured_m - current_m) * blend
}

#[cfg(any(feature = "sim", feature = "end2end"))]
fn slew_speed_mps(current_mps: f32, target_mps: f32, elapsed_s: f32) -> f32 {
    let elapsed_s = elapsed_s.clamp(0.0, 1.0);
    let max_change_mps = if target_mps < current_mps {
        SLOW_DOWN_RATE_MPS2 * elapsed_s
    } else {
        SPEED_UP_RATE_MPS2 * elapsed_s
    };
    current_mps + (target_mps - current_mps).clamp(-max_change_mps, max_change_mps)
}

/// Match ViTFly's upstream command conditioning while retaining a northward
/// component for mission progress. The learned output is a direction in its
/// fixed world `[north, west, up]` frame, not a body-frame vector or an
/// already-normalized velocity setpoint.
#[cfg(any(feature = "sim", feature = "end2end"))]
fn conditioned_world_velocity(raw: [f32; 3], desired_speed_mps: f32) -> Option<[f32; 3]> {
    if !desired_speed_mps.is_finite()
        || desired_speed_mps <= 0.0
        || !raw.iter().all(|axis| axis.is_finite())
    {
        return None;
    }

    // Upstream clips the learned forward direction before normalizing the
    // complete vector and scaling it by the requested speed.
    let direction = [
        raw[0].clamp(-desired_speed_mps, desired_speed_mps),
        raw[1],
        raw[2],
    ];
    let norm = libm::sqrtf(direction.iter().map(|axis| axis * axis).sum());
    let mut command = if norm > f32::EPSILON {
        direction.map(|axis| axis * desired_speed_mps / norm)
    } else {
        [desired_speed_mps, 0.0, 0.0]
    };

    // The original simulator enforces 1 m/s during launch and the hardware
    // deployment adds a 1 m/s forward bias then clamps it non-negative.
    command[0] = command[0].max(desired_speed_mps.min(MIN_FORWARD_SPEED_MPS));
    Some(command)
}

/// Simulation-preempted source for ViTFly's non-camera inputs.
#[cfg(feature = "sim")]
#[allow(dead_code)]
#[derive(Reflect)]
pub struct VitFlyContextSource;

#[cfg(feature = "sim")]
impl Freezable for VitFlyContextSource {}

#[cfg(feature = "sim")]
impl CuSrcTask for VitFlyContextSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(cu_ahrs::AhrsPose, cu29::units::si::f32::Velocity);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.0.clear_payload();
        output.1.clear_payload();
        Ok(())
    }
}

/// Terminal for the ViTFly prediction in the simulator.
#[cfg(feature = "sim")]
#[allow(dead_code)]
#[derive(Reflect)]
pub struct VitFlySimListener;

#[cfg(feature = "sim")]
impl Freezable for VitFlySimListener {}

#[cfg(feature = "sim")]
impl CuSinkTask for VitFlySimListener {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(cu_vitfly::VitFlyVelocity);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

/// Compute boundary for the future VitFly image/depth pipeline.
///
/// The simulator observes this task's Copper inputs to render the depth preview;
/// the task itself intentionally performs no inference yet.
#[cfg(any(feature = "end2end", all(feature = "sim_core", not(feature = "sim"))))]
#[allow(dead_code)]
#[derive(Reflect)]
pub struct NoopVitFlyTask;

#[cfg(any(feature = "end2end", all(feature = "sim_core", not(feature = "sim"))))]
impl Freezable for NoopVitFlyTask {}

#[cfg(any(feature = "end2end", all(feature = "sim_core", not(feature = "sim"))))]
impl CuTask for NoopVitFlyTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(
        'm,
        cu_zed::ZedStereoImages,
        cu_zed::ZedDepthMap<Vec<f32>>
    );
    type Output<'m> = output_msg!(());

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.clear_payload();
        output.tov = input.1.tov;
        Ok(())
    }
}

#[cfg(all(test, feature = "sim"))]
mod tests {
    use super::*;

    #[test]
    fn vitfly_direction_is_normalized_with_positive_forward_progress() {
        let clear_path = conditioned_world_velocity([0.5, 0.0, 0.0], 4.0).unwrap();
        assert_eq!(clear_path, [4.0, 0.0, 0.0]);

        let avoidance = conditioned_world_velocity([-0.5, 0.5, 0.0], 4.0).unwrap();
        assert!(avoidance[0] >= MIN_FORWARD_SPEED_MPS);
        assert!(avoidance[1] > 0.0);

        let degenerate = conditioned_world_velocity([0.0; 3], 4.0).unwrap();
        assert_eq!(degenerate, [4.0, 0.0, 0.0]);
    }

    #[test]
    fn corridor_clearance_ignores_sides_and_detects_a_centered_obstacle() {
        let width = 40;
        let height = 40;
        let mut depth = vec![10.0; width * height];

        // Close trees outside the center corridor leave a flyable gap.
        for y in 12..28 {
            for x in 0..14 {
                depth[y * width + x] = 2.0;
                depth[y * width + width - x - 1] = 2.0;
            }
        }
        depth[12 * width + 14] = 0.2;
        assert!(robust_corridor_clearance_m(&depth, width, height, width).unwrap() > 9.0);

        // Ten pixels are just over five percent of the 12 x 16 center corridor.
        for x in 14..24 {
            depth[12 * width + x] = 2.0;
        }
        assert!(robust_corridor_clearance_m(&depth, width, height, width).unwrap() < 2.2);
    }

    #[test]
    fn obstacle_speed_is_fast_when_clear_and_slow_when_close() {
        assert_eq!(
            obstacle_speed_mps(OBSTACLE_STANDOFF_M),
            OBSTACLE_SPEED_MIN_MPS
        );
        assert_eq!(obstacle_speed_mps(12.0), OBSTACLE_SPEED_MAX_MPS);
        assert!(obstacle_speed_mps(6.0) > OBSTACLE_SPEED_MIN_MPS);
        assert!(obstacle_speed_mps(6.0) < OBSTACLE_SPEED_MAX_MPS);
    }

    #[test]
    fn speed_governor_slows_down_faster_than_it_speeds_up() {
        let slower = slew_speed_mps(7.0, 2.0, 0.1);
        let faster = slew_speed_mps(2.0, 7.0, 0.1);
        assert_eq!(slower, 6.7);
        assert_eq!(faster, 2.15);
    }

    #[test]
    fn clearance_filter_responds_faster_to_approach_than_release() {
        let approach = filter_clearance_m(Some(10.0), 4.0, 0.1);
        let release = filter_clearance_m(Some(4.0), 10.0, 0.1);
        assert!(10.0 - approach > release - 4.0);
    }
}
