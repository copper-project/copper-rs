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

/// Adapts MCU context to ViTFly's pose and desired-speed inputs.
#[cfg(any(feature = "sim", feature = "end2end"))]
#[derive(Reflect, Default)]
pub struct VitFlyContextAdapter {
    last_context: Option<AutonomyContext>,
}

#[cfg(any(feature = "sim", feature = "end2end"))]
impl Freezable for VitFlyContextAdapter {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        Encode::encode(&self.last_context, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.last_context = Decode::decode(decoder)?;
        Ok(())
    }
}

#[cfg(any(feature = "sim", feature = "end2end"))]
impl CuTask for VitFlyContextAdapter {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(AutonomyContext);
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
        if let Some(context) = input.payload() {
            self.last_context = Some(*context);
        }

        let Some(context) = self.last_context else {
            output.0.clear_payload();
            output.1.clear_payload();
            return Ok(());
        };

        let mission_speed_mps = context.desired_speed.get::<meter_per_second>();

        // The MCU context is periodic, but ViTFly is a continuous compute task.
        // Keep inference running at the untouched mission speed; the active
        // flag gates actuation downstream.
        let now = Tov::Time(ctx.now());
        output.0.tov = now;
        output.1.tov = now;
        output.0.set_payload(context.pose);
        output
            .1
            .set_payload(Velocity::new::<meter_per_second>(mission_speed_mps));
        Ok(())
    }
}

/// Packages ViTFly output for the typed compute -> MCU bridge channel.
#[cfg(any(feature = "sim", feature = "end2end"))]
#[derive(Reflect, Default)]
pub struct VitFlyCommandAdapter {
    last_context: Option<AutonomyContext>,
}

#[cfg(any(feature = "sim", feature = "end2end"))]
impl Freezable for VitFlyCommandAdapter {
    fn freeze<E: cu29::bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), cu29::bincode::error::EncodeError> {
        Encode::encode(&self.last_context, encoder)
    }

    fn thaw<D: cu29::bincode::de::Decoder>(
        &mut self,
        decoder: &mut D,
    ) -> Result<(), cu29::bincode::error::DecodeError> {
        self.last_context = Decode::decode(decoder)?;
        Ok(())
    }
}

#[cfg(any(feature = "sim", feature = "end2end"))]
impl CuTask for VitFlyCommandAdapter {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, AutonomyContext, cu_vitfly::VitFlyVelocity);
    type Output<'m> = CuMsg<AutonomyVelocityCommand>;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.tov = input.1.tov;
        if let Some(context) = input.0.payload() {
            self.last_context = Some(*context);
        }

        let (Some(context), Some(velocity)) = (self.last_context, input.1.payload()) else {
            output.clear_payload();
            return Ok(());
        };
        if !context.active {
            output.clear_payload();
            return Ok(());
        }
        let Some([north, west, up]) = conditioned_world_velocity(
            velocity.map(|axis| axis.get::<meter_per_second>()),
            context.desired_speed.get::<meter_per_second>(),
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

/// Match ViTFly's upstream command conditioning while retaining a northward
/// component for mission progress. The learned output is a direction in its
/// fixed world `[north, west, up]` frame, not a body-frame vector or an
/// already-normalized velocity setpoint.
#[cfg(any(feature = "sim", feature = "end2end"))]
fn conditioned_world_velocity(raw: [f32; 3], desired_speed_mps: f32) -> Option<[f32; 3]> {
    if !desired_speed_mps.is_finite()
        || desired_speed_mps < 0.0
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

        // A stopped command must remain live so the MCU velocity controller
        // actively brakes instead of timing out into attitude-neutral hover.
        let stopped = conditioned_world_velocity([0.5, 0.5, 0.0], 0.0).unwrap();
        assert_eq!(stopped, [0.0; 3]);
    }
}
