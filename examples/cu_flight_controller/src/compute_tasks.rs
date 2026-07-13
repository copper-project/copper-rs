//! Tasks owned by the onboard-compute Copper subsystem.

use cu29::prelude::*;

/// Keeps the compute-only graph runnable before optional sensors are enabled.
#[derive(Reflect)]
pub struct ComputeHeartbeatSource;

impl Freezable for ComputeHeartbeatSource {}

impl CuSrcTask for ComputeHeartbeatSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u64);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(ctx.cl_id());
        output.tov = Tov::Time(ctx.now());
        Ok(())
    }
}

/// Terminal placeholder for the compute-local heartbeat.
#[derive(Reflect)]
pub struct ComputeHeartbeatSink;

impl Freezable for ComputeHeartbeatSink {}

impl CuSinkTask for ComputeHeartbeatSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u64);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
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
