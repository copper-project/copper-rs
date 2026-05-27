//! Confirms `logging: (handle_content: touched_only)` is accepted by the runtime
//! macro and codegen emits the per-slot policy block without errors. Behavioral
//! verification of the encode path lives in the cu_sensor_payloads end-to-end tests.

use cu29::prelude::*;
use cu29_derive::copper_runtime;

#[derive(Reflect)]
struct FrameSource;

impl Freezable for FrameSource {}

impl CuSrcTask for FrameSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(i32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(0);
        Ok(())
    }
}

#[derive(Reflect)]
struct FrameSink;

impl Freezable for FrameSink {}

impl CuSinkTask for FrameSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(i32);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, _input: &Self::Input<'_>) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(
    config = "config/handle_content_touched_only_valid.ron",
    sim_mode = true,
    ignore_resources = true
)]
struct App {}

fn main() {}
