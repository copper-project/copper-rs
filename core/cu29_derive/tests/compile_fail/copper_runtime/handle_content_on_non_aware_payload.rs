//! Pins the silent-no-op gate: setting `handle_content: touched_only` on a source
//! whose payload type doesn't impl `HandleContentAware` must be a compile error,
//! not a runtime no-op. The companion compile_pass test exercises the
//! marker-present path; this one exercises the marker-absent path.

use cu29::prelude::*;

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
    config = "config/handle_content_on_non_aware_payload.ron",
    sim_mode = true,
    ignore_resources = true
)]
struct App {}

fn main() {}
