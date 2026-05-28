//! Marker-present path of the HandleContentAware gate. Behavioral verification of
//! the encode path lives in the cu_sensor_payloads end-to-end tests.

use bincode::{Decode, Encode};
use cu29::prelude::*;

// Marker-only payload — enough to exercise the codegen gate. Real payloads (e.g.
// CuImage) also forward the inherent methods to an inner CuHandle.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
struct AwareFrame {
    seq: u32,
}

impl cu29::pool::HandleContentAware for AwareFrame {}

#[derive(Reflect)]
struct FrameSource;

impl Freezable for FrameSource {}

impl CuSrcTask for FrameSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(AwareFrame);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(AwareFrame::default());
        Ok(())
    }
}

#[derive(Reflect)]
struct FrameSink;

impl Freezable for FrameSink {}

impl CuSinkTask for FrameSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(AwareFrame);

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
