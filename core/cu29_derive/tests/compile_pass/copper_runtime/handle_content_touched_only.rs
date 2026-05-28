//! Confirms `logging: (handle_content: touched_only)` is accepted by the runtime
//! macro when the source's payload type opts into the policy via
//! `HandleContentAware`. The codegen-emitted per-slot block contains a compile-time
//! `HandleContentAware` bound check, so this test failing to compile would mean the
//! gate falsely rejects a well-formed payload. Behavioral verification of the
//! encode path lives in the cu_sensor_payloads end-to-end tests; the matching
//! compile_fail test in this directory pins the other side (unmarked payload =>
//! clear compile error).

use bincode::{Decode, Encode};
use cu29::prelude::*;

// Minimal HandleContentAware payload. The marker is the *gate* — the type also
// needs inherent `apply_handle_content_policy` / `payload_should_log` methods (or
// to wrap a CuHandle) for the policy to fire at runtime; a real payload like
// CuImage provides them. For this compile-only test the marker alone is enough
// to exercise the codegen gate.
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
