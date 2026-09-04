#![cfg(feature = "logstream")]

use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Encode, Decode, Serialize, Deserialize, Reflect)]
struct StreamMsg(u64);

#[derive(Default, Reflect)]
#[allow(dead_code)]
struct StreamSource;

impl Freezable for StreamSource {}

impl CuSrcTask for StreamSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(StreamMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(StreamMsg(1));
        Ok(())
    }
}

#[copper_runtime(config = "tests/logstream_runtime_config.ron", sim_mode = true)]
struct LogstreamSimApp {}

#[test]
fn generated_sim_logstream_builder_compiles() {
    let _ = core::mem::size_of::<LogstreamSimApp>();
}
