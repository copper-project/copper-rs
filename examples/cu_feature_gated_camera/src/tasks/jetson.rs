use cu_gstreamer::CuGstBuffer;
use cu29::prelude::*;

#[derive(Reflect)]
pub struct JetsonFrameSink;

impl Freezable for JetsonFrameSink {}

impl CuSinkTask for JetsonFrameSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CuGstBuffer);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(frame) = input.payload() {
            let bytes = frame.map_readable()?;
            let _frame_size = bytes.len();
        }
        Ok(())
    }
}
