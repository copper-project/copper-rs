use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

use crate::messages::MyPayload;

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct MyInternalState {
    pub runs: u32,
}

#[derive(Default, Reflect)]
pub struct MySource;

impl Freezable for MySource {}

impl CuSrcTask for MySource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(MyPayload);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(MyPayload { value: 42 });
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct MyTask {
    pub state: MyInternalState,
}

impl Freezable for MyTask {}

impl CuTask for MyTask {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(MyPayload);
    type Output<'m> = output_msg!(MyPayload);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        self.state.runs = self.state.runs.saturating_add(1);
        let value = input.payload().map_or(43, |payload| payload.value + 1);
        output.set_payload(MyPayload { value });
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct MySink;

impl Freezable for MySink {}

impl CuSinkTask for MySink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(MyPayload);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        debug!("Sink received message: {}", input.payload().map_or(-1, |p| p.value));
        Ok(())
    }
}
