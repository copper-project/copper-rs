use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct UdpMessage(pub u64);

#[derive(Default, Reflect)]
pub struct UdpSource {
    next: u64,
}

impl Freezable for UdpSource {}

impl CuSrcTask for UdpSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(UdpMessage);

    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(UdpMessage(self.next));
        self.next += 1;
        Ok(())
    }
}
