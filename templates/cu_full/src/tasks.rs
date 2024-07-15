use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::clock::{OptionCuTime, RobotClock};
use cu29::cutask::{CuMsg, CuSrcTask, CuTask, CuTaskLifecycle, Freezable};
use cu29::CuResult;
use cu29::config::NodeInstanceConfig;


// Define a message type
#[derive(Debug, Clone, Encode, Decode)]
pub struct MyMsg {
    value: i32,
}

// Defines a source (ie. driver)
#[derive(Default)]
pub struct MySource {}

impl CuTaskLifecycle for MySource {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { })
    }
}

impl CuSrcTask for MySource {
    type Output = MyMsg;

    fn process(&mut self, clock: &RobotClock, output: &mut CuMsg<Self::Output>) -> CuResult<()> {
        // Generated a 42 message.
        output.payload = MyMsg {
            value: 42,
        };
        Ok(())
    }
}

// Defines a processing task
pub struct MyTask {
    // if you add some task state here, you need to implement the Freezable trait
}

impl CuTask for CaterpillarTask {
    type Input = MyMsg;
    type Output = ();

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &CuMsg<Self::Input>,
        output: &mut CuMsg<Self::Output>,
    ) -> CuResult<()> {
        debug!("Received message: {}", input.payload);
        Ok(())
    }
}

