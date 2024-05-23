use serde::{Deserialize, Serialize};

use crate::config::NodeInstanceConfig;
use crate::CuResult;

// Everything that is stateful in copper for zero copy constraints need to be restricted to this trait.
pub trait CuMsgPayload: Default + Serialize + for<'a> Deserialize<'a> + Sized {}

// Also anything that follows this contract can be a payload (blanket implementation)
impl<T> CuMsgPayload for T where T: Default + Serialize + for<'a> Deserialize<'a> + Sized {}

pub struct CuMsg<T>
where
    T: CuMsgPayload,
{
    pub payload: T,
}

impl<T: CuMsgPayload> CuMsg<T> {
    pub fn new(payload: T) -> Self {
        CuMsg { payload }
    }
}

pub trait CuTaskLifecycle {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized;

    fn start(&mut self) -> CuResult<()> {
        Ok(())
    }

    fn preprocess(&mut self) -> CuResult<()> {
        Ok(())
    }

    fn postprocess(&mut self) -> CuResult<()> {
        Ok(())
    }

    fn stop(&mut self) -> CuResult<()> {
        Ok(())
    }
}

pub trait CuSrcTask: CuTaskLifecycle {
    type Payload: CuMsgPayload;
    fn process(&mut self, empty_msg: &mut CuMsg<Self::Payload>) -> CuResult<()>;
}

pub trait CuTask: CuTaskLifecycle {
    type Input: CuMsgPayload;
    type Output: CuMsgPayload;
    fn process(
        &mut self,
        input: &CuMsg<Self::Input>,
        output: &mut CuMsg<Self::Output>,
    ) -> CuResult<()>;
}

pub trait CuSinkTask: CuTaskLifecycle {
    type Input: CuMsgPayload;
    fn process(&mut self, input: &CuMsg<Self::Input>) -> CuResult<()>;
}
