use serde::{Deserialize, Serialize};

use crate::config::NodeConfig;

// Everything that is stateful in copper for zero copy constraints need to be restricted to this trait.
pub trait CuMsg: Default + Serialize + for<'a> Deserialize<'a> + Sized {}

// Also anything that follows this contract can be a message
impl<T> CuMsg for T where T: Default + Serialize + for<'a> Deserialize<'a> + Sized {}

pub trait CuMsgLifecycle: Sync + Send + Clone + 'static {
    type Msg: CuMsg;

    fn create(&self) -> &mut Self::Msg;
    fn send(&self, msg: &Self::Msg);
}

pub trait CuSrcTask<O: CuMsg, L: CuMsgLifecycle<Msg = O>> {
    fn new(config: NodeConfig, msgif: L) -> Self;
}

pub trait CuTask<I: CuMsg, O: CuMsg> {
    fn new(config: NodeConfig) -> Self;
    fn process(&self, input: &I, output: &O) -> Result<(), String>;
}

pub trait CuSinkTask<I: CuMsg> {
    fn new(&self, config: NodeConfig) -> Self;
    fn process(&self, input: &I) -> Result<(), String>;
}
