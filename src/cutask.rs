use crate::config::NodeConfig;
use serde::{Deserialize, Serialize};

pub trait CuStateful<'a>: Default + Serialize + Deserialize<'a> {}

pub trait CuMsg<'a>: CuStateful<'a> {}

pub trait CuSrcTask<'a, O>: CuStateful<'a> {
    fn initialize(&self, config: NodeConfig, get_buffer: dyn Fn() -> &'a mut O, push_buffer: dyn Fn(&O));
}

pub trait CuTask<'a, I,O>: CuStateful<'a> {
    fn initialize(&self, config: NodeConfig);
    fn process(&self, input: &I, output: &O) -> Result<(), String>;
}

pub trait CuSinkTask<'a, I>: CuStateful<'a> {
    fn initialize(&self, config: NodeConfig);
    fn process(&self, input: &I) -> Result<(), String>;
}
