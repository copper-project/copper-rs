use crate::config::NodeConfig;
use serde::{Deserialize, Serialize};

trait CuStateful<'a>: Default + Serialize + Deserialize<'a> {}

trait CuTask<'a, I,O>: CuStateful<'a> {
    fn initialize(&self, config: NodeConfig);
    fn process(&self, input: &I, output: &O) -> Result<(), String>;
}
