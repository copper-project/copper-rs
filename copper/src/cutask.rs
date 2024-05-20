use serde::{Deserialize, Serialize};

use crate::config::NodeInstanceConfig;
use crate::CuResult;

// Everything that is stateful in copper for zero copy constraints need to be restricted to this trait.
pub trait CuMsg: Default + Serialize + for<'a> Deserialize<'a> + Sized {}

// Also anything that follows this contract can be a message
impl<T> CuMsg for T where T: Default + Serialize + for<'a> Deserialize<'a> + Sized {}

// Because of the Rust orphan rule, we need to define the common methods in a macro.
// This can be cleaned up with a proc macro or with a negative impl
// https://doc.rust-lang.org/beta/unstable-book/language-features/negative-impls.html when
// they are stabilized.
macro_rules! cu_task_common {
    () => {
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
    };
}

pub trait CuSrcTask {
    type Msg: CuMsg;
    cu_task_common!();
    fn process(&mut self, empty_msg: &mut Self::Msg) -> CuResult<()>;
}

pub trait CuTask {
    type Input: CuMsg;
    type Output: CuMsg;
    cu_task_common!();
    fn process(&mut self, input: &Self::Input, output: &Self::Output) -> CuResult<()>;
}

pub trait CuSinkTask {
    type Input: CuMsg;
    cu_task_common!();
    fn process(&mut self, input: &Self::Input) -> CuResult<()>;
}
