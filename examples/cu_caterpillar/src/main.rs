use copper::cutask::{CuMsg, CuTask, CuTaskLifecycle};
use copper::CuResult;
use copper_derive::copper_runtime;
use copper_log::debug;
use serde::{Deserialize, Serialize};

#[copper_runtime(config = "copperconfig.ron")]
struct TheVeryHungryCaterpillar {}

#[derive(Serialize, Deserialize, Default)]
pub struct CaterpillarMsg(bool);

pub struct CaterpillarTask {}

impl CuTaskLifecycle for CaterpillarTask {
    fn new(_config: Option<&copper::config::NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }
}

impl CuTask for CaterpillarTask {
    type Input = CaterpillarMsg;
    type Output = CaterpillarMsg;

    fn process(
        &mut self,
        input: &CuMsg<Self::Input>,
        output: &mut CuMsg<Self::Output>,
    ) -> CuResult<()> {
        // forward the state to the next task
        output.payload = CaterpillarMsg(input.payload.0);
        Ok(())
    }
}

fn main() {
    debug!("Application created.");
    let application = TheVeryHungryCaterpillar::new().expect("Failed to create runtime.");
    debug!("End of program.");
}
