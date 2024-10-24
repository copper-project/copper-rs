use crate::config::ComponentConfig;
use crate::cutask::{
    CuMsg, CuMsgPack, CuMsgPayload, CuSinkTask, CuSrcTask, CuTaskLifecycle, Freezable,
};
use crate::{input_msg, output_msg};
use cu29_clock::RobotClock;
use cu29_traits::CuResult;
use std::marker::PhantomData;

/// This is the state that will be passed to the simulation support to hook
/// into the lifecycle of the tasks.
pub enum CuTaskCallbackState<'cl, I, O>
where
    I: CuMsgPack<'cl>,
    O: CuMsgPack<'cl>,
{
    /// Callbacked when a task is created.
    /// It gives you the opportunity to adapt the sim to the given config.
    New(Option<ComponentConfig>),
    /// Callbacked when a task is started.
    Start,
    /// Callbacked when a task is getting called on pre-process.
    Preprocess,
    /// Callbacked when a task is getting called on process.
    /// I and O are the input and output messages of the task.
    /// if this is a source task, I will be CuMsg<()>
    /// if this is a sink task, O will be CuMsg<()>
    Process(I, O),
    /// Callbacked when a task is getting called on post-process.
    Postprocess,
    /// Callbacked when a task is stopped.
    Stop,
    // We don't hold the lifetime for the copper list backing the message so we need to
    // hold a phantom reference to it.
    _Phantom(PhantomData<&'cl ()>),
}

/// This is the answer the simulator can give to control the simulation flow.
#[derive(PartialEq)]
pub enum SimOverride {
    /// The callback took care of the logic on the simulation side and the actual
    /// implementation needs to be skipped.
    ExecutedBySim,
    /// The actual implementation needs to be executed.
    ExecuteByRuntime,
}

/// This is a placeholder task for a source task for the simulations.
/// It basically does nothing in place of a real driver so it won't try to initialize any hardware.
pub struct CuSimSrcTask<T> {
    pub config: Option<ComponentConfig>,
    boo: PhantomData<T>,
}

impl<T> CuTaskLifecycle for CuSimSrcTask<T> {
    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            config: config.and_then(|c: &ComponentConfig| Some(c.clone())),
            boo: PhantomData,
        })
    }
}

impl<T> Freezable for CuSimSrcTask<T> {}

impl<'cl, T: CuMsgPayload + 'cl> CuSrcTask<'cl> for CuSimSrcTask<T> {
    type Output = output_msg!('cl, T);

    fn process(&mut self, _clock: &RobotClock, _new_msg: Self::Output) -> CuResult<()> {
        unimplemented!("A placeholder for sim was called for a source, you need answer SimOverride to ExecutedBySim for the Process step.")
    }
}

/// This is a placeholder task for a sink task for the simulations.
/// It basically does nothing in place of a real driver so it won't try to initialize any hardware.
pub struct CuSimSinkTask<T> {
    pub config: Option<ComponentConfig>,
    boo: PhantomData<T>,
}

impl<T> CuTaskLifecycle for CuSimSinkTask<T> {
    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            config: config.and_then(|c: &ComponentConfig| Some(c.clone())),
            boo: PhantomData,
        })
    }
}
impl<T> Freezable for CuSimSinkTask<T> {}

impl<'cl, T: CuMsgPayload + 'cl> CuSinkTask<'cl> for CuSimSinkTask<T> {
    type Input = input_msg!('cl, T);

    fn process(&mut self, _clock: &RobotClock, _input: Self::Input) -> CuResult<()> {
        unimplemented!("A placeholder for sim was called for a sink, you need answer SimOverride to ExecutedBySim for the Process step.")
    }
}
