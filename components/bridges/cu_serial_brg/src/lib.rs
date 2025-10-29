use cu29::prelude::*;
use std::marker::PhantomData;

pub struct CuSerialBrg<I, O>
where
    I: CuMsgPayload,
    O: CuMsgPayload,
{
    _boos: PhantomData<(I, O)>,
}

impl<I: CuMsgPayload, O: CuMsgPayload> Freezable for CuSerialBrg<I, O> {}

impl<I: CuMsgPayload, O: CuMsgPayload> CuBridge for CuSerialBrg<I, O> {
    type Input<'m> = I;
    type Output<'m> = O;

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        todo!()
    }

    fn send<'i>(&mut self, clock: &RobotClock, msg: &Self::Input<'i>) -> CuResult<()> {
        todo!()
    }

    fn receive<'o>(&mut self, clock: &RobotClock, msg: &mut Self::Output<'o>) -> CuResult<()> {
        todo!()
    }
}
