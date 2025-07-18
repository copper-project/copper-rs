use crate::config::ComponentConfig;
use crate::cutask::{CuTask, Freezable};
use cu29_clock::RobotClock;
use cu29_traits::CuResult;
use std::marker::PhantomData;

pub struct AsyncTask<'a, T>
where
    T: CuTask<'a>,
{
    task: T,
    _boo: PhantomData<&'a ()>,
}

impl<'cl, T> AsyncTask<'cl, T>
where
    T: CuTask<'cl>,
{
    pub fn from_task(task: T, config: Option<&ComponentConfig>) -> CuResult<Self> {
        let task = T::new(config)?;
        Ok(Self {
            task,
            _boo: PhantomData,
        })
    }
}

impl<'cl, T> Freezable for AsyncTask<'cl, T> where T: CuTask<'cl> {}

impl<'cl, T> CuTask<'cl> for AsyncTask<'cl, T>
where
    T: CuTask<'cl>,
{
    type Input = T::Input;
    type Output = T::Output;

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        todo!()
    }

    fn process(
        &mut self,
        clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        self.task.process(clock, input, output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::ComponentConfig;
    use crate::cutask::CuMsg;
    use crate::cutask::Freezable;
    use crate::input_msg;
    use crate::output_msg;
    use cu29_clock::RobotClock;
    use cu29_traits::CuResult;
    use std::borrow::BorrowMut;
    struct TestTask {}

    impl Freezable for TestTask {}

    impl<'cl> CuTask<'cl> for TestTask {
        type Input = input_msg!('cl, u32);
        type Output = output_msg!('cl, u32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(
            &mut self,
            _clock: &RobotClock,
            input: Self::Input,
            output: Self::Output,
        ) -> CuResult<()> {
            output.borrow_mut().set_payload(*input.payload().unwrap());
            Ok(())
        }
    }

    #[test]
    fn test_lifecycle() {
        let config = ComponentConfig::default();
        let clock = RobotClock::default();
        let task = TestTask::new(Some(&config)).unwrap();
        let mut async_task = AsyncTask::from_task(task, None).unwrap();
        let input = CuMsg::new(Some(42u32));
        let mut output = CuMsg::new(None);

        // Simulate processing
        async_task.process(&clock, &input, &mut output).unwrap();

        assert_eq!(output.payload(), Some(&42u32));
    }
}
