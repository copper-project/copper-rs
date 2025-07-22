use crate::config::ComponentConfig;
use crate::cutask::{CuMsg, CuMsgPayload, CuTask, Freezable};
use cu29_clock::RobotClock;
use cu29_traits::CuResult;
use rayon::ThreadPool;
use std::marker::PhantomData;
use std::sync::{Arc, Mutex, MutexGuard};

pub struct AsyncTask<'cl, T, O>
where
    T: CuTask<'cl, Output = &'cl mut CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + 'cl + Send + 'static,
{
    task: Arc<Mutex<T>>,
    output: Arc<Mutex<CuMsg<O>>>,
    processing: Arc<Mutex<bool>>,
    _boo: PhantomData<&'cl ()>,
    tp: ThreadPool,
}

impl<'cl, T, O> AsyncTask<'cl, T, O>
where
    T: CuTask<'cl, Output = &'cl mut CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + 'cl + Send + 'static,
{
    pub fn new(config: Option<&ComponentConfig>, tp: ThreadPool) -> CuResult<Self> {
        let task = Arc::new(Mutex::new(T::new(config)?));
        let output = Arc::new(Mutex::new(CuMsg::default()));
        Ok(Self {
            task,
            output,
            processing: Arc::new(Mutex::new(false)),
            _boo: PhantomData,
            tp,
        })
    }
}

impl<'cl, T, O> Freezable for AsyncTask<'cl, T, O>
where
    T: CuTask<'cl, Output = &'cl mut CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + 'cl + Send + 'static,
{
}

impl<'cl, T, I, O> CuTask<'cl> for AsyncTask<'cl, T, O>
where
    T: CuTask<'cl, Input = &'cl CuMsg<I>, Output = &'cl mut CuMsg<O>> + Send + 'static,
    I: CuMsgPayload + 'cl + Send + Sync + 'static,
    O: CuMsgPayload + 'cl + Send + 'static,
{
    type Input = T::Input;
    type Output = T::Output;

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Err("AsyncTask cannot be instantiated directly, use from_task()".into())
    }

    fn process(
        &mut self,
        clock: &RobotClock,
        input: Self::Input,
        real_output: Self::Output,
    ) -> CuResult<()> {
        let mut processing = self.processing.lock().unwrap();
        if *processing {
            // if the background task is still processing, returns an empty result.
            return Ok(());
        }

        *processing = true; // Reset the done flag for the next processing
        let buffered_output = self.output.lock().unwrap(); // Clear the output if the task is done
        *real_output = buffered_output.clone();

        // immediately requeue a task based on the new input
        self.tp.spawn_fifo({
            let clock = clock.clone();
            let input = (*input).clone();
            let output = self.output.clone();
            let task = self.task.clone();
            let processing = self.processing.clone();
            move || {
                let input_ref: &CuMsg<I> = &input;
                let mut output: MutexGuard<CuMsg<O>> = output.lock().unwrap();

                // Safety: because copied the input and output, their lifetime are bound to the task and we control its lifetime.
                let input_ref: &'cl CuMsg<I> = unsafe { std::mem::transmute(input_ref) };
                let output_ref: &'cl mut MutexGuard<CuMsg<O>> =
                    unsafe { std::mem::transmute(&mut output) };
                task.lock()
                    .unwrap()
                    .process(&clock, input_ref, output_ref)
                    .unwrap();
                *processing.lock().unwrap() = false; // Mark processing as done
            }
        });
        Ok(())
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
        let tp = rayon::ThreadPoolBuilder::new()
            .num_threads(1)
            .build()
            .unwrap();

        let config = ComponentConfig::default();
        let clock = RobotClock::default();
        let mut async_task: AsyncTask<TestTask, u32> = AsyncTask::new(Some(&config), tp).unwrap();
        let input = CuMsg::new(Some(42u32));
        let mut output = CuMsg::new(None);

        async_task.process(&clock, &input, &mut output).unwrap();
        let result = *output.payload().unwrap();
        assert_eq!(result, 42u32);
    }
}
