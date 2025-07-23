use crate::config::ComponentConfig;
use crate::cutask::{CuMsg, CuMsgPayload, CuTask, Freezable};
use cu29_clock::RobotClock;
use cu29_traits::CuResult;
use rayon::ThreadPool;
use std::sync::{Arc, Mutex, MutexGuard};

pub struct AsyncTask<T, O>
where
    T: CuTask<Output = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    task: Arc<Mutex<T>>,
    output: Arc<Mutex<CuMsg<O>>>,
    processing: Arc<Mutex<bool>>,
    tp: ThreadPool,
}

impl<T, O> AsyncTask<T, O>
where
    T: CuTask<Output = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    #[allow(unused)]
    pub fn new(config: Option<&ComponentConfig>, tp: ThreadPool) -> CuResult<Self> {
        let task = Arc::new(Mutex::new(T::new(config)?));
        let output = Arc::new(Mutex::new(CuMsg::default()));
        Ok(Self {
            task,
            output,
            processing: Arc::new(Mutex::new(false)),
            tp,
        })
    }
}

impl<T, O> Freezable for AsyncTask<T, O>
where
    T: CuTask<Output = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
}

impl<T, I, O> CuTask for AsyncTask<T, O>
where
    T: for<'m> CuTask<Input<'m> = CuMsg<I>, Output = CuMsg<O>> + Send + 'static,
    I: CuMsgPayload + Send + Sync + 'static,
    O: CuMsgPayload + Send + 'static,
{
    type Input<'m> = T::Input<'m>;
    type Output = T::Output;

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Err("AsyncTask cannot be instantiated directly, use from_task()".into())
    }

    fn process<'m>(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'m>,
        real_output: &mut Self::Output,
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
                let input_ref: &CuMsg<I> = unsafe { std::mem::transmute(input_ref) };
                let output_ref: &mut MutexGuard<CuMsg<O>> =
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

    impl CuTask for TestTask {
        type Input<'m> = input_msg!(u32);
        type Output = output_msg!(u32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process<'m>(
            &mut self,
            _clock: &RobotClock,
            input: &Self::Input<'m>,
            output: &mut Self::Output,
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

        loop {
            {
                let output_ref: &mut CuMsg<u32> = &mut output;
                async_task.process(&clock, &input, output_ref).unwrap();
            }

            if let Some(val) = output.payload() {
                assert_eq!(*val, 42u32);
                break;
            }
        }
    }
}
