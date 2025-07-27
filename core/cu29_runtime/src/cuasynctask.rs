use crate::config::ComponentConfig;
use crate::cutask::{CuMsg, CuMsgPayload, CuTask, Freezable};
use cu29_clock::RobotClock;
use cu29_traits::CuResult;
use rayon::ThreadPool;
use std::sync::{Arc, Mutex, MutexGuard};

pub struct CuAsyncTask<T, O>
where
    T: for<'m> CuTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    task: Arc<Mutex<T>>,
    output: Arc<Mutex<CuMsg<O>>>,
    processing: Arc<Mutex<bool>>, // TODO: an atomic should be enough.
    tp: Arc<ThreadPool>,
}

impl<T, O> CuAsyncTask<T, O>
where
    T: for<'m> CuTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    #[allow(unused)]
    pub fn new(config: Option<&ComponentConfig>, tp: Arc<ThreadPool>) -> CuResult<Self> {
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

impl<T, O> Freezable for CuAsyncTask<T, O>
where
    T: for<'m> CuTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
}

impl<T, I, O> CuTask for CuAsyncTask<T, O>
where
    T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
    I: CuMsgPayload + Send + Sync + 'static,
    O: CuMsgPayload + Send + 'static,
{
    type Input<'m> = T::Input<'m>;
    type Output<'m> = T::Output<'m>;

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Err("AsyncTask cannot be instantiated directly, use Async_task::new()".into())
    }

    fn process<'i, 'o>(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'i>,
        real_output: &mut Self::Output<'o>,
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
        type Output<'m> = output_msg!(u32);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(
            &mut self,
            _clock: &RobotClock,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            output.borrow_mut().set_payload(*input.payload().unwrap());
            Ok(())
        }
    }

    #[test]
    fn test_lifecycle() {
        let tp = Arc::new(
            rayon::ThreadPoolBuilder::new()
                .num_threads(1)
                .build()
                .unwrap(),
        );

        let config = ComponentConfig::default();
        let clock = RobotClock::default();
        let mut async_task: CuAsyncTask<TestTask, u32> =
            CuAsyncTask::new(Some(&config), tp).unwrap();
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
