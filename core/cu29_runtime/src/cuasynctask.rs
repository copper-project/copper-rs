use crate::config::ComponentConfig;
use crate::cutask::{CuMsg, CuMsgPayload, CuTask, Freezable};
use cu29_clock::{CuTime, RobotClock};
use cu29_traits::{CuError, CuResult};
use rayon::ThreadPool;
use std::sync::{Arc, Mutex};

struct AsyncState {
    processing: bool,
    ready_at: Option<CuTime>,
    last_error: Option<CuError>,
}

fn record_async_error(state: &Mutex<AsyncState>, error: CuError) {
    let mut guard = match state.lock() {
        Ok(guard) => guard,
        Err(poison) => poison.into_inner(),
    };
    guard.processing = false;
    guard.ready_at = None;
    guard.last_error = Some(error);
}

pub struct CuAsyncTask<T, O>
where
    T: for<'m> CuTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    task: Arc<Mutex<T>>,
    output: Arc<Mutex<CuMsg<O>>>,
    state: Arc<Mutex<AsyncState>>,
    tp: Arc<ThreadPool>,
}

/// Resource bundle required by a backgrounded task.
pub struct CuAsyncTaskResources<'r, T: CuTask> {
    pub inner: T::Resources<'r>,
    pub threadpool: Arc<ThreadPool>,
}

impl<T, O> CuAsyncTask<T, O>
where
    T: for<'m> CuTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    #[allow(unused)]
    pub fn new(
        config: Option<&ComponentConfig>,
        resources: T::Resources<'_>,
        tp: Arc<ThreadPool>,
    ) -> CuResult<Self> {
        let task = Arc::new(Mutex::new(T::new(config, resources)?));
        let output = Arc::new(Mutex::new(CuMsg::default()));
        Ok(Self {
            task,
            output,
            state: Arc::new(Mutex::new(AsyncState {
                processing: false,
                ready_at: None,
                last_error: None,
            })),
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
    type Resources<'r> = CuAsyncTaskResources<'r, T>;
    type Input<'m> = T::Input<'m>;
    type Output<'m> = T::Output<'m>;

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let task = Arc::new(Mutex::new(T::new(config, resources.inner)?));
        let output = Arc::new(Mutex::new(CuMsg::default()));
        Ok(Self {
            task,
            output,
            state: Arc::new(Mutex::new(AsyncState {
                processing: false,
                ready_at: None,
                last_error: None,
            })),
            tp: resources.threadpool,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        clock: &RobotClock,
        input: &Self::Input<'i>,
        real_output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        {
            let mut state = self.state.lock().map_err(|_| {
                CuError::from("Async task state mutex poisoned while scheduling background work")
            })?;
            if let Some(error) = state.last_error.take() {
                return Err(error);
            }
            if state.processing {
                // background task still running
                return Ok(());
            }

            if let Some(ready_at) = state.ready_at
                && clock.now() < ready_at
            {
                // result not yet allowed to surface based on recorded completion time
                return Ok(());
            }

            // mark as processing before spawning the next job
            state.processing = true;
            state.ready_at = None;
        }

        // clone the last finished output (if any) as the visible result for this polling round
        let buffered_output = self
            .output
            .lock()
            .map_err(|_| {
                let error = CuError::from("Async task output mutex poisoned");
                record_async_error(&self.state, error.clone());
                error
            })?;
        *real_output = buffered_output.clone();

        // immediately requeue a task based on the new input
        self.tp.spawn_fifo({
            let clock = clock.clone();
            let input = (*input).clone();
            let output = self.output.clone();
            let task = self.task.clone();
            let state = self.state.clone();
            move || {
                let input_ref: &CuMsg<I> = &input;
                let mut output_guard = match output.lock() {
                    Ok(guard) => guard,
                    Err(_) => {
                        record_async_error(
                            &state,
                            CuError::from("Async task output mutex poisoned"),
                        );
                        return;
                    }
                };
                let output_ref: &mut CuMsg<O> = &mut output_guard;

                // Track the actual processing interval so replay can honor it.
                if output_ref.metadata.process_time.start.is_none() {
                    output_ref.metadata.process_time.start = clock.now().into();
                }
                let task_result = match task.lock() {
                    Ok(mut task_guard) => task_guard.process(&clock, input_ref, output_ref),
                    Err(poison) => Err(CuError::from(format!(
                        "Async task mutex poisoned: {poison}"
                    ))),
                };

                let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
                guard.processing = false;

                match task_result {
                    Ok(()) => {
                        let end_from_metadata: Option<CuTime> =
                            output_ref.metadata.process_time.end.into();
                        let end_time = end_from_metadata.unwrap_or_else(|| {
                            let now = clock.now();
                            output_ref.metadata.process_time.end = now.into();
                            now
                        });
                        guard.ready_at = Some(end_time);
                    }
                    Err(error) => {
                        guard.ready_at = None;
                        guard.last_error = Some(error);
                    }
                }
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
    use rayon::ThreadPoolBuilder;
    use std::borrow::BorrowMut;
    use std::sync::OnceLock;
    use std::sync::mpsc;
    use std::time::Duration;

    static READY_RX: OnceLock<Arc<Mutex<mpsc::Receiver<CuTime>>>> = OnceLock::new();
    static DONE_TX: OnceLock<mpsc::Sender<()>> = OnceLock::new();
    struct TestTask {}

    impl Freezable for TestTask {}

    impl CuTask for TestTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
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
            CuAsyncTask::new(Some(&config), (), tp).unwrap();
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

    struct ControlledTask;

    impl Freezable for ControlledTask {}

    impl CuTask for ControlledTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(
            &mut self,
            clock: &RobotClock,
            _input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let rx = READY_RX
                .get()
                .expect("ready channel not set")
                .lock()
                .unwrap();
            let ready_time = rx
                .recv_timeout(Duration::from_secs(1))
                .expect("timed out waiting for ready signal");

            output.set_payload(ready_time.as_nanos() as u32);
            output.metadata.process_time.start = clock.now().into();
            output.metadata.process_time.end = ready_time.into();

            if let Some(done_tx) = DONE_TX.get() {
                let _ = done_tx.send(());
            }
            Ok(())
        }
    }

    #[test]
    fn background_respects_recorded_ready_time() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (clock, clock_mock) = RobotClock::mock();

        // Install the control channels for the task.
        let (ready_tx, ready_rx) = mpsc::channel::<CuTime>();
        let (done_tx, done_rx) = mpsc::channel::<()>();
        READY_RX
            .set(Arc::new(Mutex::new(ready_rx)))
            .expect("ready channel already set");
        DONE_TX
            .set(done_tx)
            .expect("completion channel already set");

        let mut async_task: CuAsyncTask<ControlledTask, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), (), tp.clone()).unwrap();
        let input = CuMsg::new(Some(1u32));
        let mut output = CuMsg::new(None);

        // Copperlist 0: kick off processing, nothing ready yet.
        clock_mock.set_value(0);
        async_task.process(&clock, &input, &mut output).unwrap();
        assert!(output.payload().is_none());

        // Copperlist 1 at time 10: still running in the background.
        clock_mock.set_value(10);
        async_task.process(&clock, &input, &mut output).unwrap();
        assert!(output.payload().is_none());

        // The background thread finishes at time 30 (recorded in metadata).
        clock_mock.set_value(30);
        ready_tx.send(CuTime::from(30u64)).unwrap();
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("background task never finished");
        // Wait until the async wrapper has cleared its processing flag and captured ready_at.
        let mut ready_at_recorded = None;
        for _ in 0..100 {
            let state = async_task.state.lock().unwrap();
            if !state.processing {
                ready_at_recorded = state.ready_at;
                if ready_at_recorded.is_some() {
                    break;
                }
            }
            drop(state);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(
            ready_at_recorded.is_some(),
            "background task finished without recording ready_at"
        );

        // Replay earlier than the recorded end time: the output should be held back.
        clock_mock.set_value(20);
        async_task.process(&clock, &input, &mut output).unwrap();
        assert!(
            output.payload().is_none(),
            "Output surfaced before recorded ready time"
        );

        // Once the mock clock reaches the recorded end time, the result is released.
        clock_mock.set_value(30);
        async_task.process(&clock, &input, &mut output).unwrap();
        assert_eq!(output.payload(), Some(&30u32));

        // Allow the background worker spawned by the last poll to complete so the thread pool shuts down cleanly.
        ready_tx.send(CuTime::from(40u64)).unwrap();
        let _ = done_rx.recv_timeout(Duration::from_secs(1));
    }
}
