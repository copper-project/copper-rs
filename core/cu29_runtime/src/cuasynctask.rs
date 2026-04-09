use crate::config::ComponentConfig;
use crate::context::CuContext;
use crate::cutask::{CuMsg, CuMsgPayload, CuSrcTask, CuTask, Freezable};
use crate::reflect::{Reflect, TypePath};
use cu29_clock::CuTime;
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

fn begin_background_poll<O>(
    ctx: &CuContext,
    state: &Mutex<AsyncState>,
    buffered_output: &Mutex<CuMsg<O>>,
    real_output: &mut CuMsg<O>,
) -> CuResult<bool>
where
    O: CuMsgPayload + Send + 'static,
{
    {
        let mut state = state.lock().map_err(|_| {
            CuError::from("Async task state mutex poisoned while scheduling background work")
        })?;
        if let Some(error) = state.last_error.take() {
            return Err(error);
        }
        if state.processing {
            *real_output = CuMsg::default();
            return Ok(false);
        }

        if let Some(ready_at) = state.ready_at
            && ctx.now() < ready_at
        {
            *real_output = CuMsg::default();
            return Ok(false);
        }

        state.processing = true;
        state.ready_at = None;
    }

    let buffered_output = buffered_output.lock().map_err(|_| {
        let error = CuError::from("Async task output mutex poisoned");
        record_async_error(state, error.clone());
        error
    })?;
    *real_output = buffered_output.clone();
    Ok(true)
}

fn finalize_background_run<O>(
    state: &Mutex<AsyncState>,
    output_ref: &mut CuMsg<O>,
    fallback_end: CuTime,
    task_result: CuResult<()>,
) where
    O: CuMsgPayload + Send + 'static,
{
    let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
    guard.processing = false;

    match task_result {
        Ok(()) => {
            let end_from_metadata: Option<CuTime> = output_ref.metadata.process_time.end.into();
            let end_time = end_from_metadata.unwrap_or_else(|| {
                output_ref.metadata.process_time.end = fallback_end.into();
                fallback_end
            });
            guard.ready_at = Some(end_time);
        }
        Err(error) => {
            guard.ready_at = None;
            guard.last_error = Some(error);
        }
    }
}

#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuAsyncTask<T, O>
where
    T: for<'m> CuTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    #[reflect(ignore)]
    task: Arc<Mutex<T>>,
    #[reflect(ignore)]
    output: Arc<Mutex<CuMsg<O>>>,
    #[reflect(ignore)]
    state: Arc<Mutex<AsyncState>>,
    #[reflect(ignore)]
    tp: Arc<ThreadPool>,
}

impl<T, O> TypePath for CuAsyncTask<T, O>
where
    T: for<'m> CuTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    fn type_path() -> &'static str {
        "cu29_runtime::cuasynctask::CuAsyncTask"
    }

    fn short_type_path() -> &'static str {
        "CuAsyncTask"
    }

    fn type_ident() -> Option<&'static str> {
        Some("CuAsyncTask")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu29_runtime")
    }

    fn module_path() -> Option<&'static str> {
        Some("cuasynctask")
    }
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
        CuAsyncTask::new(config, resources.inner, resources.threadpool)
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        let mut task = self
            .task
            .lock()
            .map_err(|_| CuError::from("Async task mutex poisoned during start"))?;
        task.start(ctx)
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        real_output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        if !begin_background_poll(ctx, &self.state, &self.output, real_output)? {
            return Ok(());
        }

        // immediately requeue a task based on the new input
        self.tp.spawn_fifo({
            let ctx = ctx.clone();
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

                // Each async run starts from an empty output so a task that
                // chooses not to publish does not leak the previous payload.
                *output_ref = CuMsg::default();

                // Track the actual processing interval so replay can honor it.
                if output_ref.metadata.process_time.start.is_none() {
                    output_ref.metadata.process_time.start = ctx.now().into();
                }
                let task_result = match task.lock() {
                    Ok(mut task_guard) => task_guard.process(&ctx, input_ref, output_ref),
                    Err(poison) => Err(CuError::from(format!(
                        "Async task mutex poisoned: {poison}"
                    ))),
                };
                finalize_background_run(&state, output_ref, ctx.now(), task_result);
            }
        });
        Ok(())
    }

    fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
        let mut task = self
            .task
            .lock()
            .map_err(|_| CuError::from("Async task mutex poisoned during stop"))?;
        task.stop(ctx)
    }
}

#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuAsyncSrcTask<T, O>
where
    T: for<'m> CuSrcTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    #[reflect(ignore)]
    task: Arc<Mutex<T>>,
    #[reflect(ignore)]
    output: Arc<Mutex<CuMsg<O>>>,
    #[reflect(ignore)]
    state: Arc<Mutex<AsyncState>>,
    #[reflect(ignore)]
    tp: Arc<ThreadPool>,
}

impl<T, O> TypePath for CuAsyncSrcTask<T, O>
where
    T: for<'m> CuSrcTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    fn type_path() -> &'static str {
        "cu29_runtime::cuasynctask::CuAsyncSrcTask"
    }

    fn short_type_path() -> &'static str {
        "CuAsyncSrcTask"
    }

    fn type_ident() -> Option<&'static str> {
        Some("CuAsyncSrcTask")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu29_runtime")
    }

    fn module_path() -> Option<&'static str> {
        Some("cuasynctask")
    }
}

/// Resource bundle required by a backgrounded source.
pub struct CuAsyncSrcTaskResources<'r, T: CuSrcTask> {
    pub inner: T::Resources<'r>,
    pub threadpool: Arc<ThreadPool>,
}

impl<T, O> CuAsyncSrcTask<T, O>
where
    T: for<'m> CuSrcTask<Output<'m> = CuMsg<O>> + Send + 'static,
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

impl<T, O> Freezable for CuAsyncSrcTask<T, O>
where
    T: for<'m> CuSrcTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
}

impl<T, O> CuSrcTask for CuAsyncSrcTask<T, O>
where
    T: for<'m> CuSrcTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    type Resources<'r> = CuAsyncSrcTaskResources<'r, T>;
    type Output<'m> = T::Output<'m>;

    fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        CuAsyncSrcTask::new(config, resources.inner, resources.threadpool)
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        let mut task = self
            .task
            .lock()
            .map_err(|_| CuError::from("Async source mutex poisoned during start"))?;
        task.start(ctx)
    }

    fn process<'o>(&mut self, ctx: &CuContext, real_output: &mut Self::Output<'o>) -> CuResult<()> {
        if !begin_background_poll(ctx, &self.state, &self.output, real_output)? {
            return Ok(());
        }

        self.tp.spawn_fifo({
            let ctx = ctx.clone();
            let output = self.output.clone();
            let task = self.task.clone();
            let state = self.state.clone();
            move || {
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

                *output_ref = CuMsg::default();

                if output_ref.metadata.process_time.start.is_none() {
                    output_ref.metadata.process_time.start = ctx.now().into();
                }
                let task_result = match task.lock() {
                    Ok(mut task_guard) => task_guard.process(&ctx, output_ref),
                    Err(poison) => Err(CuError::from(format!(
                        "Async source mutex poisoned: {poison}"
                    ))),
                };
                finalize_background_run(&state, output_ref, ctx.now(), task_result);
            }
        });
        Ok(())
    }

    fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
        let mut task = self
            .task
            .lock()
            .map_err(|_| CuError::from("Async source mutex poisoned during stop"))?;
        task.stop(ctx)
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
    use cu29_traits::CuResult;
    use rayon::ThreadPoolBuilder;
    use std::borrow::BorrowMut;
    use std::sync::OnceLock;
    use std::sync::mpsc;
    use std::time::Duration;

    static READY_RX: OnceLock<Arc<Mutex<mpsc::Receiver<CuTime>>>> = OnceLock::new();
    static DONE_TX: OnceLock<mpsc::Sender<()>> = OnceLock::new();
    #[derive(Reflect)]
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
            _ctx: &CuContext,
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
        let context = CuContext::new_with_clock();
        let mut async_task: CuAsyncTask<TestTask, u32> =
            CuAsyncTask::new(Some(&config), (), tp).unwrap();
        let input = CuMsg::new(Some(42u32));
        let mut output = CuMsg::new(None);

        loop {
            {
                let output_ref: &mut CuMsg<u32> = &mut output;
                async_task.process(&context, &input, output_ref).unwrap();
            }

            if let Some(val) = output.payload() {
                assert_eq!(*val, 42u32);
                break;
            }
        }
    }

    #[derive(Reflect)]
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
            ctx: &CuContext,
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
            output.metadata.process_time.start = ctx.now().into();
            output.metadata.process_time.end = ready_time.into();

            if let Some(done_tx) = DONE_TX.get() {
                let _ = done_tx.send(());
            }
            Ok(())
        }
    }

    fn wait_until_async_idle<T, O>(async_task: &CuAsyncTask<T, O>)
    where
        T: for<'m> CuTask<Output<'m> = CuMsg<O>> + Send + 'static,
        O: CuMsgPayload + Send + 'static,
    {
        for _ in 0..100 {
            let state = async_task.state.lock().unwrap();
            if !state.processing {
                return;
            }
            drop(state);
            std::thread::sleep(Duration::from_millis(1));
        }
        panic!("background task never became idle");
    }

    fn wait_until_async_src_idle<T, O>(async_task: &CuAsyncSrcTask<T, O>)
    where
        T: for<'m> CuSrcTask<Output<'m> = CuMsg<O>> + Send + 'static,
        O: CuMsgPayload + Send + 'static,
    {
        for _ in 0..100 {
            let state = async_task.state.lock().unwrap();
            if !state.processing {
                return;
            }
            drop(state);
            std::thread::sleep(Duration::from_millis(1));
        }
        panic!("background source never became idle");
    }

    #[derive(Clone)]
    struct ActionTaskResources {
        actions: Arc<Mutex<mpsc::Receiver<Option<u32>>>>,
        done: mpsc::Sender<()>,
    }

    #[derive(Reflect)]
    #[reflect(no_field_bounds, from_reflect = false)]
    struct ActionTask {
        #[reflect(ignore)]
        actions: Arc<Mutex<mpsc::Receiver<Option<u32>>>>,
        #[reflect(ignore)]
        done: mpsc::Sender<()>,
    }

    impl Freezable for ActionTask {}

    impl CuTask for ActionTask {
        type Resources<'r> = ActionTaskResources;
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);

        fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let _ = config;
            Ok(Self {
                actions: resources.actions,
                done: resources.done,
            })
        }

        fn process(
            &mut self,
            _ctx: &CuContext,
            _input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let action = self
                .actions
                .lock()
                .unwrap()
                .recv_timeout(Duration::from_secs(1))
                .expect("timed out waiting for action");
            if let Some(value) = action {
                output.set_payload(value);
            }
            let _ = self.done.send(());
            Ok(())
        }
    }

    #[derive(Reflect)]
    #[reflect(no_field_bounds, from_reflect = false)]
    struct ActionSrc {
        #[reflect(ignore)]
        actions: Arc<Mutex<mpsc::Receiver<Option<u32>>>>,
        #[reflect(ignore)]
        done: mpsc::Sender<()>,
    }

    impl Freezable for ActionSrc {}

    impl CuSrcTask for ActionSrc {
        type Resources<'r> = ActionTaskResources;
        type Output<'m> = output_msg!(u32);

        fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let _ = config;
            Ok(Self {
                actions: resources.actions,
                done: resources.done,
            })
        }

        fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            let action = self
                .actions
                .lock()
                .unwrap()
                .recv_timeout(Duration::from_secs(1))
                .expect("timed out waiting for source action");
            if let Some(value) = action {
                output.set_payload(value);
            }
            let _ = self.done.send(());
            Ok(())
        }
    }

    #[derive(Clone)]
    struct ControlledSrcResources {
        ready_times: Arc<Mutex<mpsc::Receiver<CuTime>>>,
        done: mpsc::Sender<()>,
    }

    #[derive(Reflect)]
    #[reflect(no_field_bounds, from_reflect = false)]
    struct ControlledSrc {
        #[reflect(ignore)]
        ready_times: Arc<Mutex<mpsc::Receiver<CuTime>>>,
        #[reflect(ignore)]
        done: mpsc::Sender<()>,
    }

    impl Freezable for ControlledSrc {}

    impl CuSrcTask for ControlledSrc {
        type Resources<'r> = ControlledSrcResources;
        type Output<'m> = output_msg!(u32);

        fn new(config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let _ = config;
            Ok(Self {
                ready_times: resources.ready_times,
                done: resources.done,
            })
        }

        fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            let ready_time = self
                .ready_times
                .lock()
                .unwrap()
                .recv_timeout(Duration::from_secs(1))
                .expect("timed out waiting for ready signal");
            output.set_payload(ready_time.as_nanos() as u32);
            output.metadata.process_time.start = ctx.now().into();
            output.metadata.process_time.end = ready_time.into();
            let _ = self.done.send(());
            Ok(())
        }
    }

    #[test]
    fn background_clears_output_while_processing() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let context = CuContext::new_with_clock();
        let (action_tx, action_rx) = mpsc::channel::<Option<u32>>();
        let (done_tx, done_rx) = mpsc::channel::<()>();
        let resources = ActionTaskResources {
            actions: Arc::new(Mutex::new(action_rx)),
            done: done_tx,
        };

        let mut async_task: CuAsyncTask<ActionTask, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), resources, tp).unwrap();
        let input = CuMsg::new(Some(1u32));
        let mut output = CuMsg::new(None);

        async_task.process(&context, &input, &mut output).unwrap();
        assert!(output.payload().is_none());

        output.set_payload(999);
        async_task.process(&context, &input, &mut output).unwrap();
        assert!(
            output.payload().is_none(),
            "background poll should clear stale output while the worker is still running"
        );

        action_tx.send(Some(7)).unwrap();
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("background worker never finished");
    }

    #[test]
    fn background_empty_run_does_not_reemit_previous_payload() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let context = CuContext::new_with_clock();
        let (action_tx, action_rx) = mpsc::channel::<Option<u32>>();
        let (done_tx, done_rx) = mpsc::channel::<()>();
        let resources = ActionTaskResources {
            actions: Arc::new(Mutex::new(action_rx)),
            done: done_tx,
        };

        let mut async_task: CuAsyncTask<ActionTask, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), resources, tp).unwrap();
        let some_input = CuMsg::new(Some(1u32));
        let no_input = CuMsg::new(None::<u32>);
        let mut output = CuMsg::new(None);

        action_tx.send(Some(42)).unwrap();
        async_task
            .process(&context, &some_input, &mut output)
            .expect("failed to start first background run");
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("first background run never finished");
        wait_until_async_idle(&async_task);

        action_tx.send(None).unwrap();
        async_task
            .process(&context, &no_input, &mut output)
            .expect("failed to start empty background run");
        assert_eq!(output.payload(), Some(&42));
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("empty background run never finished");
        wait_until_async_idle(&async_task);

        action_tx.send(None).unwrap();
        async_task
            .process(&context, &no_input, &mut output)
            .expect("failed to poll after empty background run");
        assert!(
            output.payload().is_none(),
            "background task re-emitted the previous payload after an empty run"
        );
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("cleanup background run never finished");
    }

    #[test]
    fn background_source_clears_output_while_processing() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let context = CuContext::new_with_clock();
        let (action_tx, action_rx) = mpsc::channel::<Option<u32>>();
        let (done_tx, done_rx) = mpsc::channel::<()>();
        let resources = ActionTaskResources {
            actions: Arc::new(Mutex::new(action_rx)),
            done: done_tx,
        };

        let mut async_src: CuAsyncSrcTask<ActionSrc, u32> =
            CuAsyncSrcTask::new(Some(&ComponentConfig::default()), resources, tp).unwrap();
        let mut output = CuMsg::new(None);

        async_src.process(&context, &mut output).unwrap();
        assert!(output.payload().is_none());

        output.set_payload(999);
        async_src.process(&context, &mut output).unwrap();
        assert!(
            output.payload().is_none(),
            "background source poll should clear stale output while the worker is still running"
        );

        action_tx.send(Some(7)).unwrap();
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("background source never finished");
    }

    #[test]
    fn background_source_empty_run_does_not_reemit_previous_payload() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let context = CuContext::new_with_clock();
        let (action_tx, action_rx) = mpsc::channel::<Option<u32>>();
        let (done_tx, done_rx) = mpsc::channel::<()>();
        let resources = ActionTaskResources {
            actions: Arc::new(Mutex::new(action_rx)),
            done: done_tx,
        };

        let mut async_src: CuAsyncSrcTask<ActionSrc, u32> =
            CuAsyncSrcTask::new(Some(&ComponentConfig::default()), resources, tp).unwrap();
        let mut output = CuMsg::new(None);

        action_tx.send(Some(42)).unwrap();
        async_src
            .process(&context, &mut output)
            .expect("failed to start first background source run");
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("first background source run never finished");
        wait_until_async_src_idle(&async_src);

        action_tx.send(None).unwrap();
        async_src
            .process(&context, &mut output)
            .expect("failed to start empty background source run");
        assert_eq!(output.payload(), Some(&42));
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("empty background source run never finished");
        wait_until_async_src_idle(&async_src);

        action_tx.send(None).unwrap();
        async_src
            .process(&context, &mut output)
            .expect("failed to poll background source after empty run");
        assert!(
            output.payload().is_none(),
            "background source re-emitted the previous payload after an empty run"
        );
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("cleanup background source run never finished");
    }

    #[test]
    fn background_respects_recorded_ready_time() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (context, clock_mock) = CuContext::new_mock_clock();

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
        async_task.process(&context, &input, &mut output).unwrap();
        assert!(output.payload().is_none());

        // Copperlist 1 at time 10: still running in the background.
        clock_mock.set_value(10);
        async_task.process(&context, &input, &mut output).unwrap();
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
        async_task.process(&context, &input, &mut output).unwrap();
        assert!(
            output.payload().is_none(),
            "Output surfaced before recorded ready time"
        );

        // Once the mock clock reaches the recorded end time, the result is released.
        clock_mock.set_value(30);
        async_task.process(&context, &input, &mut output).unwrap();
        assert_eq!(output.payload(), Some(&30u32));

        // Allow the background worker spawned by the last poll to complete so the thread pool shuts down cleanly.
        ready_tx.send(CuTime::from(40u64)).unwrap();
        let _ = done_rx.recv_timeout(Duration::from_secs(1));
    }

    #[test]
    fn background_source_respects_recorded_ready_time() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (context, clock_mock) = CuContext::new_mock_clock();
        let (ready_tx, ready_rx) = mpsc::channel::<CuTime>();
        let (done_tx, done_rx) = mpsc::channel::<()>();
        let resources = ControlledSrcResources {
            ready_times: Arc::new(Mutex::new(ready_rx)),
            done: done_tx,
        };

        let mut async_src: CuAsyncSrcTask<ControlledSrc, u32> =
            CuAsyncSrcTask::new(Some(&ComponentConfig::default()), resources, tp.clone()).unwrap();
        let mut output = CuMsg::new(None);

        clock_mock.set_value(0);
        async_src.process(&context, &mut output).unwrap();
        assert!(output.payload().is_none());

        clock_mock.set_value(10);
        async_src.process(&context, &mut output).unwrap();
        assert!(output.payload().is_none());

        clock_mock.set_value(30);
        ready_tx.send(CuTime::from(30u64)).unwrap();
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("background source never finished");

        let mut ready_at_recorded = None;
        for _ in 0..100 {
            let state = async_src.state.lock().unwrap();
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
            "background source finished without recording ready_at"
        );

        clock_mock.set_value(20);
        async_src.process(&context, &mut output).unwrap();
        assert!(
            output.payload().is_none(),
            "background source surfaced output before recorded ready time"
        );

        clock_mock.set_value(30);
        async_src.process(&context, &mut output).unwrap();
        assert_eq!(output.payload(), Some(&30u32));

        ready_tx.send(CuTime::from(40u64)).unwrap();
        let _ = done_rx.recv_timeout(Duration::from_secs(1));
    }
}
