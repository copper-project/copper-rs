use crate::config::ComponentConfig;
use crate::context::CuContext;
use crate::cutask::{BincodeAdapter, CuMsg, CuMsgPayload, CuSrcTask, CuTask, Freezable};
use crate::reflect::{Reflect, TypePath};
use bincode::config::standard;
use bincode::de::read::Reader;
use bincode::de::{Decode, Decoder};
use bincode::enc::write::Writer;
use bincode::enc::{Encode, Encoder, EncoderImpl};
use bincode::error::{DecodeError, EncodeError};
use cu29_clock::CuTime;
use cu29_traits::{CuError, CuResult};
use rayon::ThreadPool;
use std::any::Any;
use std::cell::UnsafeCell;
use std::sync::{Arc, Mutex};

const ASYNC_IDLE_TAG: u8 = 0xA0;
const ASYNC_WAITING_TAG: u8 = 0xA1;
const ASYNC_FAILED_TAG: u8 = 0xA2;
const ASYNC_PENDING_TAG: u8 = 0xA3;

enum AsyncStatus {
    Idle,
    Waiting(CuTime),
    Failed(String),
    Running(u64),
    ReplayPending(u64),
}

struct AsyncState<O: CuMsgPayload> {
    status: AsyncStatus,
    committed_task: Vec<u8>,
    task_scratch: Vec<u8>,
    committed_output: CuMsg<O>,
}

impl<O: CuMsgPayload> AsyncState<O> {
    fn new() -> Self {
        Self {
            status: AsyncStatus::Idle,
            committed_task: Vec::new(),
            task_scratch: Vec::new(),
            committed_output: CuMsg::default(),
        }
    }
}

fn commit_initial_snapshot<O: CuMsgPayload>(state: &mut AsyncState<O>, snapshot: Vec<u8>) {
    let mut scratch = core::mem::replace(&mut state.committed_task, snapshot);
    if scratch.capacity() < state.committed_task.len() {
        scratch.reserve_exact(state.committed_task.len() - scratch.len());
    }
    state.task_scratch = scratch;
}

struct BufferWriter<'a>(&'a mut Vec<u8>);

impl Writer for BufferWriter<'_> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        self.0.extend_from_slice(bytes);
        Ok(())
    }
}

fn encode_value_into(value: &impl Encode, buffer: &mut Vec<u8>) -> Result<(), EncodeError> {
    buffer.clear();
    let mut encoder = EncoderImpl::new(BufferWriter(buffer), standard());
    value.encode(&mut encoder)
}

fn freeze_into(task: &impl Freezable, buffer: &mut Vec<u8>) -> Result<(), EncodeError> {
    encode_value_into(&BincodeAdapter(task), buffer)
}

fn thaw_from(task: &mut impl Freezable, buffer: &[u8]) -> Result<(), DecodeError> {
    let reader = bincode::de::read::SliceReader::new(buffer);
    let mut decoder = bincode::de::DecoderImpl::new(reader, standard(), ());
    task.thaw(&mut decoder)
}

struct DynWriter<'a>(&'a mut dyn Writer);

impl Writer for DynWriter<'_> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        self.0.write(bytes)
    }
}

struct DynReader<'a>(&'a mut dyn Reader);

impl Reader for DynReader<'_> {
    fn read(&mut self, bytes: &mut [u8]) -> Result<(), DecodeError> {
        self.0.read(bytes)
    }

    fn peek_read(&mut self, length: usize) -> Option<&[u8]> {
        self.0.peek_read(length)
    }

    fn consume(&mut self, length: usize) {
        self.0.consume(length);
    }
}

trait ErasedDispatch: Any + Send + Sync {
    fn as_any(&self) -> &dyn Any;
    fn encode_input(&self, writer: &mut dyn Writer) -> Result<(), EncodeError>;
    fn decode_input(&self, reader: &mut dyn Reader) -> Result<(), DecodeError>;
}

struct DispatchSlot<I: CuMsgPayload> {
    input: UnsafeCell<CuMsg<I>>,
}

// SAFETY: the slot is written only before a run is published or while thaw holds the
// inner-task mutex. While a run is published, workers and keyframes only read it.
unsafe impl<I: CuMsgPayload + Send + Sync> Sync for DispatchSlot<I> {}

impl<I> DispatchSlot<I>
where
    I: CuMsgPayload + Send + Sync + 'static,
{
    fn new() -> Self {
        Self {
            input: UnsafeCell::new(CuMsg::default()),
        }
    }

    fn replace(&self, input: CuMsg<I>) -> CuMsg<I> {
        // SAFETY: callers write only while no worker can hold a reference to the slot.
        unsafe { core::mem::replace(&mut *self.input.get(), input) }
    }

    fn get(&self) -> &CuMsg<I> {
        // SAFETY: published dispatch input remains immutable until the worker finishes.
        unsafe { &*self.input.get() }
    }
}

impl<I> ErasedDispatch for DispatchSlot<I>
where
    I: CuMsgPayload + Send + Sync + 'static,
{
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn encode_input(&self, writer: &mut dyn Writer) -> Result<(), EncodeError> {
        let mut encoder = EncoderImpl::new(DynWriter(writer), standard());
        self.get().encode(&mut encoder)
    }

    fn decode_input(&self, reader: &mut dyn Reader) -> Result<(), DecodeError> {
        let mut decoder = bincode::de::DecoderImpl::new(DynReader(reader), standard(), ());
        drop(self.replace(CuMsg::<I>::decode(&mut decoder)?));
        Ok(())
    }
}

fn failure(error: CuError) -> AsyncStatus {
    AsyncStatus::Failed(error.to_string())
}

fn encode_async_state<O, E>(state: &AsyncState<O>, encoder: &mut E) -> Result<bool, EncodeError>
where
    O: CuMsgPayload + Send + 'static,
    E: Encoder,
{
    Encode::encode(&state.committed_task, encoder)?;
    Encode::encode(&state.committed_output, encoder)?;
    match &state.status {
        AsyncStatus::Idle => {
            ASYNC_IDLE_TAG.encode(encoder)?;
            Ok(false)
        }
        AsyncStatus::Waiting(ready_at) => {
            ASYNC_WAITING_TAG.encode(encoder)?;
            ready_at.encode(encoder)?;
            Ok(false)
        }
        AsyncStatus::Failed(snapshot) => {
            ASYNC_FAILED_TAG.encode(encoder)?;
            snapshot.encode(encoder)?;
            Ok(false)
        }
        AsyncStatus::Running(cl_id) | AsyncStatus::ReplayPending(cl_id) => {
            ASYNC_PENDING_TAG.encode(encoder)?;
            cl_id.encode(encoder)?;
            Ok(true)
        }
    }
}

fn decode_async_state<O, D>(decoder: &mut D) -> Result<AsyncState<O>, DecodeError>
where
    O: CuMsgPayload + Send + 'static,
    D: Decoder,
{
    let committed_task = Decode::decode(decoder)?;
    let committed_output = CuMsg::<O>::decode(&mut decoder.with_context(()))?;
    let status = match u8::decode(decoder)? {
        ASYNC_IDLE_TAG => AsyncStatus::Idle,
        ASYNC_WAITING_TAG => AsyncStatus::Waiting(Decode::decode(decoder)?),
        ASYNC_FAILED_TAG => {
            let snapshot: String = Decode::decode(decoder)?;
            AsyncStatus::Failed(snapshot)
        }
        ASYNC_PENDING_TAG => AsyncStatus::ReplayPending(u64::decode(decoder)?),
        tag => {
            return Err(DecodeError::OtherString(format!(
                "unsupported async keyframe payload tag {tag:#04x}; expected version 1"
            )));
        }
    };
    Ok(AsyncState {
        status,
        committed_task,
        task_scratch: Vec::new(),
        committed_output,
    })
}

type ErasedDispatchSlot = Arc<dyn ErasedDispatch>;

fn dispatch_slot<I>(slot: &ErasedDispatchSlot) -> CuResult<&DispatchSlot<I>>
where
    I: CuMsgPayload + Send + Sync + 'static,
{
    slot.as_any()
        .downcast_ref::<DispatchSlot<I>>()
        .ok_or_else(|| CuError::from("Async task dispatch slot type did not match its input"))
}

fn record_async_error<O: CuMsgPayload>(state: &Mutex<AsyncState<O>>, error: CuError) {
    let mut guard = match state.lock() {
        Ok(guard) => guard,
        Err(poison) => poison.into_inner(),
    };
    guard.status = failure(error);
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
    state: Arc<Mutex<AsyncState<O>>>,
    #[reflect(ignore)]
    dispatch: Option<ErasedDispatchSlot>,
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
        Ok(Self {
            task,
            state: Arc::new(Mutex::new(AsyncState::new())),
            dispatch: None,
            tp,
        })
    }

    fn initialize_dispatch<I>(&mut self) -> CuResult<()>
    where
        I: CuMsgPayload + Send + Sync + 'static,
    {
        if let Some(dispatch) = self.dispatch.as_ref() {
            let _ = dispatch_slot::<I>(dispatch)?;
        } else {
            self.dispatch = Some(Arc::new(DispatchSlot::<I>::new()));
        }
        Ok(())
    }
}

impl<T, O> Freezable for CuAsyncTask<T, O>
where
    T: for<'m> CuTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let state = self
            .state
            .lock()
            .map_err(|_| EncodeError::OtherString("async task state mutex poisoned".to_string()))?;
        let pending = encode_async_state(&state, encoder)?;
        if pending {
            self.dispatch
                .as_ref()
                .ok_or_else(|| {
                    EncodeError::OtherString(
                        "async task pending before dispatch initialization".to_string(),
                    )
                })?
                .encode_input(encoder.writer())?;
        }
        Ok(())
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        let mut task = self
            .task
            .lock()
            .map_err(|_| DecodeError::OtherString("async task mutex poisoned".to_string()))?;
        let restored_state = decode_async_state(decoder)?;
        if matches!(restored_state.status, AsyncStatus::ReplayPending(_)) {
            self.dispatch
                .as_ref()
                .ok_or_else(|| {
                    DecodeError::OtherString(
                        "async task restored before dispatch initialization".to_string(),
                    )
                })?
                .decode_input(decoder.reader())?;
        }
        thaw_from(&mut *task, &restored_state.committed_task)?;

        let mut state = self
            .state
            .lock()
            .map_err(|_| DecodeError::OtherString("async task state mutex poisoned".to_string()))?;
        *state = restored_state;
        Ok(())
    }
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
        self.initialize_dispatch::<I>()?;
        let mut task = self
            .task
            .lock()
            .map_err(|_| CuError::from("Async task mutex poisoned during start"))?;
        task.start(ctx)?;
        let mut state = self
            .state
            .lock()
            .map_err(|_| CuError::from("Async task state mutex poisoned during start"))?;
        let mut snapshot = core::mem::take(&mut state.task_scratch);
        freeze_into(&*task, &mut snapshot).map_err(|error| {
            CuError::from("Failed to snapshot async task after start").with_cause(error)
        })?;
        commit_initial_snapshot(&mut state, snapshot);
        Ok(())
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        real_output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (dispatch, dispatch_cl_id, mut task_snapshot, retired_input) = {
            let mut state = self.state.lock().map_err(|_| {
                CuError::from("Async task state mutex poisoned while scheduling background work")
            })?;
            if matches!(state.status, AsyncStatus::Failed(_)) {
                let AsyncStatus::Failed(error) =
                    core::mem::replace(&mut state.status, AsyncStatus::Idle)
                else {
                    unreachable!();
                };
                return Err(CuError::from(error));
            }
            if matches!(state.status, AsyncStatus::Running(_)) {
                *real_output = CuMsg::default();
                return Ok(());
            }
            if let AsyncStatus::Waiting(ready_at) = state.status
                && ctx.now() < ready_at
            {
                *real_output = CuMsg::default();
                return Ok(());
            }

            let dispatch = self
                .dispatch
                .as_ref()
                .ok_or_else(|| CuError::from("Async task dispatch slot was not initialized"))?;
            let typed_dispatch = dispatch_slot::<I>(dispatch)?;
            let (dispatch_cl_id, replay_pending) =
                if let AsyncStatus::ReplayPending(cl_id) = state.status {
                    (cl_id, true)
                } else {
                    (ctx.cl_id(), false)
                };
            let retired_input = if replay_pending {
                CuMsg::default()
            } else {
                typed_dispatch.replace((*input).clone())
            };
            *real_output = state.committed_output.clone();
            state.status = AsyncStatus::Running(dispatch_cl_id);
            (
                dispatch.clone(),
                dispatch_cl_id,
                core::mem::take(&mut state.task_scratch),
                retired_input,
            )
        };

        self.tp.spawn_fifo({
            let ctx = ctx.with_cl_id(dispatch_cl_id);
            let task = self.task.clone();
            let state = self.state.clone();
            move || {
                let mut worker_output = CuMsg::default();
                let typed_dispatch =
                    if let Some(slot) = dispatch.as_any().downcast_ref::<DispatchSlot<I>>() {
                        slot
                    } else {
                        record_async_error(
                            &state,
                            CuError::from("Async task dispatch slot type did not match its input"),
                        );
                        return;
                    };
                let input_ref = typed_dispatch.get();
                let mut task_guard = match task.lock() {
                    Ok(guard) => guard,
                    Err(poison) => {
                        record_async_error(
                            &state,
                            CuError::from(format!("Async task mutex poisoned: {poison}")),
                        );
                        return;
                    }
                };
                // Each async run starts from an empty output so a task that
                // chooses not to publish does not leak the previous payload.
                // Track the actual processing interval so replay can honor it.
                if worker_output.metadata.process_time.start.is_none() {
                    worker_output.metadata.process_time.start = ctx.now().into();
                }
                let task_result = task_guard.process(&ctx, input_ref, &mut worker_output);
                let fallback_end = ctx.now();
                let end_from_metadata: Option<CuTime> =
                    worker_output.metadata.process_time.end.into();
                let ready_at = end_from_metadata.unwrap_or_else(|| {
                    worker_output.metadata.process_time.end = fallback_end.into();
                    fallback_end
                });
                let snapshot_result = freeze_into(&*task_guard, &mut task_snapshot);
                let (commit_snapshot, status) = match snapshot_result {
                    Ok(()) => (
                        true,
                        match task_result {
                            Ok(()) => AsyncStatus::Waiting(ready_at),
                            Err(error) => failure(error),
                        },
                    ),
                    Err(error) => (
                        false,
                        failure(
                            CuError::from("Failed to snapshot completed async task")
                                .with_cause(error),
                        ),
                    ),
                };

                let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
                let retired_output = if commit_snapshot {
                    guard.task_scratch =
                        core::mem::replace(&mut guard.committed_task, task_snapshot);
                    Some(core::mem::replace(
                        &mut guard.committed_output,
                        worker_output,
                    ))
                } else {
                    guard.task_scratch = task_snapshot;
                    None
                };
                guard.status = status;
                drop(guard);
                drop(task_guard);
                drop(retired_output);
                drop(retired_input);
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
    state: Arc<Mutex<AsyncState<O>>>,
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
        Ok(Self {
            task,
            state: Arc::new(Mutex::new(AsyncState::new())),
            tp,
        })
    }
}

impl<T, O> Freezable for CuAsyncSrcTask<T, O>
where
    T: for<'m> CuSrcTask<Output<'m> = CuMsg<O>> + Send + 'static,
    O: CuMsgPayload + Send + 'static,
{
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let state = self.state.lock().map_err(|_| {
            EncodeError::OtherString("async source state mutex poisoned".to_string())
        })?;
        let _ = encode_async_state(&state, encoder)?;
        Ok(())
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        let mut task = self
            .task
            .lock()
            .map_err(|_| DecodeError::OtherString("async source mutex poisoned".to_string()))?;
        let restored_state = decode_async_state(decoder)?;
        thaw_from(&mut *task, &restored_state.committed_task)?;

        let mut state = self.state.lock().map_err(|_| {
            DecodeError::OtherString("async source state mutex poisoned".to_string())
        })?;
        *state = restored_state;
        Ok(())
    }
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
        task.start(ctx)?;
        let mut state = self
            .state
            .lock()
            .map_err(|_| CuError::from("Async source state mutex poisoned during start"))?;
        let mut snapshot = core::mem::take(&mut state.task_scratch);
        freeze_into(&*task, &mut snapshot).map_err(|error| {
            CuError::from("Failed to snapshot async source after start").with_cause(error)
        })?;
        commit_initial_snapshot(&mut state, snapshot);
        Ok(())
    }

    fn process<'o>(&mut self, ctx: &CuContext, real_output: &mut Self::Output<'o>) -> CuResult<()> {
        let (dispatch_cl_id, mut task_snapshot) = {
            let mut state = self.state.lock().map_err(|_| {
                CuError::from("Async source state mutex poisoned while scheduling background work")
            })?;
            if matches!(state.status, AsyncStatus::Failed(_)) {
                let AsyncStatus::Failed(error) =
                    core::mem::replace(&mut state.status, AsyncStatus::Idle)
                else {
                    unreachable!();
                };
                return Err(CuError::from(error));
            }
            if matches!(state.status, AsyncStatus::Running(_)) {
                *real_output = CuMsg::default();
                return Ok(());
            }
            if let AsyncStatus::Waiting(ready_at) = state.status
                && ctx.now() < ready_at
            {
                *real_output = CuMsg::default();
                return Ok(());
            }

            let dispatch_cl_id = if let AsyncStatus::ReplayPending(cl_id) = state.status {
                cl_id
            } else {
                ctx.cl_id()
            };
            *real_output = state.committed_output.clone();
            state.status = AsyncStatus::Running(dispatch_cl_id);
            (dispatch_cl_id, core::mem::take(&mut state.task_scratch))
        };

        self.tp.spawn_fifo({
            let ctx = ctx.with_cl_id(dispatch_cl_id);
            let task = self.task.clone();
            let state = self.state.clone();
            move || {
                let mut worker_output = CuMsg::default();
                let mut task_guard = match task.lock() {
                    Ok(guard) => guard,
                    Err(poison) => {
                        record_async_error(
                            &state,
                            CuError::from(format!("Async source mutex poisoned: {poison}")),
                        );
                        return;
                    }
                };
                if worker_output.metadata.process_time.start.is_none() {
                    worker_output.metadata.process_time.start = ctx.now().into();
                }
                let task_result = task_guard.process(&ctx, &mut worker_output);
                let fallback_end = ctx.now();
                let end_from_metadata: Option<CuTime> =
                    worker_output.metadata.process_time.end.into();
                let ready_at = end_from_metadata.unwrap_or_else(|| {
                    worker_output.metadata.process_time.end = fallback_end.into();
                    fallback_end
                });
                let snapshot_result = freeze_into(&*task_guard, &mut task_snapshot);
                let (commit_snapshot, status) = match snapshot_result {
                    Ok(()) => (
                        true,
                        match task_result {
                            Ok(()) => AsyncStatus::Waiting(ready_at),
                            Err(error) => failure(error),
                        },
                    ),
                    Err(error) => (
                        false,
                        failure(
                            CuError::from("Failed to snapshot completed async source")
                                .with_cause(error),
                        ),
                    ),
                };

                let mut guard = state.lock().unwrap_or_else(|poison| poison.into_inner());
                let retired_output = if commit_snapshot {
                    guard.task_scratch =
                        core::mem::replace(&mut guard.committed_task, task_snapshot);
                    Some(core::mem::replace(
                        &mut guard.committed_output,
                        worker_output,
                    ))
                } else {
                    guard.task_scratch = task_snapshot;
                    None
                };
                guard.status = status;
                drop(guard);
                drop(task_guard);
                drop(retired_output);
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
    use crate::cutask_anytime::{
        AnytimePolicy, AnytimeStatus, CuAnytimeRunner, CuAnytimeTask, Quality, quality_from_f32,
    };
    use crate::input_msg;
    use crate::output_msg;
    use cu29_clock::CuDuration;
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
        async_task.start(&context).unwrap();
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
            if !matches!(state.status, AsyncStatus::Running(_)) {
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
            if !matches!(state.status, AsyncStatus::Running(_)) {
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
        async_task.start(&context).unwrap();
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
        async_task.start(&context).unwrap();
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
        async_task.start(&context).unwrap();
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
        let mut ready_at_recorded = false;
        for _ in 0..100 {
            let state = async_task.state.lock().unwrap();
            if matches!(state.status, AsyncStatus::Waiting(_)) {
                ready_at_recorded = true;
                break;
            }
            drop(state);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(
            ready_at_recorded,
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

    /// Anytime task under the wrapper: one increment per quantum, quality
    /// climbing toward the input target.
    #[derive(Reflect)]
    struct IncrementalPlanner {
        target: u32,
        acc: u32,
    }

    impl Freezable for IncrementalPlanner {}

    impl CuAnytimeTask for IncrementalPlanner {
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);
        type Resources<'r> = ();
        type Quality = Quality;

        fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
            Ok(Self { target: 0, acc: 0 })
        }

        fn base(
            &mut self,
            _ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<AnytimeStatus<Quality>> {
            self.target = input.payload().copied().ok_or("planner: no input")?;
            self.acc = 0;
            output.set_payload(self.acc);
            Ok(AnytimeStatus::Improved(quality_from_f32(0.0)))
        }

        fn refine(
            &mut self,
            _ctx: &CuContext,
            output: &mut Self::Output<'_>,
        ) -> CuResult<AnytimeStatus<Quality>> {
            self.acc += 1;
            output.set_payload(self.acc);
            Ok(AnytimeStatus::Improved(quality_from_f32(
                self.acc as f32 / self.target as f32,
            )))
        }
    }

    /// Mirrors codegen for `anytime: (max_refines: 3)` on a background node.
    struct ThreeQuantaPolicy;
    impl<Q: Copy + PartialOrd> AnytimePolicy<Q> for ThreeQuantaPolicy {
        const TIME_BUDGET: Option<CuDuration> = None;
        const MAX_AGE: Option<CuDuration> = None;
        const MAX_STALL: Option<u32> = None;
        const MAX_REFINES: Option<u32> = Some(3);
    }

    #[test]
    fn background_anytime_job_lands_with_its_status_stamp() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let context = CuContext::new_with_clock();
        let mut task: CuAsyncTask<CuAnytimeRunner<IncrementalPlanner, ThreeQuantaPolicy>, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), (), tp).unwrap();
        task.start(&context).unwrap();

        let input = CuMsg::new(Some(5u32));
        let mut output = CuMsg::new(None);

        // Poll until the worker's job comes back through the buffered output.
        for _ in 0..1000 {
            task.process(&context, &input, &mut output).unwrap();
            if output.payload().is_some() {
                break;
            }
            std::thread::sleep(Duration::from_millis(1));
        }

        // Three quanta of a job needing five: stopped by the quanta bound, and
        // the stamp the runner wrote survived the buffered-output copy.
        assert_eq!(output.payload(), Some(&3));
        assert_eq!(output.metadata.status_txt.0.as_str(), "any:3it q=0.60 max");
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

        let mut ready_at_recorded = false;
        for _ in 0..100 {
            let state = async_src.state.lock().unwrap();
            if matches!(state.status, AsyncStatus::Waiting(_)) {
                ready_at_recorded = true;
                break;
            }
            drop(state);
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(
            ready_at_recorded,
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

    type ReplayTaskObservation = (u32, u64, u32, u32);

    #[derive(Clone)]
    struct ReplayTaskResources {
        release: Arc<Mutex<mpsc::Receiver<()>>>,
        observed: mpsc::Sender<ReplayTaskObservation>,
    }

    #[derive(Reflect)]
    #[reflect(no_field_bounds, from_reflect = false)]
    struct ReplayTask {
        counter: u32,
        #[reflect(ignore)]
        release: Arc<Mutex<mpsc::Receiver<()>>>,
        #[reflect(ignore)]
        observed: mpsc::Sender<ReplayTaskObservation>,
    }

    impl Freezable for ReplayTask {
        fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
            self.counter.encode(encoder)
        }

        fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
            self.counter = u32::decode(decoder)?;
            Ok(())
        }
    }

    impl CuTask for ReplayTask {
        type Resources<'r> = ReplayTaskResources;
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);

        fn new(
            _config: Option<&ComponentConfig>,
            resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self {
                counter: 0,
                release: resources.release,
                observed: resources.observed,
            })
        }

        fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
            self.counter = 10;
            Ok(())
        }

        fn process(
            &mut self,
            ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            self.release
                .lock()
                .unwrap()
                .recv_timeout(Duration::from_secs(1))
                .expect("timed out waiting to release replay task");
            let input = input.payload().copied().expect("replay task input");
            self.observed
                .send((input, ctx.cl_id(), ctx.instance_id(), self.counter))
                .expect("failed to record replay task dispatch");
            self.counter += 1;
            output.set_payload(input + self.counter);
            Ok(())
        }
    }

    fn replay_task_resources() -> (
        ReplayTaskResources,
        mpsc::Sender<()>,
        mpsc::Receiver<ReplayTaskObservation>,
    ) {
        let (release_tx, release_rx) = mpsc::channel();
        let (observed_tx, observed_rx) = mpsc::channel();
        (
            ReplayTaskResources {
                release: Arc::new(Mutex::new(release_rx)),
                observed: observed_tx,
            },
            release_tx,
            observed_rx,
        )
    }

    #[test]
    fn background_freeze_mid_run_replays_original_dispatch_from_committed_state() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (resources, release_tx, observed_rx) = replay_task_resources();
        let (base_context, _) = CuContext::new_mock_clock();
        let mut dispatch_context = CuContext::builder(base_context.clock.clone())
            .cl_id(7)
            .instance_id(3)
            .task_ids(&["replay"])
            .build();
        dispatch_context.set_current_task(0);
        let mut original: CuAsyncTask<ReplayTask, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), resources, tp).unwrap();
        original.start(&dispatch_context).unwrap();
        let original_input = CuMsg::new(Some(41u32));
        let mut output = CuMsg::default();
        original
            .process(&dispatch_context, &original_input, &mut output)
            .unwrap();

        let frozen = bincode::encode_to_vec(BincodeAdapter(&original), standard())
            .expect("mid-run async freeze failed");
        let committed_task = bincode::encode_to_vec(10u32, standard()).unwrap();
        let expected = bincode::encode_to_vec(
            (
                committed_task,
                CuMsg::<u32>::default(),
                ASYNC_PENDING_TAG,
                7u64,
                original_input.clone(),
            ),
            standard(),
        )
        .unwrap();
        assert_eq!(
            frozen, expected,
            "pending task frames contain only committed state, CL id, and input"
        );
        assert!(
            matches!(
                original.state.lock().unwrap().status,
                AsyncStatus::Running(7)
            ),
            "freezing must not disturb the live worker"
        );
        release_tx.send(()).unwrap();
        assert_eq!(
            observed_rx.recv_timeout(Duration::from_secs(1)).unwrap(),
            (41, 7, 3, 10)
        );

        let replay_tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (replay_resources, replay_release_tx, replay_observed_rx) = replay_task_resources();
        let mut restored: CuAsyncTask<ReplayTask, u32> = CuAsyncTask::new(
            Some(&ComponentConfig::default()),
            replay_resources,
            replay_tp,
        )
        .unwrap();
        restored.start(&dispatch_context).unwrap();
        let reader = bincode::de::read::SliceReader::new(&frozen);
        let mut decoder = bincode::de::DecoderImpl::new(reader, standard(), ());
        restored.thaw(&mut decoder).unwrap();
        assert!(matches!(
            restored.state.lock().unwrap().status,
            AsyncStatus::ReplayPending(7)
        ));

        let mut current_context = CuContext::builder(base_context.clock.clone())
            .cl_id(99)
            .instance_id(8)
            .task_ids(&["replay"])
            .build();
        current_context.set_current_task(0);
        let current_input = CuMsg::new(Some(999u32));
        restored
            .process(&current_context, &current_input, &mut output)
            .unwrap();
        replay_release_tx.send(()).unwrap();
        assert_eq!(
            replay_observed_rx
                .recv_timeout(Duration::from_secs(1))
                .unwrap(),
            (41, 7, 8, 10),
            "replay must retain only the original input and CopperList id"
        );
        wait_until_async_idle(&restored);

        restored
            .process(&current_context, &current_input, &mut output)
            .unwrap();
        assert_eq!(output.payload(), Some(&52));
        replay_release_tx.send(()).unwrap();
        let _ = replay_observed_rx.recv_timeout(Duration::from_secs(1));
    }

    #[cfg(feature = "memory_monitoring")]
    #[test]
    fn background_freeze_mid_run_does_not_allocate() {
        const SNAPSHOT_CAPACITY: usize = 4 * 1024;

        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (resources, release_tx, observed_rx) = replay_task_resources();
        let (context, _) = CuContext::new_mock_clock();
        let mut async_task: CuAsyncTask<ReplayTask, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), resources, tp).unwrap();
        async_task.start(&context).unwrap();
        let input = CuMsg::new(Some(41u32));
        let mut output = CuMsg::default();
        async_task.process(&context, &input, &mut output).unwrap();

        let mut snapshot = [0u8; SNAPSHOT_CAPACITY];
        let allocations = crate::monitoring::ScopedAllocCounter::new();
        let writer = bincode::enc::write::SliceWriter::new(&mut snapshot);
        let mut encoder = EncoderImpl::new(writer, standard());
        BincodeAdapter(&async_task).encode(&mut encoder).unwrap();
        assert_eq!(allocations.allocated(), 0);

        release_tx.send(()).unwrap();
        let _ = observed_rx.recv_timeout(Duration::from_secs(1));
    }

    #[test]
    fn background_freeze_thaw_preserves_pending_error() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let context = CuContext::new_with_clock();
        let mut original: CuAsyncTask<TestTask, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), (), tp).unwrap();
        original.start(&context).unwrap();
        {
            let mut state = original.state.lock().unwrap();
            let error = CuError::from("expected async failure");
            state.status = failure(error);
        }
        let frozen = bincode::encode_to_vec(BincodeAdapter(&original), standard()).unwrap();

        let replay_tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let mut restored: CuAsyncTask<TestTask, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), (), replay_tp).unwrap();
        restored.start(&context).unwrap();
        let reader = bincode::de::read::SliceReader::new(&frozen);
        let mut decoder = bincode::de::DecoderImpl::new(reader, standard(), ());
        restored.thaw(&mut decoder).unwrap();

        let input = CuMsg::new(Some(1u32));
        let mut output = CuMsg::default();
        let error = restored.process(&context, &input, &mut output).unwrap_err();
        assert!(error.to_string().contains("expected async failure"));
    }

    #[test]
    fn worker_completion_racing_freeze_is_an_atomic_committed_snapshot() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (resources, release_tx, observed_rx) = replay_task_resources();
        let (context, _) = CuContext::new_mock_clock();
        let mut async_task: CuAsyncTask<ReplayTask, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), resources, tp).unwrap();
        async_task.start(&context).unwrap();
        let input = CuMsg::new(Some(41u32));
        let mut output = CuMsg::default();
        async_task.process(&context, &input, &mut output).unwrap();

        let mut snapshots = vec![
            bincode::encode_to_vec(BincodeAdapter(&async_task), standard())
                .expect("freeze running async task"),
        ];
        release_tx.send(()).unwrap();
        observed_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("worker did not complete");
        for _ in 0..100 {
            snapshots.push(
                bincode::encode_to_vec(BincodeAdapter(&async_task), standard())
                    .expect("freeze async task racing completion"),
            );
            if !matches!(
                async_task.state.lock().unwrap().status,
                AsyncStatus::Running(_)
            ) {
                break;
            }
            std::thread::yield_now();
        }
        wait_until_async_idle(&async_task);
        snapshots.push(
            bincode::encode_to_vec(BincodeAdapter(&async_task), standard())
                .expect("freeze completed async task"),
        );

        let mut saw_running = false;
        let mut saw_idle = false;
        for snapshot in snapshots {
            let reader = bincode::de::read::SliceReader::new(&snapshot);
            let mut decoder = bincode::de::DecoderImpl::new(reader, standard(), ());
            let dispatch = DispatchSlot::<u32>::new();
            let state: AsyncState<u32> =
                decode_async_state(&mut decoder).expect("decode raced snapshot");
            if matches!(state.status, AsyncStatus::ReplayPending(_)) {
                dispatch.decode_input(decoder.reader()).unwrap();
            }
            let (task_counter, bytes_read): (u32, usize) =
                bincode::decode_from_slice(&state.committed_task, standard())
                    .expect("decode committed task");
            assert_eq!(bytes_read, state.committed_task.len());
            match state.status {
                AsyncStatus::ReplayPending(0) => {
                    saw_running = true;
                    assert_eq!(task_counter, 10);
                    assert!(state.committed_output.payload().is_none());
                    assert_eq!(dispatch.get().payload(), Some(&41));
                }
                AsyncStatus::Waiting(_) => {
                    saw_idle = true;
                    assert_eq!(task_counter, 11);
                    assert_eq!(state.committed_output.payload(), Some(&52));
                }
                _ => panic!("decoded keyframe was neither pending nor committed"),
            }
        }
        assert!(saw_running, "race did not capture the pre-completion state");
        assert!(
            saw_idle,
            "race did not capture the committed completion state"
        );
    }

    #[derive(Clone)]
    struct ReplaySrcResources {
        release: Arc<Mutex<mpsc::Receiver<()>>>,
        observed: mpsc::Sender<(u64, u32)>,
    }

    #[derive(Reflect)]
    #[reflect(no_field_bounds, from_reflect = false)]
    struct ReplaySrc {
        counter: u32,
        #[reflect(ignore)]
        release: Arc<Mutex<mpsc::Receiver<()>>>,
        #[reflect(ignore)]
        observed: mpsc::Sender<(u64, u32)>,
    }

    impl Freezable for ReplaySrc {
        fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
            self.counter.encode(encoder)
        }

        fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
            self.counter = u32::decode(decoder)?;
            Ok(())
        }
    }

    impl CuSrcTask for ReplaySrc {
        type Resources<'r> = ReplaySrcResources;
        type Output<'m> = output_msg!(u32);

        fn new(
            _config: Option<&ComponentConfig>,
            resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self {
                counter: 0,
                release: resources.release,
                observed: resources.observed,
            })
        }

        fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
            self.counter = 20;
            Ok(())
        }

        fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            self.release
                .lock()
                .unwrap()
                .recv_timeout(Duration::from_secs(1))
                .expect("timed out waiting to release replay source");
            self.observed
                .send((ctx.cl_id(), self.counter))
                .expect("failed to record replay source dispatch");
            self.counter += 1;
            output.set_payload(self.counter);
            Ok(())
        }
    }

    fn replay_src_resources() -> (
        ReplaySrcResources,
        mpsc::Sender<()>,
        mpsc::Receiver<(u64, u32)>,
    ) {
        let (release_tx, release_rx) = mpsc::channel();
        let (observed_tx, observed_rx) = mpsc::channel();
        (
            ReplaySrcResources {
                release: Arc::new(Mutex::new(release_rx)),
                observed: observed_tx,
            },
            release_tx,
            observed_rx,
        )
    }

    #[test]
    fn background_source_freeze_mid_run_replays_original_context() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (resources, release_tx, observed_rx) = replay_src_resources();
        let (base_context, _) = CuContext::new_mock_clock();
        let dispatch_context = CuContext::builder(base_context.clock.clone())
            .cl_id(7)
            .build();
        let mut original: CuAsyncSrcTask<ReplaySrc, u32> =
            CuAsyncSrcTask::new(Some(&ComponentConfig::default()), resources, tp).unwrap();
        original.start(&dispatch_context).unwrap();
        let mut output = CuMsg::default();
        original.process(&dispatch_context, &mut output).unwrap();
        let frozen = bincode::encode_to_vec(BincodeAdapter(&original), standard())
            .expect("mid-run async source freeze failed");
        let committed_task = bincode::encode_to_vec(20u32, standard()).unwrap();
        let expected = bincode::encode_to_vec(
            (
                committed_task,
                CuMsg::<u32>::default(),
                ASYNC_PENDING_TAG,
                7u64,
            ),
            standard(),
        )
        .unwrap();
        assert_eq!(
            frozen, expected,
            "pending source frames contain no task-only dispatch marker"
        );
        release_tx.send(()).unwrap();
        assert_eq!(
            observed_rx.recv_timeout(Duration::from_secs(1)).unwrap(),
            (7, 20)
        );

        let replay_tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (replay_resources, replay_release_tx, replay_observed_rx) = replay_src_resources();
        let mut restored: CuAsyncSrcTask<ReplaySrc, u32> = CuAsyncSrcTask::new(
            Some(&ComponentConfig::default()),
            replay_resources,
            replay_tp,
        )
        .unwrap();
        restored.start(&dispatch_context).unwrap();
        let reader = bincode::de::read::SliceReader::new(&frozen);
        let mut decoder = bincode::de::DecoderImpl::new(reader, standard(), ());
        restored.thaw(&mut decoder).unwrap();
        let current_context = CuContext::builder(base_context.clock.clone())
            .cl_id(99)
            .build();
        restored.process(&current_context, &mut output).unwrap();
        replay_release_tx.send(()).unwrap();
        assert_eq!(
            replay_observed_rx
                .recv_timeout(Duration::from_secs(1))
                .unwrap(),
            (7, 20)
        );
    }
}
