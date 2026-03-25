use crate::config::ComponentConfig;
use crate::context::CuContext;
#[cfg(test)]
use crate::cutask::BincodeAdapter;
use crate::cutask::{CuMsg, CuMsgPayload, CuTask, Freezable};
use crate::reflect::{Reflect, TypePath};
use bincode::de::Decoder;
#[cfg(test)]
use bincode::de::{DecoderImpl, read::SliceReader};
use bincode::enc::Encoder;
#[cfg(test)]
use bincode::enc::{EncoderImpl, write::SizeWriter, write::SliceWriter};
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29_clock::{CuDuration, CuTime, Tov};
use cu29_traits::{CuError, CuResult};
use rayon::ThreadPool;
use std::sync::{Arc, Mutex, MutexGuard};

fn lock_or_recover<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(|poison| poison.into_inner())
}

#[cfg(test)]
fn encode_freezable(task: &impl Freezable, bytes: &mut Vec<u8>) -> Result<(), EncodeError> {
    encode_value(&BincodeAdapter(task), bytes)
}

#[cfg(test)]
fn encode_value(value: &impl Encode, bytes: &mut Vec<u8>) -> Result<(), EncodeError> {
    let config = bincode::config::standard();
    let mut sizer = EncoderImpl::<_, _>::new(SizeWriter::default(), config);
    value.encode(&mut sizer)?;
    let required = sizer.into_writer().bytes_written as usize;

    bytes.clear();
    bytes.resize(required, 0);
    let mut encoder = EncoderImpl::<_, _>::new(SliceWriter::new(bytes.as_mut_slice()), config);
    value.encode(&mut encoder)?;
    Ok(())
}

#[cfg(test)]
fn thaw_freezable(task: &mut impl Freezable, bytes: &[u8]) -> Result<(), DecodeError> {
    let config = bincode::config::standard();
    let reader = SliceReader::new(bytes);
    let mut decoder = DecoderImpl::new(reader, config, ());
    task.thaw(&mut decoder)
}

fn mutex_poison_encode_error(name: &str) -> EncodeError {
    EncodeError::OtherString(format!("{name} mutex poisoned"))
}

fn mutex_poison_decode_error(name: &str) -> DecodeError {
    DecodeError::OtherString(format!("{name} mutex poisoned"))
}

#[derive(Clone, Debug, Encode, Decode)]
struct FrozenCuError {
    text: String,
}

impl From<&CuError> for FrozenCuError {
    fn from(error: &CuError) -> Self {
        Self {
            text: error.to_string(),
        }
    }
}

impl From<FrozenCuError> for CuError {
    fn from(error: FrozenCuError) -> Self {
        CuError::from(error.text)
    }
}

struct AsyncState<O, P>
where
    O: CuMsgPayload,
    P: Default,
{
    inflight: bool,
    keyframe_gap: bool,
    ready_output: CuMsg<O>,
    ready_at: Option<CuTime>,
    last_error: Option<CuError>,
    policy: P,
}

impl<O, P> Default for AsyncState<O, P>
where
    O: CuMsgPayload,
    P: Default,
{
    fn default() -> Self {
        Self {
            inflight: false,
            keyframe_gap: false,
            ready_output: CuMsg::default(),
            ready_at: None,
            last_error: None,
            policy: P::default(),
        }
    }
}

impl<O, P> AsyncState<O, P>
where
    O: CuMsgPayload,
    P: Default,
{
    fn blocks_new_launches(&self, now: CuTime) -> bool {
        self.inflight || self.ready_at.is_some_and(|ready_at| now < ready_at)
    }

    fn take_ready_output(&mut self) -> CuMsg<O> {
        self.ready_at = None;
        std::mem::take(&mut self.ready_output)
    }
}

fn background_keyframe_gap_error() -> CuError {
    CuError::from(
        "Background task state unavailable: keyframe captured while the task was in flight",
    )
}

const ASYNC_STATE_QUIESCENT: u8 = 0;
const ASYNC_STATE_UNAVAILABLE: u8 = 1;

trait AsyncPolicy<I>: Clone + Default + Send + 'static
where
    I: CuMsgPayload + Send + Sync + 'static,
{
    fn encode_state<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError>;
    fn decode_state<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError>
    where
        Self: Sized;
    fn observe_blocked_input(&mut self, input: &CuMsg<I>, now: CuTime);
    fn decide_launch(&mut self, input: &CuMsg<I>, now: CuTime) -> LaunchDecision<I>;
}

enum LaunchDecision<I>
where
    I: CuMsgPayload,
{
    Launch(CuMsg<I>),
    Hold,
}

impl<I> AsyncPolicy<I> for ()
where
    I: CuMsgPayload + Send + Sync + 'static,
{
    fn encode_state<E: Encoder>(&self, _encoder: &mut E) -> Result<(), EncodeError> {
        Ok(())
    }

    fn decode_state<D: Decoder<Context = ()>>(_decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(())
    }

    fn observe_blocked_input(&mut self, _input: &CuMsg<I>, _now: CuTime) {}

    fn decide_launch(&mut self, input: &CuMsg<I>, _now: CuTime) -> LaunchDecision<I> {
        LaunchDecision::Launch(input.clone())
    }
}

impl<I> AsyncPolicy<I> for Option<CuMsg<I>>
where
    I: CuMsgPayload + Send + Sync + 'static,
{
    fn encode_state<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.encode(encoder)
    }

    fn decode_state<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Option::<CuMsg<I>>::decode(decoder)
    }

    fn observe_blocked_input(&mut self, input: &CuMsg<I>, _now: CuTime) {
        *self = Some(input.clone());
    }

    fn decide_launch(&mut self, input: &CuMsg<I>, _now: CuTime) -> LaunchDecision<I> {
        if let Some(buffered) = self.take() {
            *self = Some(input.clone());
            LaunchDecision::Launch(buffered)
        } else {
            LaunchDecision::Launch(input.clone())
        }
    }
}

#[derive(Default, Copy, Clone, Debug, Encode, Decode)]
struct ArrivalPredictor {
    last_observed: Option<CuTime>,
    interval: Option<CuDuration>,
}

impl ArrivalPredictor {
    fn observe<I>(&mut self, msg: &CuMsg<I>, now: CuTime)
    where
        I: CuMsgPayload,
    {
        let observed = observed_time(msg, now);
        if let Some(previous) = self.last_observed {
            if observed < previous {
                return;
            }
            self.interval = Some(observed - previous);
        }
        self.last_observed = Some(observed);
    }

    fn projected_next(&self) -> Option<CuTime> {
        self.last_observed
            .zip(self.interval)
            .map(|(last, interval)| last + interval)
    }

    fn should_wait_for<I>(&self, pending: &CuMsg<I>, now: CuTime) -> bool
    where
        I: CuMsgPayload,
    {
        let Some(projected_next) = self.projected_next() else {
            return false;
        };
        if projected_next <= now {
            return false;
        }

        time_distance(projected_next, now) < msg_distance_to_now(pending, now)
    }
}

#[derive(Clone, Debug, Encode, Decode)]
struct ClosestPolicyState<I>
where
    I: CuMsgPayload,
{
    pending: Option<CuMsg<I>>,
    predictor: ArrivalPredictor,
}

impl<I> Default for ClosestPolicyState<I>
where
    I: CuMsgPayload,
{
    fn default() -> Self {
        Self {
            pending: None,
            predictor: ArrivalPredictor::default(),
        }
    }
}

impl<I> AsyncPolicy<I> for ClosestPolicyState<I>
where
    I: CuMsgPayload + Send + Sync + 'static,
{
    fn encode_state<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.pending.encode(encoder)?;
        self.predictor.last_observed.encode(encoder)?;
        self.predictor.interval.encode(encoder)?;
        Ok(())
    }

    fn decode_state<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            pending: Option::<CuMsg<I>>::decode(decoder)?,
            predictor: ArrivalPredictor {
                last_observed: Option::<CuTime>::decode(decoder)?,
                interval: Option::<CuDuration>::decode(decoder)?,
            },
        })
    }

    fn observe_blocked_input(&mut self, input: &CuMsg<I>, now: CuTime) {
        self.predictor.observe(input, now);
        self.pending = Some(input.clone());
    }

    fn decide_launch(&mut self, input: &CuMsg<I>, now: CuTime) -> LaunchDecision<I> {
        self.predictor.observe(input, now);

        let Some(buffered) = self.pending.take() else {
            return LaunchDecision::Launch(input.clone());
        };

        if msg_distance_to_now(input, now) < msg_distance_to_now(&buffered, now) {
            return LaunchDecision::Launch(input.clone());
        }

        if self.predictor.should_wait_for(&buffered, now) {
            self.pending = Some(buffered);
            return LaunchDecision::Hold;
        }

        self.pending = Some(input.clone());
        LaunchDecision::Launch(buffered)
    }
}

fn metadata_time<T>(msg: &CuMsg<T>) -> Option<CuTime>
where
    T: CuMsgPayload,
{
    let end: Option<CuTime> = msg.metadata.process_time.end.into();
    if end.is_some() {
        return end;
    }
    msg.metadata.process_time.start.into()
}

fn observed_time<T>(msg: &CuMsg<T>, fallback: CuTime) -> CuTime
where
    T: CuMsgPayload,
{
    match msg.tov {
        Tov::Time(time) => time,
        Tov::Range(range) => range.end,
        Tov::None => metadata_time(msg).unwrap_or(fallback),
    }
}

fn time_distance(lhs: CuTime, rhs: CuTime) -> CuDuration {
    if lhs >= rhs { lhs - rhs } else { rhs - lhs }
}

fn msg_distance_to_now<T>(msg: &CuMsg<T>, now: CuTime) -> CuDuration
where
    T: CuMsgPayload,
{
    match msg.tov {
        Tov::Time(time) => time_distance(time, now),
        Tov::Range(range) => {
            if now < range.start {
                range.start - now
            } else if now > range.end {
                now - range.end
            } else {
                CuDuration::default()
            }
        }
        Tov::None => time_distance(observed_time(msg, now), now),
    }
}

struct AsyncTaskCore<T, I, O, P>
where
    T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
    I: CuMsgPayload + Send + Sync + 'static,
    O: CuMsgPayload + Send + 'static,
    P: AsyncPolicy<I>,
{
    task: Arc<Mutex<T>>,
    state: Arc<Mutex<AsyncState<O, P>>>,
    tp: Arc<ThreadPool>,
}

impl<T, I, O, P> AsyncTaskCore<T, I, O, P>
where
    T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
    I: CuMsgPayload + Send + Sync + 'static,
    O: CuMsgPayload + Send + 'static,
    P: AsyncPolicy<I>,
{
    fn new(
        config: Option<&ComponentConfig>,
        resources: T::Resources<'_>,
        tp: Arc<ThreadPool>,
    ) -> CuResult<Self> {
        Ok(Self {
            task: Arc::new(Mutex::new(T::new(config, resources)?)),
            state: Arc::new(Mutex::new(AsyncState::default())),
            tp,
        })
    }

    fn spawn_worker(&self, dispatch_ctx: CuContext, worker_input: CuMsg<I>) {
        {
            let mut state = lock_or_recover(&self.state);
            state.inflight = true;
        }

        self.tp.spawn_fifo({
            let task = self.task.clone();
            let state = self.state.clone();
            move || {
                let mut output = CuMsg::<O>::default();
                if output.metadata.process_time.start.is_none() {
                    output.metadata.process_time.start = dispatch_ctx.now().into();
                }

                let task_result = match task.lock() {
                    Ok(mut task_guard) => {
                        task_guard.process(&dispatch_ctx, &worker_input, &mut output)
                    }
                    Err(poison) => Err(CuError::from(format!(
                        "Async task mutex poisoned: {poison}"
                    ))),
                };

                let mut guard = lock_or_recover(&state);
                guard.inflight = false;

                match task_result {
                    Ok(()) => {
                        let end_from_metadata: Option<CuTime> =
                            output.metadata.process_time.end.into();
                        let ready_at = end_from_metadata.unwrap_or_else(|| {
                            let now = dispatch_ctx.now();
                            output.metadata.process_time.end = now.into();
                            now
                        });
                        guard.ready_output = output;
                        guard.ready_at = Some(ready_at);
                        guard.last_error = None;
                    }
                    Err(error) => {
                        guard.ready_output = CuMsg::default();
                        guard.ready_at = None;
                        guard.last_error = Some(error);
                    }
                }
            }
        });
    }

    fn process(
        &self,
        ctx: &CuContext,
        input: &CuMsg<I>,
        real_output: &mut CuMsg<O>,
    ) -> CuResult<()> {
        let launch = {
            let mut state = self.state.lock().map_err(|_| {
                CuError::from("Async task state mutex poisoned while scheduling background work")
            })?;

            if state.keyframe_gap {
                return Err(background_keyframe_gap_error());
            }

            if let Some(error) = state.last_error.take() {
                return Err(error);
            }

            if state.blocks_new_launches(ctx.now()) {
                state.policy.observe_blocked_input(input, ctx.now());
                *real_output = CuMsg::default();
                None
            } else {
                *real_output = state.take_ready_output();
                match state.policy.decide_launch(input, ctx.now()) {
                    LaunchDecision::Launch(next_input) => Some(next_input),
                    LaunchDecision::Hold => None,
                }
            }
        };

        let Some(next_input) = launch else {
            return Ok(());
        };

        self.spawn_worker(ctx.clone(), next_input);
        Ok(())
    }

    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let state = self
            .state
            .lock()
            .map_err(|_| mutex_poison_encode_error("Async task state"))?;

        if state.inflight || state.keyframe_gap {
            ASYNC_STATE_UNAVAILABLE.encode(encoder)?;
        } else {
            ASYNC_STATE_QUIESCENT.encode(encoder)?;
            state.ready_output.encode(encoder)?;
            state.ready_at.encode(encoder)?;
            match state.last_error.as_ref() {
                Some(error) => {
                    true.encode(encoder)?;
                    FrozenCuError::from(error).text.encode(encoder)?;
                }
                None => false.encode(encoder)?,
            }
            state.policy.encode_state(encoder)?;
            let task = self
                .task
                .lock()
                .map_err(|_| mutex_poison_encode_error("Async task"))?;
            task.freeze(encoder)?;
        }

        Ok(())
    }

    fn thaw<D: Decoder>(&self, decoder: &mut D) -> Result<(), DecodeError> {
        let mut decoder = decoder.with_context(());
        let tag = u8::decode(&mut decoder)?;

        let mut state = self
            .state
            .lock()
            .map_err(|_| mutex_poison_decode_error("Async task state"))?;

        match tag {
            ASYNC_STATE_QUIESCENT => {
                let ready_output = CuMsg::<O>::decode(&mut decoder)?;
                let ready_at = Option::<CuTime>::decode(&mut decoder)?;
                let has_error = bool::decode(&mut decoder)?;
                let last_error = if has_error {
                    Some(CuError::from(String::decode(&mut decoder)?))
                } else {
                    None
                };
                let policy = P::decode_state(&mut decoder)?;
                let mut task = self
                    .task
                    .lock()
                    .map_err(|_| mutex_poison_decode_error("Async task"))?;
                task.thaw(&mut decoder)?;
                state.inflight = false;
                state.keyframe_gap = false;
                state.ready_output = ready_output;
                state.ready_at = ready_at;
                state.last_error = last_error;
                state.policy = policy;
            }
            ASYNC_STATE_UNAVAILABLE => {
                state.inflight = false;
                state.keyframe_gap = true;
                state.ready_output = CuMsg::default();
                state.ready_at = None;
                state.last_error = None;
                state.policy = P::default();
            }
            _ => {
                return Err(DecodeError::OtherString(
                    "Unknown async task state tag".into(),
                ));
            }
        }
        Ok(())
    }
}

/// Resource bundle required by a backgrounded task.
pub struct CuAsyncTaskResources<'r, T: CuTask> {
    pub inner: T::Resources<'r>,
    pub threadpool: Arc<ThreadPool>,
}

#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuAsyncTask<T, I, O>
where
    T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
    I: CuMsgPayload + Send + Sync + 'static,
    O: CuMsgPayload + Send + 'static,
{
    #[reflect(ignore)]
    core: AsyncTaskCore<T, I, O, ()>,
}

#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuAsyncTaskPrevious<T, I, O>
where
    T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
    I: CuMsgPayload + Send + Sync + 'static,
    O: CuMsgPayload + Send + 'static,
{
    #[reflect(ignore)]
    core: AsyncTaskCore<T, I, O, Option<CuMsg<I>>>,
}

#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuAsyncTaskClosest<T, I, O>
where
    T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
    I: CuMsgPayload + Send + Sync + 'static,
    O: CuMsgPayload + Send + 'static,
{
    #[reflect(ignore)]
    core: AsyncTaskCore<T, I, O, ClosestPolicyState<I>>,
}

macro_rules! impl_async_type_path {
    ($name:ident, $path:literal, $short:literal) => {
        impl<T, I, O> TypePath for $name<T, I, O>
        where
            T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
            I: CuMsgPayload + Send + Sync + 'static,
            O: CuMsgPayload + Send + 'static,
        {
            fn type_path() -> &'static str {
                $path
            }

            fn short_type_path() -> &'static str {
                $short
            }

            fn type_ident() -> Option<&'static str> {
                Some($short)
            }

            fn crate_name() -> Option<&'static str> {
                Some("cu29_runtime")
            }

            fn module_path() -> Option<&'static str> {
                Some("cuasynctask")
            }
        }
    };
}

impl_async_type_path!(
    CuAsyncTask,
    "cu29_runtime::cuasynctask::CuAsyncTask",
    "CuAsyncTask"
);
impl_async_type_path!(
    CuAsyncTaskPrevious,
    "cu29_runtime::cuasynctask::CuAsyncTaskPrevious",
    "CuAsyncTaskPrevious"
);
impl_async_type_path!(
    CuAsyncTaskClosest,
    "cu29_runtime::cuasynctask::CuAsyncTaskClosest",
    "CuAsyncTaskClosest"
);

macro_rules! impl_async_wrapper {
    ($name:ident, $policy:ty) => {
        impl<T, I, O> $name<T, I, O>
        where
            T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
            I: CuMsgPayload + Send + Sync + 'static,
            O: CuMsgPayload + Send + 'static,
        {
            #[allow(unused)]
            pub fn new(
                config: Option<&ComponentConfig>,
                resources: T::Resources<'_>,
                tp: Arc<ThreadPool>,
            ) -> CuResult<Self> {
                Ok(Self {
                    core: AsyncTaskCore::<T, I, O, $policy>::new(config, resources, tp)?,
                })
            }
        }

        impl<T, I, O> Freezable for $name<T, I, O>
        where
            T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
            I: CuMsgPayload + Send + Sync + 'static,
            O: CuMsgPayload + Send + 'static,
        {
            fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                self.core.freeze(encoder)
            }

            fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
                self.core.thaw(decoder)
            }
        }

        impl<T, I, O> CuTask for $name<T, I, O>
        where
            T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
            I: CuMsgPayload + Send + Sync + 'static,
            O: CuMsgPayload + Send + 'static,
        {
            type Resources<'r> = CuAsyncTaskResources<'r, T>;
            type Input<'m> = T::Input<'m>;
            type Output<'m> = T::Output<'m>;

            fn new(
                config: Option<&ComponentConfig>,
                resources: Self::Resources<'_>,
            ) -> CuResult<Self>
            where
                Self: Sized,
            {
                Ok(Self {
                    core: AsyncTaskCore::<T, I, O, $policy>::new(
                        config,
                        resources.inner,
                        resources.threadpool,
                    )?,
                })
            }

            fn process<'i, 'o>(
                &mut self,
                ctx: &CuContext,
                input: &Self::Input<'i>,
                real_output: &mut Self::Output<'o>,
            ) -> CuResult<()> {
                self.core.process(ctx, input, real_output)
            }
        }
    };
}

impl_async_wrapper!(CuAsyncTask, ());
impl_async_wrapper!(CuAsyncTaskPrevious, Option<CuMsg<I>>);
impl_async_wrapper!(CuAsyncTaskClosest, ClosestPolicyState<I>);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::ComponentConfig;
    use crate::cutask::CuMsg;
    use crate::input_msg;
    use crate::output_msg;
    use cu29_clock::CuTimeRange;
    use rayon::ThreadPoolBuilder;
    use std::borrow::BorrowMut;
    use std::sync::OnceLock;
    use std::sync::mpsc;
    use std::time::Duration;

    static READY_RX: OnceLock<Arc<Mutex<mpsc::Receiver<CuTime>>>> = OnceLock::new();
    static DONE_TX: OnceLock<mpsc::Sender<()>> = OnceLock::new();

    fn freeze_bytes(task: &impl Freezable) -> Vec<u8> {
        let mut bytes = Vec::new();
        encode_freezable(task, &mut bytes).expect("failed to encode task");
        bytes
    }

    fn thaw_bytes(task: &mut impl Freezable, bytes: &[u8]) {
        thaw_freezable(task, bytes).expect("failed to thaw task");
    }

    fn wait_until_state_idle<O, P>(state: &Arc<Mutex<AsyncState<O, P>>>)
    where
        O: CuMsgPayload + Send + 'static,
        P: Default,
    {
        for _ in 0..100 {
            let guard = lock_or_recover(state);
            if !guard.inflight {
                return;
            }
            drop(guard);
            std::thread::sleep(Duration::from_millis(1));
        }
        panic!("background task never became idle");
    }

    fn wait_until_async_idle<T, I, O>(async_task: &CuAsyncTask<T, I, O>)
    where
        T: for<'i, 'o> CuTask<Input<'i> = CuMsg<I>, Output<'o> = CuMsg<O>> + Send + 'static,
        I: CuMsgPayload + Send + Sync + 'static,
        O: CuMsgPayload + Send + 'static,
    {
        wait_until_state_idle(&async_task.core.state);
    }

    #[derive(Reflect)]
    struct TestTask;

    impl Freezable for TestTask {}

    impl CuTask for TestTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);

        fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self)
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
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let config = ComponentConfig::default();
        let context = CuContext::new_with_clock();
        let mut async_task: CuAsyncTask<TestTask, u32, u32> =
            CuAsyncTask::new(Some(&config), (), tp).unwrap();
        let input = CuMsg::new(Some(42u32));
        let mut output = CuMsg::new(None);

        loop {
            async_task.process(&context, &input, &mut output).unwrap();
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
            Ok(Self)
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

    #[derive(Clone)]
    struct BlockingTaskResources {
        release_rx: Arc<Mutex<mpsc::Receiver<()>>>,
        done_tx: mpsc::Sender<()>,
    }

    #[derive(Reflect)]
    #[reflect(no_field_bounds, from_reflect = false)]
    struct BlockingStampTask {
        seq: u32,
        #[reflect(ignore)]
        release_rx: Arc<Mutex<mpsc::Receiver<()>>>,
        #[reflect(ignore)]
        done_tx: mpsc::Sender<()>,
    }

    impl Freezable for BlockingStampTask {
        fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
            self.seq.encode(encoder)
        }

        fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
            self.seq = u32::decode(decoder)?;
            Ok(())
        }
    }

    impl CuTask for BlockingStampTask {
        type Resources<'r> = BlockingTaskResources;
        type Input<'m> = input_msg!(u32);
        type Output<'m> = output_msg!(u32);

        fn new(_config: Option<&ComponentConfig>, resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                seq: 0,
                release_rx: resources.release_rx,
                done_tx: resources.done_tx,
            })
        }

        fn process(
            &mut self,
            ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            self.release_rx
                .lock()
                .unwrap()
                .recv_timeout(Duration::from_secs(1))
                .expect("timed out waiting for release");

            let payload = input.payload().copied().unwrap_or_default();
            output.metadata.process_time.start = ctx.now().into();
            output.metadata.process_time.end = ctx.now().into();
            output.set_payload(payload + self.seq);
            self.seq = self.seq.saturating_add(1);
            let _ = self.done_tx.send(());
            Ok(())
        }
    }

    fn blocking_resources() -> (BlockingTaskResources, mpsc::Sender<()>, mpsc::Receiver<()>) {
        let (release_tx, release_rx) = mpsc::channel();
        let (done_tx, done_rx) = mpsc::channel();
        (
            BlockingTaskResources {
                release_rx: Arc::new(Mutex::new(release_rx)),
                done_tx,
            },
            release_tx,
            done_rx,
        )
    }

    fn msg_with_tov(payload: u32, tov_ns: u64) -> CuMsg<u32> {
        let mut msg = CuMsg::new(Some(payload));
        msg.tov = Tov::Time(CuTime::from(tov_ns));
        msg
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

        let mut async_task: CuAsyncTask<ActionTask, u32, u32> =
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

        let mut async_task: CuAsyncTask<ActionTask, u32, u32> =
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
    fn background_respects_recorded_ready_time() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (context, clock_mock) = CuContext::new_mock_clock();

        let (ready_tx, ready_rx) = mpsc::channel::<CuTime>();
        let (done_tx, done_rx) = mpsc::channel::<()>();
        READY_RX
            .set(Arc::new(Mutex::new(ready_rx)))
            .expect("ready channel already set");
        DONE_TX
            .set(done_tx)
            .expect("completion channel already set");

        let mut async_task: CuAsyncTask<ControlledTask, u32, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), (), tp.clone()).unwrap();
        let input = CuMsg::new(Some(1u32));
        let mut output = CuMsg::new(None);

        clock_mock.set_value(0);
        async_task.process(&context, &input, &mut output).unwrap();
        assert!(output.payload().is_none());

        clock_mock.set_value(10);
        async_task.process(&context, &input, &mut output).unwrap();
        assert!(output.payload().is_none());

        clock_mock.set_value(30);
        ready_tx.send(CuTime::from(30u64)).unwrap();
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("background task never finished");
        wait_until_async_idle(&async_task);

        clock_mock.set_value(20);
        async_task.process(&context, &input, &mut output).unwrap();
        assert!(
            output.payload().is_none(),
            "Output surfaced before recorded ready time"
        );

        clock_mock.set_value(30);
        async_task.process(&context, &input, &mut output).unwrap();
        assert_eq!(output.payload(), Some(&30u32));

        ready_tx.send(CuTime::from(40u64)).unwrap();
        let _ = done_rx.recv_timeout(Duration::from_secs(1));
    }

    #[test]
    fn background_inflight_keyframe_restores_gap_marker() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (context, clock) = CuContext::new_mock_clock();
        let (resources, release_tx, done_rx) = blocking_resources();
        let mut original: CuAsyncTask<BlockingStampTask, u32, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), resources, tp.clone()).unwrap();
        let mut output = CuMsg::default();

        clock.set_value(10);
        original
            .process(&context, &CuMsg::new(Some(1u32)), &mut output)
            .unwrap();
        assert!(output.payload().is_none());

        let frozen = freeze_bytes(&original);

        release_tx.send(()).unwrap();
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("original worker never finished");

        let (replay_context, replay_clock) = CuContext::new_mock_clock();
        let (replay_resources, _replay_release_tx, _replay_done_rx) = blocking_resources();
        let mut replayed: CuAsyncTask<BlockingStampTask, u32, u32> =
            CuAsyncTask::new(Some(&ComponentConfig::default()), replay_resources, tp).unwrap();
        thaw_bytes(&mut replayed, &frozen);

        assert!(lock_or_recover(&replayed.core.state).keyframe_gap);

        replay_clock.set_value(20);
        let error = replayed
            .process(&replay_context, &CuMsg::new(Some(2u32)), &mut output)
            .expect_err("partial background keyframe should not resume transparently");
        assert!(
            error
                .to_string()
                .contains("Background task state unavailable"),
            "unexpected error: {error}"
        );
    }

    #[test]
    fn previous_policy_restores_pending_input_from_keyframe() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (context, clock) = CuContext::new_mock_clock();
        let (resources, release_tx, done_rx) = blocking_resources();
        let mut original: CuAsyncTaskPrevious<BlockingStampTask, u32, u32> =
            CuAsyncTaskPrevious::new(Some(&ComponentConfig::default()), resources, tp.clone())
                .unwrap();
        let mut output = CuMsg::default();

        clock.set_value(10);
        original
            .process(&context, &CuMsg::new(Some(1u32)), &mut output)
            .unwrap();

        clock.set_value(11);
        original
            .process(&context, &CuMsg::new(Some(2u32)), &mut output)
            .unwrap();

        release_tx.send(()).unwrap();
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("worker never finished");
        wait_until_state_idle(&original.core.state);

        let frozen = freeze_bytes(&original);

        let (replay_context, replay_clock) = CuContext::new_mock_clock();
        let (replay_resources, replay_release_tx, replay_done_rx) = blocking_resources();
        let mut replayed: CuAsyncTaskPrevious<BlockingStampTask, u32, u32> =
            CuAsyncTaskPrevious::new(
                Some(&ComponentConfig::default()),
                replay_resources,
                tp.clone(),
            )
            .unwrap();
        thaw_bytes(&mut replayed, &frozen);

        replay_clock.set_value(15);
        replayed
            .process(&replay_context, &CuMsg::new(Some(3u32)), &mut output)
            .unwrap();
        assert_eq!(output.payload(), Some(&1u32));

        replay_release_tx.send(()).unwrap();
        replay_done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("second worker never finished");
        wait_until_state_idle(&replayed.core.state);

        replay_clock.set_value(20);
        replayed
            .process(&replay_context, &CuMsg::new(Some(4u32)), &mut output)
            .unwrap();
        assert_eq!(output.payload(), Some(&3u32));

        replay_release_tx.send(()).unwrap();
        replay_done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("cleanup worker never finished");
    }

    #[test]
    fn closest_policy_waits_for_predicted_fresher_message_after_restore() {
        let tp = Arc::new(ThreadPoolBuilder::new().num_threads(1).build().unwrap());
        let (context, clock) = CuContext::new_mock_clock();
        let (resources, release_tx, done_rx) = blocking_resources();
        let mut original: CuAsyncTaskClosest<BlockingStampTask, u32, u32> =
            CuAsyncTaskClosest::new(Some(&ComponentConfig::default()), resources, tp.clone())
                .unwrap();
        let mut output = CuMsg::default();

        clock.set_value(0);
        original
            .process(&context, &CuMsg::new(Some(1u32)), &mut output)
            .unwrap();

        clock.set_value(100);
        original
            .process(&context, &msg_with_tov(2, 100), &mut output)
            .unwrap();

        clock.set_value(200);
        original
            .process(&context, &msg_with_tov(3, 200), &mut output)
            .unwrap();

        release_tx.send(()).unwrap();
        done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("worker never finished");
        wait_until_state_idle(&original.core.state);

        let frozen = freeze_bytes(&original);

        let (replay_context, replay_clock) = CuContext::new_mock_clock();
        let (replay_resources, replay_release_tx, replay_done_rx) = blocking_resources();
        let mut replayed: CuAsyncTaskClosest<BlockingStampTask, u32, u32> =
            CuAsyncTaskClosest::new(
                Some(&ComponentConfig::default()),
                replay_resources,
                tp.clone(),
            )
            .unwrap();
        thaw_bytes(&mut replayed, &frozen);

        replay_clock.set_value(290);
        replayed
            .process(&replay_context, &msg_with_tov(9, 150), &mut output)
            .unwrap();
        assert_eq!(output.payload(), Some(&1u32));

        replay_clock.set_value(300);
        replayed
            .process(&replay_context, &msg_with_tov(4, 300), &mut output)
            .unwrap();
        assert!(output.payload().is_none());

        replay_release_tx.send(()).unwrap();
        replay_done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("closest worker never finished");
        wait_until_state_idle(&replayed.core.state);

        replay_clock.set_value(305);
        replayed
            .process(&replay_context, &msg_with_tov(5, 305), &mut output)
            .unwrap();
        assert_eq!(output.payload(), Some(&5u32));

        replay_release_tx.send(()).unwrap();
        replay_done_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("cleanup closest worker never finished");
    }

    #[test]
    fn closest_distance_uses_tov_ranges() {
        let now = CuTime::from(250u64);
        let mut msg = CuMsg::new(Some(1u32));
        msg.tov = Tov::Range(CuTimeRange {
            start: CuTime::from(200u64),
            end: CuTime::from(300u64),
        });
        assert_eq!(msg_distance_to_now(&msg, now), CuDuration::default());
    }
}
