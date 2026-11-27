//! Typed operator channels shared by a console and a robot.
use crate::CuStreamTxError;
use alloc::sync::Arc;
use bincode::{Decode, Encode};
use core::marker::PhantomData;
use crossbeam_queue::ArrayQueue;
use cu29_clock::{RobotClock, Tov};
use cu29_runtime::cutask::{CuMsg, CuMsgPayload};
use cu29_runtime::payload::CuLatchedStateUpdate;
use cu29_traits::{CuError, CuResult};

/// One replaceable sample.
pub struct Latest<T>(PhantomData<T>);
/// One replaceable set or clear transition.
pub struct Latched<T>(PhantomData<T>);
/// Up to N pending samples in arrival order.
pub struct Event<T, const N: usize>(PhantomData<T>);

/// Declared delivery semantics for a schema field.
pub trait Channel {
    type Batch: Default + Encode + Decode<()> + IntoIterator<Item = Sample<Self::Payload>>;
    fn sample_count(batch: &Self::Batch) -> usize;
    type Payload: CuMsgPayload + Send + Sync + 'static;
    const ONE_PER_CL: bool = false;
    const CAPACITY: usize;
    const REPLACE: bool;
}
impl<T: CuMsgPayload + Send + Sync + 'static> Channel for Latest<T> {
    const ONE_PER_CL: bool = true;
    type Batch = Option<Sample<T>>;
    fn sample_count(batch: &Self::Batch) -> usize {
        usize::from(batch.is_some())
    }
    type Payload = T;
    const CAPACITY: usize = 1;
    const REPLACE: bool = true;
}
impl<T: CuMsgPayload + Send + Sync + 'static> Channel for Latched<T> {
    type Batch = Option<Sample<CuLatchedStateUpdate<T>>>;
    fn sample_count(batch: &Self::Batch) -> usize {
        usize::from(batch.is_some())
    }
    type Payload = CuLatchedStateUpdate<T>;
    const CAPACITY: usize = 1;
    const REPLACE: bool = true;
}
impl<T: CuMsgPayload + Send + Sync + 'static, const N: usize> Channel for Event<T, N> {
    type Batch = Events<T, N>;
    fn sample_count(batch: &Self::Batch) -> usize {
        batch.0.len()
    }
    type Payload = T;
    const CAPACITY: usize = N;
    const REPLACE: bool = false;
}

/// Captures the time of operator input before queueing or transport retries.
#[derive(Clone)]
pub struct InputSampler(Option<RobotClock>);

impl InputSampler {
    /// Supply a clock synchronized to robot time, or `None` to use the displayed CopperList's tick.
    pub fn new(synced_clock: Option<RobotClock>) -> Self {
        Self(synced_clock)
    }

    /// Sample a fresh input, such as a button press, at the point it occurs.
    pub fn sample<T>(&self, value: T) -> Sample<T> {
        Sample {
            value,
            tov: self
                .0
                .as_ref()
                .map_or(Tov::None, |clock| Tov::Time(clock.now())),
        }
    }

    /// Forward an existing message. Its original ToV is usable when the clocks are synchronized.
    pub fn from_msg<T: CuMsgPayload>(&self, mut msg: CuMsg<T>) -> Option<Sample<T>> {
        let value = msg.payload_mut().take()?;
        let tov = if self.0.is_some() { msg.tov } else { Tov::None };
        Some(Sample { value, tov })
    }
}

/// One captured input. An absent ToV tells the robot to use the observed CopperList's tick.
#[derive(Encode, Decode)]
pub struct Sample<T> {
    pub value: T,
    tov: Tov,
}
impl<T> Sample<T> {
    pub fn tov(&self) -> Tov {
        self.tov
    }
}

/// Events selected for this update, in arrival order.
#[derive(Default)]
pub struct Events<T, const N: usize>(arrayvec::ArrayVec<Sample<T>, N>);
impl<T, const N: usize> Events<T, N> {
    /// Append a sample, returning it if this update is already full.
    pub fn try_push(&mut self, sample: Sample<T>) -> Result<(), Sample<T>> {
        self.0.try_push(sample).map_err(|e| e.element())
    }
}
impl<T, const N: usize> IntoIterator for Events<T, N> {
    type Item = Sample<T>;
    type IntoIter = arrayvec::IntoIter<Sample<T>, N>;
    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}
impl<T: Encode, const N: usize> Encode for Events<T, N> {
    fn encode<E: bincode::enc::Encoder>(
        &self,
        encoder: &mut E,
    ) -> Result<(), bincode::error::EncodeError> {
        self.0.as_slice().encode(encoder)
    }
}
impl<T: Decode<()>, const N: usize> Decode<()> for Events<T, N> {
    fn decode<D: bincode::de::Decoder<Context = ()>>(
        decoder: &mut D,
    ) -> Result<Self, bincode::error::DecodeError> {
        let count = usize::decode(decoder)?;
        if count > N {
            return Err(bincode::error::DecodeError::Other(
                "teleops event capacity exceeded",
            ));
        }
        let mut items = arrayvec::ArrayVec::new();
        for _ in 0..count {
            items.push(Sample::decode(decoder)?);
        }
        Ok(Self(items))
    }
}

/// Submit all selected inputs together. `observed` identifies the displayed robot CopperList.
pub trait Submit {
    fn submit<U: Encode>(&mut self, observed: u64, update: &U) -> Result<(), CuStreamTxError>;
}

/// Bounded source-side resource. A source moves one admitted value into the graph per iteration.
pub struct Inbox<T>(Arc<ArrayQueue<(u64, Tov, T)>>, Option<fn(u64, u64)>);
impl<T> Clone for Inbox<T> {
    fn clone(&self) -> Self {
        Self(self.0.clone(), self.1)
    }
}
impl<T> Inbox<T> {
    /// Install a bounded, nonblocking recorder before moving this resource to its source.
    pub fn on_emit(mut self, record: fn(u64, u64)) -> Self {
        self.1 = Some(record);
        self
    }
    #[doc(hidden)]
    pub fn record_emission(&self, id: u64, list: u64) {
        if let Some(record) = self.1 {
            record(id, list)
        }
    }

    pub fn take(&self, current_cl: u64, max_lag_cl: u64) -> Option<(u64, Tov, T)> {
        self.0.pop().filter(|(observed, _, _)| {
            current_cl
                .checked_sub(*observed)
                .is_some_and(|age| age <= max_lag_cl)
        })
    }
}
/// Single producer for a typed inbox.
pub struct Publisher<C: Channel> {
    last_cl: Option<u64>,
    inbox: Inbox<C::Payload>,
}
impl<C: Channel> Publisher<C> {
    pub fn pair() -> CuResult<(Self, Inbox<C::Payload>)> {
        if C::CAPACITY == 0 {
            return Err(CuError::from("teleops event capacity must be positive"));
        }
        let inbox = Inbox(Arc::new(ArrayQueue::new(C::CAPACITY)), None);
        Ok((
            Self {
                last_cl: None,
                inbox: inbox.clone(),
            },
            inbox,
        ))
    }
    pub fn publish(&mut self, id: u64, tov: Tov, value: C::Payload) -> bool {
        if C::ONE_PER_CL && self.last_cl.is_some_and(|previous| id <= previous) {
            return false;
        }
        self.last_cl = Some(id);
        let sample = (id, tov, value);
        if C::REPLACE {
            self.inbox.0.force_push(sample);
            true
        } else {
            self.inbox.0.push(sample).is_ok()
        }
    }
}

/// Application schema implemented by derive(Teleops).
pub trait Teleops {
    type Operator<S: Submit>;
    fn operator<S: Submit>(sender: S, synced_clock: Option<RobotClock>) -> Self::Operator<S>;
}
impl<S: Submit> Submit for &mut S {
    fn submit<U: Encode>(&mut self, observed: u64, update: &U) -> Result<(), CuStreamTxError> {
        (**self).submit(observed, update)
    }
}

/// Construct a console sender after the twin validates the session header.
#[cfg(feature = "std")]
pub trait Connect {
    type Sender: Submit;
    fn connect(self, session: [u8; 16]) -> Self::Sender;
}
#[cfg(feature = "std")]
impl<F: FnOnce([u8; 16]) -> S, S: Submit> Connect for F {
    type Sender = S;
    fn connect(self, session: [u8; 16]) -> S {
        self(session)
    }
}
#[cfg(feature = "std")]
pub(crate) struct TwinSubmit<C: Connect> {
    connect: Option<C>,
    sender: Option<C::Sender>,
    session: Arc<std::sync::OnceLock<[u8; 16]>>,
}
#[cfg(feature = "std")]
impl<C: Connect> TwinSubmit<C> {
    pub(crate) fn new(connect: C, session: Arc<std::sync::OnceLock<[u8; 16]>>) -> Self {
        Self {
            connect: Some(connect),
            session,
            sender: None,
        }
    }
}
#[cfg(feature = "std")]
impl<F: Connect> Submit for TwinSubmit<F> {
    fn submit<U: Encode>(&mut self, observed: u64, update: &U) -> Result<(), CuStreamTxError> {
        let session = self.session.get().ok_or(CuStreamTxError::WouldBlock)?;
        if let Some(connect) = self.connect.take() {
            self.sender = Some(connect.connect(*session))
        }
        self.sender
            .as_mut()
            .ok_or(CuStreamTxError::Failed("missing operator sender"))?
            .submit(observed, update)
    }
}
