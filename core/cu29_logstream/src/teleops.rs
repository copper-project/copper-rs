//! Typed operator channels shared by a console and a robot.
use crate::CuStreamTxError;
use alloc::sync::Arc;
use bincode::{Decode, Encode};
use core::marker::PhantomData;
use crossbeam_queue::ArrayQueue;
use cu29_clock::Tov;
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
    type Payload: CuMsgPayload + Send + Sync + 'static;
    const ONE_PER_CL: bool = false;
    const CAPACITY: usize;
    const REPLACE: bool;
}
impl<T: CuMsgPayload + Send + Sync + 'static> Channel for Latest<T> {
    const ONE_PER_CL: bool = true;
    type Payload = T;
    const CAPACITY: usize = 1;
    const REPLACE: bool = true;
}
impl<T: CuMsgPayload + Send + Sync + 'static> Channel for Latched<T> {
    type Payload = CuLatchedStateUpdate<T>;
    const CAPACITY: usize = 1;
    const REPLACE: bool = true;
}
impl<T: CuMsgPayload + Send + Sync + 'static, const N: usize> Channel for Event<T, N> {
    type Payload = T;
    const CAPACITY: usize = N;
    const REPLACE: bool = false;
}

/// How to interpret the sample's original validity.
#[derive(Clone, Copy, Encode, Decode)]
pub enum Validity {
    Observed,
    Robot(Tov),
}

/// Console submission contract. The caller binds each selection to its observed CopperList.
pub trait Submit {
    fn submit<C: Channel>(
        &mut self,
        field: u16,
        observed: u64,
        validity: Validity,
        value: &C::Payload,
    ) -> Result<(), CuStreamTxError>;
}
/// Typed console handle borrowed for one channel.
pub struct Input<'a, S, C> {
    sender: &'a mut S,
    field: u16,
    observed: u64,
    last_sent: &'a mut Option<u64>,
    marker: PhantomData<C>,
}
impl<'a, S: Submit, C: Channel> Input<'a, S, C> {
    #[doc(hidden)]
    pub fn new(
        sender: &'a mut S,
        field: u16,
        observed: u64,
        last_sent: &'a mut Option<u64>,
    ) -> Self {
        Self {
            sender,
            field,
            observed,
            last_sent,
            marker: PhantomData,
        }
    }
    pub fn try_send(&mut self, value: &C::Payload) -> Result<(), CuStreamTxError> {
        self.send(Validity::Observed, value)
    }
    fn send(&mut self, validity: Validity, value: &C::Payload) -> Result<(), CuStreamTxError> {
        if C::ONE_PER_CL && self.last_sent.is_some_and(|cl| cl >= self.observed) {
            return Ok(());
        }
        self.sender
            .submit::<C>(self.field, self.observed, validity, value)?;
        *self.last_sent = Some(self.observed);
        Ok(())
    }
    /// Forward a message whose validity is already in robot time.
    pub fn try_send_msg(&mut self, msg: &CuMsg<C::Payload>) -> Result<(), CuStreamTxError> {
        self.try_send_mapped_msg(msg, Some)
    }
    /// Map stamped validity into robot time before submission. Unknown validity
    /// stays unknown. A failed mapping rejects the sample locally.
    pub fn try_send_mapped_msg(
        &mut self,
        msg: &CuMsg<C::Payload>,
        map: impl FnOnce(Tov) -> Option<Tov>,
    ) -> Result<(), CuStreamTxError> {
        let Some(value) = msg.payload() else {
            return Ok(());
        };
        let tov = match msg.tov {
            Tov::None => Tov::None,
            stamp => map(stamp)
                .filter(|t| *t != Tov::None)
                .ok_or(CuStreamTxError::Failed("unmappable teleops timestamp"))?,
        };
        self.send(Validity::Robot(tov), value)
    }
}
impl<S: Submit, T: CuMsgPayload + Send + Sync + 'static> Input<'_, S, Latest<T>> {
    /// Select the newest sample collected for this observed frame. Submit once
    /// at the sampling boundary. An empty batch emits nothing.
    pub fn try_send_latest<'v>(
        &mut self,
        samples: impl DoubleEndedIterator<Item = &'v T>,
    ) -> Result<(), CuStreamTxError> {
        match samples.last() {
            Some(value) => self.try_send(value),
            None => Ok(()),
        }
    }
}
impl<S: Submit, T: CuMsgPayload + Send + Sync + 'static> Input<'_, S, Latched<T>> {
    pub fn set(&mut self, value: &T) -> Result<(), CuStreamTxError> {
        self.try_send(&CuLatchedStateUpdate::Set(value.clone()))
    }
    pub fn clear(&mut self) -> Result<(), CuStreamTxError> {
        self.try_send(&CuLatchedStateUpdate::Clear)
    }
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
    last_sequence: Option<u64>,
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
                last_sequence: None,
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
    fn operator<S: Submit>(sender: S) -> Self::Operator<S>;
}
impl<S: Submit> Submit for &mut S {
    fn submit<C: Channel>(
        &mut self,
        field: u16,
        observed: u64,
        validity: Validity,
        value: &C::Payload,
    ) -> Result<(), CuStreamTxError> {
        (**self).submit::<C>(field, observed, validity, value)
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
    fn submit<C: Channel>(
        &mut self,
        field: u16,
        observed: u64,
        validity: Validity,
        value: &C::Payload,
    ) -> Result<(), CuStreamTxError> {
        let session = self.session.get().ok_or(CuStreamTxError::WouldBlock)?;
        if let Some(connect) = self.connect.take() {
            self.sender = Some(connect.connect(*session))
        }
        self.sender
            .as_mut()
            .ok_or(CuStreamTxError::Failed("missing operator sender"))?
            .submit::<C>(field, observed, validity, value)
    }
}

mod link;
pub use link::*;

/// Reason the return reader dropped an input.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rejected {
    Framing,
    Session,
    Channel,
    OutOfOrder,
    UnknownFrame,
    Overflow,
}

#[doc(hidden)]
pub trait Dispatch {
    fn dispatch(&mut self, field: u16, cl: u64, tov: Tov, bytes: &[u8]) -> Result<(), Rejected>;
}
impl<C: Channel> Publisher<C> {
    #[doc(hidden)]
    pub fn decode(&mut self, cl: u64, tov: Tov, bytes: &[u8]) -> Result<(), Rejected> {
        let config = bincode::config::standard().with_limit::<1200>();
        let (sequence, bytes) = if C::ONE_PER_CL {
            if self.last_cl.is_some_and(|last| cl <= last) {
                return Err(Rejected::OutOfOrder);
            }
            (None, bytes)
        } else {
            let (seq, n) =
                bincode::decode_from_slice(bytes, config).map_err(|_| Rejected::Framing)?;
            if self.last_sequence.is_some_and(|last| seq <= last) {
                return Err(Rejected::OutOfOrder);
            }
            (Some(seq), &bytes[n..])
        };
        let (value, n) =
            bincode::decode_from_slice(bytes, config).map_err(|_| Rejected::Framing)?;
        if n != bytes.len() {
            return Err(Rejected::Framing);
        }
        self.last_sequence = sequence;
        self.publish(cl, tov, value)
            .then_some(())
            .ok_or(Rejected::Overflow)
    }
}
