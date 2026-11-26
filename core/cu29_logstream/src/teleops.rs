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
    const CAPACITY: usize;
    const REPLACE: bool;
}
impl<T: CuMsgPayload + Send + Sync + 'static> Channel for Latest<T> {
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

/// Identity retained across admission and diagnostics.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub struct CommandId {
    pub session: [u8; 16],
    pub epoch: u64,
    pub field: u16,
    pub sequence: u64,
    pub observed_copperlist_id: Option<u64>,
}

/// Console submission contract. Implementations bind a robot session and epoch.
pub trait Submit {
    fn submit<T: Encode>(
        &mut self,
        field: u16,
        observed: Option<u64>,
        validity: Validity,
        value: &T,
    ) -> Result<(), CuStreamTxError>;
}
/// Typed console handle borrowed for one channel.
pub struct Input<'a, S, C> {
    sender: &'a mut S,
    field: u16,
    observed: Option<u64>,
    marker: PhantomData<C>,
}
impl<'a, S: Submit, C: Channel> Input<'a, S, C> {
    #[doc(hidden)]
    pub fn new(sender: &'a mut S, field: u16, observed: Option<u64>) -> Self {
        Self {
            sender,
            field,
            observed,
            marker: PhantomData,
        }
    }
    pub fn try_send(&mut self, value: &C::Payload) -> Result<(), CuStreamTxError> {
        self.sender
            .submit(self.field, self.observed, Validity::Observed, value)
    }
    /// The caller supplies a ToV already mapped into this robot's clock.
    pub fn try_send_msg(&mut self, msg: &CuMsg<C::Payload>) -> Result<(), CuStreamTxError> {
        let Some(value) = msg.payload() else {
            return Ok(());
        };
        self.sender
            .submit(self.field, self.observed, Validity::Robot(msg.tov), value)
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
pub struct Inbox<T>(Arc<ArrayQueue<(CommandId, Tov, T)>>);
impl<T> Clone for Inbox<T> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}
impl<T> Inbox<T> {
    pub fn take(&self) -> Option<(CommandId, Tov, T)> {
        self.0.pop()
    }
}
/// Single producer for a typed inbox.
pub struct Publisher<C: Channel> {
    last_sequence: Option<u64>,
    inbox: Inbox<C::Payload>,
}
impl<C: Channel> Publisher<C> {
    pub fn pair() -> CuResult<(Self, Inbox<C::Payload>)> {
        if C::CAPACITY == 0 {
            return Err(CuError::from("teleops event capacity must be positive"));
        }
        let inbox = Inbox(Arc::new(ArrayQueue::new(C::CAPACITY)));
        Ok((
            Self {
                last_sequence: None,
                inbox: inbox.clone(),
            },
            inbox,
        ))
    }
    pub fn publish(&mut self, id: CommandId, tov: Tov, value: C::Payload) -> bool {
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
    fn submit<T: Encode>(
        &mut self,
        field: u16,
        observed: Option<u64>,
        validity: Validity,
        value: &T,
    ) -> Result<(), CuStreamTxError> {
        (**self).submit(field, observed, validity, value)
    }
}

mod link;
pub use link::*;

/// Robot-side admission failure for one command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rejected {
    Framing,
    Session,
    Epoch,
    Channel,
    Sequence,
    Freshness,
    Overflow,
}

/// Generated routing from private wire tags to typed publishers.
pub trait Dispatch {
    fn dispatch(&mut self, id: CommandId, tov: Tov, bytes: &[u8]) -> Result<(), Rejected>;
    fn reset(&mut self);
}
impl<C: Channel> Publisher<C> {
    #[doc(hidden)]
    pub fn decode(&mut self, id: CommandId, tov: Tov, bytes: &[u8]) -> Result<(), Rejected> {
        if self.last_sequence.is_some_and(|last| id.sequence <= last) {
            return Err(Rejected::Sequence);
        }
        let (value, len) =
            bincode::decode_from_slice(bytes, bincode::config::standard().with_limit::<1200>())
                .map_err(|_| Rejected::Framing)?;
        if len != bytes.len() {
            return Err(Rejected::Framing);
        }
        self.last_sequence = Some(id.sequence);
        self.publish(id, tov, value)
            .then_some(())
            .ok_or(Rejected::Overflow)
    }
    #[doc(hidden)]
    pub fn reset(&mut self) {
        while self.inbox.take().is_some() {}
        self.last_sequence = None;
    }
}
