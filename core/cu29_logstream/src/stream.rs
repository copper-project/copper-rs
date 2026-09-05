//! Transport-independent, packet-oriented stream resource contracts.

use core::fmt::Debug;

/// Failure to submit one complete logstream packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CuStreamTxError {
    /// The bounded transport cannot accept this packet immediately.
    WouldBlock,
    /// The transport has failed and cannot accept the packet.
    Failed(&'static str),
}

/// Failure to receive one complete logstream packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CuStreamRxError {
    /// The next packet does not fit in the caller-provided buffer.
    BufferTooSmall { needed: usize },
    /// The transport has failed and cannot receive packets.
    Failed(&'static str),
}

/// One-way, fire-and-forget transmission of complete logstream packets.
///
/// Implementations must return immediately without waiting, locking, allocating,
/// retrying, acknowledging, or accepting a partial packet. `Ok(())` means only
/// that the resource accepted this packet once; it does not guarantee delivery.
pub trait CuStreamTx: Debug + Send + Sync {
    fn try_send(&mut self, packet: &[u8]) -> core::result::Result<(), CuStreamTxError>;
}

impl<T: CuStreamTx + ?Sized> CuStreamTx for alloc::boxed::Box<T> {
    #[inline]
    fn try_send(&mut self, packet: &[u8]) -> core::result::Result<(), CuStreamTxError> {
        (**self).try_send(packet)
    }
}

/// Nonblocking reception of complete logstream packets.
///
/// `Ok(None)` means no complete packet is available. `Ok(Some(len))` places one
/// complete packet in `packet[..len]`. This is the receive half of the one-way
/// data plane; it does not acknowledge traffic or imply a reverse channel.
pub trait CuStreamRx: Debug + Send + Sync {
    fn try_recv(
        &mut self,
        packet: &mut [u8],
    ) -> core::result::Result<Option<usize>, CuStreamRxError>;
}

impl<T: CuStreamRx + ?Sized> CuStreamRx for alloc::boxed::Box<T> {
    #[inline]
    fn try_recv(
        &mut self,
        packet: &mut [u8],
    ) -> core::result::Result<Option<usize>, CuStreamRxError> {
        (**self).try_recv(packet)
    }
}

/// Optional advisory receive direction. This logical interface may share the
/// physical stream carrier. A shared endpoint must have one receive owner and
/// route complete packet kinds; cloned competing readers cannot provide routing.
/// Protocol decoding, capability negotiation, and feedback policy belong above it.
pub trait CuFeedbackRx: Debug + Send + Sync {
    fn try_recv_feedback(
        &mut self,
        packet: &mut [u8],
    ) -> core::result::Result<Option<usize>, CuStreamRxError>;
}

/// Optional advisory transmit direction; same atomic, bounded contract as stream TX.
pub trait CuFeedbackTx: Debug + Send + Sync {
    fn try_send_feedback(&mut self, packet: &[u8]) -> core::result::Result<(), CuStreamTxError>;
}

/// Static autonomous wiring. There is no feedback polling or capability discovery.
#[derive(Debug)]
pub struct OneWay<T> {
    tx: T,
}
impl<T> OneWay<T> {
    pub const fn new(tx: T) -> Self {
        Self { tx }
    }
    pub fn into_inner(self) -> T {
        self.tx
    }
}
impl<T: CuStreamTx> CuStreamTx for OneWay<T> {
    fn try_send(&mut self, packet: &[u8]) -> core::result::Result<(), CuStreamTxError> {
        self.tx.try_send(packet)
    }
}
impl<T: CuStreamTx> CuFeedbackRx for OneWay<T> {
    fn try_recv_feedback(
        &mut self,
        _packet: &mut [u8],
    ) -> core::result::Result<Option<usize>, CuStreamRxError> {
        Ok(None)
    }
}

/// Explicit feedback wiring for separate logical endpoints. The endpoints may be
/// resource handles backed by one carrier; physical out-of-band links are optional.
#[derive(Debug)]
pub struct SeparateFeedback<T, R> {
    pub tx: T,
    pub feedback_rx: R,
}
impl<T: CuStreamTx, R: CuFeedbackRx> CuStreamTx for SeparateFeedback<T, R> {
    fn try_send(&mut self, packet: &[u8]) -> core::result::Result<(), CuStreamTxError> {
        self.tx.try_send(packet)
    }
}
impl<T: CuStreamTx, R: CuFeedbackRx> CuFeedbackRx for SeparateFeedback<T, R> {
    fn try_recv_feedback(
        &mut self,
        packet: &mut [u8],
    ) -> core::result::Result<Option<usize>, CuStreamRxError> {
        self.feedback_rx.try_recv_feedback(packet)
    }
}
