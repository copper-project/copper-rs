#![cfg_attr(not(feature = "std"), no_std)]

//! Transport-independent Copper log framing and continuous-stream recovery.
//!
//! The crate deliberately stops at datagrams. Runtime ownership handoff,
//! pacing, and concrete transports are composed outside this layer.

extern crate alloc;

pub use cu_fec::{DensityThreshold, EncodingSymbolId, Field, RlcConfig};

mod copper;
mod error;
mod manifest;
mod object;
mod record;
mod recovery;
mod rlc;
mod router;
mod sender;
mod stream;
mod wire;

/// Utilities for testing log streaming over unreliable datagram links.
///
/// This module is not part of the normal runtime API and is available only
/// with the `test-utils` feature.
#[cfg(feature = "test-utils")]
pub mod test_support;

pub use copper::{decode_copperlist, encode_copperlist, encode_copperlist_record_into};
pub use error::{Error, Result};
#[cfg(feature = "std")]
pub use manifest::new_session_id;
pub use manifest::{
    ApplicationOutputSchema, ApplicationSchema, LogStreamPlan, ResolvedContentPolicy,
    ResolvedContinuousFec, ResolvedObjectFec, ResolvedRlcField, SESSION_MANIFEST_VERSION,
    SessionManifest,
};
pub use object::{
    FiniteObjectDecoder, FiniteObjectEncoder, FiniteObjectLimits, FiniteObjectRecoveryStats,
    FiniteObjectSenderConfig,
};
pub use record::{DecodedRecord, RecordKind, decode_record, encode_record};
pub use recovery::{
    Anchor, decode_anchor, decode_keyframe, encode_anchor, encode_keyframe,
    encode_keyframe_and_anchor,
};
pub use rlc::{
    ContinuousDecoder, ContinuousEncoder, ContinuousReceiveEvent, ContinuousRecoveryStats,
    CopperListGap, GapReason, ReceiveError, ReceiverLimits, ReceiverOccupancy, RecoveredRecord,
    StreamIdentity,
};
pub use router::{SessionEvent, SessionRouter, SessionRouterLimits, SessionRouterStats};
pub use sender::{
    ContinuousCopperListSink, ContinuousSenderConfig, ContinuousSenderStats,
    DEFAULT_MAX_SYMBOL_SIZE, DEFAULT_MAX_WINDOW_SYMBOLS, DefaultContinuousCopperListSink,
    KeyFrameAnchorSink, LogStreamSenderConfig, RecoverySenderConfig, RecoverySenderStats,
};
pub use stream::{CuStreamRx, CuStreamRxError, CuStreamTx, CuStreamTxError};
pub use wire::{
    FecScheme, FecSymbolKind, Lane, PACKET_HEADER_LEN, WIRE_VERSION, WireHeader, WirePacket,
    encode_packet_into,
};
