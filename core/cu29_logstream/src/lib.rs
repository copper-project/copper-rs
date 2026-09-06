#![cfg_attr(not(feature = "std"), no_std)]

//! Transport-independent Copper log framing and continuous-stream recovery.
//!
//! The crate owns bounded FEC, pacing, and autonomous recovery repetition above
//! packet transports. The host driver consumes encoded records from existing
//! semantic output workers; runtime ownership handoff stays outside this layer.

extern crate alloc;

pub use cu_fec::{DensityThreshold, EncodingSymbolId, Field, RlcConfig};

#[cfg(feature = "std")]
mod archive;
#[cfg(feature = "std")]
pub use archive::{CaptureArchive, NativeArchive};
#[cfg(feature = "std")]
pub mod twin;
#[cfg(feature = "std")]
mod twin_session;
#[cfg(feature = "std")]
pub use twin_session::{CuTwin, CuTwinBuilder, CuTwinReader, CuTwinRecordingState, CuTwinStatus};

/// Bounded ground-side delivery to caller-owned telemetry consumers.
#[cfg(feature = "std")]
pub mod telemetry;

pub mod capture;
mod copper;
mod error;
mod manifest;
mod object;
mod pacing;
mod record;
mod recovery;
mod rlc;
mod router;
mod sender;
mod stream;
mod wire;
#[cfg(feature = "std")]
mod worker;
#[cfg(feature = "std")]
pub use worker::{ScheduledCopperListSink, ScheduledKeyFrameSink, SenderMonitor, scheduled_sinks};

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
    ApplicationOutputSchema, ApplicationSchema, LogStreamPlan, ResolvedContinuousFec,
    ResolvedObjectFec, ResolvedRlcField, SESSION_MANIFEST_VERSION, SessionManifest,
};
pub use object::{
    FiniteObjectDecoder, FiniteObjectEncoder, FiniteObjectLimits, FiniteObjectRecoveryStats,
    FiniteObjectSenderConfig,
};
pub use pacing::{PacingConfig, RECOVERY_REPEAT_INTERVAL, SenderCore, SenderStats};
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
pub use stream::{
    CuFeedbackRx, CuFeedbackTx, CuStreamRx, CuStreamRxError, CuStreamTx, CuStreamTxError, OneWay,
    SeparateFeedback,
};
pub use wire::{
    FecScheme, FecSymbolKind, Lane, PACKET_HEADER_LEN, WIRE_VERSION, WireHeader, WirePacket,
    encode_packet_into,
};
