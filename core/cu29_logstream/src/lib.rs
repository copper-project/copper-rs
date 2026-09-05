#![cfg_attr(not(feature = "std"), no_std)]

//! Transport-independent Copper log framing and continuous-stream recovery.
//!
//! The crate deliberately stops at datagrams. Runtime ownership handoff,
//! pacing, and concrete transports are composed outside this layer.

extern crate alloc;

pub use cu_fec::{DensityThreshold, EncodingSymbolId, Field, RlcConfig};

mod copper;
mod error;
mod record;
mod rlc;
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
pub use record::{DecodedRecord, RecordKind, encode_record};
pub use rlc::{
    ContinuousDecoder, ContinuousEncoder, ContinuousRecoveryStats, ReceiverLimits, RecoveredRecord,
    StreamIdentity,
};
pub use sender::{
    ContinuousCopperListSink, ContinuousSenderConfig, ContinuousSenderStats,
    DEFAULT_MAX_SYMBOL_SIZE, DEFAULT_MAX_WINDOW_SYMBOLS, DefaultContinuousCopperListSink,
};
pub use stream::{CuStreamRx, CuStreamRxError, CuStreamTx, CuStreamTxError};
pub use wire::{
    FecScheme, FecSymbolKind, Lane, PACKET_HEADER_LEN, WIRE_VERSION, WireHeader, WirePacket,
    encode_packet_into,
};
