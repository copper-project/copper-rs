use crate::rlc::RepairSchedule;
use crate::{
    ContinuousEncoder, CuStreamTx, CuStreamTxError, Error, Lane, PACKET_HEADER_LEN, Result,
    StreamIdentity, encode_copperlist_record_into,
};
use alloc::{boxed::Box, string::ToString, vec, vec::Vec};
use core::{fmt::Debug, marker::PhantomData};
use cu_fec::{DensityThreshold, EncodingSymbolId, RlcConfig};
use cu29_runtime::copperlist::CopperList;
use cu29_traits::{CopperListTuple, CuError, CuResult, WriteStream};

/// Maximum source-symbol storage used by generated runtime integration.
pub const DEFAULT_MAX_SYMBOL_SIZE: usize = 1128;
/// Maximum RLC coding-window storage used by generated runtime integration.
pub const DEFAULT_MAX_WINDOW_SYMBOLS: usize = 64;

/// Concrete sender shape emitted by generated applications.
pub type DefaultContinuousCopperListSink<P, T> =
    ContinuousCopperListSink<P, T, DEFAULT_MAX_SYMBOL_SIZE, DEFAULT_MAX_WINDOW_SYMBOLS>;

/// Configuration for the continuous CopperList sender lane.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContinuousSenderConfig {
    pub identity: StreamIdentity,
    pub first_packet_sequence: u64,
    pub lane: Lane,
    pub fec: RlcConfig,
    pub max_record_bytes: usize,
    pub initial_esi: EncodingSymbolId,
    /// Emit one repair after this many source symbols.
    pub repair_every_source_symbols: usize,
    pub first_repair_key: u16,
    pub repair_density: DensityThreshold,
}

/// Sender-side delivery counters. Transport backpressure is observable but never blocks.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ContinuousSenderStats {
    pub records_encoded: u64,
    pub source_datagrams: u64,
    pub repair_datagrams: u64,
    pub datagrams_sent: u64,
    pub datagrams_dropped: u64,
}

/// CopperList semantic sink backed by continuous RLC and a nonblocking datagram transport.
pub struct ContinuousCopperListSink<
    P,
    T,
    const MAX_SYMBOL_SIZE: usize,
    const MAX_WINDOW_SYMBOLS: usize,
> where
    P: CopperListTuple,
    T: CuStreamTx,
{
    transport: T,
    encoder: Box<ContinuousEncoder<MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>>,
    record: Vec<u8>,
    datagram: Vec<u8>,
    repair_schedule: RepairSchedule,
    stats: ContinuousSenderStats,
    _payload: PhantomData<fn() -> P>,
}

impl<P, T, const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize> Debug
    for ContinuousCopperListSink<P, T, MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>
where
    P: CopperListTuple,
    T: CuStreamTx,
{
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter
            .debug_struct("ContinuousCopperListSink")
            .field("transport", &self.transport)
            .field("config", &self.encoder.config())
            .field("stats", &self.stats)
            .finish_non_exhaustive()
    }
}

impl<P, T, const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize>
    ContinuousCopperListSink<P, T, MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>
where
    P: CopperListTuple,
    T: CuStreamTx,
{
    pub fn new(transport: T, config: ContinuousSenderConfig) -> Result<Self> {
        if config.repair_every_source_symbols == 0 {
            return Err(Error::InvalidConfig(
                "repair interval must contain at least one source symbol",
            ));
        }
        let datagram_bytes = PACKET_HEADER_LEN
            .checked_add(config.fec.symbol_size())
            .ok_or(Error::InvalidConfig("datagram length overflow"))?;
        let encoder = ContinuousEncoder::new(
            config.identity,
            config.first_packet_sequence,
            config.lane,
            config.fec,
            config.max_record_bytes,
            config.initial_esi,
        )?;
        Ok(Self {
            transport,
            encoder: Box::new(encoder),
            record: vec![0; config.max_record_bytes],
            datagram: vec![0; datagram_bytes],
            repair_schedule: RepairSchedule::new(
                config.repair_every_source_symbols,
                config.first_repair_key,
                config.repair_density,
            ),
            stats: ContinuousSenderStats::default(),
            _payload: PhantomData,
        })
    }

    pub const fn stats(&self) -> ContinuousSenderStats {
        self.stats
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn into_transport(self) -> T {
        self.transport
    }

    fn emit_source_record(&mut self, record_len: usize) -> Result<()> {
        let transport = &mut self.transport;
        let stats = &mut self.stats;
        let (source_datagrams, repair_datagrams) = self.encoder.push_record_with_repairs(
            &self.record[..record_len],
            &mut self.datagram,
            &mut |datagram| submit_datagram(transport, stats, datagram),
            &mut self.repair_schedule,
        )?;
        stats.source_datagrams = stats
            .source_datagrams
            .saturating_add(source_datagrams as u64);
        stats.repair_datagrams = stats
            .repair_datagrams
            .saturating_add(repair_datagrams as u64);
        Ok(())
    }
}

impl<P, T, const MAX_SYMBOL_SIZE: usize, const MAX_WINDOW_SYMBOLS: usize> WriteStream<CopperList<P>>
    for ContinuousCopperListSink<P, T, MAX_SYMBOL_SIZE, MAX_WINDOW_SYMBOLS>
where
    P: CopperListTuple + Send + Sync,
    T: CuStreamTx,
{
    fn log(&mut self, copperlist: &CopperList<P>) -> CuResult<()> {
        let record_len = encode_copperlist_record_into(copperlist, &mut self.record)
            .map_err(|error| CuError::from(error.to_string()))?;
        self.emit_source_record(record_len)
            .map_err(|error| CuError::from(error.to_string()))?;
        self.stats.records_encoded = self.stats.records_encoded.saturating_add(1);
        Ok(())
    }
}

fn submit_datagram<T: CuStreamTx>(
    transport: &mut T,
    stats: &mut ContinuousSenderStats,
    datagram: &[u8],
) -> Result<()> {
    match transport.try_send(datagram) {
        Ok(()) => stats.datagrams_sent = stats.datagrams_sent.saturating_add(1),
        Err(CuStreamTxError::WouldBlock) => {
            stats.datagrams_dropped = stats.datagrams_dropped.saturating_add(1);
        }
        Err(CuStreamTxError::Failed(message)) => return Err(Error::Transport(message)),
    }
    Ok(())
}
