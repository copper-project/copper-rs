#![cfg(feature = "logstream")]

use bincode::{Decode, Encode};
use cu29::logstream::test_support::link_sim::{LinkSimulationConfig, simulate_bad_link};
use cu29::logstream::{
    ContinuousDecoder, ContinuousSenderConfig, CuStreamTx, CuStreamTxError, DensityThreshold,
    EncodingSymbolId, FecSymbolKind, Field, Lane, ReceiverLimits, RlcConfig, StreamIdentity,
    WirePacket, decode_copperlist,
};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

const SYMBOL_SIZE: usize = 256;
const WINDOW_SYMBOLS: usize = 64;
const MAX_EQUATIONS: usize = 32;
const RECORD_BYTES: usize = 4_096;

#[derive(Clone, Debug, Default, PartialEq, Eq, Encode, Decode, Serialize, Deserialize, Reflect)]
struct StreamMsg(u64);

#[derive(Default, Reflect)]
struct StreamSource {
    next: u64,
}

impl Freezable for StreamSource {}

impl CuSrcTask for StreamSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(StreamMsg);

    fn new(_config: Option<&ComponentConfig>, _resources: ()) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process(&mut self, _ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(StreamMsg(self.next));
        self.next += 1;
        Ok(())
    }
}

const MAX_CAPTURED_PACKET_BYTES: usize = 1_024;
const CAPTURED_PACKET_COUNT: usize = 128;
type CapturedPacket = heapless::Vec<u8, MAX_CAPTURED_PACKET_BYTES>;
type CapturedPackets = heapless::mpmc::Queue<CapturedPacket, CAPTURED_PACKET_COUNT>;

#[derive(Clone, Default)]
struct CapturingTx {
    packets: Arc<CapturedPackets>,
}

impl core::fmt::Debug for CapturingTx {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str("CapturingTx")
    }
}

impl CapturingTx {
    fn drain(&self) -> Vec<Vec<u8>> {
        core::iter::from_fn(|| self.packets.dequeue())
            .map(|packet| packet.as_slice().to_vec())
            .collect()
    }
}

impl CuStreamTx for CapturingTx {
    fn try_send(&mut self, datagram: &[u8]) -> Result<(), CuStreamTxError> {
        let mut packet = CapturedPacket::new();
        packet
            .extend_from_slice(datagram)
            .map_err(|_| CuStreamTxError::Failed("test packet exceeds capture capacity"))?;
        self.packets
            .enqueue(packet)
            .map_err(|_| CuStreamTxError::WouldBlock)
    }
}

#[copper_runtime(config = "tests/logstream_runtime_config.ron")]
struct LogstreamRuntimeApp {}

#[test]
fn generated_runtime_streams_without_local_copperlist_logging() -> CuResult<()> {
    const ITERATIONS: usize = 4;
    let transport = CapturingTx::default();
    let captured = transport.clone();
    let identity = StreamIdentity {
        session_id: *b"runtime-stream01",
        sender_id: 23,
    };
    let fec = RlcConfig::new(SYMBOL_SIZE, WINDOW_SYMBOLS, Field::Gf256)
        .map_err(|error| CuError::from(error.to_string()))?;
    let sender = ContinuousSenderConfig {
        identity,
        first_packet_sequence: 0,
        lane: Lane::ReplayCritical,
        fec,
        max_record_bytes: RECORD_BYTES,
        initial_esi: EncodingSymbolId::new(0),
        repair_every_source_symbols: 1,
        first_repair_key: 1,
        repair_density: DensityThreshold::FULL,
    };

    let app = LogstreamRuntimeApp::builder()
        .with_instance_id(identity.sender_id)
        .with_logstream(transport, sender)
        .build()?;
    let mut running = app.start()?;
    for _ in 0..ITERATIONS {
        running.run_one_iteration()?;
    }
    drop(running.stop()?);

    let datagrams = captured.drain();
    assert!(
        datagrams.len() > ITERATIONS,
        "expected source and repair datagrams, got {}",
        datagrams.len()
    );
    let source_symbols = datagrams
        .iter()
        .filter(|datagram| {
            WirePacket::decode(datagram)
                .is_ok_and(|packet| packet.header.symbol_kind == FecSymbolKind::Source)
        })
        .count();
    let simulated_link = simulate_bad_link(
        &datagrams,
        LinkSimulationConfig {
            seed: 0x71_6d_e5,
            drop_basis_points: 1_500,
            corrupt_basis_points: 500,
            duplicate_basis_points: 500,
            reorder: true,
        },
    );
    assert!(simulated_link.stats.dropped_datagrams > 0);
    let mut decoder = ContinuousDecoder::<
        SYMBOL_SIZE,
        { cu29::logstream::DEFAULT_MAX_WINDOW_SYMBOLS },
        MAX_EQUATIONS,
    >::new(
        identity,
        Lane::ReplayCritical,
        fec,
        MAX_EQUATIONS,
        ReceiverLimits::new(RECORD_BYTES, ITERATIONS, WINDOW_SYMBOLS, 64),
    )
    .map_err(|error| CuError::from(error.to_string()))?;
    for datagram in &simulated_link.datagrams {
        decoder
            .receive_datagram(datagram)
            .map_err(|error| CuError::from(error.to_string()))?;
    }

    assert_eq!(decoder.recovered_records().len(), ITERATIONS);
    assert!(decoder.stats().source_symbols_received < source_symbols);
    assert!(decoder.stats().source_symbols_recovered > 0);
    for expected_id in 0..ITERATIONS {
        let record = decoder
            .recovered_records()
            .iter()
            .find(|record| {
                record
                    .decoded()
                    .is_ok_and(|record| record.object_id == expected_id as u64)
            })
            .expect("every generated CopperList should be recovered")
            .decoded()
            .map_err(|error| CuError::from(error.to_string()))?;
        let copperlist: default::CuList =
            decode_copperlist(record.payload).map_err(|error| CuError::from(error.to_string()))?;
        assert_eq!(copperlist.id, expected_id as u64);
        assert_eq!(
            copperlist.msgs.0.0.payload(),
            Some(&StreamMsg(expected_id as u64))
        );
    }
    Ok(())
}
