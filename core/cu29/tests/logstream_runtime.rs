#![cfg(feature = "logstream")]

use bincode::{Decode, Encode};
use cu29::logstream::test_support::link_sim::{LinkSimulationConfig, simulate_bad_link};
use cu29::logstream::{
    ContinuousDecoder, ContinuousReceiveEvent, ContinuousSenderConfig, CuStreamTx, CuStreamTxError,
    DensityThreshold, EncodingSymbolId, FecScheme, FecSymbolKind, Field, FiniteObjectDecoder,
    FiniteObjectLimits, FiniteObjectSenderConfig, Lane, LogStreamSenderConfig, ReceiverLimits,
    RecordKind, RecoverySenderConfig, RlcConfig, StreamIdentity, WirePacket, decode_copperlist,
    decode_record, decode_recovery_point, encode_record,
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
    let sender = sender_config(identity)?;
    let fec = sender.continuous.fec;
    let manifest = sender.recovery.manifest_record.clone();

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
            WirePacket::decode(datagram).is_ok_and(|packet| {
                packet.header.fec_scheme == FecScheme::RlcGf256
                    && packet.header.symbol_kind == FecSymbolKind::Source
            })
        })
        .count();
    // Scheduling changes cross-lane arrival order. Select a known source loss
    // by semantic identity so this integration test stays inside its FEC budget;
    // random loss/corruption matrices are exercised by the codec tests.
    let impaired: Vec<_> = datagrams
        .iter()
        .filter(|datagram| {
            let header = WirePacket::decode(datagram).unwrap().header;
            !(header.record_kind == RecordKind::CopperList
                && header.symbol_kind == FecSymbolKind::Source
                && header.object_id == 1)
        })
        .cloned()
        .collect();
    assert!(impaired.len() < datagrams.len());
    let simulated_link = simulate_bad_link(
        &impaired,
        LinkSimulationConfig {
            seed: 0x71_6d_e5,
            drop_basis_points: 0,
            corrupt_basis_points: 0,
            duplicate_basis_points: 500,
            reorder: true,
        },
    );
    let mut decoder = ContinuousDecoder::<
        SYMBOL_SIZE,
        { cu29::logstream::DEFAULT_MAX_WINDOW_SYMBOLS },
        MAX_EQUATIONS,
    >::new(
        identity,
        Lane::ReplayCritical,
        fec,
        MAX_EQUATIONS,
        0,
        ReceiverLimits::new(RECORD_BYTES, WINDOW_SYMBOLS),
    )
    .map_err(|error| CuError::from(error.to_string()))?;
    let mut records = Vec::new();
    let mut gaps = Vec::new();
    for datagram in &simulated_link.datagrams {
        decoder
            .receive_datagram(datagram, |event| {
                match event {
                    ContinuousReceiveEvent::Record(record) => records.push(record.clone()),
                    ContinuousReceiveEvent::Gap(gap) => gaps.push(gap),
                }
                Ok::<(), core::convert::Infallible>(())
            })
            .map_err(|error| CuError::from(format!("{error:?}")))?;
    }

    assert_eq!(records.len(), ITERATIONS);
    assert!(gaps.is_empty());
    assert!(decoder.stats().source_symbols_received < source_symbols);
    assert!(decoder.stats().source_symbols_recovered > 0);
    for expected_id in 0..ITERATIONS {
        let record = records
            .iter()
            .find(|record| record.decoded().object_id == expected_id as u64)
            .expect("every generated CopperList should be recovered")
            .decoded();
        let copperlist: default::CuList =
            decode_copperlist(record.payload).map_err(|error| CuError::from(error.to_string()))?;
        assert_eq!(copperlist.id, expected_id as u64);
        assert_eq!(
            copperlist.msgs.0.0.payload(),
            Some(&StreamMsg(expected_id as u64))
        );
    }

    let mut object_decoder = FiniteObjectDecoder::new(
        identity,
        Lane::LargeObject,
        FiniteObjectLimits::new(64 * 1024, SYMBOL_SIZE as u16, 4),
    )
    .map_err(|error| CuError::from(error.to_string()))?;
    let mut object_records = Vec::new();
    for datagram in &datagrams {
        object_decoder
            .receive_datagram(datagram, |record| {
                object_records.push(record.bytes().to_vec());
                Ok::<(), core::convert::Infallible>(())
            })
            .map_err(|error| CuError::from(format!("{error:?}")))?;
    }
    let keyframe = object_records
        .iter()
        .map(|record| decode_record(record).unwrap())
        .find(|record| record.kind == RecordKind::KeyFrame)
        .expect("generated runtime should stream its keyframe");
    let recovery_point = object_records
        .iter()
        .map(|record| decode_record(record).unwrap())
        .find(|record| record.kind == RecordKind::RecoveryPoint)
        .map(|record| decode_recovery_point(record.payload).unwrap())
        .expect("generated runtime should stream its recovery point");
    assert_eq!(recovery_point.copperlist_id, 0);
    assert!(recovery_point.references_keyframe(keyframe));
    assert!(recovery_point.references_manifest(decode_record(&manifest).unwrap()));
    Ok(())
}

fn sender_config(identity: StreamIdentity) -> CuResult<LogStreamSenderConfig> {
    let fec = RlcConfig::new(SYMBOL_SIZE, WINDOW_SYMBOLS, Field::Gf256)
        .map_err(|error| CuError::from(error.to_string()))?;
    let continuous = ContinuousSenderConfig {
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
    let manifest = encode_record(RecordKind::Manifest, 0, b"runtime-test-manifest")
        .map_err(|error| CuError::from(error.to_string()))?;
    Ok(LogStreamSenderConfig {
        pacing: cu29::logstream::PacingConfig {
            bitrate_bps: 10_000_000,
            burst_packets: 8,
            max_latency: cu29::clock::CuDuration::from_millis(1000),
            memory_budget_bytes: 1024 * 1024,
        },
        continuous,
        recovery: RecoverySenderConfig {
            finite: FiniteObjectSenderConfig {
                identity,
                first_packet_sequence: 0,
                lane: Lane::LargeObject,
                symbol_size: SYMBOL_SIZE as u16,
                max_object_bytes: 64 * 1024,
                repair_symbols_per_block: 8,
            },
            manifest_record: manifest.clone(),
            recovery_interval: 1,
        },
    })
}

#[test]
fn scheduled_worker_shutdown_is_bounded_with_frozen_robotclock() -> CuResult<()> {
    let mut config = sender_config(StreamIdentity {
        session_id: *b"frozen-stream001",
        sender_id: 1,
    })?;
    config.pacing.bitrate_bps = 1;
    config.pacing.burst_packets = 1;
    config.pacing.max_latency = CuDuration::from_millis(20);
    let (clock, _mock) = RobotClock::mock();
    let (mut lists, keyframes, monitor) = cu29::logstream::scheduled_sinks::<
        default::CuStampedDataSet,
        _,
    >(CapturingTx::default(), config, clock)?;
    lists.log(&default::CuList::default())?;
    let started = std::time::Instant::now();
    drop(lists);
    drop(keyframes);
    assert!(started.elapsed() < std::time::Duration::from_secs(1));
    assert!(!monitor.failed());
    assert!(monitor.final_stats().unwrap().shutdown_drops > 0);
    Ok(())
}

#[derive(Debug)]
struct FailedTx;
impl CuStreamTx for FailedTx {
    fn try_send(&mut self, _: &[u8]) -> Result<(), CuStreamTxError> {
        Err(CuStreamTxError::Failed("test carrier failure"))
    }
}

#[test]
fn scheduled_worker_reports_carrier_failure_to_its_producer() -> CuResult<()> {
    let config = sender_config(StreamIdentity {
        session_id: *b"failed-stream001",
        sender_id: 1,
    })?;
    let (mut lists, keyframes, monitor) = cu29::logstream::scheduled_sinks::<
        default::CuStampedDataSet,
        _,
    >(FailedTx, config, RobotClock::new())?;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    while !monitor.failed() && std::time::Instant::now() < deadline {
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
    assert!(monitor.failed());
    assert!(lists.log(&default::CuList::default()).is_err());
    drop(lists);
    drop(keyframes);
    Ok(())
}
