use bincode::{Decode, Encode};
use clap::{Parser, ValueEnum};
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use serde::{Deserialize, Serialize};
use std::hint::black_box;
use std::sync::{LazyLock, Mutex};
use std::time::Instant;

pub const DEFAULT_CLOCK_STEP_TICKS: u64 = 10;
pub const DEFAULT_BG_DELAY_TICKS: u64 = 20;
const DEFAULT_EMIT_LIMIT: u64 = 64;
const DEFAULT_LOG_SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);
const DEFAULT_COMPUTE_WORDS: usize = 512;
const DEFAULT_COMPUTE_ROUNDS: u32 = 8;
const BENCH_DEFAULT_COMPUTE_WORDS: usize = 2048;
const BENCH_DEFAULT_COMPUTE_ROUNDS: u32 = 16;
const BENCH_DEFAULT_MIN_SECONDS: f64 = 1.0;

const OTM_SINK_ID: u8 = 1;
const MTO_SINK_ID: u8 = 2;
const MTM_SINK_ID: u8 = 3;

const OTM_LEFT_STAGE: u8 = 11;
const OTM_RIGHT_STAGE: u8 = 12;
const MTO_STAGE: u8 = 21;
const MTM_LEFT_STAGE: u8 = 31;
const MTM_RIGHT_STAGE: u8 = 32;
const BRIDGE_LEFT_STAGE: u8 = 41;
const BRIDGE_RIGHT_STAGE: u8 = 42;

#[derive(Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct SeqPayload {
    pub source_id: u8,
    pub seq: u32,
    pub stage_id: u8,
}

#[derive(Default, Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Encode, Decode, Reflect)]
pub struct JoinPayload {
    pub left_source_id: u8,
    pub right_source_id: u8,
    pub left_seq: u32,
    pub right_seq: u32,
    pub stage_id: u8,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TraceEntry {
    SeqPair {
        sink_id: u8,
        tick: u64,
        left_seq: u32,
        right_seq: u32,
        left_stage: u8,
        right_stage: u8,
    },
    Join {
        sink_id: u8,
        tick: u64,
        left_seq: u32,
        right_seq: u32,
        stage_id: u8,
    },
    JoinPair {
        sink_id: u8,
        tick: u64,
        left_left_seq: u32,
        left_right_seq: u32,
        right_left_seq: u32,
        right_right_seq: u32,
        left_stage: u8,
        right_stage: u8,
    },
    BridgePair {
        tick: u64,
        left_seq: u32,
        right_seq: u32,
        left_stage: u8,
        right_stage: u8,
    },
}

static TRACE_LOG: LazyLock<Mutex<Vec<TraceEntry>>> = LazyLock::new(|| Mutex::new(Vec::new()));

fn record_trace(entry: TraceEntry) {
    TRACE_LOG.lock().unwrap().push(entry);
}

#[cfg(test)]
fn clear_trace() {
    TRACE_LOG.lock().unwrap().clear();
}

#[cfg(test)]
fn take_trace() -> Vec<TraceEntry> {
    TRACE_LOG.lock().unwrap().drain(..).collect()
}

fn param_u64(config: Option<&ComponentConfig>, key: &str, default: u64) -> CuResult<u64> {
    Ok(config
        .and_then(|cfg| cfg.get::<u64>(key).ok().flatten())
        .unwrap_or(default))
}

fn param_u8(config: Option<&ComponentConfig>, key: &str, default: u8) -> CuResult<u8> {
    let value = param_u64(config, key, u64::from(default))?;
    u8::try_from(value).map_err(|_| CuError::from(format!("`{key}` does not fit in u8")))
}

fn param_u32(config: Option<&ComponentConfig>, key: &str, default: u32) -> CuResult<u32> {
    let value = param_u64(config, key, u64::from(default))?;
    u32::try_from(value).map_err(|_| CuError::from(format!("`{key}` does not fit in u32")))
}

fn param_usize(config: Option<&ComponentConfig>, key: &str, default: usize) -> CuResult<usize> {
    let value = param_u64(config, key, default as u64)?;
    usize::try_from(value).map_err(|_| CuError::from(format!("`{key}` does not fit in usize")))
}

fn busy_spin(iterations: u64, seed: u64) {
    let mut acc = seed ^ 0x9E37_79B9_7F4A_7C15;
    for i in 0..iterations {
        acc = acc.wrapping_mul(6364136223846793005).wrapping_add(i ^ seed);
    }
    black_box(acc);
}

fn init_compute_scratch(words: usize, salt: u64) -> Vec<u64> {
    (0..words)
        .map(|index| {
            (index as u64)
                .wrapping_mul(0x9E37_79B9_7F4A_7C15)
                .wrapping_add(salt.rotate_left((index % 31) as u32))
        })
        .collect()
}

fn run_compute_kernel(scratch: &mut [u64], seed: u64, rounds: u32) -> u64 {
    let mut acc = seed ^ 0xD6E8_FD50_7A1C_7A11;
    for round in 0..rounds {
        let round_mix = (round as u64).wrapping_mul(0x94D0_49BB_1331_11EB);
        for (index, slot) in scratch.iter_mut().enumerate() {
            let rotated = slot.rotate_left(((index as u32) ^ round) & 31);
            let mixed = rotated
                .wrapping_add(acc)
                .wrapping_add(round_mix)
                .wrapping_mul(0xBF58_476D_1CE4_E5B9)
                ^ (seed
                    .wrapping_add(index as u64)
                    .rotate_right((round & 31) + 1));
            *slot = mixed ^ (*slot).rotate_right(11);
            acc ^= mixed
                .wrapping_add(index as u64)
                .rotate_left(((round + index as u32) & 31) + 1);
        }
        acc = acc.rotate_left(17) ^ round_mix;
    }
    black_box(acc);
    acc
}

fn stamp_now(metadata: &mut CuMsgMetadata, now: CuTime) {
    metadata.process_time.start = now.into();
    metadata.process_time.end = now.into();
}

fn stamp_with_delay(metadata: &mut CuMsgMetadata, now: CuTime, delay_ticks: u64) {
    metadata.process_time.start = now.into();
    metadata.process_time.end = (now + CuDuration::from_nanos(delay_ticks)).into();
}

pub mod tasks {
    use super::*;

    #[derive(Reflect)]
    pub struct SequenceSrc {
        source_id: u8,
        emit_limit: u32,
        busy_spin_iters: u64,
        compute_rounds: u32,
        #[reflect(ignore)]
        scratch: Vec<u64>,
        next_seq: u32,
    }

    impl Freezable for SequenceSrc {}

    impl CuSrcTask for SequenceSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(SeqPayload);

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                source_id: param_u8(config, "source_id", 1)?,
                emit_limit: param_u32(config, "emit_limit", DEFAULT_EMIT_LIMIT as u32)?,
                busy_spin_iters: param_u64(config, "busy_spin_iters", 0)?,
                compute_rounds: param_u32(config, "compute_rounds", DEFAULT_COMPUTE_ROUNDS)?,
                scratch: init_compute_scratch(
                    param_usize(config, "compute_words", DEFAULT_COMPUTE_WORDS)?,
                    0x51C0_0001,
                ),
                next_seq: 0,
            })
        }

        fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
            let now = ctx.now();
            stamp_now(&mut output.metadata, now);
            if self.next_seq >= self.emit_limit {
                output.clear_payload();
                return Ok(());
            }

            let compute_digest = run_compute_kernel(
                &mut self.scratch,
                (u64::from(self.source_id) << 32) | u64::from(self.next_seq),
                self.compute_rounds,
            );
            busy_spin(self.busy_spin_iters, compute_digest);
            output.set_payload(SeqPayload {
                source_id: self.source_id,
                seq: self.next_seq,
                stage_id: 0,
            });
            self.next_seq = self.next_seq.saturating_add(1);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct TransformTask {
        stage_id: u8,
        logical_delay_ticks: u64,
        busy_spin_iters: u64,
        compute_rounds: u32,
        #[reflect(ignore)]
        scratch: Vec<u64>,
    }

    impl Freezable for TransformTask {}

    impl CuTask for TransformTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(SeqPayload);
        type Output<'m> = output_msg!(SeqPayload);

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                stage_id: param_u8(config, "stage_id", 1)?,
                logical_delay_ticks: param_u64(config, "logical_delay_ticks", 0)?,
                busy_spin_iters: param_u64(config, "busy_spin_iters", 0)?,
                compute_rounds: param_u32(config, "compute_rounds", DEFAULT_COMPUTE_ROUNDS)?,
                scratch: init_compute_scratch(
                    param_usize(config, "compute_words", DEFAULT_COMPUTE_WORDS)?,
                    u64::from(param_u8(config, "stage_id", 1)?),
                ),
            })
        }

        fn process(
            &mut self,
            ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let now = ctx.now();
            if self.logical_delay_ticks == 0 {
                stamp_now(&mut output.metadata, now);
            } else {
                stamp_with_delay(&mut output.metadata, now, self.logical_delay_ticks);
            }

            let Some(payload) = input.payload() else {
                output.clear_payload();
                return Ok(());
            };

            let compute_digest = run_compute_kernel(
                &mut self.scratch,
                (u64::from(payload.seq) << 16)
                    ^ u64::from(payload.source_id)
                    ^ u64::from(payload.stage_id),
                self.compute_rounds,
            );
            busy_spin(self.busy_spin_iters, compute_digest);
            output.set_payload(SeqPayload {
                source_id: payload.source_id,
                seq: payload.seq,
                stage_id: self.stage_id,
            });
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct JoinTask {
        stage_id: u8,
        logical_delay_ticks: u64,
        busy_spin_iters: u64,
        compute_rounds: u32,
        #[reflect(ignore)]
        scratch: Vec<u64>,
    }

    impl Freezable for JoinTask {}

    impl CuTask for JoinTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!('m, SeqPayload, SeqPayload);
        type Output<'m> = output_msg!(JoinPayload);

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                stage_id: param_u8(config, "stage_id", 1)?,
                logical_delay_ticks: param_u64(config, "logical_delay_ticks", 0)?,
                busy_spin_iters: param_u64(config, "busy_spin_iters", 0)?,
                compute_rounds: param_u32(config, "compute_rounds", DEFAULT_COMPUTE_ROUNDS)?,
                scratch: init_compute_scratch(
                    param_usize(config, "compute_words", DEFAULT_COMPUTE_WORDS)?,
                    u64::from(param_u8(config, "stage_id", 1)?),
                ),
            })
        }

        fn process(
            &mut self,
            ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let now = ctx.now();
            if self.logical_delay_ticks == 0 {
                stamp_now(&mut output.metadata, now);
            } else {
                stamp_with_delay(&mut output.metadata, now, self.logical_delay_ticks);
            }

            let (left, right): (&CuMsg<SeqPayload>, &CuMsg<SeqPayload>) = *input;
            let (Some(left), Some(right)) = (left.payload(), right.payload()) else {
                output.clear_payload();
                return Ok(());
            };

            if left.seq != right.seq {
                return Err(CuError::from(format!(
                    "join inputs drifted: left={}, right={}",
                    left.seq, right.seq
                )));
            }

            let compute_digest = run_compute_kernel(
                &mut self.scratch,
                (u64::from(left.seq) << 32)
                    ^ u64::from(right.seq)
                    ^ (u64::from(left.source_id) << 8)
                    ^ u64::from(right.source_id),
                self.compute_rounds,
            );
            busy_spin(self.busy_spin_iters, compute_digest);
            output.set_payload(JoinPayload {
                left_source_id: left.source_id,
                right_source_id: right.source_id,
                left_seq: left.seq,
                right_seq: right.seq,
                stage_id: self.stage_id,
            });
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct JoinDelayTask {
        stage_id: u8,
        logical_delay_ticks: u64,
        busy_spin_iters: u64,
        compute_rounds: u32,
        #[reflect(ignore)]
        scratch: Vec<u64>,
    }

    impl Freezable for JoinDelayTask {}

    impl CuTask for JoinDelayTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(JoinPayload);
        type Output<'m> = output_msg!(JoinPayload);

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                stage_id: param_u8(config, "stage_id", 1)?,
                logical_delay_ticks: param_u64(config, "logical_delay_ticks", 0)?,
                busy_spin_iters: param_u64(config, "busy_spin_iters", 0)?,
                compute_rounds: param_u32(config, "compute_rounds", DEFAULT_COMPUTE_ROUNDS)?,
                scratch: init_compute_scratch(
                    param_usize(config, "compute_words", DEFAULT_COMPUTE_WORDS)?,
                    u64::from(param_u8(config, "stage_id", 1)?),
                ),
            })
        }

        fn process(
            &mut self,
            ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let now = ctx.now();
            if self.logical_delay_ticks == 0 {
                stamp_now(&mut output.metadata, now);
            } else {
                stamp_with_delay(&mut output.metadata, now, self.logical_delay_ticks);
            }

            let Some(payload) = input.payload() else {
                output.clear_payload();
                return Ok(());
            };

            let compute_digest = run_compute_kernel(
                &mut self.scratch,
                (u64::from(payload.left_seq) << 32)
                    ^ u64::from(payload.right_seq)
                    ^ u64::from(payload.stage_id),
                self.compute_rounds,
            );
            busy_spin(self.busy_spin_iters, compute_digest);
            let mut delayed = payload.clone();
            delayed.stage_id = self.stage_id;
            output.set_payload(delayed);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct BridgeStampTask {
        stage_id: u8,
        logical_delay_ticks: u64,
        busy_spin_iters: u64,
        next_seq: u32,
        compute_rounds: u32,
        #[reflect(ignore)]
        scratch: Vec<u64>,
    }

    impl Freezable for BridgeStampTask {}

    impl CuTask for BridgeStampTask {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(SeqPayload);
        type Output<'m> = output_msg!(SeqPayload);

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                stage_id: param_u8(config, "stage_id", 1)?,
                logical_delay_ticks: param_u64(config, "logical_delay_ticks", 0)?,
                busy_spin_iters: param_u64(config, "busy_spin_iters", 0)?,
                next_seq: 0,
                compute_rounds: param_u32(config, "compute_rounds", DEFAULT_COMPUTE_ROUNDS)?,
                scratch: init_compute_scratch(
                    param_usize(config, "compute_words", DEFAULT_COMPUTE_WORDS)?,
                    u64::from(param_u8(config, "stage_id", 1)?),
                ),
            })
        }

        fn process(
            &mut self,
            ctx: &CuContext,
            input: &Self::Input<'_>,
            output: &mut Self::Output<'_>,
        ) -> CuResult<()> {
            let now = ctx.now();
            if self.logical_delay_ticks == 0 {
                stamp_now(&mut output.metadata, now);
            } else {
                stamp_with_delay(&mut output.metadata, now, self.logical_delay_ticks);
            }

            if input.payload().is_none() {
                output.clear_payload();
                return Ok(());
            }

            let compute_digest = run_compute_kernel(
                &mut self.scratch,
                (u64::from(self.stage_id) << 32) ^ u64::from(self.next_seq),
                self.compute_rounds,
            );
            busy_spin(self.busy_spin_iters, compute_digest);
            output.set_payload(SeqPayload {
                source_id: 9,
                seq: self.next_seq,
                stage_id: self.stage_id,
            });
            self.next_seq = self.next_seq.saturating_add(1);
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct SeqPairSink {
        sink_id: u8,
    }

    impl Freezable for SeqPairSink {}

    impl CuSinkTask for SeqPairSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!('m, SeqPayload, SeqPayload);

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                sink_id: param_u8(config, "sink_id", 1)?,
            })
        }

        fn process(&mut self, ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            let (left, right): (&CuMsg<SeqPayload>, &CuMsg<SeqPayload>) = *input;
            let (Some(left), Some(right)) = (left.payload(), right.payload()) else {
                return Ok(());
            };
            if left.seq != right.seq {
                return Ok(());
            }
            record_trace(TraceEntry::SeqPair {
                sink_id: self.sink_id,
                tick: ctx.now().as_nanos(),
                left_seq: left.seq,
                right_seq: right.seq,
                left_stage: left.stage_id,
                right_stage: right.stage_id,
            });
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct JoinSink {
        sink_id: u8,
    }

    impl Freezable for JoinSink {}

    impl CuSinkTask for JoinSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(JoinPayload);

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                sink_id: param_u8(config, "sink_id", 1)?,
            })
        }

        fn process(&mut self, ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            let Some(payload) = input.payload() else {
                return Ok(());
            };
            record_trace(TraceEntry::Join {
                sink_id: self.sink_id,
                tick: ctx.now().as_nanos(),
                left_seq: payload.left_seq,
                right_seq: payload.right_seq,
                stage_id: payload.stage_id,
            });
            Ok(())
        }
    }

    #[derive(Reflect)]
    pub struct JoinPairSink {
        sink_id: u8,
    }

    impl Freezable for JoinPairSink {}

    impl CuSinkTask for JoinPairSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!('m, JoinPayload, JoinPayload);

        fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                sink_id: param_u8(config, "sink_id", 1)?,
            })
        }

        fn process(&mut self, ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            let (left, right): (&CuMsg<JoinPayload>, &CuMsg<JoinPayload>) = *input;
            let (Some(left), Some(right)) = (left.payload(), right.payload()) else {
                return Ok(());
            };
            if left.left_seq != right.left_seq || left.right_seq != right.right_seq {
                return Ok(());
            }
            record_trace(TraceEntry::JoinPair {
                sink_id: self.sink_id,
                tick: ctx.now().as_nanos(),
                left_left_seq: left.left_seq,
                left_right_seq: left.right_seq,
                right_left_seq: right.left_seq,
                right_right_seq: right.right_seq,
                left_stage: left.stage_id,
                right_stage: right.stage_id,
            });
            Ok(())
        }
    }
}

pub mod bridges {
    use super::*;

    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub enum AlphaTxId {}

    pub struct AlphaTxChannels;

    impl BridgeChannelSet for AlphaTxChannels {
        type Id = AlphaTxId;

        const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[];
    }

    rx_channels! {
        pub struct AlphaRxChannels : AlphaRxId {
            ingress => SeqPayload = "alpha/ingress",
        }
    }

    #[derive(Reflect)]
    pub struct AlphaBridge {
        emit_limit: u32,
        busy_spin_iters: u64,
        compute_rounds: u32,
        #[reflect(ignore)]
        scratch: Vec<u64>,
        next_seq: u32,
    }

    impl Freezable for AlphaBridge {}

    impl CuBridge for AlphaBridge {
        type Resources<'r> = ();
        type Tx = AlphaTxChannels;
        type Rx = AlphaRxChannels;

        fn new(
            config: Option<&ComponentConfig>,
            _tx: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
            _rx: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {
                emit_limit: param_u32(config, "emit_limit", DEFAULT_EMIT_LIMIT as u32)?,
                busy_spin_iters: param_u64(config, "busy_spin_iters", 0)?,
                compute_rounds: param_u32(config, "compute_rounds", DEFAULT_COMPUTE_ROUNDS)?,
                scratch: init_compute_scratch(
                    param_usize(config, "compute_words", DEFAULT_COMPUTE_WORDS)?,
                    0xA17A_0001,
                ),
                next_seq: 0,
            })
        }

        fn send<'a, Payload>(
            &mut self,
            _ctx: &CuContext,
            _channel: &'static BridgeChannel<AlphaTxId, Payload>,
            _msg: &CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            Ok(())
        }

        fn receive<'a, Payload>(
            &mut self,
            ctx: &CuContext,
            _channel: &'static BridgeChannel<AlphaRxId, Payload>,
            msg: &mut CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            let now = ctx.now();
            stamp_now(&mut msg.metadata, now);
            if self.next_seq >= self.emit_limit {
                msg.clear_payload();
                return Ok(());
            }
            let compute_digest = run_compute_kernel(
                &mut self.scratch,
                (u64::from(self.next_seq) << 8) ^ 0xA17A,
                self.compute_rounds,
            );
            busy_spin(self.busy_spin_iters, compute_digest);
            msg.set_payload(Payload::default());
            self.next_seq = self.next_seq.saturating_add(1);
            Ok(())
        }
    }

    tx_channels! {
        pub struct BetaTxChannels : BetaTxId {
            left => SeqPayload = "beta/left",
            right => SeqPayload = "beta/right",
        }
    }

    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    pub enum BetaRxId {}

    pub struct BetaRxChannels;

    impl BridgeChannelSet for BetaRxChannels {
        type Id = BetaRxId;

        const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[];
    }

    #[derive(Default, Reflect)]
    pub struct BetaBridge {
        pending_left: bool,
        pending_right: bool,
        completed_pairs: u32,
    }

    impl Freezable for BetaBridge {}

    impl CuBridge for BetaBridge {
        type Resources<'r> = ();
        type Tx = BetaTxChannels;
        type Rx = BetaRxChannels;

        fn new(
            _config: Option<&ComponentConfig>,
            _tx: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
            _rx: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self::default())
        }

        fn send<'a, Payload>(
            &mut self,
            ctx: &CuContext,
            channel: &'static BridgeChannel<BetaTxId, Payload>,
            msg: &CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            if msg.payload().is_none() {
                return Ok(());
            }
            match channel.id() {
                BetaTxId::Left => self.pending_left = true,
                BetaTxId::Right => self.pending_right = true,
            }

            if self.pending_left && self.pending_right {
                record_trace(TraceEntry::BridgePair {
                    tick: ctx.now().as_nanos(),
                    left_seq: self.completed_pairs,
                    right_seq: self.completed_pairs,
                    left_stage: BRIDGE_LEFT_STAGE,
                    right_stage: BRIDGE_RIGHT_STAGE,
                });
                self.pending_left = false;
                self.pending_right = false;
                self.completed_pairs = self.completed_pairs.saturating_add(1);
            }

            Ok(())
        }

        fn receive<'a, Payload>(
            &mut self,
            _ctx: &CuContext,
            _channel: &'static BridgeChannel<BetaRxId, Payload>,
            _msg: &mut CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            Ok(())
        }
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct RuntimeMatrixApp {}

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
pub enum MissionArg {
    #[value(name = "OneToMany")]
    OneToMany,
    #[value(name = "OneToManyBackground")]
    OneToManyBackground,
    #[value(name = "ManyToOne")]
    ManyToOne,
    #[value(name = "ManyToOneBackground")]
    ManyToOneBackground,
    #[value(name = "ManyToMany")]
    ManyToMany,
    #[value(name = "ManyToManyBackground")]
    ManyToManyBackground,
    #[value(name = "BridgeFanout")]
    BridgeFanout,
    #[value(name = "BridgeFanoutBackground")]
    BridgeFanoutBackground,
}

impl MissionArg {
    pub const ALL: [MissionArg; 8] = [
        MissionArg::OneToMany,
        MissionArg::OneToManyBackground,
        MissionArg::ManyToOne,
        MissionArg::ManyToOneBackground,
        MissionArg::ManyToMany,
        MissionArg::ManyToManyBackground,
        MissionArg::BridgeFanout,
        MissionArg::BridgeFanoutBackground,
    ];

    pub fn all() -> &'static [MissionArg] {
        &Self::ALL
    }

    pub fn as_str(self) -> &'static str {
        match self {
            MissionArg::OneToMany => "OneToMany",
            MissionArg::OneToManyBackground => "OneToManyBackground",
            MissionArg::ManyToOne => "ManyToOne",
            MissionArg::ManyToOneBackground => "ManyToOneBackground",
            MissionArg::ManyToMany => "ManyToMany",
            MissionArg::ManyToManyBackground => "ManyToManyBackground",
            MissionArg::BridgeFanout => "BridgeFanout",
            MissionArg::BridgeFanoutBackground => "BridgeFanoutBackground",
        }
    }

    pub fn uses_background(self) -> bool {
        matches!(
            self,
            MissionArg::OneToManyBackground
                | MissionArg::ManyToOneBackground
                | MissionArg::ManyToManyBackground
                | MissionArg::BridgeFanoutBackground
        )
    }

    pub fn drain_iterations(self) -> usize {
        if self.uses_background() { 3 } else { 1 }
    }

    pub fn expected_trace(self, emit_limit: u64) -> Vec<TraceEntry> {
        let delay_steps = if self.uses_background() {
            DEFAULT_BG_DELAY_TICKS / DEFAULT_CLOCK_STEP_TICKS
        } else {
            0
        };
        let bg_stride = usize::try_from(delay_steps.max(1)).expect("delay steps should fit");

        match self {
            MissionArg::OneToMany => (0..emit_limit)
                .map(|seq| TraceEntry::SeqPair {
                    sink_id: OTM_SINK_ID,
                    tick: DEFAULT_CLOCK_STEP_TICKS.saturating_mul(seq),
                    left_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    right_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    left_stage: OTM_LEFT_STAGE,
                    right_stage: OTM_RIGHT_STAGE,
                })
                .collect(),
            MissionArg::OneToManyBackground => (0..emit_limit)
                .step_by(bg_stride)
                .map(|seq| TraceEntry::SeqPair {
                    sink_id: OTM_SINK_ID,
                    tick: DEFAULT_CLOCK_STEP_TICKS.saturating_mul(seq.saturating_add(delay_steps)),
                    left_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    right_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    left_stage: OTM_LEFT_STAGE,
                    right_stage: OTM_RIGHT_STAGE,
                })
                .collect(),
            MissionArg::ManyToOne => (0..emit_limit)
                .map(|seq| TraceEntry::Join {
                    sink_id: MTO_SINK_ID,
                    tick: DEFAULT_CLOCK_STEP_TICKS.saturating_mul(seq),
                    left_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    right_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    stage_id: MTO_STAGE,
                })
                .collect(),
            MissionArg::ManyToOneBackground => (0..emit_limit)
                .step_by(bg_stride)
                .map(|seq| TraceEntry::Join {
                    sink_id: MTO_SINK_ID,
                    tick: DEFAULT_CLOCK_STEP_TICKS.saturating_mul(seq.saturating_add(delay_steps)),
                    left_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    right_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    stage_id: MTO_STAGE,
                })
                .collect(),
            MissionArg::ManyToMany => (0..emit_limit)
                .map(|seq| TraceEntry::JoinPair {
                    sink_id: MTM_SINK_ID,
                    tick: DEFAULT_CLOCK_STEP_TICKS.saturating_mul(seq),
                    left_left_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    left_right_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    right_left_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    right_right_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    left_stage: MTM_LEFT_STAGE,
                    right_stage: MTM_RIGHT_STAGE,
                })
                .collect(),
            MissionArg::ManyToManyBackground => (0..emit_limit)
                .step_by(bg_stride)
                .map(|seq| TraceEntry::JoinPair {
                    sink_id: MTM_SINK_ID,
                    tick: DEFAULT_CLOCK_STEP_TICKS.saturating_mul(seq.saturating_add(delay_steps)),
                    left_left_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    left_right_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    right_left_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    right_right_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    left_stage: MTM_LEFT_STAGE,
                    right_stage: MTM_RIGHT_STAGE,
                })
                .collect(),
            MissionArg::BridgeFanout => (0..emit_limit)
                .map(|seq| TraceEntry::BridgePair {
                    tick: DEFAULT_CLOCK_STEP_TICKS.saturating_mul(seq),
                    left_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    right_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                    left_stage: BRIDGE_LEFT_STAGE,
                    right_stage: BRIDGE_RIGHT_STAGE,
                })
                .collect(),
            MissionArg::BridgeFanoutBackground => {
                let output_count = emit_limit.div_ceil(delay_steps.max(1));
                (0..output_count)
                    .map(|seq| TraceEntry::BridgePair {
                        tick: DEFAULT_CLOCK_STEP_TICKS.saturating_mul(
                            seq.saturating_mul(delay_steps).saturating_add(delay_steps),
                        ),
                        left_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                        right_seq: u32::try_from(seq).expect("emit limit should fit in u32"),
                        left_stage: BRIDGE_LEFT_STAGE,
                        right_stage: BRIDGE_RIGHT_STAGE,
                    })
                    .collect()
            }
        }
    }
}

enum MissionApp {
    OneToMany(OneToMany::RuntimeMatrixApp),
    OneToManyBackground(OneToManyBackground::RuntimeMatrixApp),
    ManyToOne(ManyToOne::RuntimeMatrixApp),
    ManyToOneBackground(ManyToOneBackground::RuntimeMatrixApp),
    ManyToMany(ManyToMany::RuntimeMatrixApp),
    ManyToManyBackground(ManyToManyBackground::RuntimeMatrixApp),
    BridgeFanout(BridgeFanout::RuntimeMatrixApp),
    BridgeFanoutBackground(BridgeFanoutBackground::RuntimeMatrixApp),
}

impl MissionApp {
    fn start_all_tasks(&mut self) -> CuResult<()> {
        match self {
            MissionApp::OneToMany(app) => app.start_all_tasks(),
            MissionApp::OneToManyBackground(app) => app.start_all_tasks(),
            MissionApp::ManyToOne(app) => app.start_all_tasks(),
            MissionApp::ManyToOneBackground(app) => app.start_all_tasks(),
            MissionApp::ManyToMany(app) => app.start_all_tasks(),
            MissionApp::ManyToManyBackground(app) => app.start_all_tasks(),
            MissionApp::BridgeFanout(app) => app.start_all_tasks(),
            MissionApp::BridgeFanoutBackground(app) => app.start_all_tasks(),
        }
    }

    fn run_one_iteration(&mut self) -> CuResult<()> {
        match self {
            MissionApp::OneToMany(app) => app.run_one_iteration(),
            MissionApp::OneToManyBackground(app) => app.run_one_iteration(),
            MissionApp::ManyToOne(app) => app.run_one_iteration(),
            MissionApp::ManyToOneBackground(app) => app.run_one_iteration(),
            MissionApp::ManyToMany(app) => app.run_one_iteration(),
            MissionApp::ManyToManyBackground(app) => app.run_one_iteration(),
            MissionApp::BridgeFanout(app) => app.run_one_iteration(),
            MissionApp::BridgeFanoutBackground(app) => app.run_one_iteration(),
        }
    }

    fn stop_all_tasks(&mut self) -> CuResult<()> {
        match self {
            MissionApp::OneToMany(app) => app.stop_all_tasks(),
            MissionApp::OneToManyBackground(app) => app.stop_all_tasks(),
            MissionApp::ManyToOne(app) => app.stop_all_tasks(),
            MissionApp::ManyToOneBackground(app) => app.stop_all_tasks(),
            MissionApp::ManyToMany(app) => app.stop_all_tasks(),
            MissionApp::ManyToManyBackground(app) => app.stop_all_tasks(),
            MissionApp::BridgeFanout(app) => app.stop_all_tasks(),
            MissionApp::BridgeFanoutBackground(app) => app.stop_all_tasks(),
        }
    }
}

pub fn load_runtime_matrix_config() -> CuResult<CuConfig> {
    CuConfig::deserialize_ron(&OneToMany::RuntimeMatrixApp::original_config()).map_err(|err| {
        CuError::from(format!(
            "failed to deserialize runtime matrix config: {err}"
        ))
    })
}

pub fn configure_runtime_matrix(
    config: &mut CuConfig,
    emit_limit: u64,
    busy_spin_iters: u64,
    compute_words: usize,
    compute_rounds: u32,
    enable_task_logging: bool,
) -> CuResult<()> {
    let compute_words_u64 = u64::try_from(compute_words)
        .map_err(|_| CuError::from("`compute_words` does not fit in u64"))?;
    let logging = config
        .logging
        .get_or_insert_with(cu29::config::LoggingConfig::default);
    logging.enable_task_logging = enable_task_logging;

    for mission in MissionArg::all() {
        let graph = config.get_graph_mut(Some(mission.as_str()))?;
        for node_index in graph.node_indices() {
            let node_id = u32::try_from(node_index.index()).expect("node index should fit in u32");
            let Some(node) = graph.get_node_mut(node_id) else {
                continue;
            };

            match node.get_type() {
                "tasks::SequenceSrc" => {
                    node.set_param("emit_limit", emit_limit);
                    node.set_param("busy_spin_iters", busy_spin_iters);
                    node.set_param("compute_words", compute_words_u64);
                    node.set_param("compute_rounds", u64::from(compute_rounds));
                }
                "tasks::TransformTask"
                | "tasks::JoinTask"
                | "tasks::JoinDelayTask"
                | "tasks::BridgeStampTask" => {
                    node.set_param("busy_spin_iters", busy_spin_iters);
                    node.set_param("compute_words", compute_words_u64);
                    node.set_param("compute_rounds", u64::from(compute_rounds));
                }
                _ => {}
            }
        }
    }

    for bridge in &mut config.bridges {
        if bridge.type_ == "bridges::AlphaBridge" {
            let bridge_config = bridge.config.get_or_insert_with(ComponentConfig::default);
            bridge_config.set("emit_limit", emit_limit);
            bridge_config.set("busy_spin_iters", busy_spin_iters);
            bridge_config.set("compute_words", compute_words_u64);
            bridge_config.set("compute_rounds", u64::from(compute_rounds));
        }
    }

    Ok(())
}

fn build_mission_app(
    mission: MissionArg,
    copper_ctx: &CopperContext,
    config: CuConfig,
) -> CuResult<MissionApp> {
    match mission {
        MissionArg::OneToMany => Ok(MissionApp::OneToMany(
            OneToMany::RuntimeMatrixAppBuilder::new()
                .with_context(copper_ctx)
                .with_config(config)
                .build()?,
        )),
        MissionArg::OneToManyBackground => Ok(MissionApp::OneToManyBackground(
            OneToManyBackground::RuntimeMatrixAppBuilder::new()
                .with_context(copper_ctx)
                .with_config(config)
                .build()?,
        )),
        MissionArg::ManyToOne => Ok(MissionApp::ManyToOne(
            ManyToOne::RuntimeMatrixAppBuilder::new()
                .with_context(copper_ctx)
                .with_config(config)
                .build()?,
        )),
        MissionArg::ManyToOneBackground => Ok(MissionApp::ManyToOneBackground(
            ManyToOneBackground::RuntimeMatrixAppBuilder::new()
                .with_context(copper_ctx)
                .with_config(config)
                .build()?,
        )),
        MissionArg::ManyToMany => Ok(MissionApp::ManyToMany(
            ManyToMany::RuntimeMatrixAppBuilder::new()
                .with_context(copper_ctx)
                .with_config(config)
                .build()?,
        )),
        MissionArg::ManyToManyBackground => Ok(MissionApp::ManyToManyBackground(
            ManyToManyBackground::RuntimeMatrixAppBuilder::new()
                .with_context(copper_ctx)
                .with_config(config)
                .build()?,
        )),
        MissionArg::BridgeFanout => Ok(MissionApp::BridgeFanout(
            BridgeFanout::RuntimeMatrixAppBuilder::new()
                .with_context(copper_ctx)
                .with_config(config)
                .build()?,
        )),
        MissionArg::BridgeFanoutBackground => Ok(MissionApp::BridgeFanoutBackground(
            BridgeFanoutBackground::RuntimeMatrixAppBuilder::new()
                .with_context(copper_ctx)
                .with_config(config)
                .build()?,
        )),
    }
}

fn mode_label() -> &'static str {
    match (cfg!(feature = "async-cl-io"), cfg!(feature = "parallel-rt")) {
        (false, false) => "sync+serial",
        (true, false) => "async-io+serial",
        (false, true) => "sync+parallel",
        (true, true) => "async-io+parallel",
    }
}

fn percentile(sorted: &[u64], num: u64, den: u64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    let len = sorted.len();
    let rank = ((len - 1) as u128 * num as u128) / den as u128;
    sorted[rank as usize]
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
enum BenchOutputFormat {
    #[value(name = "human")]
    Human,
    #[value(name = "tsv")]
    Tsv,
}

#[derive(Debug, Clone)]
struct BenchResultRow {
    mode: &'static str,
    mission: MissionArg,
    samples: usize,
    elapsed_s: f64,
    throughput_hz: f64,
    mean_us: f64,
    p50_us: f64,
    p90_us: f64,
    p99_us: f64,
    max_us: f64,
}

fn print_human_results(rows: &[BenchResultRow], args: &BenchArgs) {
    println!(
        "mode={} warmup={} min_iterations={} busy_spin_iters={} compute_words={} compute_rounds={} min_seconds={:.2} logging={}",
        mode_label(),
        args.warmup,
        args.iterations,
        args.busy_spin_iters,
        args.compute_words,
        args.compute_rounds,
        args.min_seconds,
        args.enable_task_logging,
    );
    println!(
        "throughput_hz: higher is better; latency columns are in microseconds and lower is better"
    );
    if args.min_seconds > 0.0 {
        println!(
            "duration gate: each mission runs until both min_iterations and min_seconds are satisfied; `samples` reports the actual measured iterations"
        );
    }
    println!(
        "{:<24} {:>10} {:>10} {:>14} {:>10} {:>10} {:>10} {:>10}",
        "mission", "samples", "secs", "throughput_hz", "mean_us", "p50_us", "p99_us", "max_us"
    );
    for row in rows {
        println!(
            "{:<24} {:>10} {:>10.2} {:>14.2} {:>10.2} {:>10.2} {:>10.2} {:>10.2}",
            row.mission.as_str(),
            row.samples,
            row.elapsed_s,
            row.throughput_hz,
            row.mean_us,
            row.p50_us,
            row.p99_us,
            row.max_us,
        );
    }
    if rows.iter().any(|row| row.mission.uses_background()) {
        println!(
            "note: background missions measure scheduler loop cost while work is in flight; they can sample outputs and are not throughput-equivalent to foreground missions"
        );
    }
}

fn print_tsv_results(rows: &[BenchResultRow]) {
    for row in rows {
        println!(
            "{}\t{}\t{}\t{:.4}\t{:.2}\t{:.2}\t{:.2}\t{:.2}\t{:.2}\t{:.2}",
            row.mode,
            row.mission.as_str(),
            row.samples,
            row.elapsed_s,
            row.throughput_hz,
            row.mean_us,
            row.p50_us,
            row.p90_us,
            row.p99_us,
            row.max_us,
        );
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, ValueEnum)]
enum BenchMission {
    #[value(name = "all")]
    All,
    #[value(name = "OneToMany")]
    OneToMany,
    #[value(name = "OneToManyBackground")]
    OneToManyBackground,
    #[value(name = "ManyToOne")]
    ManyToOne,
    #[value(name = "ManyToOneBackground")]
    ManyToOneBackground,
    #[value(name = "ManyToMany")]
    ManyToMany,
    #[value(name = "ManyToManyBackground")]
    ManyToManyBackground,
    #[value(name = "BridgeFanout")]
    BridgeFanout,
    #[value(name = "BridgeFanoutBackground")]
    BridgeFanoutBackground,
}

impl BenchMission {
    fn missions(self) -> Vec<MissionArg> {
        match self {
            BenchMission::All => MissionArg::all().to_vec(),
            BenchMission::OneToMany => vec![MissionArg::OneToMany],
            BenchMission::OneToManyBackground => vec![MissionArg::OneToManyBackground],
            BenchMission::ManyToOne => vec![MissionArg::ManyToOne],
            BenchMission::ManyToOneBackground => vec![MissionArg::ManyToOneBackground],
            BenchMission::ManyToMany => vec![MissionArg::ManyToMany],
            BenchMission::ManyToManyBackground => vec![MissionArg::ManyToManyBackground],
            BenchMission::BridgeFanout => vec![MissionArg::BridgeFanout],
            BenchMission::BridgeFanoutBackground => vec![MissionArg::BridgeFanoutBackground],
        }
    }
}

#[derive(Parser, Debug)]
#[command(name = "cu-runtime-matrix")]
struct BenchArgs {
    #[arg(long, value_enum, default_value_t = BenchMission::All)]
    mission: BenchMission,
    #[arg(long, default_value_t = 128)]
    warmup: u32,
    #[arg(long, default_value_t = 256)]
    iterations: u32,
    #[arg(long, default_value_t = 0)]
    busy_spin_iters: u64,
    #[arg(long, default_value_t = BENCH_DEFAULT_COMPUTE_WORDS)]
    compute_words: usize,
    #[arg(long, default_value_t = BENCH_DEFAULT_COMPUTE_ROUNDS)]
    compute_rounds: u32,
    #[arg(long, default_value_t = BENCH_DEFAULT_MIN_SECONDS)]
    min_seconds: f64,
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    enable_task_logging: bool,
    #[arg(long, value_enum, default_value_t = BenchOutputFormat::Human)]
    output_format: BenchOutputFormat,
}

pub fn run_bench_cli() -> CuResult<()> {
    let args = BenchArgs::parse();
    let mut results = Vec::new();
    let min_duration =
        (args.min_seconds > 0.0).then(|| std::time::Duration::from_secs_f64(args.min_seconds));
    for mission in args.mission.missions() {
        let total_emit_limit = if min_duration.is_some() {
            u64::from(u32::MAX)
        } else {
            u64::from(args.warmup)
                .saturating_add(u64::from(args.iterations))
                .saturating_add(u64::try_from(mission.drain_iterations()).expect("drain fits"))
        };
        let tmp_dir = tempfile::TempDir::new()
            .map_err(|err| CuError::new_with_cause("failed to create temp benchmark dir", err))?;
        let logger_path = tmp_dir.path().join(format!("{}.copper", mission.as_str()));
        let (clock, clock_mock) = RobotClock::mock();
        let ctx = basic_copper_setup(
            &logger_path,
            DEFAULT_LOG_SLAB_SIZE,
            false,
            Some(clock.clone()),
        )?;

        let mut config = load_runtime_matrix_config()?;
        configure_runtime_matrix(
            &mut config,
            total_emit_limit,
            args.busy_spin_iters,
            args.compute_words,
            args.compute_rounds,
            args.enable_task_logging,
        )?;
        let mut app = build_mission_app(mission, &ctx, config)?;

        app.start_all_tasks()?;
        for i in 0..args.warmup {
            clock_mock.set_value(DEFAULT_CLOCK_STEP_TICKS.saturating_mul(u64::from(i)));
            app.run_one_iteration()?;
        }

        let mut samples_ns = Vec::with_capacity(args.iterations as usize);
        let bench_start = Instant::now();
        let mut measured_iterations = 0_u32;
        loop {
            if measured_iterations >= args.iterations
                && min_duration
                    .map(|required| bench_start.elapsed() >= required)
                    .unwrap_or(true)
            {
                break;
            }
            let iter = args.warmup.saturating_add(measured_iterations);
            clock_mock.set_value(DEFAULT_CLOCK_STEP_TICKS.saturating_mul(u64::from(iter)));
            let iter_start = Instant::now();
            app.run_one_iteration()?;
            samples_ns.push(iter_start.elapsed().as_nanos() as u64);
            measured_iterations = measured_iterations.saturating_add(1);
        }
        let total_elapsed = bench_start.elapsed();

        for drain in 0..mission.drain_iterations() {
            let iter = u64::from(args.warmup)
                .saturating_add(u64::from(measured_iterations))
                .saturating_add(u64::try_from(drain).expect("drain index fits"));
            clock_mock.set_value(DEFAULT_CLOCK_STEP_TICKS.saturating_mul(iter));
            app.run_one_iteration()?;
        }
        app.stop_all_tasks()?;

        let mut sorted = samples_ns.clone();
        sorted.sort_unstable();
        let elapsed_s = total_elapsed.as_secs_f64();
        let throughput_hz = if elapsed_s > 0.0 {
            sorted.len() as f64 / elapsed_s
        } else {
            0.0
        };
        let mean_ns = if sorted.is_empty() {
            0.0
        } else {
            sorted.iter().copied().sum::<u64>() as f64 / sorted.len() as f64
        };
        results.push(BenchResultRow {
            mode: mode_label(),
            mission,
            samples: sorted.len(),
            elapsed_s,
            throughput_hz,
            mean_us: mean_ns / 1_000.0,
            p50_us: percentile(&sorted, 50, 100) as f64 / 1_000.0,
            p90_us: percentile(&sorted, 90, 100) as f64 / 1_000.0,
            p99_us: percentile(&sorted, 99, 100) as f64 / 1_000.0,
            max_us: sorted.last().copied().unwrap_or(0) as f64 / 1_000.0,
        });
    }

    match args.output_format {
        BenchOutputFormat::Human => print_human_results(&results, &args),
        BenchOutputFormat::Tsv => print_tsv_results(&results),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_export::copperlists_reader;
    use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
    use serde_json::Value;
    use std::path::Path;
    use std::time::Duration;

    static TEST_MUTEX: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));
    const TEST_EMIT_LIMIT: u64 = 8;
    const TEST_COMPUTE_WORDS: usize = 64;
    const TEST_COMPUTE_ROUNDS: u32 = 2;

    #[derive(Debug, Clone, PartialEq)]
    struct NormalizedCuMsg {
        payload: Option<Value>,
        tov: Tov,
        status_txt: String,
    }

    #[derive(Debug, Clone, PartialEq)]
    struct NormalizedCopperList {
        id: u64,
        msgs: Vec<NormalizedCuMsg>,
    }

    fn erased_serialize_to_json(value: &dyn erased_serde::Serialize) -> CuResult<Value> {
        let mut bytes = Vec::new();
        {
            let mut serializer = serde_json::Serializer::new(&mut bytes);
            erased_serde::serialize(value, &mut serializer).map_err(|err| {
                CuError::from(format!("failed to serialize erased payload: {err}"))
            })?;
        }
        serde_json::from_slice(&bytes)
            .map_err(|err| CuError::new_with_cause("failed to parse serialized payload JSON", err))
    }

    fn read_copperlists_normalized<P>(log_base: &Path) -> CuResult<Vec<NormalizedCopperList>>
    where
        P: CopperListTuple,
    {
        let UnifiedLogger::Read(read_logger) = UnifiedLoggerBuilder::new()
            .file_base_name(log_base)
            .build()
            .expect("failed to open runtime matrix log for read")
        else {
            panic!("expected read logger");
        };

        let mut reader = UnifiedLoggerIOReader::new(read_logger, UnifiedLogType::CopperList);
        let iter = copperlists_reader::<P>(&mut reader);
        let mut out = Vec::new();
        for entry in iter {
            let msgs = entry
                .cumsgs()
                .into_iter()
                .map(|msg| {
                    Ok(NormalizedCuMsg {
                        payload: msg.payload().map(erased_serialize_to_json).transpose()?,
                        tov: msg.tov(),
                        status_txt: msg.metadata().status_txt().0.to_string(),
                    })
                })
                .collect::<CuResult<Vec<_>>>()?;
            out.push(NormalizedCopperList { id: entry.id, msgs });
        }
        Ok(out)
    }

    fn read_mission_copperlists(
        mission: MissionArg,
        log_base: &Path,
    ) -> CuResult<Vec<NormalizedCopperList>> {
        match mission {
            MissionArg::OneToMany => {
                read_copperlists_normalized::<OneToMany::CuStampedDataSet>(log_base)
            }
            MissionArg::OneToManyBackground => {
                read_copperlists_normalized::<OneToManyBackground::CuStampedDataSet>(log_base)
            }
            MissionArg::ManyToOne => {
                read_copperlists_normalized::<ManyToOne::CuStampedDataSet>(log_base)
            }
            MissionArg::ManyToOneBackground => {
                read_copperlists_normalized::<ManyToOneBackground::CuStampedDataSet>(log_base)
            }
            MissionArg::ManyToMany => {
                read_copperlists_normalized::<ManyToMany::CuStampedDataSet>(log_base)
            }
            MissionArg::ManyToManyBackground => {
                read_copperlists_normalized::<ManyToManyBackground::CuStampedDataSet>(log_base)
            }
            MissionArg::BridgeFanout => {
                read_copperlists_normalized::<BridgeFanout::CuStampedDataSet>(log_base)
            }
            MissionArg::BridgeFanoutBackground => {
                read_copperlists_normalized::<BridgeFanoutBackground::CuStampedDataSet>(log_base)
            }
        }
    }

    fn run_mission_trace_and_logs(
        mission: MissionArg,
    ) -> CuResult<(Vec<TraceEntry>, Vec<NormalizedCopperList>)> {
        let tmp_dir = tempfile::TempDir::new()
            .map_err(|err| CuError::new_with_cause("failed to create temp test dir", err))?;
        let log_path = tmp_dir.path().join(format!("{}.copper", mission.as_str()));
        let (clock, clock_mock) = RobotClock::mock();
        let ctx = basic_copper_setup(&log_path, DEFAULT_LOG_SLAB_SIZE, false, Some(clock.clone()))?;

        let mut config = load_runtime_matrix_config()?;
        configure_runtime_matrix(
            &mut config,
            TEST_EMIT_LIMIT,
            0,
            TEST_COMPUTE_WORDS,
            TEST_COMPUTE_ROUNDS,
            true,
        )?;
        clear_trace();

        let mut app = build_mission_app(mission, &ctx, config)?;
        app.start_all_tasks()?;

        let total_iterations = TEST_EMIT_LIMIT
            .saturating_add(u64::try_from(mission.drain_iterations()).expect("drain fits"));
        for iter in 0..total_iterations {
            clock_mock.set_value(DEFAULT_CLOCK_STEP_TICKS.saturating_mul(iter));
            app.run_one_iteration()?;
            std::thread::yield_now();
            if mission.uses_background() {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        if mission.uses_background() {
            std::thread::sleep(Duration::from_millis(5));
        }
        app.stop_all_tasks()?;
        drop(app);

        let trace = take_trace();
        let copperlists = read_mission_copperlists(mission, &log_path)?;
        Ok((trace, copperlists))
    }

    #[test]
    fn missions_match_expected_async_traces() {
        let _guard = TEST_MUTEX
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        for mission in MissionArg::all() {
            let (trace, copperlists) = run_mission_trace_and_logs(*mission).expect("run mission");
            assert_eq!(
                trace,
                mission.expected_trace(TEST_EMIT_LIMIT),
                "trace mismatch for mission {:?}",
                mission
            );
            assert!(
                !copperlists.is_empty(),
                "expected copperlists for mission {:?}",
                mission
            );
        }
    }

    #[test]
    fn missions_record_identical_normalized_copperlists_across_two_runs() {
        let _guard = TEST_MUTEX
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        for mission in MissionArg::all() {
            let (_, first) = run_mission_trace_and_logs(*mission).expect("first run");
            let (_, second) = run_mission_trace_and_logs(*mission).expect("second run");
            assert_eq!(
                first, second,
                "normalized copperlist stream is not deterministic for mission {:?}",
                mission
            );
        }
    }
}
