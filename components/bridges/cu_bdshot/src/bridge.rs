use bincode::de::{Decode, Decoder};
use bincode::enc::{Encode, Encoder};
use bincode::error::{DecodeError, EncodeError};
use cu29::prelude::*;
use spin::Mutex;

use crate::board::{BdshotBoard, encode_frame};
use crate::messages::{DShotTelemetry, EscCommand, EscTelemetry};

pub trait BdshotBoardProvider {
    type Board: BdshotBoard;

    fn create_board() -> CuResult<Self::Board>;
}

// channel mappings generation
tx_channels! {
    esc0_tx => EscCommand,
    esc1_tx => EscCommand,
    esc2_tx => EscCommand,
    esc3_tx => EscCommand,
}

rx_channels! {
    esc0_rx => EscTelemetry,
    esc1_rx => EscTelemetry,
    esc2_rx => EscTelemetry,
    esc3_rx => EscTelemetry,
}

const MAX_ESC_CHANNELS: usize = 4;

#[cfg(all(
    feature = "messages-only",
    not(any(feature = "rp2350", feature = "stm32h7"))
))]
fn telemetry_status(idx: usize, telemetry: Option<DShotTelemetry>) -> CuCompactString {
    match telemetry {
        Some(DShotTelemetry::Erpm(v)) => format!("esc{} {}rpm", idx, v).into(),
        Some(DShotTelemetry::Temp(v)) => format!("esc{} t{}C", idx, v).into(),
        Some(DShotTelemetry::Voltage(v)) => format!("esc{} {}V", idx, v).into(),
        Some(DShotTelemetry::Amps(v)) => format!("esc{} {}A", idx, v).into(),
        Some(DShotTelemetry::EncodingError) => format!("esc{} enc_err", idx).into(),
        Some(DShotTelemetry::Debug1(v)) => format!("esc{} d1 {}", idx, v).into(),
        Some(DShotTelemetry::Debug2(v)) => format!("esc{} d2 {}", idx, v).into(),
        Some(DShotTelemetry::Debug3(v)) => format!("esc{} d3 {}", idx, v).into(),
        Some(DShotTelemetry::Event(v)) => format!("esc{} ev {}", idx, v).into(),
        None => format!("esc{} no_tlm", idx).into(),
    }
}

#[derive(Reflect)]
#[reflect(from_reflect = false, no_field_bounds, type_path = false)]
pub struct CuBdshotBridge<P: BdshotBoardProvider + 'static>
where
    P::Board: Send + 'static,
{
    #[reflect(ignore)]
    board: Mutex<P::Board>,
    #[reflect(ignore)]
    telemetry_cache: [Option<EscTelemetry>; MAX_ESC_CHANNELS],
    configured_channels: [bool; MAX_ESC_CHANNELS],
    active_channels: [bool; MAX_ESC_CHANNELS],
    wait_for_esc_power: bool,
    send_interval: Option<CuDuration>,
    last_send: [Option<CuTime>; MAX_ESC_CHANNELS],
}

impl<P: BdshotBoardProvider + 'static> cu29::reflect::TypePath for CuBdshotBridge<P>
where
    P::Board: Send + 'static,
{
    fn type_path() -> &'static str {
        "cu_bdshot::CuBdshotBridge"
    }

    fn short_type_path() -> &'static str {
        "CuBdshotBridge"
    }

    fn type_ident() -> Option<&'static str> {
        Some("CuBdshotBridge")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_bdshot")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_bdshot")
    }
}

impl<P: BdshotBoardProvider + 'static> Freezable for CuBdshotBridge<P>
where
    P::Board: Send + 'static,
{
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.telemetry_cache, encoder)?;
        Encode::encode(&self.last_send, encoder)?;
        Ok(())
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.telemetry_cache = Decode::decode(decoder)?;
        self.last_send = Decode::decode(decoder)?;
        Ok(())
    }
}

impl<P: BdshotBoardProvider + 'static> CuBridge for CuBdshotBridge<P>
where
    P::Board: Send + 'static,
{
    type Resources<'r> = ();
    type Tx = TxChannels;
    type Rx = RxChannels;

    fn new(
        config: Option<&ComponentConfig>,
        tx: &[BridgeChannelConfig<TxId>],
        _rx: &[BridgeChannelConfig<RxId>],
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        let board = P::create_board()?;
        if P::Board::CHANNEL_COUNT > MAX_ESC_CHANNELS {
            return Err(CuError::from(
                "BDShot board exposes more channels than supported",
            ));
        }
        if tx.len() > P::Board::CHANNEL_COUNT {
            return Err(CuError::from("Too many Tx channels for board"));
        }
        let mut active = [false; MAX_ESC_CHANNELS];
        for ch in tx {
            active[ch.channel.id.as_index()] = true;
        }
        let wait_for_esc_power = match config {
            Some(cfg) => cfg.get::<bool>("wait_for_esc_power")?.unwrap_or(false),
            None => false,
        };
        let send_interval = if let Some(cfg) = config {
            if let Some(rate_hz) = cfg.get::<f64>("rate_hz")? {
                if rate_hz <= 0.0 {
                    return Err(CuError::from("BDShot rate_hz must be > 0"));
                }
                Some(CuDuration::from((1e9 / rate_hz) as u64))
            } else {
                None
            }
        } else {
            None
        };
        Ok(Self {
            board: Mutex::new(board),
            telemetry_cache: Default::default(),
            configured_channels: active,
            active_channels: [false; MAX_ESC_CHANNELS],
            wait_for_esc_power,
            send_interval,
            last_send: [None; MAX_ESC_CHANNELS],
        })
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        let idle_frame = encode_frame(EscCommand::disarm());

        // A monitor may continue after a start error. Only enable control frames
        // once every configured ESC has answered during this startup.
        self.active_channels = [false; MAX_ESC_CHANNELS];
        self.telemetry_cache.fill(None);
        self.last_send.fill(None);
        const START_ATTEMPTS: usize = 2048;
        // Keep synchronous logging out of the disarm pulse train. A slow log
        // sink can space frames too far apart for the ESCs to initialize.
        let mut attempts_remaining = START_ATTEMPTS;
        loop {
            let mut ready = true;
            // Arbitrary long, TODO(gbin): move that to the config
            for idx in 0..P::Board::CHANNEL_COUNT {
                if !self.configured_channels[idx] {
                    continue;
                }
                let sample = {
                    let mut board = self.board.lock();
                    board.delay(200);
                    board.exchange(idx, idle_frame)
                };
                if let Some(sample) = sample.filter(|s| *s != DShotTelemetry::EncodingError) {
                    self.telemetry_cache[idx] = Some(EscTelemetry {
                        sample: Some(sample),
                    });
                } else {
                    ready = false;
                }
            }
            if ready {
                break;
            }
            if self.wait_for_esc_power {
                continue;
            }
            attempts_remaining -= 1;
            if attempts_remaining == 0 {
                error!(ctx, "Timeout waiting for ESC to start up");
                return Err(CuError::from("Timeout waiting for ESC to start up"));
            }
        }

        let telemetry_cmd = EscCommand {
            throttle: 13,
            request_telemetry: true,
        };
        let telemetry_frame = encode_frame(telemetry_cmd);
        for _ in 0..6 {
            for idx in 0..P::Board::CHANNEL_COUNT {
                if !self.configured_channels[idx] {
                    continue;
                }
                let mut board = self.board.lock();
                board.delay(200);
                let _ = board.exchange(idx, telemetry_frame);
            }
        }

        self.active_channels = self.configured_channels;
        Ok(())
    }

    fn send<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let idx = channel.id().as_index();
        if idx >= P::Board::CHANNEL_COUNT {
            return Err(CuError::from("Channel not supported by board"));
        }
        if !self.active_channels[idx] {
            return Ok(());
        }
        if let Some(interval) = self.send_interval {
            let now = ctx.recent();
            if let Some(last) = self.last_send[idx]
                && now - last < interval
            {
                return Ok(());
            }
            self.last_send[idx] = Some(now);
        }
        let payload: &CuMsg<EscCommand> = msg.downcast_ref()?;
        let command = payload.payload().cloned().unwrap_or_default();
        let frame = encode_frame(command);
        let telemetry = {
            let mut board = self.board.lock();
            board.exchange_in_loop(idx, frame)
        };
        if let Some(telemetry) = telemetry {
            self.telemetry_cache[idx] = Some(EscTelemetry {
                sample: Some(telemetry),
            });
        }
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        ctx: &CuContext,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        msg.tov = Tov::Time(ctx.now());
        let idx = channel.id().as_index();
        if idx >= P::Board::CHANNEL_COUNT {
            msg.clear_payload();
            return Ok(());
        }
        if !self.active_channels[idx] {
            msg.clear_payload();
            return Ok(());
        }
        let telemetry_msg: &mut CuMsg<EscTelemetry> = msg.downcast_mut()?;
        if let Some(sample) = self.telemetry_cache[idx].take() {
            #[cfg(all(
                feature = "messages-only",
                not(any(feature = "rp2350", feature = "stm32h7"))
            ))]
            telemetry_msg
                .metadata
                .set_status(telemetry_status(idx, sample.sample));
            telemetry_msg.set_payload(sample);
        } else {
            #[cfg(all(
                feature = "messages-only",
                not(any(feature = "rp2350", feature = "stm32h7"))
            ))]
            telemetry_msg
                .metadata
                .set_status(telemetry_status(idx, None));
            telemetry_msg.clear_payload();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestBoard {
        polls: usize,
        control_frames: usize,
        valid_after: usize,
    }

    impl BdshotBoardProvider for TestBoard {
        type Board = Self;

        fn create_board() -> CuResult<Self> {
            Ok(Self {
                polls: 0,
                control_frames: 0,
                valid_after: 1,
            })
        }
    }

    impl BdshotBoard for TestBoard {
        const CHANNEL_COUNT: usize = 2;

        fn exchange(&mut self, channel: usize, _: u32) -> Option<DShotTelemetry> {
            if channel == 0 {
                self.polls += 1;
            }
            Some(if channel == 1 && self.polls <= self.valid_after {
                DShotTelemetry::EncodingError
            } else {
                DShotTelemetry::Erpm(0)
            })
        }

        fn exchange_in_loop(&mut self, _: usize, _: u32) -> Option<DShotTelemetry> {
            self.control_frames += 1;
            None
        }

        fn delay(&mut self, _: u32) {}
    }

    #[test]
    fn startup_retries_and_a_failed_restart_blocks_control_frames() {
        let mut bridge = CuBdshotBridge::<TestBoard>::new(None, &[], &[], ()).unwrap();
        bridge.configured_channels = [true, true, false, false];
        let (ctx, _) = CuContext::new_mock_clock();
        let command = CuMsg::new(Some(EscCommand {
            throttle: 256,
            request_telemetry: true,
        }));
        bridge.start(&ctx).unwrap();
        assert_eq!(bridge.board.lock().polls, 2 + 6);
        bridge.send(&ctx, &TxChannels::ESC0_TX, &command).unwrap();
        bridge.board.lock().valid_after = usize::MAX;
        assert!(bridge.start(&ctx).is_err());
        bridge.send(&ctx, &TxChannels::ESC0_TX, &command).unwrap();
        assert_eq!(bridge.board.lock().control_frames, 1);

        bridge.wait_for_esc_power = true;
        let previous_polls = bridge.board.lock().polls;
        bridge.board.lock().valid_after = previous_polls + 2050;
        bridge.start(&ctx).unwrap();
        assert_eq!(bridge.board.lock().polls, previous_polls + 2051 + 6);
        bridge.send(&ctx, &TxChannels::ESC0_TX, &command).unwrap();
        assert_eq!(bridge.board.lock().control_frames, 2);
    }
}
