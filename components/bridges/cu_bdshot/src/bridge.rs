use cu29::prelude::*;

use crate::board::{encode_frame, BdshotBoard};
use crate::messages::{EscCommand, EscTelemetry};

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

pub struct CuBdshotBridge<P: BdshotBoardProvider> {
    board: P::Board,
    telemetry_cache: [Option<EscTelemetry>; MAX_ESC_CHANNELS],
    active_channels: [bool; MAX_ESC_CHANNELS],
}

impl<P: BdshotBoardProvider> Freezable for CuBdshotBridge<P> {}

impl<P: BdshotBoardProvider> CuBridge for CuBdshotBridge<P> {
    type Resources<'r> = ();
    type Tx = TxChannels;
    type Rx = RxChannels;

    fn new(
        _config: Option<&ComponentConfig>,
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
        Ok(Self {
            board,
            telemetry_cache: Default::default(),
            active_channels: active,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let idle_frame = encode_frame(EscCommand::disarm());

        let mut ready = true;
        for i in 0..2048 {
            // Arbitrary long, TODO(gbin): move that to the config
            for idx in 0..P::Board::CHANNEL_COUNT {
                if !self.active_channels[idx] {
                    continue;
                }
                debug!("Sending disarm frames {}", idx);
                self.board.delay(200);
                if let Some(sample) = self.board.exchange(idx, idle_frame) {
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
            info!("Waiting for ESCs startup {}...", i);
        }

        if !ready {
            error!("ESC TIMEOUT");
            return Err(CuError::from("Timeout waiting for ESC to start up"));
        }

        let telemetry_cmd = EscCommand {
            throttle: 13,
            request_telemetry: true,
        };
        let telemetry_frame = encode_frame(telemetry_cmd);
        for _ in 0..6 {
            for idx in 0..P::Board::CHANNEL_COUNT {
                if !self.active_channels[idx] {
                    continue;
                }
                self.board.delay(200);
                let _ = self.board.exchange(idx, telemetry_frame);
            }
        }

        Ok(())
    }

    fn send<'a, Payload>(
        &mut self,
        _clock: &RobotClock,
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
        let payload: &CuMsg<EscCommand> = msg.downcast_ref()?;
        let command = payload.payload().cloned().unwrap_or_default();
        let frame = encode_frame(command);
        if let Some(telemetry) = self.board.exchange(idx, frame) {
            self.telemetry_cache[idx] = Some(EscTelemetry {
                sample: Some(telemetry),
            });
        }
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        _clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
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
            telemetry_msg.set_payload(sample);
        } else {
            telemetry_msg.clear_payload();
        }
        Ok(())
    }
}
