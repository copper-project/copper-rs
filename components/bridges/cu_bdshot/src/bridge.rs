use crate::board::{encode_frame, BdshotBoard};
use crate::messages::{EscCommand, EscTelemetry};
use cu29::cubridge::{
    BridgeChannel, BridgeChannelConfig, BridgeChannelInfo, BridgeChannelSet, CuBridge,
};
use cu29::prelude::{info, CuError, CuMsg, CuMsgPayload, CuResult, Freezable, RobotClock};
use serde::{Deserialize, Serialize};

pub trait BdshotBoardProvider {
    type Board: BdshotBoard;

    fn create_board() -> CuResult<Self::Board>;
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TxId {
    Esc0,
    Esc1,
    Esc2,
    Esc3,
}

pub struct TxChannels;

impl TxChannels {
    pub const ESC_0: BridgeChannel<TxId, EscCommand> = BridgeChannel::new(TxId::Esc0, "esc0");
    pub const ESC_1: BridgeChannel<TxId, EscCommand> = BridgeChannel::new(TxId::Esc1, "esc1");
    pub const ESC_2: BridgeChannel<TxId, EscCommand> = BridgeChannel::new(TxId::Esc2, "esc2");
    pub const ESC_3: BridgeChannel<TxId, EscCommand> = BridgeChannel::new(TxId::Esc3, "esc3");
}

impl BridgeChannelSet for TxChannels {
    type Id = TxId;

    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] =
        &[&Self::ESC_0, &Self::ESC_1, &Self::ESC_2, &Self::ESC_3];
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RxId {
    Esc0Rx,
    Esc1Rx,
    Esc2Rx,
    Esc3Rx,
}

pub struct RxChannels;

impl RxChannels {
    pub const ESC_0_RX: BridgeChannel<RxId, EscTelemetry> =
        BridgeChannel::new(RxId::Esc0Rx, "esc0_rx");
    pub const ESC_1_RX: BridgeChannel<RxId, EscTelemetry> =
        BridgeChannel::new(RxId::Esc1Rx, "esc1_rx");
    pub const ESC_2_RX: BridgeChannel<RxId, EscTelemetry> =
        BridgeChannel::new(RxId::Esc2Rx, "esc2_rx");
    pub const ESC_3_RX: BridgeChannel<RxId, EscTelemetry> =
        BridgeChannel::new(RxId::Esc3Rx, "esc3_rx");
}

impl BridgeChannelSet for RxChannels {
    type Id = RxId;

    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] = &[
        &Self::ESC_0_RX,
        &Self::ESC_1_RX,
        &Self::ESC_2_RX,
        &Self::ESC_3_RX,
    ];
}

const MAX_ESC_CHANNELS: usize = 4;

pub struct CuBdshotBridge<P: BdshotBoardProvider> {
    board: P::Board,
    telemetry_cache: [Option<EscTelemetry>; MAX_ESC_CHANNELS],
    active_channels: [bool; MAX_ESC_CHANNELS],
}

impl<P: BdshotBoardProvider> Freezable for CuBdshotBridge<P> {}

impl<P: BdshotBoardProvider> CuBdshotBridge<P> {
    fn channel_index(id: TxId) -> usize {
        match id {
            TxId::Esc0 => 0,
            TxId::Esc1 => 1,
            TxId::Esc2 => 2,
            TxId::Esc3 => 3,
        }
    }

    fn rx_index(id: RxId) -> usize {
        match id {
            RxId::Esc0Rx => 0,
            RxId::Esc1Rx => 1,
            RxId::Esc2Rx => 2,
            RxId::Esc3Rx => 3,
        }
    }

    unsafe fn msg_as_command<Payload>(msg: &CuMsg<Payload>) -> &CuMsg<EscCommand>
    where
        Payload: CuMsgPayload,
    {
        &*(msg as *const CuMsg<Payload> as *const CuMsg<EscCommand>)
    }

    unsafe fn msg_as_telemetry<Payload>(msg: &mut CuMsg<Payload>) -> &mut CuMsg<EscTelemetry>
    where
        Payload: CuMsgPayload,
    {
        &mut *(msg as *mut CuMsg<Payload> as *mut CuMsg<EscTelemetry>)
    }
}

impl<P: BdshotBoardProvider> CuBridge for CuBdshotBridge<P> {
    type Tx = TxChannels;
    type Rx = RxChannels;

    fn new(
        _config: Option<&cu29::config::ComponentConfig>,
        tx: &[BridgeChannelConfig<TxId>],
        _rx: &[BridgeChannelConfig<RxId>],
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
            let idx = Self::channel_index(ch.channel.id);
            active[idx] = true;
        }
        Ok(Self {
            board,
            telemetry_cache: Default::default(),
            active_channels: active,
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let idle_frame = encode_frame(EscCommand::disarm());
        for _ in 0..64 {
            let mut ready = true;
            for idx in 0..P::Board::CHANNEL_COUNT {
                if !self.active_channels[idx] {
                    continue;
                }
                info!("Sending disarm frames {}", idx);
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
                info!("Sending telemetry on frames {}", idx);
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
        let idx = Self::channel_index(channel.id());
        if idx >= P::Board::CHANNEL_COUNT {
            return Err(CuError::from("Channel not supported by board"));
        }
        if !self.active_channels[idx] {
            return Ok(());
        }
        let payload = unsafe { Self::msg_as_command(msg) };
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
        let idx = Self::rx_index(channel.id());
        if idx >= P::Board::CHANNEL_COUNT {
            msg.clear_payload();
            return Ok(());
        }
        if !self.active_channels[idx] {
            msg.clear_payload();
            return Ok(());
        }
        let telemetry_msg = unsafe { Self::msg_as_telemetry(msg) };
        if let Some(sample) = self.telemetry_cache[idx].take() {
            telemetry_msg.set_payload(sample);
        } else {
            telemetry_msg.clear_payload();
        }
        Ok(())
    }
}
