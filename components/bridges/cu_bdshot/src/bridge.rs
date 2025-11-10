use crate::board::{encode_frame, BdshotBoard};
use crate::messages::{EscCommand, EscTelemetry};
use cu29::cubridge::{
    BridgeChannel, BridgeChannelConfig, BridgeChannelInfo, BridgeChannelSet, CuBridge,
};
use cu29::prelude::{CuError, CuMsg, CuResult, RobotClock};
use cu29::CuOther;

const MAX_ESC_CHANNELS: usize = 4;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum TxId {
    Esc0,
    Esc1,
    Esc2,
    Esc3,
}

pub struct TxChannels;

impl TxChannels {
    pub const ESC0: BridgeChannel<TxId, EscCommand> = BridgeChannel::new(TxId::Esc0, "esc0");
    pub const ESC1: BridgeChannel<TxId, EscCommand> = BridgeChannel::new(TxId::Esc1, "esc1");
    pub const ESC2: BridgeChannel<TxId, EscCommand> = BridgeChannel::new(TxId::Esc2, "esc2");
    pub const ESC3: BridgeChannel<TxId, EscCommand> = BridgeChannel::new(TxId::Esc3, "esc3");
}

impl BridgeChannelSet for TxChannels {
    type Id = TxId;

    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] =
        &[&Self::ESC0, &Self::ESC1, &Self::ESC2, &Self::ESC3];
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum RxId {
    Esc0,
    Esc1,
    Esc2,
    Esc3,
}

pub struct RxChannels;

impl RxChannels {
    pub const ESC0: BridgeChannel<RxId, EscTelemetry> = BridgeChannel::new(RxId::Esc0, "esc0");
    pub const ESC1: BridgeChannel<RxId, EscTelemetry> = BridgeChannel::new(RxId::Esc1, "esc1");
    pub const ESC2: BridgeChannel<RxId, EscTelemetry> = BridgeChannel::new(RxId::Esc2, "esc2");
    pub const ESC3: BridgeChannel<RxId, EscTelemetry> = BridgeChannel::new(RxId::Esc3, "esc3");
}

impl BridgeChannelSet for RxChannels {
    type Id = RxId;

    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] =
        &[&Self::ESC0, &Self::ESC1, &Self::ESC2, &Self::ESC3];
}

pub struct CuBdshotBridge<B: BdshotBoard> {
    board: B,
    telemetry_cache: [Option<EscTelemetry>; MAX_ESC_CHANNELS],
    active_channels: [bool; MAX_ESC_CHANNELS],
}

impl<B: BdshotBoard> CuBdshotBridge<B> {
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
            RxId::Esc0 => 0,
            RxId::Esc1 => 1,
            RxId::Esc2 => 2,
            RxId::Esc3 => 3,
        }
    }
}

impl<B: BdshotBoard> CuBridge for CuBdshotBridge<B> {
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
        let board = B::init()?;
        if tx.len() > B::CHANNEL_COUNT {
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
            for idx in 0..B::CHANNEL_COUNT {
                if !self.active_channels[idx] {
                    continue;
                }
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
            for idx in 0..B::CHANNEL_COUNT {
                if !self.active_channels[idx] {
                    continue;
                }
                self.board.delay(200);
                let _ = self.board.exchange(idx, telemetry_frame);
            }
        }

        Ok(())
    }

    fn send(
        &mut self,
        _clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, CuMsg<EscCommand>>,
        msg: &CuMsg<EscCommand>,
    ) -> CuResult<()> {
        let idx = Self::channel_index(channel.id());
        if idx >= B::CHANNEL_COUNT {
            return Err(CuError::from("Channel not supported by board"));
        }
        if !self.active_channels[idx] {
            return Ok(());
        }
        let command = msg.payload().copied().unwrap_or_default();
        let frame = encode_frame(command);
        if let Some(telemetry) = self.board.exchange(idx, frame) {
            self.telemetry_cache[idx] = Some(EscTelemetry {
                sample: Some(telemetry),
            });
        }
        Ok(())
    }

    fn receive(
        &mut self,
        _clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, CuMsg<EscTelemetry>>,
        msg: &mut CuMsg<EscTelemetry>,
    ) -> CuResult<()> {
        let idx = Self::rx_index(channel.id());
        if idx >= B::CHANNEL_COUNT {
            msg.clear_payload();
            return Ok(());
        }
        if !self.active_channels[idx] {
            msg.clear_payload();
            return Ok(());
        }
        if let Some(sample) = self.telemetry_cache[idx].take() {
            msg.set_payload(sample);
        } else {
            msg.clear_payload();
        }
        Ok(())
    }
}
