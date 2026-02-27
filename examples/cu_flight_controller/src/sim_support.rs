#![cfg(feature = "sim")]

use cu_bdshot::{DShotTelemetry, EscCommand, EscTelemetry};
use cu_crsf::messages::{LinkStatisticsPayload, RcChannelsPayload};
use cu_msp_bridge::{MspRequestBatch, MspResponseBatch};
use cu_sensor_payloads::{BarometerPayload, ImuPayload, MagnetometerPayload};
use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
use cu29::prelude::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, OnceLock};

static SIM_ACTIVITY_LED_STATE: OnceLock<Arc<AtomicBool>> = OnceLock::new();

fn sim_activity_led_state() -> Arc<AtomicBool> {
    SIM_ACTIVITY_LED_STATE
        .get_or_init(|| Arc::new(AtomicBool::new(false)))
        .clone()
}

#[derive(Clone, Default)]
pub struct SimActivityLed {
    state: Arc<AtomicBool>,
}

impl SimActivityLed {
    pub fn set(&self, on: bool) {
        self.state.store(on, Ordering::Relaxed);
    }
}

pub fn sim_activity_led() -> SimActivityLed {
    SimActivityLed {
        state: sim_activity_led_state(),
    }
}

pub fn sim_activity_led_is_on() -> bool {
    sim_activity_led_state().load(Ordering::Relaxed)
}

#[derive(Clone, Reflect)]
pub struct SimBatteryAdc {
    base_voltage: f32,
    phase: f32,
}

impl Default for SimBatteryAdc {
    fn default() -> Self {
        Self {
            base_voltage: 16.0,
            phase: 0.0,
        }
    }
}

impl SimBatteryAdc {
    pub fn read_voltage_v(&mut self) -> f32 {
        // Keep a small deterministic ripple so downstream battery logic sees live updates.
        self.phase += 0.05;
        let ripple = self.phase.sin() * 0.2;
        (self.base_voltage + ripple).max(0.0)
    }
}

pub fn sim_battery_adc(base_voltage: f32) -> SimBatteryAdc {
    SimBatteryAdc {
        base_voltage,
        phase: 0.0,
    }
}

#[derive(Default, Reflect)]
pub struct SimBmi088Source {
    step: u64,
}

impl Freezable for SimBmi088Source {}

impl CuSrcTask for SimBmi088Source {
    type Resources<'r> = ();
    type Output<'m> = CuMsg<ImuPayload>;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { step: 0 })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let t = (self.step as f32) * 0.01;
        self.step = self.step.saturating_add(1);
        output.tov = Tov::Time(ctx.now());
        output.set_payload(ImuPayload::from_raw(
            [0.0, 0.0, 9.81],
            [0.05 * t.sin(), 0.03 * t.cos(), 0.01 * t.sin()],
            32.0,
        ));
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct SimDps310Source {
    step: u64,
}

impl Freezable for SimDps310Source {}

impl CuSrcTask for SimDps310Source {
    type Resources<'r> = ();
    type Output<'m> = CuMsg<BarometerPayload>;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { step: 0 })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let t = (self.step as f32) * 0.02;
        self.step = self.step.saturating_add(1);
        output.tov = Tov::Time(ctx.now());
        output.set_payload(BarometerPayload::from_raw(
            101_325.0 + (t.sin() * 20.0),
            28.0,
        ));
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct SimIst8310Source {
    step: u64,
}

impl Freezable for SimIst8310Source {}

impl CuSrcTask for SimIst8310Source {
    type Resources<'r> = ();
    type Output<'m> = CuMsg<MagnetometerPayload>;

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self { step: 0 })
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        let t = (self.step as f32) * 0.015;
        self.step = self.step.saturating_add(1);
        output.tov = Tov::Time(ctx.now());
        output.set_payload(MagnetometerPayload::from_raw([
            35.0 + 2.0 * t.cos(),
            0.5 * t.sin(),
            -42.0,
        ]));
        Ok(())
    }
}

tx_channels! {
    pub struct RcTxChannels : RcTxId {
        lq_tx => LinkStatisticsPayload,
    }
}

rx_channels! {
    pub struct RcRxChannels : RcRxId {
        rc_rx => RcChannelsPayload,
    }
}

#[derive(Default, Reflect)]
pub struct RcBridgeSim;

impl Freezable for RcBridgeSim {}

impl CuBridge for RcBridgeSim {
    type Tx = RcTxChannels;
    type Rx = RcRxChannels;
    type Resources<'r> = ();

    fn new(
        _config: Option<&ComponentConfig>,
        _tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn send<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        _channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
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
        _channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let rc_msg: &mut CuMsg<RcChannelsPayload> = msg.downcast_mut()?;
        let mut payload = RcChannelsPayload::default();
        let channels = &mut payload.inner_mut().0;
        channels[0] = 992; // roll
        channels[1] = 992; // pitch
        channels[2] = 172; // throttle
        channels[3] = 992; // yaw
        channels[4] = 172; // arm switch low (disarmed)
        channels[5] = 992; // mode switch mid (ANGLE)
        rc_msg.tov = Tov::Time(ctx.now());
        rc_msg.set_payload(payload);
        Ok(())
    }
}

tx_channels! {
    pub struct MspTxChannels : MspTxId {
        requests => MspRequestBatch,
    }
}

rx_channels! {
    pub struct MspRxChannels : MspRxId {
        responses => MspResponseBatch,
        incoming => MspRequestBatch,
    }
}

#[derive(Default, Reflect)]
pub struct MspBridgeSim;

impl Freezable for MspBridgeSim {}

impl CuBridge for MspBridgeSim {
    type Tx = MspTxChannels;
    type Rx = MspRxChannels;
    type Resources<'r> = ();

    fn new(
        _config: Option<&ComponentConfig>,
        _tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self)
    }

    fn send<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        _channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
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
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        msg.tov = Tov::Time(ctx.now());
        match channel.id() {
            MspRxId::Responses => {
                let response_msg: &mut CuMsg<MspResponseBatch> = msg.downcast_mut()?;
                response_msg.clear_payload();
            }
            MspRxId::Incoming => {
                let request_msg: &mut CuMsg<MspRequestBatch> = msg.downcast_mut()?;
                request_msg.clear_payload();
            }
        }
        Ok(())
    }
}

tx_channels! {
    pub struct BdshotTxChannels : BdshotTxId {
        esc0_tx => EscCommand,
        esc1_tx => EscCommand,
        esc2_tx => EscCommand,
        esc3_tx => EscCommand,
    }
}

rx_channels! {
    pub struct BdshotRxChannels : BdshotRxId {
        esc0_rx => EscTelemetry,
        esc1_rx => EscTelemetry,
        esc2_rx => EscTelemetry,
        esc3_rx => EscTelemetry,
    }
}

#[derive(Reflect)]
pub struct BdshotBridgeSim {
    last_commands: [EscCommand; 4],
}

impl Default for BdshotBridgeSim {
    fn default() -> Self {
        Self {
            last_commands: [EscCommand::default(); 4],
        }
    }
}

impl Freezable for BdshotBridgeSim {}

impl CuBridge for BdshotBridgeSim {
    type Tx = BdshotTxChannels;
    type Rx = BdshotRxChannels;
    type Resources<'r> = ();

    fn new(
        _config: Option<&ComponentConfig>,
        _tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self::default())
    }

    fn send<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let esc_msg: &CuMsg<EscCommand> = msg.downcast_ref()?;
        if let Some(command) = esc_msg.payload() {
            let index = match channel.id() {
                BdshotTxId::Esc0Tx => 0,
                BdshotTxId::Esc1Tx => 1,
                BdshotTxId::Esc2Tx => 2,
                BdshotTxId::Esc3Tx => 3,
            };
            self.last_commands[index] = *command;
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
        let telemetry_msg: &mut CuMsg<EscTelemetry> = msg.downcast_mut()?;
        let motor_index = match channel.id() {
            BdshotRxId::Esc0Rx => 0,
            BdshotRxId::Esc1Rx => 1,
            BdshotRxId::Esc2Rx => 2,
            BdshotRxId::Esc3Rx => 3,
        };
        let _ = self.last_commands[motor_index];
        let command = self.last_commands[motor_index];
        let erpm = ((u32::from(command.throttle) * 30).min(u32::from(u16::MAX))) as u16;
        telemetry_msg.tov = Tov::Time(ctx.now());
        telemetry_msg
            .metadata
            .set_status(format!("esc{} {}rpm", motor_index, erpm));
        telemetry_msg.set_payload(EscTelemetry {
            sample: Some(DShotTelemetry::Erpm(erpm)),
        });
        Ok(())
    }
}
