use core::marker::PhantomData;
use cu_bdshot::{DShotTelemetry, EscCommand, EscTelemetry};
use cu_crsf::messages::RcChannelsPayload;
use cu29::prelude::*;

const CRSF_THROTTLE_INDEX: usize = 2; // 0 is aileron, 1, elevator, 2 is throttle, 3 is rudder

#[derive(Default)]
pub struct ThrottleControl {}

impl Freezable for ThrottleControl {}

impl CuTask for ThrottleControl {
    type Resources<'r> = ();
    type Input<'m> = CuMsg<RcChannelsPayload>;
    type Output<'m> = CuMsg<EscCommand>;

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process<'i, 'o>(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        if let Some(channels) = input.payload() {
            let throttle = channels.inner()[CRSF_THROTTLE_INDEX];
            let command = EscCommand {
                throttle,
                request_telemetry: true,
            };
            info!("Sending throttle {}.", throttle);
            output.set_payload(command);
        }

        Ok(())
    }
}

pub struct TelemetrySink<const ESC: usize> {
    _marker: PhantomData<EscTelemetry>,
}

impl<const ESC: usize> Freezable for TelemetrySink<ESC> {}

impl<const ESC: usize> Default for TelemetrySink<ESC> {
    fn default() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl<const ESC: usize> CuSinkTask for TelemetrySink<ESC> {
    type Input<'m> = CuMsg<EscTelemetry>;
    type Resources<'r> = ();

    fn new_with(
        _config: Option<&ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process<'i>(&mut self, _clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
        if let Some(payload) = input.payload().and_then(|t| t.sample) {
            match payload {
                DShotTelemetry::EncodingError => {
                    info!("ESC{} telemetry encoding error", ESC)
                }
                DShotTelemetry::Erpm(v) => info!("ESC{} eRPM {}", ESC, v),
                DShotTelemetry::Temp(v) => info!("ESC{} temp {}C", ESC, v),
                DShotTelemetry::Voltage(v) => info!("ESC{} voltage {}x0.25V", ESC, v),
                DShotTelemetry::Amps(v) => info!("ESC{} current {}A", ESC, v),
                DShotTelemetry::Debug1(v) => info!("ESC{} dbg1 {}", ESC, v),
                DShotTelemetry::Debug2(v) => info!("ESC{} dbg2 {}", ESC, v),
                DShotTelemetry::Debug3(v) => info!("ESC{} dbg3 {}", ESC, v),
                DShotTelemetry::Event(v) => info!("ESC{} event {}", ESC, v),
            }
        } else {
            info!("No Telemetry {}", ESC);
        }
        Ok(())
    }
}

pub type TelemetrySink0 = TelemetrySink<0>;
pub type TelemetrySink1 = TelemetrySink<1>;
pub type TelemetrySink2 = TelemetrySink<2>;
pub type TelemetrySink3 = TelemetrySink<3>;
