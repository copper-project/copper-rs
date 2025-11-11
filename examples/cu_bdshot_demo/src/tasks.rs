use core::marker::PhantomData;
use cu29::prelude::*;
use cu_bdshot::{DShotTelemetry, EscCommand, EscTelemetry};

const MAX_THROTTLE: u16 = 600;
const STEP: u16 = 10;

pub struct ThrottleSource<const ESC: usize> {
    value: u16,
    ascending: bool,
}

impl<const ESC: usize> Freezable for ThrottleSource<ESC> {}

impl<const ESC: usize> Default for ThrottleSource<ESC> {
    fn default() -> Self {
        Self {
            value: 0,
            ascending: true,
        }
    }
}

impl<const ESC: usize> CuSrcTask for ThrottleSource<ESC> {
    type Output<'m> = CuMsg<EscCommand>;

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self::default())
    }

    fn process<'o>(&mut self, _clock: &RobotClock, output: &mut CuMsg<EscCommand>) -> CuResult<()> {
        let throttle = self.value;
        let command = EscCommand {
            throttle,
            request_telemetry: true,
        };
        info!("Sending throttle {}.", throttle);
        output.set_payload(command);

        if self.ascending {
            self.value = (self.value + STEP).min(MAX_THROTTLE);
            if self.value >= MAX_THROTTLE {
                info!("ESC {} MAXED OUT", ESC);
                self.ascending = false;
            }
        } else if self.value > STEP {
            self.value -= STEP;
        } else {
            info!("ESC {} ZEROED", ESC);
            self.value = 0;
            self.ascending = true;
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

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
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

pub type ThrottleSource0 = ThrottleSource<0>;
pub type ThrottleSource1 = ThrottleSource<1>;
pub type ThrottleSource2 = ThrottleSource<2>;
pub type ThrottleSource3 = ThrottleSource<3>;

pub type TelemetrySink0 = TelemetrySink<0>;
pub type TelemetrySink1 = TelemetrySink<1>;
pub type TelemetrySink2 = TelemetrySink<2>;
pub type TelemetrySink3 = TelemetrySink<3>;
