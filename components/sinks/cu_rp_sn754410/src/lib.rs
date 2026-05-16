use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use cu29::units::si::f32::Ratio;
use cu29::units::si::ratio::ratio;
use serde::{Deserialize, Serialize};

#[cfg(hardware)]
use rppal::pwm::{Channel, Polarity, Pwm};

#[cfg(hardware)]
const PWM_FREQUENCY: f64 = 1000.0; // Frequency in Hz

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct SN754410 {
    current_power: f32, // retain what was the last state so we don't bang the hardware at each iteration.
    deadzone: f32,
    dryrun: bool,
    #[cfg_attr(hardware, reflect(ignore))]
    #[cfg(hardware)]
    pwm0: Pwm,
    #[cfg_attr(hardware, reflect(ignore))]
    #[cfg(hardware)]
    pwm1: Pwm,
    last_update: CuTime,
}

#[derive(
    Debug, Clone, Copy, Default, Encode, Decode, PartialEq, Serialize, Deserialize, Reflect,
)]
pub struct MotorPayload {
    pub power: Ratio, // -1.0 to 1.0
}

#[cfg(hardware)]
impl SN754410 {
    #[inline]
    fn forward(&mut self, _ctx: &CuContext, pwm: f64) -> CuResult<()> {
        self.pwm0
            .set_duty_cycle(pwm)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM0 duty cycle", e))?;
        self.pwm1
            .set_duty_cycle(0.0)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM1 duty cycle", e))?;
        Ok(())
    }

    #[inline]
    fn reverse(&mut self, _ctx: &CuContext, pwm: f64) -> CuResult<()> {
        self.pwm0
            .set_duty_cycle(0.0)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM0 duty cycle", e))?;
        self.pwm1
            .set_duty_cycle(pwm)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM1 duty cycle", e))?;
        Ok(())
    }

    #[inline]
    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.pwm0
            .set_duty_cycle(0.0)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM0 duty cycle", e))?;
        self.pwm1
            .set_duty_cycle(0.0)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM1 duty cycle", e))?;
        Ok(())
    }

    #[inline]
    fn enable_pwms(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.pwm0
            .enable()
            .map_err(|e| CuError::new_with_cause("Failed to enable PWM0", e))?;
        self.pwm1
            .enable()
            .map_err(|e| CuError::new_with_cause("Failed to enable PWM1", e))?;
        Ok(())
    }

    #[inline]
    fn disable_pwms(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.pwm0
            .disable()
            .map_err(|e| CuError::new_with_cause("Failed to disable PWM0", e))?;
        self.pwm1
            .disable()
            .map_err(|e| CuError::new_with_cause("Failed to disable PWM1", e))?;
        Ok(())
    }
}

#[cfg(mock)]
impl SN754410 {
    #[inline]
    fn forward(&mut self, ctx: &CuContext, pwm: f64) -> CuResult<()> {
        debug!(ctx, "Forwarding with power {}", pwm);
        Ok(())
    }

    #[inline]
    fn reverse(&mut self, ctx: &CuContext, pwm: f64) -> CuResult<()> {
        debug!(ctx, "Reversing with power {}", pwm);
        Ok(())
    }

    #[inline]
    fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
        debug!(ctx, "Stopping.");
        Ok(())
    }

    #[inline]
    fn enable_pwms(&mut self, ctx: &CuContext) -> CuResult<()> {
        debug!(ctx, "Enabling.");
        Ok(())
    }

    #[inline]
    fn disable_pwms(&mut self, ctx: &CuContext) -> CuResult<()> {
        debug!(ctx, "Disabling.");
        Ok(())
    }
}

impl Freezable for SN754410 {
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.current_power, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.current_power = Decode::decode(decoder)?;
        Ok(())
    }
}

impl CuSinkTask for SN754410 {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(MotorPayload);
    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let (deadzone, dryrun) = match config {
            Some(config) => (
                config.get::<f64>("deadzone")?.unwrap_or(0.0) as f32,
                config.get::<bool>("dryrun")?.unwrap_or(false),
            ),
            None => (0.0, false),
        };

        #[cfg(hardware)]
        let (pwm0, pwm1) = (
            Pwm::with_frequency(Channel::Pwm0, PWM_FREQUENCY, 0.0, Polarity::Normal, false)
                .map_err(|e| CuError::new_with_cause("Failed to create PWM0", e))?,
            Pwm::with_frequency(Channel::Pwm1, PWM_FREQUENCY, 0.0, Polarity::Normal, false)
                .map_err(|e| CuError::new_with_cause("Failed to create PWM0", e))?,
        );

        Ok(Self {
            current_power: 0.0f32,
            #[cfg(hardware)]
            pwm0,
            #[cfg(hardware)]
            pwm1,
            last_update: CuTime::default(),
            deadzone,
            dryrun,
        })
    }
    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        debug!(ctx, "Enabling SN754410.");
        self.enable_pwms(ctx)
    }
    fn process(&mut self, ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        let Some(power) = input.payload() else {
            debug!(ctx, "No payload in the message, stopping for safety.");
            self.stop(ctx)?;
            return Err("Safety Mode.".into());
        };

        if self.dryrun {
            debug!(
                ctx,
                "In Dryrun mode ignore any command: power would have been {}.",
                power.power.get::<ratio>()
            );
            self.stop(ctx)?;
            return Ok(());
        }

        let power_ratio = power.power.get::<ratio>();
        let deadzone_compensated = if power_ratio != 0.0f32 {
            // proportinally on the [deadzone, 1.0] range
            let deadzone = self.deadzone;
            if power_ratio > 0.0 {
                deadzone + (1.0 - deadzone) * power_ratio
            } else {
                -deadzone + (1.0 + deadzone) * power_ratio
            }
        } else {
            power_ratio
        };

        if deadzone_compensated == self.current_power {
            debug!(ctx, "Power is the same {}, skipping.", deadzone_compensated);
            return Ok(());
        }

        self.current_power = deadzone_compensated;
        if deadzone_compensated > 0.0 {
            self.forward(ctx, self.current_power as f64)?;
        } else if self.current_power < 0.0 {
            self.reverse(ctx, -self.current_power as f64)?;
        } else {
            self.stop(ctx)?;
        }
        self.last_update = ctx.now();
        Ok(())
    }

    fn stop(&mut self, ctx: &CuContext) -> CuResult<()> {
        debug!(ctx, "Disabling SN754410.");
        self.disable_pwms(ctx)
    }
}

pub mod test_support {
    use crate::MotorPayload;
    use cu29::prelude::*;

    #[derive(Reflect)]
    pub struct SN754410TestSrc;

    impl Freezable for SN754410TestSrc {}

    impl CuSrcTask for SN754410TestSrc {
        type Resources<'r> = ();
        type Output<'m> = output_msg!(MotorPayload);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self {})
        }

        fn process(&mut self, _ctx: &CuContext, _new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            todo!()
        }
    }
}
