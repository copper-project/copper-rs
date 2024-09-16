use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::clock::{CuTime, RobotClock};
use cu29::config::NodeInstanceConfig;
use cu29::cutask::{CuMsg, CuSinkTask, CuTaskLifecycle, Freezable};
use cu29::{input_msg, CuResult};
use serde::{Deserialize, Serialize};

use cu29_log_derive::debug;
use cu29_traits::CuError;
use rppal::pwm::{Channel, Polarity, Pwm};

const PWM_FREQUENCY: f64 = 1000.0; // Frequency in Hz

pub struct SN754410 {
    current_power: f32, // retain what was the last state so we don't bang the hardware at each iteration.
    deadzone: f32,
    pwm0: Pwm,
    pwm1: Pwm,
    last_update: CuTime,
}

#[derive(Debug, Clone, Copy, Default, Encode, Decode, PartialEq, Serialize, Deserialize)]
pub struct MotorMsg {
    pub power: f32, // -1.0 to 1.0
}

impl MotorMsg {}

impl SN754410 {
    #[inline]
    fn forward(&mut self, pwm: f64) -> CuResult<()> {
        self.pwm0
            .set_duty_cycle(pwm)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM0 duty cycle", e))?;
        self.pwm1
            .set_duty_cycle(0.0)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM1 duty cycle", e))?;
        Ok(())
    }

    #[inline]
    fn reverse(&mut self, pwm: f64) -> CuResult<()> {
        self.pwm0
            .set_duty_cycle(0.0)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM0 duty cycle", e))?;
        self.pwm1
            .set_duty_cycle(pwm)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM1 duty cycle", e))?;
        Ok(())
    }

    #[inline]
    fn stop(&mut self) -> CuResult<()> {
        self.pwm0
            .set_duty_cycle(0.0)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM0 duty cycle", e))?;
        self.pwm1
            .set_duty_cycle(0.0)
            .map_err(|e| CuError::new_with_cause("Failed to set PWM1 duty cycle", e))?;
        Ok(())
    }

    #[inline]
    fn enable_pwms(&mut self) -> CuResult<()> {
        self.pwm0
            .enable()
            .map_err(|e| CuError::new_with_cause("Failed to enable PWM0", e))?;
        self.pwm1
            .enable()
            .map_err(|e| CuError::new_with_cause("Failed to enable PWM1", e))?;
        Ok(())
    }

    #[inline]
    fn disable_pwms(&mut self) -> CuResult<()> {
        self.pwm0
            .disable()
            .map_err(|e| CuError::new_with_cause("Failed to disable PWM0", e))?;
        self.pwm1
            .disable()
            .map_err(|e| CuError::new_with_cause("Failed to disable PWM1", e))?;
        Ok(())
    }
}

impl CuTaskLifecycle for SN754410 {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let deadzone: f32 = config.and_then(|c| c.get::<f64>("deadzone")).unwrap_or(0.0) as f32;
        let pwm0 = Pwm::with_frequency(Channel::Pwm0, PWM_FREQUENCY, 0.0, Polarity::Normal, false)
            .map_err(|e| CuError::new_with_cause("Failed to create PWM0", e))?;
        let pwm1 = Pwm::with_frequency(Channel::Pwm1, PWM_FREQUENCY, 0.0, Polarity::Normal, false)
            .map_err(|e| CuError::new_with_cause("Failed to create PWM0", e))?;

        Ok(Self {
            current_power: 0.0f32,
            pwm0,
            pwm1,
            last_update: CuTime::default(),
            deadzone,
        })
    }
    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        debug!("Enabling SN754410.");
        self.enable_pwms()
    }
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        debug!("Disabling SN754410.");
        self.disable_pwms()
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

impl<'cl> CuSinkTask<'cl> for SN754410 {
    type Input = input_msg!('cl, MotorMsg);

    fn process(&mut self, clock: &RobotClock, input: &mut CuMsg<Self::Input>) -> CuResult<()> {
        let power = input.payload().unwrap().power;
        let deadzone_compensated = if power != 0.0f32 {
            // proportinally on the [deadzone, 1.0] range*/
            let deadzone = self.deadzone;
            if power > 0.0 {
                deadzone + (1.0 - deadzone) * power
            } else {
                -deadzone + (1.0 + deadzone) * power
            }
        } else {
            power
        };

        if deadzone_compensated != self.current_power {
            self.current_power = deadzone_compensated;
            if deadzone_compensated > 0.0 {
                self.forward(self.current_power as f64)?;
            } else if self.current_power < 0.0 {
                self.reverse(-self.current_power as f64)?;
            } else {
                self.stop()?;
            }
            self.last_update = clock.now();
        } else {
            debug!("Power is the same {}, skipping.", deadzone_compensated);
        }
        Ok(())
    }
}

pub mod test_support {
    use crate::MotorMsg;
    use cu29::clock::RobotClock;
    use cu29::config::NodeInstanceConfig;
    use cu29::cutask::{CuMsg, CuSrcTask, CuTaskLifecycle, Freezable};
    use cu29::{output_msg, CuResult};

    pub struct SN754410TestSrc;

    impl Freezable for SN754410TestSrc {}

    impl CuTaskLifecycle for SN754410TestSrc {
        fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self> {
            Ok(Self {})
        }
    }

    impl<'cl> CuSrcTask<'cl> for SN754410TestSrc {
        type Output = output_msg!('cl, MotorMsg);

        fn process(&mut self, _clock: &RobotClock, _new_msg: Self::Output) -> CuResult<()> {
            todo!()
        }
    }
}
