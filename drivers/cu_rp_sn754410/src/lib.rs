use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::clock::RobotClock;
use cu29::config::NodeInstanceConfig;
use cu29::cutask::{CuMsg, CuSinkTask, CuTaskLifecycle, Freezable};
use cu29::CuResult;
use serde::{Deserialize, Serialize};

use cu29_traits::CuError;
use rppal::pwm::{Channel, Polarity, Pwm};

const PWM_FREQUENCY: f64 = 1000.0; // Frequency in Hz

pub struct SN754410 {
    current_power: f32, // retain what was the last state so we don't bang the hardware at each iteration.
    pwm0: Pwm,
    pwm1: Pwm,
}

#[derive(Debug, Clone, Copy, Default, Encode, Decode, PartialEq, Serialize, Deserialize)]
pub struct MotorMsg {
    pub power: f32, // -1.0 to 1.0
}

impl MotorMsg {}

impl CuTaskLifecycle for SN754410 {
    fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let pwm0 = Pwm::with_frequency(Channel::Pwm0, PWM_FREQUENCY, 0.0, Polarity::Normal, false)
            .map_err(|e| CuError::new_with_cause("Failed to create PWM0", e))?;
        let pwm1 = Pwm::with_frequency(Channel::Pwm1, PWM_FREQUENCY, 0.0, Polarity::Normal, false)
            .map_err(|e| CuError::new_with_cause("Failed to create PWM0", e))?;

        Ok(Self {
            current_power: 0.0f32,
            pwm0,
            pwm1,
        })
    }
    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.pwm0
            .enable()
            .map_err(|e| CuError::new_with_cause("Failed to enable PWM0", e))?;
        self.pwm1
            .enable()
            .map_err(|e| CuError::new_with_cause("Failed to enable PWM1", e))?;
        Ok(())
    }
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.pwm0
            .disable()
            .map_err(|e| CuError::new_with_cause("Failed to disable PWM0", e))?;
        self.pwm1
            .disable()
            .map_err(|e| CuError::new_with_cause("Failed to disable PWM1", e))?;
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
    type Input = MotorMsg;

    fn process(&mut self, _clock: &RobotClock, input: &CuMsg<Self::Input>) -> CuResult<()> {
        let power = input.payload().unwrap().power;
        if power != self.current_power {
            self.current_power = power;
            if self.current_power > 0.0 {
                self.pwm1
                    .set_duty_cycle(0.0)
                    .map_err(|e| CuError::new_with_cause("Failed to set PWM1 duty cycle", e))?;
                self.pwm0
                    .set_duty_cycle(self.current_power.abs() as f64)
                    .map_err(|e| CuError::new_with_cause("Failed to set PWM0 duty cycle", e))?;
            } else if self.current_power < 0.0 {
                self.pwm0
                    .set_duty_cycle(0.0)
                    .map_err(|e| CuError::new_with_cause("Failed to set PWM0 duty cycle", e))?;
                self.pwm1
                    .set_duty_cycle(self.current_power.abs() as f64)
                    .map_err(|e| CuError::new_with_cause("Failed to set PWM1 duty cycle", e))?;
            } else {
                self.pwm0
                    .set_duty_cycle(0.0)
                    .map_err(|e| CuError::new_with_cause("Failed to set PWM0 duty cycle", e))?;
                self.pwm1
                    .set_duty_cycle(0.0)
                    .map_err(|e| CuError::new_with_cause("Failed to set PWM1 duty cycle", e))?;
            }
        }
        Ok(())
    }
}

pub mod test_support {
    use crate::MotorMsg;
    use cu29::clock::RobotClock;
    use cu29::config::NodeInstanceConfig;
    use cu29::cutask::{CuMsg, CuSrcTask, CuTaskLifecycle, Freezable};
    use cu29::CuResult;

    pub struct SN754410TestSrc;

    impl Freezable for SN754410TestSrc {}

    impl CuTaskLifecycle for SN754410TestSrc {
        fn new(_config: Option<&NodeInstanceConfig>) -> CuResult<Self> {
            Ok(Self {})
        }
    }

    impl CuSrcTask for SN754410TestSrc {
        type Output = MotorMsg;

        fn process(
            &mut self,
            _clock: &RobotClock,
            _new_msg: &mut CuMsg<Self::Output>,
        ) -> CuResult<()> {
            todo!()
        }
    }
}
