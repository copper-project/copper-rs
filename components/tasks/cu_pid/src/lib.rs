#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use core::marker::PhantomData;
use cu29::prelude::*;
use cu29::reflect::{Reflect, ReflectTypePath};
use serde::{Deserialize, Serialize};

#[cfg(not(feature = "std"))]
use alloc::format;

/// Output of the PID controller.
#[derive(Debug, Default, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct PIDControlOutputPayload {
    /// Proportional term
    pub p: f32,
    /// Integral term
    pub i: f32,
    /// Derivative term
    pub d: f32,
    /// Final output
    pub output: f32,
}

/// This is the underlying standard PID controller.
#[derive(Reflect)]
pub struct PIDController {
    // Configuration
    kp: f32,
    ki: f32,
    kd: f32,
    setpoint: f32,
    p_limit: f32,
    i_limit: f32,
    d_limit: f32,
    output_limit: f32,
    sampling: CuDuration,
    // Internal state
    integral: f32,
    last_error: f32,
    elapsed: CuDuration,
    last_output: PIDControlOutputPayload,
}

impl PIDController {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        setpoint: f32,
        p_limit: f32,
        i_limit: f32,
        d_limit: f32,
        output_limit: f32,
        sampling: CuDuration, // to avoid oversampling and get a bunch of zeros.
    ) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            setpoint,
            integral: 0.0,
            last_error: 0.0,
            p_limit,
            i_limit,
            d_limit,
            output_limit,
            elapsed: CuDuration::default(),
            sampling,
            last_output: PIDControlOutputPayload::default(),
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0f32;
        self.last_error = 0.0f32;
    }

    pub fn reset_integral(&mut self) {
        self.integral = 0.0f32;
    }

    pub fn init_measurement(&mut self, measurement: f32) {
        self.last_error = self.setpoint - measurement;
        self.elapsed = self.sampling; // force the computation on the first next_control_output
    }

    pub fn next_control_output(
        &mut self,
        measurement: f32,
        dt: CuDuration,
    ) -> PIDControlOutputPayload {
        self.elapsed += dt;

        if self.elapsed < self.sampling {
            // if we bang too fast the PID controller, just keep on giving the same answer
            return self.last_output.clone();
        }

        let error = self.setpoint - measurement;
        let CuDuration(elapsed) = self.elapsed;
        let dt = elapsed as f32 / 1_000_000f32; // the unit is kind of arbitrary.
        if dt == 0.0 {
            return self.last_output.clone();
        }

        // Proportional term
        let p_unbounded = self.kp * error;
        let p = p_unbounded.clamp(-self.p_limit, self.p_limit);

        // Integral term (accumulated over time)
        self.integral += error * dt;
        let i_unbounded = self.ki * self.integral;
        let i = i_unbounded.clamp(-self.i_limit, self.i_limit);

        // Derivative term (rate of change)
        let derivative = (error - self.last_error) / dt;
        let d_unbounded = self.kd * derivative;
        let d = d_unbounded.clamp(-self.d_limit, self.d_limit);

        // Update last error for next calculation
        self.last_error = error;

        // Final output: sum of P, I, D with output limit
        let output_unbounded = p + i + d;
        let output = output_unbounded.clamp(-self.output_limit, self.output_limit);

        let output = PIDControlOutputPayload { p, i, d, output };

        self.last_output = output.clone();
        self.elapsed = CuDuration::default();
        output
    }
}

/// This is the Copper task encapsulating the PID controller.
#[derive(Reflect)]
pub struct GenericPIDTask<I>
where
    f32: for<'a> From<&'a I>,
{
    #[reflect(ignore)]
    _marker: PhantomData<fn() -> I>,
    pid: PIDController,
    first_run: bool,
    last_tov: CuTime,
    setpoint: f32,
    cutoff: f32,
}

impl<I> CuTask for GenericPIDTask<I>
where
    f32: for<'a> From<&'a I>,
    I: CuMsgPayload + ReflectTypePath + 'static,
{
    type Resources<'r> = ();
    type Input<'m> = input_msg!(I);
    type Output<'m> = output_msg!(PIDControlOutputPayload);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        match config {
            Some(config) => {
                debug!("PIDTask config loaded");
                let setpoint: f32 = config
                    .get::<f64>("setpoint")?
                    .ok_or("'setpoint' not found in config")?
                    as f32;

                let cutoff: f32 = config.get::<f64>("cutoff")?.ok_or(
                    "'cutoff' not found in config, please set an operating +/- limit on the input.",
                )? as f32;

                // p is mandatory
                let kp = match config.get::<f64>("kp")? {
                    Some(kp) => Ok(kp as f32),
                    None => Err(CuError::from(
                        "'kp' not found in the config. We need at least 'kp' to make the PID algorithm work.",
                    )),
                }?;

                let p_limit = getcfg(config, "pl", 2.0f32)?;
                let ki = getcfg(config, "ki", 0.0f32)?;
                let i_limit = getcfg(config, "il", 1.0f32)?;
                let kd = getcfg(config, "kd", 0.0f32)?;
                let d_limit = getcfg(config, "dl", 2.0f32)?;
                let output_limit = getcfg(config, "ol", 1.0f32)?;

                let sampling = if let Some(value) = config.get::<u32>("sampling_ms")? {
                    CuDuration::from(value as u64 * 1_000_000u64)
                } else {
                    CuDuration::default()
                };

                let pid: PIDController = PIDController::new(
                    kp,
                    ki,
                    kd,
                    setpoint,
                    p_limit,
                    i_limit,
                    d_limit,
                    output_limit,
                    sampling,
                );

                Ok(Self {
                    _marker: PhantomData,
                    pid,
                    first_run: true,
                    last_tov: CuTime::default(),
                    setpoint,
                    cutoff,
                })
            }
            None => Err(CuError::from("PIDTask needs a config.")),
        }
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.tov = input.tov;
        match input.payload() {
            Some(payload) => {
                let tov = match input.tov {
                    Tov::Time(single) => single,
                    _ => return Err("Unexpected variant for a TOV of PID".into()),
                };

                let measure: f32 = payload.into();

                if self.first_run {
                    self.first_run = false;
                    self.last_tov = tov;
                    self.pid.init_measurement(measure);
                    output.clear_payload();
                    return Ok(());
                }
                let dt = tov - self.last_tov;
                self.last_tov = tov;

                // update the status of the pid.
                let state = self.pid.next_control_output(measure, dt);
                // But safety check if the input is within operational margins and cut power if it is not.
                let upper_limit = self.setpoint + self.cutoff;
                let lower_limit = self.setpoint - self.cutoff;
                if measure > upper_limit {
                    return Err(format!("{} > {} (cutoff)", measure, upper_limit).into());
                }
                if measure < lower_limit {
                    return Err(format!("{} < {} (cutoff)", measure, lower_limit).into());
                }
                output.metadata.set_status(format!(
                    "{:>5.2} {:>5.2} {:>5.2} {:>5.2}",
                    &state.output, &state.p, &state.i, &state.d
                ));
                output.set_payload(state);
            }
            None => output.clear_payload(),
        };
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.pid.reset();
        self.first_run = true;
        Ok(())
    }
}

/// Store/Restore the internal state of the PID controller.
impl<I> Freezable for GenericPIDTask<I>
where
    f32: for<'a> From<&'a I>,
{
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.pid.integral, encoder)?;
        Encode::encode(&self.pid.last_error, encoder)?;
        Encode::encode(&self.pid.elapsed, encoder)?;
        Encode::encode(&self.pid.last_output, encoder)?;
        Encode::encode(&self.first_run, encoder)?;
        Encode::encode(&self.last_tov, encoder)?;
        Ok(())
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.pid.integral = Decode::decode(decoder)?;
        self.pid.last_error = Decode::decode(decoder)?;
        self.pid.elapsed = Decode::decode(decoder)?;
        self.pid.last_output = Decode::decode(decoder)?;
        self.first_run = Decode::decode(decoder)?;
        self.last_tov = Decode::decode(decoder)?;
        Ok(())
    }
}

// Small helper befause we do this again and again
fn getcfg(config: &ComponentConfig, key: &str, default: f32) -> Result<f32, ConfigError> {
    Ok(config
        .get::<f64>(key)?
        .map(|value| value as f32)
        .unwrap_or(default))
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config::standard;
    use bincode::de::DecoderImpl;
    use bincode::de::read::SliceReader;
    use bincode::encode_to_vec;

    #[derive(Clone, Copy)]
    struct TestInput;

    impl From<&TestInput> for f32 {
        fn from(_: &TestInput) -> Self {
            0.0
        }
    }

    fn sample_task() -> GenericPIDTask<TestInput> {
        GenericPIDTask {
            _marker: PhantomData,
            pid: PIDController {
                kp: 1.0,
                ki: 2.0,
                kd: 3.0,
                setpoint: 4.0,
                p_limit: 5.0,
                i_limit: 6.0,
                d_limit: 7.0,
                output_limit: 8.0,
                sampling: CuDuration::from(9),
                integral: 10.0,
                last_error: 11.0,
                elapsed: CuDuration::from(12),
                last_output: PIDControlOutputPayload {
                    p: 13.0,
                    i: 14.0,
                    d: 15.0,
                    output: 16.0,
                },
            },
            first_run: false,
            last_tov: CuTime::from(17_u64),
            setpoint: 18.0,
            cutoff: 19.0,
        }
    }

    #[test]
    fn freeze_thaw_restores_pid_timekeeping_state() {
        let original = sample_task();
        let bytes =
            encode_to_vec(BincodeAdapter(&original), standard()).expect("encode pid task state");

        let mut restored = sample_task();
        restored.pid.integral = -1.0;
        restored.pid.last_error = -2.0;
        restored.pid.elapsed = CuDuration::from(999);
        restored.pid.last_output = PIDControlOutputPayload {
            p: -3.0,
            i: -4.0,
            d: -5.0,
            output: -6.0,
        };
        restored.first_run = true;
        restored.last_tov = CuTime::from(1_000_u64);

        let reader = SliceReader::new(&bytes);
        let mut decoder = DecoderImpl::new(reader, standard(), ());
        restored.thaw(&mut decoder).expect("thaw pid task state");

        assert_eq!(restored.pid.integral, original.pid.integral);
        assert_eq!(restored.pid.last_error, original.pid.last_error);
        assert_eq!(restored.pid.elapsed, original.pid.elapsed);
        assert_eq!(restored.pid.last_output.p, original.pid.last_output.p);
        assert_eq!(restored.pid.last_output.i, original.pid.last_output.i);
        assert_eq!(restored.pid.last_output.d, original.pid.last_output.d);
        assert_eq!(
            restored.pid.last_output.output,
            original.pid.last_output.output
        );
        assert_eq!(restored.first_run, original.first_run);
        assert_eq!(restored.last_tov, original.last_tov);
    }
}
