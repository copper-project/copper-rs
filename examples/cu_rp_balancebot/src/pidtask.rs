use cu29::clock::RobotClock;
use cu29::config::NodeInstanceConfig;
use cu29::cutask::{CuMsg, CuTask, CuTaskLifecycle, Freezable};
use cu29::{input_msg, output_msg, CuResult};
use cu29_log_derive::debug;
use cu29_traits::CuError;
use cu_ads7883::ADSReadingPayload;
use cu_rp_encoder::EncoderPayload;
use cu_rp_sn754410::MotorPayload;
use pid::Pid;

pub struct PIDTask {
    pid: Pid<f32>,
    setpoint: f32,
    cutoff: f32,
}

impl Freezable for PIDTask {}

impl CuTaskLifecycle for PIDTask {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        match config {
            Some(config) => {
                debug!("PIDTask config: {:?}", config);
                let setpoint: f32 = config
                    .get::<f64>("setpoint")
                    .ok_or_else(|| "'setpoint' not found in config")?
                    as f32;
                let cutoff: f32 = config
                    .get::<f64>("cutoff")
                    .ok_or_else(|| "'cutoff' not found in config")?
                    as f32;

                let mut pid: Pid<f32> = Pid::new(setpoint, 1.0f32);

                // p is mandatory
                let kp: f64 = config.get("kp").ok_or_else(|| {
                    "'kp' not found in config, you need at least a proportional term"
                })?;

                pid.p(kp as f32, 1.0f32);

                // i and d are optional
                if let Some(ki) = config.get::<f64>("ki") {
                    pid.i(ki as f32, 1.0f32);
                }
                if let Some(kd) = config.get::<f64>("kd") {
                    pid.d(kd as f32, 2.0f32);
                }
                Ok(Self {
                    pid,
                    setpoint,
                    cutoff,
                })
            }
            None => Err(CuError::from("PIDTask needs a config.")),
        }
    }

    fn start(&mut self, clock: &RobotClock) -> CuResult<()> {
        debug!("PIDTask started at {}", clock.now());
        self.pid.reset_integral_term();
        Ok(())
    }
}

impl<'cl> CuTask<'cl> for PIDTask {
    type Input = input_msg!('cl, ADSReadingPayload, EncoderPayload);
    type Output = output_msg!('cl, MotorPayload);

    fn process(
        &mut self,
        clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        let (bal_pos, rail_pos) = input;
        let bal_pos = bal_pos.payload().unwrap();
        let rail_pos = rail_pos.payload().unwrap();

        debug!(
            "{}: PIDTask processing bal: {} rail:{}",
            clock.now(),
            bal_pos,
            rail_pos
        );
        let power = self.pid.next_control_output(bal_pos.analog_value as f32);
        debug!(
            "PIDTask output: input: {} p:{} i:{} d:{} total:{}",
            bal_pos.analog_value, power.p, power.i, power.d, power.output
        );
        match bal_pos.analog_value as f32 {
            value if value < self.setpoint - self.cutoff => {
                debug!("********** Rod position too low, stopping motors");
                output.set_payload(MotorPayload { power: 0.0 });
            }
            value if value > self.setpoint + self.cutoff => {
                debug!("********** Rod position too high, stopping motors");
                output.set_payload(MotorPayload { power: 0.0 });
            }
            _ => {
                output.set_payload(MotorPayload {
                    power: power.output,
                });
            }
        }

        Ok(())
    }
}
