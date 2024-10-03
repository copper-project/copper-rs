use cu29::clock::{CuDuration, CuTime, RobotClock};
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuTask, CuTaskLifecycle, Freezable};
use cu29::{input_msg, output_msg, CuResult};
use cu29_log_derive::debug;
use cu29_traits::CuError;
use cu_ads7883::ADSReadingPayload;
use cu_rp_encoder::EncoderPayload;
use cu_rp_sn754410::MotorPayload;

struct PIDControlOutput {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub output: f32,
}

struct PIDController {
    kp: f32,
    ki: f32,
    kd: f32,
    setpoint: f32,
    integral: f32,
    last_error: f32,
    p_limit: f32,
    i_limit: f32,
    d_limit: f32,
    output_limit: f32,
}

impl PIDController {
    fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        setpoint: f32,
        p_limit: f32,
        i_limit: f32,
        d_limit: f32,
        output_limit: f32,
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
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0f32;
        self.last_error = 0.0f32;
    }

    pub fn init_measurement(&mut self, measurement: f32) {
        self.last_error = self.setpoint - measurement;
    }

    pub fn next_control_output(&mut self, measurement: f32, dt: CuDuration) -> PIDControlOutput {
        let error = self.setpoint - measurement;
        let dt = dt.0 as f32 / 1_000_000f32; // the unit is kind of arbitrary.
        debug!("DT: {}", dt);

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

        PIDControlOutput { p, i, d, output }
    }
}

pub struct PIDTask {
    pid_balance: PIDController,
    pid_position: PIDController,
    cutoff: f32,
    // retain the last measurement to be able to only act on the change
    first_run: bool,
    last_tov_bal: CuTime,
    last_tov_pos: CuTime,
    last_bal: u16,
    last_pos: i32,
    current_power_bal: f32,
    current_power_pos: f32,
}

impl Freezable for PIDTask {}

impl CuTaskLifecycle for PIDTask {
    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        match config {
            Some(config) => {
                debug!("PIDTask config: {:?}", config);
                let setpoint: f32 = config
                    .get::<f64>("setpoint")
                    .ok_or("'setpoint' not found in config")?
                    as f32;
                let cutoff: f32 = config
                    .get::<f64>("cutoff")
                    .ok_or("'cutoff' not found in config")?
                    as f32;

                // p is mandatory
                let kp = if let Some(kp) = config.get::<f64>("kp") {
                    Ok(kp as f32)
                } else {
                    Err(CuError::from(
                        "'kp' not found in the config. We need at least 'kp' to make the PID algorithm work.",
                    ))
                }?;

                let pl = getcfg(config, "pl", 2.0f32);
                let ki = getcfg(config, "ki", 0.0f32);
                let il = getcfg(config, "il", 1.0f32);
                let kd = getcfg(config, "kd", 0.0f32);
                let dl = getcfg(config, "dl", 2.0f32);

                // Get rail PID parameters
                let setpoint_rail: f32 = getcfg(config, "setpoint_rail", 0.0f32);
                let kp_rail = getcfg(config, "kp_rail", 0.0f32);
                let pl_rail = getcfg(config, "pl_rail", 1.0f32);
                let ki_rail = getcfg(config, "ki_rail", 0.0f32);
                let il_rail = getcfg(config, "il_rail", 1.0f32);
                let kd_rail = getcfg(config, "kd_rail", 0.0f32);
                let dl_rail = getcfg(config, "dl_rail", 1.0f32);

                let output_limit = 1.0f32;

                let pid_balance: PIDController =
                    PIDController::new(kp, ki, kd, setpoint, pl, il, dl, output_limit);

                let pid_position: PIDController = PIDController::new(
                    kp_rail,
                    ki_rail,
                    kd_rail,
                    setpoint_rail,
                    pl_rail,
                    il_rail,
                    dl_rail,
                    output_limit,
                );

                Ok(Self {
                    pid_balance,
                    pid_position,
                    cutoff,
                    first_run: true,
                    last_tov_bal: CuTime::default(),
                    last_tov_pos: CuTime::default(),
                    last_bal: 0,
                    last_pos: 0,
                    current_power_bal: 0.0,
                    current_power_pos: 0.0,
                })
            }
            None => Err(CuError::from("PIDTask needs a config.")),
        }
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.pid_balance.reset();
        self.pid_position.reset();
        self.first_run = true;
        Ok(())
    }
}

// Small helper befause we do this again and again
fn getcfg(config: &ComponentConfig, key: &str, default: f32) -> f32 {
    if let Some(kd) = config.get::<f64>(key) {
        kd as f32
    } else {
        default
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
        let bal_tov = bal_pos.metadata.tov.unwrap();
        let rail_tov = rail_pos.metadata.tov.unwrap();
        let bal_pos = bal_pos.payload().unwrap().analog_value;
        let rail_pos = rail_pos.payload().unwrap().ticks;
        if self.first_run {
            self.first_run = false;
            self.last_tov_bal = bal_tov;
            self.last_tov_pos = rail_tov;
            self.last_bal = bal_pos;
            self.last_pos = rail_pos;
            self.pid_balance.init_measurement(self.last_bal as f32);
            self.pid_position.init_measurement(self.last_pos as f32);
            return Ok(());
        }

        let power_bal = if bal_pos != self.last_bal {
            // debug!(
            //     "{} / {}: PIDTask processing bal: {} rail:{}",
            //     clock.now(),
            //     bal_tov,
            //     bal_pos,
            //     rail_pos
            // );
            let bal_dt = bal_tov - self.last_tov_bal;
            let power_balance = self.pid_balance.next_control_output(bal_pos as f32, bal_dt);
            // debug!(
            //     "Balance output: input: {} p:{} i:{} d:{} total:{}",
            //     bal_pos, power_balance.p, power_balance.i, power_balance.d, power_balance.output
            // );

            self.last_bal = bal_pos;
            self.last_tov_bal = bal_tov;
            self.current_power_bal = power_balance.output;
            power_balance.output
        } else {
            self.current_power_bal
        };

        let power_pos = if rail_pos != self.last_pos {
            let rail_dt = rail_tov - self.last_tov_pos;
            let power_position = self
                .pid_position
                .next_control_output(rail_pos as f32, rail_dt);
            // debug!(
            //     "Position output: input: {} p:{} i:{} d:{} total:{}",
            //     rail_pos,
            //     power_position.p,
            //     power_position.i,
            //     power_position.d,
            //     power_position.output
            // );

            self.last_pos = rail_pos;
            self.last_tov_pos = rail_tov;
            self.current_power_pos = power_position.output;
            power_position.output
        } else {
            self.current_power_pos
        };

        let mut composite_output = power_bal - power_pos;
        composite_output = composite_output.clamp(-1.0, 1.0);

        match bal_pos as f32 {
            value if value < self.pid_balance.setpoint - self.cutoff => {
                // debug!("********** Rod position too low, stopping motors");
                output.set_payload(MotorPayload { power: 0.0 });
            }
            value if value > self.pid_balance.setpoint + self.cutoff => {
                // debug!("********** Rod position too high, stopping motors");
                output.set_payload(MotorPayload { power: 0.0 });
            }
            _ => {
                output.set_payload(MotorPayload {
                    power: composite_output,
                });
            }
        }

        Ok(())
    }
}
