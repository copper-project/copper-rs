use bincode::{Decode, Encode};
use cu29::clock::{CuDuration, CuTime, RobotClock};
use cu29::config::ComponentConfig;
use cu29::cutask::{CuMsg, CuMsgPayload, CuTask, CuTaskLifecycle, Freezable};
use cu29::{input_msg, output_msg, CuResult};
use cu29_log_derive::debug;
use cu29_traits::CuError;
use cu_ads7883::ADSReadingPayload;
use cu_rp_encoder::EncoderPayload;
use cu_rp_sn754410::MotorPayload;
use std::marker::PhantomData;

#[derive(Debug, Default, Clone, Encode, Decode)]
pub struct PIDControlOutput {
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

pub struct GenericPIDTask<I>
where
    f32: for<'a> From<&'a I>,
{
    _marker: PhantomData<I>,
    pid: PIDController,
    first_run: bool,
    last_tov: CuTime,
    setpoint: f32,
    cutoff: f32,
}

impl<I> CuTaskLifecycle for GenericPIDTask<I>
where
    f32: for<'a> From<&'a I>,
{
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

                let cutoff: f32 = config.get::<f64>("cutoff").ok_or(
                    "'cutoff' not found in config, please set an operating +/- limit on the input.",
                )? as f32;

                // p is mandatory
                let kp = if let Some(kp) = config.get::<f64>("kp") {
                    Ok(kp as f32)
                } else {
                    Err(CuError::from(
                        "'kp' not found in the config. We need at least 'kp' to make the PID algorithm work.",
                    ))
                }?;

                let p_limit = getcfg(config, "pl", 2.0f32);
                let ki = getcfg(config, "ki", 0.0f32);
                let i_limit = getcfg(config, "il", 1.0f32);
                let kd = getcfg(config, "kd", 0.0f32);
                let d_limit = getcfg(config, "dl", 2.0f32);
                let output_limit = getcfg(config, "ol", 1.0f32);

                let pid: PIDController = PIDController::new(
                    kp,
                    ki,
                    kd,
                    setpoint,
                    p_limit,
                    i_limit,
                    d_limit,
                    output_limit,
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

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.pid.reset();
        self.first_run = true;
        Ok(())
    }
}

impl<I> Freezable for GenericPIDTask<I> where f32: for<'a> From<&'a I> {}

// Small helper befause we do this again and again
fn getcfg(config: &ComponentConfig, key: &str, default: f32) -> f32 {
    if let Some(value) = config.get::<f64>(key) {
        value as f32
    } else {
        default
    }
}

pub type BalPID = GenericPIDTask<ADSReadingPayload>;
pub type PosPID = GenericPIDTask<EncoderPayload>;

impl<'cl, I> CuTask<'cl> for GenericPIDTask<I>
where
    f32: for<'a> From<&'a I>,
    I: CuMsgPayload + 'cl,
{
    type Input = input_msg!('cl, I);
    type Output = output_msg!('cl, PIDControlOutput);

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        match input.payload() {
            Some(payload) => {
                let tov = input.metadata.tov.unwrap();
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
                // But safety check if the input is within operation margins and cut power if it is not.
                if measure > self.setpoint + self.cutoff {
                    return Err(
                        format!("{} > {} (cutoff)", measure, self.setpoint + self.cutoff).into(),
                    );
                }
                if measure < self.setpoint - self.cutoff {
                    return Err(
                        format!("{} < {} (cutoff)", measure, self.setpoint - self.cutoff).into(),
                    );
                }

                output.set_payload(state);
            }
            None => output.clear_payload(),
        };
        Ok(())
    }
}

pub struct PIDMerger {}

impl CuTaskLifecycle for PIDMerger {
    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }
}

impl Freezable for PIDMerger {}

impl<'cl> CuTask<'cl> for PIDMerger {
    type Input = input_msg!('cl, PIDControlOutput, PIDControlOutput);
    type Output = output_msg!('cl, MotorPayload);

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
        output: Self::Output,
    ) -> CuResult<()> {
        let (bal_pid_msg, pos_pid_msg) = input;
        let bal_pid = match bal_pid_msg.payload() {
            Some(payload) => payload,
            None => return Err(CuError::from("Safety mode [balance].")),
        };
        let pos_pid = match pos_pid_msg.payload() {
            Some(payload) => payload,
            None => return Err(CuError::from("Safety mode [rail].")),
        };
        // Take the fastest measure as the reference time for the merge
        output.metadata.tov = bal_pid_msg.metadata.tov;
        let composite_output = (bal_pid.output - pos_pid.output).clamp(-1.0, 1.0);
        output.set_payload(MotorPayload {
            power: composite_output,
        });
        Ok(())
    }
}

/////////////////////////////////////////////////////////////////////////

// pub struct PIDTask {
//     pid_balance: PIDController,
//     pid_position: PIDController,
//     cutoff: f32,
//     // retain the last measurement to be able to only act on the change
//     first_run: bool,
//     last_tov_bal: CuTime,
//     last_tov_pos: CuTime,
//     last_bal: u16,
//     last_pos: i32,
//     current_power_bal: f32,
//     current_power_pos: f32,
// }
//
// impl<'cl> CuTask<'cl> for PIDTask {
//     type Input = input_msg!('cl, ADSReadingPayload, EncoderPayload);
//     type Output = output_msg!('cl, MotorPayload);
//
//     fn process(
//         &mut self,
//         _clock: &RobotClock,
//         input: Self::Input,
//         output: Self::Output,
//     ) -> CuResult<()> {
//         let (bal_pos, rail_pos) = input;
//         let bal_tov = bal_pos.metadata.tov.unwrap();
//         let rail_tov = rail_pos.metadata.tov.unwrap();
//         let bal_pos = bal_pos.payload().unwrap().analog_value;
//         let rail_pos = rail_pos.payload().unwrap().ticks;
//         if self.first_run {
//             self.first_run = false;
//             self.last_tov_bal = bal_tov;
//             self.last_tov_pos = rail_tov;
//             self.last_bal = bal_pos;
//             self.last_pos = rail_pos;
//             self.pid_balance.init_measurement(self.last_bal as f32);
//             self.pid_position.init_measurement(self.last_pos as f32);
//             return Ok(());
//         }
//
//         let power_bal = if bal_pos != self.last_bal {
//             let bal_dt = bal_tov - self.last_tov_bal;
//             let power_balance = self.pid_balance.next_control_output(bal_pos as f32, bal_dt);
//             self.last_bal = bal_pos;
//             self.last_tov_bal = bal_tov;
//             self.current_power_bal = power_balance.output;
//             power_balance.output
//         } else {
//             self.current_power_bal
//         };
//
//         let power_pos = if rail_pos != self.last_pos {
//             let rail_dt = rail_tov - self.last_tov_pos;
//             let power_position = self
//                 .pid_position
//                 .next_control_output(rail_pos as f32, rail_dt);
//
//             self.last_pos = rail_pos;
//             self.last_tov_pos = rail_tov;
//             self.current_power_pos = power_position.output;
//             power_position.output
//         } else {
//             self.current_power_pos
//         };
//
//         let mut composite_output = power_bal - power_pos;
//         composite_output = composite_output.clamp(-1.0, 1.0);
//
//         match bal_pos as f32 {
//             value if value < self.pid_balance.setpoint - self.cutoff => {
//                 // debug!("********** Rod position too low, stopping motors");
//                 output.set_payload(MotorPayload { power: 0.0 });
//             }
//             value if value > self.pid_balance.setpoint + self.cutoff => {
//                 // debug!("********** Rod position too high, stopping motors");
//                 output.set_payload(MotorPayload { power: 0.0 });
//             }
//             _ => {
//                 output.set_payload(MotorPayload {
//                     power: composite_output,
//                 });
//             }
//         }
//
//         Ok(())
//     }
// }
// impl Freezable for PIDTask {}
//
// impl CuTaskLifecycle for PIDTask {
//     fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
//     where
//         Self: Sized,
//     {
//         match config {
//             Some(config) => {
//                 debug!("PIDTask config: {:?}", config);
//                 let setpoint: f32 = config
//                     .get::<f64>("setpoint")
//                     .ok_or("'setpoint' not found in config")?
//                     as f32;
//                 let cutoff: f32 = config
//                     .get::<f64>("cutoff")
//                     .ok_or("'cutoff' not found in config")?
//                     as f32;
//
//                 // p is mandatory
//                 let kp = if let Some(kp) = config.get::<f64>("kp") {
//                     Ok(kp as f32)
//                 } else {
//                     Err(CuError::from(
//                         "'kp' not found in the config. We need at least 'kp' to make the PID algorithm work.",
//                     ))
//                 }?;
//
//                 let pl = getcfg(config, "pl", 2.0f32);
//                 let ki = getcfg(config, "ki", 0.0f32);
//                 let il = getcfg(config, "il", 1.0f32);
//                 let kd = getcfg(config, "kd", 0.0f32);
//                 let dl = getcfg(config, "dl", 2.0f32);
//
//                 // Get rail PID parameters
//                 let setpoint_rail: f32 = getcfg(config, "setpoint_rail", 0.0f32);
//                 let kp_rail = getcfg(config, "kp_rail", 0.0f32);
//                 let pl_rail = getcfg(config, "pl_rail", 1.0f32);
//                 let ki_rail = getcfg(config, "ki_rail", 0.0f32);
//                 let il_rail = getcfg(config, "il_rail", 1.0f32);
//                 let kd_rail = getcfg(config, "kd_rail", 0.0f32);
//                 let dl_rail = getcfg(config, "dl_rail", 1.0f32);
//
//                 let output_limit = 1.0f32;
//
//                 let pid_balance: PIDController =
//                     PIDController::new(kp, ki, kd, setpoint, pl, il, dl, output_limit);
//
//                 let pid_position: PIDController = PIDController::new(
//                     kp_rail,
//                     ki_rail,
//                     kd_rail,
//                     setpoint_rail,
//                     pl_rail,
//                     il_rail,
//                     dl_rail,
//                     output_limit,
//                 );
//
//                 Ok(Self {
//                     pid_balance,
//                     pid_position,
//                     cutoff,
//                     first_run: true,
//                     last_tov_bal: CuTime::default(),
//                     last_tov_pos: CuTime::default(),
//                     last_bal: 0,
//                     last_pos: 0,
//                     current_power_bal: 0.0,
//                     current_power_pos: 0.0,
//                 })
//             }
//             None => Err(CuError::from("PIDTask needs a config.")),
//         }
//     }
//
//     fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
//         self.pid_balance.reset();
//         self.pid_position.reset();
//         self.first_run = true;
//         Ok(())
//     }
// }
