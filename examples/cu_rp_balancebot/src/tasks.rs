use cu29::prelude::*;
use cu_ads7883_new::ADSReadingPayload;
use cu_pid::{GenericPIDTask, PIDControlOutputPayload};
use cu_rp_encoder::EncoderPayload;
use cu_rp_sn754410_new::MotorPayload;

pub type BalPID = GenericPIDTask<ADSReadingPayload>;
pub type PosPID = GenericPIDTask<EncoderPayload>;

pub struct PIDMerger {}

impl Freezable for PIDMerger {}

impl<'cl> CuTask<'cl> for PIDMerger {
    type Input = input_msg!('cl, PIDControlOutputPayload, PIDControlOutputPayload);
    type Output = output_msg!('cl, MotorPayload);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

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
        output.metadata.tov = bal_pid_msg.metadata.tov.clone();
        let composite_output = (bal_pid.output - pos_pid.output).clamp(-1.0, 1.0);
        output.set_payload(MotorPayload {
            power: composite_output,
        });
        output
            .metadata
            .set_status(format!("Comp:{:.2}", composite_output));

        Ok(())
    }
}
