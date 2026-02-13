use cu_ads7883_new::ADSReadingPayload;
use cu_pid::{GenericPIDTask, PIDControlOutputPayload};
use cu_rp_encoder::EncoderPayload;
use cu_rp_sn754410_new::MotorPayload;
use cu29::prelude::*;
use cu29::units::si::f32::Ratio;
use cu29::units::si::ratio::ratio;

pub type BalPID = GenericPIDTask<ADSReadingPayload>;
pub type PosPID = GenericPIDTask<EncoderPayload>;

#[derive(Reflect)]
pub struct PIDMerger {}

impl Freezable for PIDMerger {}

impl CuTask for PIDMerger {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, PIDControlOutputPayload, PIDControlOutputPayload);
    type Output<'m> = output_msg!(MotorPayload);

    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let (bal_pid_msg, pos_pid_msg) = *input;
        let bal_pid = match bal_pid_msg.payload() {
            Some(payload) => payload,
            None => return Err(CuError::from("Safety mode [balance].")),
        };
        let pos_pid = match pos_pid_msg.payload() {
            Some(payload) => payload,
            None => return Err(CuError::from("Safety mode [rail].")),
        };
        output.tov = merge_tov(bal_pid_msg.tov, pos_pid_msg.tov);
        let composite_output = (bal_pid.output - pos_pid.output).clamp(-1.0, 1.0);
        output.set_payload(MotorPayload {
            power: Ratio::new::<ratio>(composite_output),
        });
        output
            .metadata
            .set_status(format!("Comp:{composite_output:.2}"));

        Ok(())
    }
}

fn merge_tov(left: Tov, right: Tov) -> Tov {
    match (left, right) {
        (Tov::None, other) | (other, Tov::None) => other,
        (Tov::Time(a), Tov::Time(b)) => {
            let (start, end) = if a <= b { (a, b) } else { (b, a) };
            Tov::Range(CuTimeRange { start, end })
        }
        (Tov::Range(range), Tov::Time(time)) | (Tov::Time(time), Tov::Range(range)) => {
            Tov::Range(CuTimeRange {
                start: range.start.min(time),
                end: range.end.max(time),
            })
        }
        (Tov::Range(left), Tov::Range(right)) => Tov::Range(CuTimeRange {
            start: left.start.min(right.start),
            end: left.end.max(right.end),
        }),
    }
}
