//! A captured tick drives the watchdog during silence and replay.
use cu29::bincode::{
    Decode, Encode,
    de::Decoder,
    enc::Encoder,
    error::{DecodeError, EncodeError},
};
use cu29::prelude::*;

#[derive(Reflect)]
pub struct Tick;
impl Freezable for Tick {}
impl CuSrcTask for Tick {
    type Resources<'r> = ();
    type Output<'m> = CuMsg<CuTime>;
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, ctx: &CuContext, out: &mut Self::Output<'_>) -> CuResult<()> {
        let now = ctx.now();
        out.set_payload(now);
        out.tov = Tov::Time(now);
        const PULSE: [&str; 8] = [
            "▁▃█▅",
            "▁▂▆▇",
            "▃▁▅█",
            "▅▁▃▇",
            "█▃▁▅",
            "▇▆▂▁",
            "▅█▃▁",
            "▃▇▅▁",
        ];
        let phase = (ctx.cl_id() / 64) as usize % PULSE.len();
        out.metadata.set_status(PULSE[phase]);
        Ok(())
    }
}

/// Commands from the operator. A resume names the robot transition it acknowledges.
#[derive(
    Default,
    Clone,
    Copy,
    Debug,
    PartialEq,
    Eq,
    Reflect,
    Encode,
    Decode,
    serde::Serialize,
    serde::Deserialize,
)]
#[bincode(crate = "cu29::bincode")]
pub enum Command {
    Permit,
    #[default]
    Stop,
    Resume(CuTime),
}

/// Captured robot state. A trip retains the input and transition timestamps
/// until an explicit resume so a lost frame cannot erase the measurement.
#[derive(
    Default,
    Clone,
    Copy,
    Debug,
    PartialEq,
    Eq,
    Reflect,
    Encode,
    Decode,
    serde::Serialize,
    serde::Deserialize,
)]
#[bincode(crate = "cu29::bincode")]
pub enum Safety {
    #[default]
    Safe,
    Armed,
    ButtonTrip {
        pressed: CuTime,
        flipped: CuTime,
    },
    PermitLost {
        since: CuTime,
    },
}

#[derive(Default, Reflect)]
pub struct Deadman {
    until: Option<CuTime>,
    state: Safety,
}
impl Freezable for Deadman {
    fn freeze<E: Encoder>(&self, e: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&(self.until, self.state), e)
    }
    fn thaw<D: Decoder>(&mut self, d: &mut D) -> Result<(), DecodeError> {
        (self.until, self.state) = Decode::decode(d)?;
        Ok(())
    }
}
impl CuTask for Deadman {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m,Command,CuTime);
    type Output<'m> = CuMsg<Safety>;
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self::default())
    }
    fn process(
        &mut self,
        ctx: &CuContext,
        (button, tick): &Self::Input<'_>,
        safe: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let now = *tick
            .payload()
            .ok_or_else(|| CuError::from("missing robot tick"))?;
        let was_armed = self.state == Safety::Armed;
        if self.until.is_some_and(|end| now >= end) {
            self.until = None;
            self.state = Safety::PermitLost { since: now };
        }
        let latch = self.state.latch();
        match (button.payload(), button.tov) {
            (Some(Command::Stop), Tov::Time(pressed)) if self.state == Safety::Armed => {
                self.until = None;
                self.state = Safety::ButtonTrip {
                    pressed,
                    flipped: ctx.now(),
                };
            }
            (Some(command), Tov::Time(t)) if t <= now && now - t < crate::PERMIT_LIFETIME => {
                let enable = match command {
                    Command::Permit => latch.is_none(),
                    Command::Resume(id) => latch == Some(*id) && t >= *id,
                    Command::Stop => false,
                };
                if enable {
                    self.until = Some(t + crate::PERMIT_LIFETIME);
                    self.state = Safety::Armed;
                }
            }
            (Some(_), _) if self.state == Safety::Armed => {
                self.until = None;
                self.state = Safety::PermitLost { since: now };
            }
            _ => {}
        }
        if was_armed && self.state != Safety::Armed {
            error!(ctx, "Deadman safety trip: {}", self.state.status());
        }
        safe.set_payload(self.state);
        Ok(())
    }
}

impl Safety {
    pub fn latch(self) -> Option<CuTime> {
        match self {
            Self::ButtonTrip { flipped, .. } => Some(flipped),
            Self::PermitLost { since } => Some(since),
            _ => None,
        }
    }

    fn status(self) -> &'static str {
        match self {
            Self::Safe => "SAFEMODE: actuation off",
            Self::Armed => "ARMED: permit active",
            Self::ButtonTrip { .. } => "SAFEMODE: red button",
            Self::PermitLost { .. } => "SAFEMODE: permit lost",
        }
    }
}

/// Emit an actuation permit only while the watchdog is armed.
#[derive(Reflect)]
pub struct ActuationGate;
impl Freezable for ActuationGate {}
impl CuTask for ActuationGate {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(Safety);
    type Output<'m> = CuMsg<()>;
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(
        &mut self,
        _: &CuContext,
        input: &Self::Input<'_>,
        permit: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let state = input.payload().copied().unwrap_or_default();
        permit.metadata.set_status(state.status());
        if state != Safety::Armed {
            permit.clear_payload();
            return Err(CuError::from(state.status()));
        }
        permit.set_payload(());
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::bincode::{
        config::standard,
        de::{DecoderImpl, read::SliceReader},
    };

    fn cycle(policy: &mut Deadman, ms: u64, sample: Option<u64>) -> bool {
        let (ctx, _) = CuContext::new_mock_clock();
        let mut button = CuMsg::default();
        if let Some(at) = sample {
            button.set_payload(Command::Permit);
            button.tov = Tov::Time(CuTime::from_millis(at));
        }
        let mut tick = CuMsg::default();
        tick.set_payload(CuTime::from_millis(ms));
        let mut out = CuMsg::default();
        policy.process(&ctx, &(&button, &tick), &mut out).unwrap();
        let mut permit = CuMsg::default();
        let result = ActuationGate.process(&ctx, &out, &mut permit);
        assert_eq!(result.is_err(), permit.payload().is_none());
        result.is_err()
    }

    #[test]
    fn a_later_capture_still_identifies_the_press_that_tripped_the_robot() {
        let (ctx, clock) = CuContext::new_mock_clock();
        let mut policy = Deadman::default();
        assert!(!cycle(&mut policy, 110, Some(100)));
        let mut tick = CuMsg::default();
        tick.set_payload(CuTime::from_millis(120));
        let mut button = CuMsg::default();
        button.set_payload(Command::Stop);
        button.tov = Tov::Time(CuTime::from_millis(119));
        clock.set_value(120_000_002);
        let mut out = CuMsg::default();
        policy.process(&ctx, &(&button, &tick), &mut out).unwrap();
        let expected = Safety::ButtonTrip {
            pressed: CuTime::from_millis(119),
            flipped: CuTime(120_000_002),
        };
        assert_eq!(out.payload(), Some(&expected));
        let mut permit = CuMsg::default();
        permit.set_payload(());
        assert!(ActuationGate.process(&ctx, &out, &mut permit).is_err());
        assert!(permit.payload().is_none());
        button.clear_payload();
        tick.set_payload(CuTime::from_millis(400));
        policy.process(&ctx, &(&button, &tick), &mut out).unwrap();
        assert_eq!(out.payload(), Some(&expected));
        assert!(ActuationGate.process(&ctx, &out, &mut permit).is_err());
        // Neither a fresh heartbeat nor a resume for an earlier trip releases the latch.
        for command in [Command::Permit, Command::Resume(CuTime::from_millis(100))] {
            button.set_payload(command);
            button.tov = Tov::Time(CuTime::from_millis(400));
            policy.process(&ctx, &(&button, &tick), &mut out).unwrap();
            assert_eq!(out.payload(), Some(&expected));
        }
        button.set_payload(Command::Resume(expected.latch().unwrap()));
        policy.process(&ctx, &(&button, &tick), &mut out).unwrap();
        assert_eq!(out.payload(), Some(&Safety::Armed));
    }

    #[test]
    fn replay_from_mid_hold_expires_on_the_same_tick() {
        let mut robot = Deadman::default();
        assert!(cycle(&mut robot, 100, None));
        assert!(cycle(&mut robot, 180, Some(20)));
        assert!(!cycle(&mut robot, 220, Some(100)));
        let bytes = cu29::bincode::encode_to_vec(BincodeAdapter(&robot), standard()).unwrap();
        let mut replay = Deadman::default();
        replay
            .thaw(&mut DecoderImpl::new(
                SliceReader::new(&bytes),
                standard(),
                (),
            ))
            .unwrap();
        for (ms, expected) in [(249, false), (250, true), (251, true)] {
            assert_eq!(cycle(&mut robot, ms, None), expected);
            assert_eq!(cycle(&mut replay, ms, None), expected);
        }
    }
}
