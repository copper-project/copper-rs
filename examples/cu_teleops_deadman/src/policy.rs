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
        Ok(())
    }
}

#[derive(Default, Reflect)]
pub struct Deadman {
    until: Option<CuTime>,
}
impl Freezable for Deadman {
    fn freeze<E: Encoder>(&self, e: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.until, e)
    }
    fn thaw<D: Decoder>(&mut self, d: &mut D) -> Result<(), DecodeError> {
        self.until = Decode::decode(d)?;
        Ok(())
    }
}
impl CuTask for Deadman {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m,bool,CuTime);
    type Output<'m> = CuMsg<bool>;
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self::default())
    }
    fn process(
        &mut self,
        _: &CuContext,
        (button, tick): &Self::Input<'_>,
        safe: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        let now = *tick.payload().ok_or(CuError::from("missing robot tick"))?;
        if let Some(pressed) = button.payload() {
            self.until = match (pressed, button.tov) {
                (false, Tov::Time(t)) if t <= now => Some(t + CuDuration::from_millis(150)),
                _ => None,
            };
        }
        safe.set_payload(self.until.is_none_or(|end| now >= end));
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
            button.set_payload(false);
            button.tov = Tov::Time(CuTime::from_millis(at));
        }
        let mut tick = CuMsg::default();
        tick.set_payload(CuTime::from_millis(ms));
        let mut out = CuMsg::default();
        policy.process(&ctx, &(&button, &tick), &mut out).unwrap();
        *out.payload().unwrap()
    }

    #[test]
    fn replay_from_mid_hold_expires_on_the_same_tick() {
        let mut robot = Deadman::default();
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
