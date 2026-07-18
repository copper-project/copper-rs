use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use std::marker::PhantomData;

fn phase_at_or_before(tov: CuTime, interval: CuDuration) -> CuTime {
    let interval_ns = interval.as_nanos();
    CuTime::from(tov.as_nanos() / interval_ns * interval_ns)
}

#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false, type_path = false)]
pub struct CuRateLimit<T>
where
    T: CuMsgPayload + 'static,
{
    #[reflect(ignore)]
    _marker: PhantomData<fn() -> T>,
    interval: CuDuration,
    last_tov: Option<CuTime>,
}

impl<T> TypePath for CuRateLimit<T>
where
    T: CuMsgPayload + 'static,
{
    fn type_path() -> &'static str {
        "cu_ratelimit::CuRateLimit"
    }

    fn short_type_path() -> &'static str {
        "CuRateLimit"
    }

    fn type_ident() -> Option<&'static str> {
        Some("CuRateLimit")
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_ratelimit")
    }

    fn module_path() -> Option<&'static str> {
        Some("")
    }
}

impl<T> Freezable for CuRateLimit<T>
where
    T: CuMsgPayload,
{
    fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.last_tov, encoder)
    }

    fn thaw<D: Decoder>(&mut self, decoder: &mut D) -> Result<(), DecodeError> {
        self.last_tov = Decode::decode(decoder)?;
        Ok(())
    }
}

impl<T> CuTask for CuRateLimit<T>
where
    T: CuMsgPayload,
{
    type Resources<'r> = ();
    type Input<'m> = input_msg!(T);
    type Output<'m> = output_msg!(T);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let hz = match config {
            Some(cfg) => cfg
                .get::<f64>("rate")?
                .ok_or("Missing required 'rate' config for CuRateLimiter")?,
            None => return Err("Missing required 'rate' config for CuRateLimiter".into()),
        };
        if !hz.is_finite() || hz <= 0.0 {
            return Err("CuRateLimiter 'rate' must be finite and greater than zero".into());
        }
        let interval_ns = (1e9 / hz) as u64;
        if interval_ns == 0 {
            return Err("CuRateLimiter 'rate' cannot exceed 1 GHz".into());
        }
        Ok(Self {
            _marker: PhantomData,
            interval: CuDuration::from(interval_ns),
            last_tov: None,
        })
    }

    fn process<'m>(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'m>,
        output: &mut Self::Output<'m>,
    ) -> CuResult<()> {
        let tov = match input.tov {
            Tov::Time(ts) => ts,
            _ => return Err("Expected single timestamp TOV".into()),
        };
        output.tov = input.tov;

        let allow = match self.last_tov {
            None => {
                self.last_tov = Some(phase_at_or_before(tov, self.interval));
                true
            }
            Some(last) if tov < last => {
                // Re-anchor after replay/simulation time is reset.
                self.last_tov = Some(phase_at_or_before(tov, self.interval));
                true
            }
            Some(last) => {
                let elapsed = tov - last;
                if elapsed < self.interval {
                    false
                } else {
                    let elapsed_intervals = elapsed.as_nanos() / self.interval.as_nanos();
                    self.last_tov = Some(last + elapsed_intervals * self.interval);
                    true
                }
            }
        };

        if allow {
            if let Some(payload) = input.payload() {
                output.set_payload(payload.clone());
            } else {
                output.clear_payload();
            }
        } else {
            output.clear_payload();
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_ratelimiter(rate: f64) -> CuRateLimit<i32> {
        let mut cfg = ComponentConfig::new();
        cfg.set("rate", rate);
        CuRateLimit::new(Some(&cfg), ()).unwrap()
    }

    #[test]
    fn test_rate_limiting() {
        let ctx = CuContext::new_with_clock();
        let mut limiter = create_test_ratelimiter(10.0); // 10 Hz = 100ms interval
        let mut input = CuMsg::<i32>::new(Some(42));
        let mut output = CuMsg::<i32>::new(None);

        // First message should pass
        input.tov = Tov::Time(CuTime::from(0));
        limiter.process(&ctx, &input, &mut output).unwrap();
        assert_eq!(output.payload(), Some(&42));

        // Message within the interval should be blocked
        input.tov = Tov::Time(CuTime::from(50_000_000)); // 50ms
        limiter.process(&ctx, &input, &mut output).unwrap();
        assert_eq!(output.payload(), None);

        // Message after the interval should pass
        input.tov = Tov::Time(CuTime::from(100_000_000)); // 100ms
        limiter.process(&ctx, &input, &mut output).unwrap();
        assert_eq!(output.payload(), Some(&42));
        assert_eq!(output.tov, input.tov);
    }

    #[test]
    fn test_payload_propagation() {
        let ctx = CuContext::new_with_clock();
        let mut limiter = create_test_ratelimiter(10.0);
        let mut input = CuMsg::<i32>::new(None);
        let mut output = CuMsg::<i32>::new(None);

        // Test payload propagation
        input.set_payload(123);
        input.tov = Tov::Time(CuTime::from(0));
        limiter.process(&ctx, &input, &mut output).unwrap();
        assert_eq!(output.payload(), Some(&123));

        // Test empty payload propagation
        input.clear_payload();
        input.tov = Tov::Time(CuTime::from(100_000_000));
        limiter.process(&ctx, &input, &mut output).unwrap();
        assert_eq!(output.payload(), None);
    }

    #[test]
    fn test_phase_preserving_64_hz_to_30_hz() {
        let ctx = CuContext::new_with_clock();
        let mut limiter = create_test_ratelimiter(30.0);
        let mut input = CuMsg::<i32>::new(Some(42));
        let mut output = CuMsg::<i32>::new(None);
        let mut emitted = 0;

        // 64 source samples span [0, 984.375 ms], which contains exactly 30
        // points of the 30 Hz output phase including the initial sample.
        for source_tick in 0..64 {
            input.tov = Tov::Time(CuTime::from(source_tick * 15_625_000));
            limiter.process(&ctx, &input, &mut output).unwrap();
            emitted += usize::from(output.payload().is_some());
        }

        assert_eq!(emitted, 30);
    }

    #[test]
    fn test_first_sample_uses_absolute_rate_phase() {
        let ctx = CuContext::new_with_clock();
        let mut limiter = create_test_ratelimiter(30.0);
        let mut input = CuMsg::<i32>::new(Some(42));
        let mut output = CuMsg::<i32>::new(None);

        // A camera that becomes ready off-phase must join the same 30 Hz
        // schedule as other graph inputs instead of creating its own phase.
        input.tov = Tov::Time(CuTime::from(78_125_000));
        limiter.process(&ctx, &input, &mut output).unwrap();
        assert!(output.payload().is_some());

        input.tov = Tov::Time(CuTime::from(93_750_000));
        limiter.process(&ctx, &input, &mut output).unwrap();
        assert!(output.payload().is_none());

        input.tov = Tov::Time(CuTime::from(109_375_000));
        limiter.process(&ctx, &input, &mut output).unwrap();
        assert!(output.payload().is_some());
    }

    #[test]
    fn test_time_rewind_reanchors_limiter() {
        let ctx = CuContext::new_with_clock();
        let mut limiter = create_test_ratelimiter(30.0);
        let mut input = CuMsg::<i32>::new(Some(42));
        let mut output = CuMsg::<i32>::new(None);

        input.tov = Tov::Time(CuTime::from(1_000_000_000));
        limiter.process(&ctx, &input, &mut output).unwrap();
        assert!(output.payload().is_some());

        input.tov = Tov::Time(CuTime::from(1_000_000));
        limiter.process(&ctx, &input, &mut output).unwrap();
        assert!(output.payload().is_some());
    }

    #[test]
    fn test_invalid_rates_are_rejected() {
        for rate in [0.0, -1.0, f64::NAN, f64::INFINITY, 1_000_000_001.0] {
            let mut cfg = ComponentConfig::new();
            cfg.set("rate", rate);
            assert!(CuRateLimit::<i32>::new(Some(&cfg), ()).is_err());
        }
    }
}
