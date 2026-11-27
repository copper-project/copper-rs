//! Discipline the station's operator clock to the robot using ordinary permit echoes.
use cu_teleops_deadman::Command;
use cu29::prelude::*;
use cu29_clock::sync::{
    ClockDomain, ClockObservation, ClockSync, SyncConfig, SyncError, SyncState,
};
use std::collections::VecDeque;
use std::fmt;

pub struct OperatorClock {
    clock: RobotClock,
    sync: ClockSync,
    domain: ClockDomain,
    preceding_permit: Option<CuInstant>,
    permits: VecDeque<(CuTime, CuInstant)>,
}

impl OperatorClock {
    pub fn new(clock: RobotClock, session: [u8; 16]) -> Result<Self, SyncError> {
        let (identity, generation) = session.split_at(8);
        let domain = ClockDomain {
            id: 0,
            identity: identity.try_into().unwrap(),
            session: u64::from_le_bytes(generation.try_into().unwrap()),
        };
        let sync = ClockSync::new(&clock, domain, SyncConfig::new(CuDuration::from_millis(10)))?;
        Ok(Self {
            clock,
            sync,
            domain,
            preceding_permit: None,
            permits: VecDeque::new(),
        })
    }

    pub fn sent_permit(&mut self, tick: CuTime) {
        if self.permits.len() == 64 {
            self.permits.pop_front();
        }
        self.permits.push_back((tick, self.clock.raw_now()));
    }

    pub fn observe(&mut self, tick: CuTime, button: &CuMsg<Command>) -> Result<(), SyncError> {
        // This tick follows the CopperList that consumed the matched permit.
        // Its actual station time lies between that permit's send and this read.
        if let Some(sent) = self.preceding_permit.take() {
            let span = self.clock.raw_now() - sent;
            let half = CuDuration::from_nanos(span.as_nanos().div_ceil(2));
            let observation = ClockObservation {
                raw_local: sent + half,
                parent_ns: tick.as_nanos(),
                uncertainty: half,
                origin: ClockOrigin {
                    source: ClockSource::LocalMonotonic,
                    epoch: ClockEpoch::Local,
                },
                domain: self.domain,
            };
            match self.sync.observe(observation) {
                Ok(()) => {
                    self.sync.update()?;
                }
                Err(SyncError::InvalidSample) => {}
                Err(SyncError::Expired | SyncError::Discontinuity) => {
                    self.sync.resync(self.domain)?
                }
                Err(err) => return Err(err),
            }
        }
        if let (Some(Command::Permit), Tov::Time(reference)) = (button.payload(), button.tov) {
            while let Some(&(observed, sent)) = self.permits.front() {
                if observed > reference {
                    break;
                }
                self.permits.pop_front();
                if observed == reference {
                    self.preceding_permit = Some(sent);
                    break;
                }
            }
        }
        Ok(())
    }

    pub fn uncertainty(&self) -> Option<CuDuration> {
        let status = self.sync.status();
        (status.state == SyncState::Locked).then_some(status.estimated_error)
    }
}

#[derive(Clone, Copy)]
pub struct Latency {
    pub pressed: CuTime,
    pub flipped: CuTime,
    pub uncertainty: Option<CuDuration>,
    pub confirmation: CuDuration,
}

impl fmt::Display for Latency {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.uncertainty {
            Some(error) => {
                let end = self.pressed + self.confirmation;
                let first = (self.flipped - error).max(self.pressed);
                let last = (self.flipped + error).min(end);
                if first > last {
                    writeln!(f, "Press to action    Clock mismatch")?;
                } else if self.flipped < self.pressed || self.flipped > end {
                    writeln!(
                        f,
                        "Press to action    {}–{}",
                        first - self.pressed,
                        last - self.pressed
                    )?;
                } else {
                    writeln!(
                        f,
                        "Press to action    {} ± {error}",
                        self.flipped - self.pressed
                    )?;
                }
            }
            None => writeln!(f, "Press to action    Clock acquiring")?,
        }
        write!(f, "Press to confirmation    {}", self.confirmation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::teleops::InputSampler;

    #[test]
    fn operator_sampler_stamps_in_the_remote_robots_epoch() {
        let (clock, raw) = RobotClock::mock();
        let sampler = InputSampler::new(Some(clock.clone()));
        let mut operator = OperatorClock::new(clock, [7; 16]).unwrap();
        let mut button = CuMsg::default();
        for i in 1..=3 {
            let local = 10 * i;
            let tick = CuTime::from_millis(50_000 + local);
            raw.set_value(local * 1_000_000);
            operator.sent_permit(tick);
            button.set_payload(Command::Permit);
            button.tov = Tov::Time(tick);
            operator.observe(tick, &button).unwrap();
            raw.set_value((local + 4) * 1_000_000);
            button.clear_payload();
            operator
                .observe(tick + CuDuration::from_millis(2), &button)
                .unwrap();
        }
        assert!(operator.uncertainty().is_some());
        assert_eq!(
            sampler.sample(true).tov(),
            Tov::Time(CuTime::from_millis(50_034))
        );
    }
}
