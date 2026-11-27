//! Synchronize the operator clock with timestamped probes over the teleops link.
use cu_teleops_deadman::Command;
use cu29::prelude::*;
use cu29_clock::sync::{
    ClockDomain, ClockObservation, ClockSync, SyncConfig, SyncError, SyncState,
};
use std::fmt;

#[derive(Default)]
pub struct LinkStatus {
    pub synced: bool,
    pub rtt: Option<CuDuration>,
}

pub struct OperatorClock {
    clock: RobotClock,
    sync: ClockSync,
    domain: ClockDomain,
    last_probe: Option<CuInstant>,
    last_reply: Option<(CuInstant, CuDuration)>,
    pending: Option<CuInstant>,
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
            last_probe: None,
            last_reply: None,
            pending: None,
        })
    }

    pub fn probe(&mut self, permit: bool) -> Option<Command> {
        let sent = self.clock.raw_now();
        if self
            .last_probe
            .is_some_and(|last| sent - last < CuDuration::from_millis(100))
        {
            return None;
        }
        if self
            .pending
            .is_some_and(|last| sent - last < CuDuration::from_secs(1))
        {
            return None;
        }
        self.last_probe = Some(sent);
        self.pending = Some(sent);
        Some(Command::ClockProbe { sent, permit })
    }

    pub fn observe(
        &mut self,
        command: Option<&Command>,
        reply: Tov,
        received: CuInstant,
    ) -> Result<(), SyncError> {
        let (Some(Command::ClockProbe { sent, .. }), Tov::Range(robot)) = (command, reply) else {
            return Ok(());
        };
        if self.pending != Some(*sent) || *sent > received || robot.end < robot.start {
            return Ok(());
        }
        self.pending = None;
        let round_trip = received - *sent;
        let processing = robot.end - robot.start;
        if processing > round_trip {
            return Ok(());
        }
        let rtt = round_trip - processing;
        self.last_reply = Some((received, rtt));
        // Symmetric paths put the robot's midpoint at the twin's midpoint.
        let sample = ClockObservation {
            raw_local: *sent + CuDuration::from_nanos(round_trip.as_nanos() / 2),
            parent_ns: (robot.start + CuDuration::from_nanos(processing.as_nanos() / 2)).as_nanos(),
            uncertainty: CuDuration::from_nanos(rtt.as_nanos().div_ceil(2)),
            origin: ClockOrigin {
                source: ClockSource::LocalMonotonic,
                epoch: ClockEpoch::Local,
            },
            domain: self.domain,
        };
        match self.sync.observe(sample) {
            Ok(()) => {
                self.sync.update()?;
            }
            Err(SyncError::InvalidSample) => {}
            Err(SyncError::Expired | SyncError::Discontinuity) => self.sync.resync(self.domain)?,
            Err(err) => return Err(err),
        }
        Ok(())
    }

    pub fn link_status(&self, now: CuInstant) -> LinkStatus {
        LinkStatus {
            synced: self.synced(),
            rtt: self
                .last_reply
                .filter(|(at, _)| now - *at < CuDuration::from_secs(1))
                .map(|(_, rtt)| rtt),
        }
    }

    pub fn synced(&self) -> bool {
        self.sync.status().state == SyncState::Locked
    }
}

#[derive(Clone, Copy)]
pub struct Latency {
    pub pressed: CuTime,
    pub flipped: CuTime,
    pub synced: bool,
    pub confirmation: CuDuration,
    pub before_send: CuDuration,
}

impl fmt::Display for Latency {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if !self.synced {
            writeln!(f, "Press to action    Clock unsynced at press")?;
        } else if self.flipped < self.pressed + self.before_send
            || self.flipped > self.pressed + self.confirmation
        {
            let transit = self.confirmation - self.before_send;
            let estimate = self.before_send + CuDuration::from_nanos(transit.as_nanos() / 2);
            writeln!(f, "Press to action    ≈ {estimate} (RTT estimate)")?;
        } else {
            writeln!(f, "Press to action    ≈ {}", self.flipped - self.pressed)?;
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
        let mut operator = OperatorClock::new(clock.clone(), [7; 16]).unwrap();
        for i in 1..=3 {
            let local = 100 * i;
            raw.set_value(local * 1_000_000);
            let probe = operator.probe(false).unwrap();
            let reply = Tov::Range(CuTimeRange {
                start: CuTime::from_millis(50_000 + local + 2),
                end: CuTime::from_millis(50_000 + local + 5),
            });
            raw.set_value((local + 7) * 1_000_000);
            operator
                .observe(Some(&probe), reply, clock.raw_now())
                .unwrap();
        }
        assert!(operator.synced());
        assert_eq!(
            operator.link_status(clock.raw_now()).rtt,
            Some(CuDuration::from_millis(4))
        );
        assert_eq!(
            sampler.sample(true).tov(),
            Tov::Time(CuTime::from_millis(50_307))
        );
        raw.set_value(1_307_000_000);
        assert!(operator.link_status(clock.raw_now()).rtt.is_none());
    }
}
