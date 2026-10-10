use cu29::clock::sync::{ClockDomain, ClockObservation, ClockSnapshot};
use cu29::prelude::*;

fn main() {
    let (clock, _) = RobotClock::mock();
    let raw = clock.raw_now();
    let execution = clock.now();
    let _ = execution - raw; //~ E0277
    let _ = raw - execution; //~ E0277
    let _ = raw == execution; //~ E0308
    let _ = raw < execution; //~ E0308
    clock.busy_wait_until(execution); //~ E0308
    let _ = Tov::Time(raw); //~ E0308
    let _ = CuTime::from(raw); //~ E0277
    let _ = ClockObservation {
        raw_local: execution, //~ E0308
        parent_ns: 0,
        uncertainty: CuDuration(0),
        domain: ClockDomain { id: 0, identity: [0; 8], session: 0 },
    };
}

fn snapshot_input(snapshot: &ClockSnapshot, execution: CuTime) {
    let _ = snapshot.at(execution); //~ E0308
    let _ = snapshot.status(execution); //~ E0308
}
