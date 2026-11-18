//! Worker contract for composing required and optional telemetry outputs.
use cu29_clock::CuTime;
use cu29_logstream::protocol::{BestEffort, CuTelemetryRecord, CuTelemetryTx, Fanout, TxMode};
use cu29_traits::CuResult;

#[derive(Default)]
struct Probe {
    deadline: Option<CuTime>,
    fail: bool,
    sends: usize,
    polls: Vec<TxMode>,
}

impl CuTelemetryTx for Probe {
    fn send(&mut self, _: CuTelemetryRecord<'_>) -> CuResult<()> {
        self.sends += 1;
        if self.fail {
            Err("send failed".into())
        } else {
            Ok(())
        }
    }

    fn poll(&mut self, mode: TxMode) -> CuResult<Option<CuTime>> {
        self.polls.push(mode);
        if self.fail {
            Err("poll failed".into())
        } else {
            Ok(self.deadline)
        }
    }
}

#[test]
fn both_children_run_on_error_and_contribute_to_drain_deadline() {
    let mut tx = Fanout(Probe::default(), Probe::default());
    tx.0.fail = true;
    assert!(
        tx.send(CuTelemetryRecord::CopperList {
            id: 3,
            capture: &[42]
        })
        .is_err()
    );
    assert_eq!(tx.1.sends, 1);
    assert!(tx.poll(TxMode::Running).is_err());
    assert_eq!(tx.1.polls, [TxMode::Running]);

    tx.0.fail = false;
    tx.0.deadline = Some(CuTime(20));
    tx.1.deadline = Some(CuTime(12));
    assert_eq!(tx.poll(TxMode::Running).unwrap(), Some(CuTime(12)));
    tx.1.deadline = None;
    assert_eq!(tx.poll(TxMode::Draining).unwrap(), Some(CuTime(20)));
    tx.0.deadline = None;
    assert_eq!(tx.poll(TxMode::Draining).unwrap(), None);
    assert_eq!(tx.0.polls, tx.1.polls);
}

#[test]
fn optional_failure_reports_once_and_leaves_the_other_branch_active() {
    for fail_in_poll in [true, false] {
        let mut errors = Vec::new();
        let optional = Probe {
            fail: true,
            ..Default::default()
        };
        let mut tx = Fanout(
            BestEffort::new(optional, |e| errors.push(e)),
            Probe::default(),
        );
        if fail_in_poll {
            tx.poll(TxMode::Running).unwrap();
        }
        let record = CuTelemetryRecord::KeyFrame {
            copperlist_id: 0,
            state: &[],
        };
        tx.send(record).unwrap();
        tx.send(record).unwrap();
        tx.poll(TxMode::Draining).unwrap();
        assert_eq!(tx.1.sends, 2);
        assert_eq!(errors.len(), 1);
        let expected = if fail_in_poll {
            "poll failed"
        } else {
            "send failed"
        };
        assert!(errors[0].to_string().contains(expected));
    }
}
