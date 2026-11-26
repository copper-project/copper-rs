//! Exercise a schema through the same import path used by robot and twin applications.
#![cfg(feature = "teleops")]
use cu29::logstream::CuStreamTxError;
use cu29::prelude::*;
use cu29::teleops::{CommandId, Submit};

#[derive(Teleops)]
#[allow(dead_code)]
struct Controls {
    stick: Latest<u8>,
    mode: Latched<bool>,
    click: Event<u8, 2>,
}

#[derive(Default)]
struct Recorder(Vec<(u16, Option<u64>)>);
impl Submit for Recorder {
    fn submit<T: bincode::Encode>(
        &mut self,
        tag: u16,
        frame: Option<u64>,
        _: cu29::teleops::Validity,
        _: &T,
    ) -> Result<(), CuStreamTxError> {
        self.0.push((tag, frame));
        Ok(())
    }
}

#[test]
fn displayed_frame_and_channel_policies() {
    use cu29::teleops::Teleops;
    let mut recorder = Recorder::default();
    let mut operator = Controls::operator(&mut recorder);
    operator.observing(14).mode().clear().unwrap();
    operator.click().try_send(&7).unwrap();
    assert_eq!(recorder.0, [(1, Some(14)), (2, None)]);
    let (mut pubs, inboxes) = Controls::inboxes().unwrap();
    let id = CommandId {
        session: [0; 16],
        epoch: 1,
        field: 2,
        sequence: 1,
        observed_copperlist_id: None,
    };
    for value in 0..3 {
        assert_eq!(pubs.click.publish(id, Tov::None, value), value < 2);
        assert!(pubs.stick.publish(id, Tov::None, value));
    }
    assert_eq!(inboxes.click.take().unwrap().2, 0);
    assert_eq!(inboxes.click.take().unwrap().2, 1);
    assert!(inboxes.click.take().is_none());
    assert_eq!(inboxes.stick.take().unwrap().2, 2);
}

#[derive(Default, Debug)]
struct Packet(Vec<u8>);
impl cu29::logstream::CuStreamTx for Packet {
    fn try_send(&mut self, packet: &[u8]) -> Result<(), CuStreamTxError> {
        self.0 = packet.to_vec();
        Ok(())
    }
}

#[test]
fn delayed_or_duplicate_commands_cannot_renew_control() {
    use cu29::teleops::{Admissions, Rejected, ReturnTx, Teleops, TickHistory};
    let (publish, inboxes) = Controls::inboxes().unwrap();
    let session = [77; 16];
    let mut admissions = Admissions::new(publish, session, 1, CuDuration::from_millis(150));
    let mut tx = ReturnTx::new(Packet::default(), session, 1);
    let packet = |tx: &mut ReturnTx<Packet>| {
        Controls::operator(&mut *tx)
            .observing(12)
            .stick()
            .try_send(&1)
            .unwrap();
        tx.transport().0.clone()
    };
    let mut ticks = TickHistory::<8>::default();
    ticks.record(12, CuTime::from_millis(100));
    ticks.record(13, CuTime::from_millis(200));
    let first = packet(&mut tx);
    let now = CuTime::from_millis(220);
    let id = admissions.receive(&first, now, |id| ticks.get(id)).unwrap();
    assert_eq!(id.observed_copperlist_id, Some(12));
    assert_eq!(
        admissions.receive(&first, now, |id| ticks.get(id)),
        Err((Some(id), Rejected::Sequence))
    );
    assert_eq!(
        inboxes.stick.take().unwrap().1,
        Tov::Time(CuTime::from_millis(100))
    );
    let late = packet(&mut tx);
    assert_eq!(
        admissions
            .receive(&late, CuTime::from_millis(260), |id| ticks.get(id))
            .unwrap_err()
            .1,
        Rejected::Freshness
    );
    admissions.takeover(2, || {}).unwrap();
    assert_eq!(
        admissions
            .receive(&first, now, |id| ticks.get(id))
            .unwrap_err()
            .1,
        Rejected::Epoch
    );
    assert!(inboxes.stick.take().is_none());
}

#[test]
fn shared_reader_services_eight_packets_even_without_outbound_captures() {
    use cu29::logstream::protocol::{CuTelemetryRecord, CuTelemetrySession, CuTelemetryTx, TxMode};
    use cu29::logstream::{CuFeedbackRx, CuStreamRx, CuStreamRxError, StreamIdentity};
    use cu29::teleops::{FeedbackInbox, ReturnPolicy, ReturnTx, Teleops, with_teleops};
    use std::collections::VecDeque;
    #[derive(Debug)]
    struct Packets(VecDeque<Vec<u8>>);
    impl CuStreamRx for Packets {
        fn try_recv(&mut self, bytes: &mut [u8]) -> Result<Option<usize>, CuStreamRxError> {
            Ok(self.0.pop_front().map(|pkt| {
                bytes[..pkt.len()].copy_from_slice(&pkt);
                pkt.len()
            }))
        }
    }
    struct Idle;
    impl CuTelemetryTx for Idle {
        fn send(&mut self, _: CuTelemetryRecord<'_>) -> CuResult<()> {
            Ok(())
        }
    }
    let (publish, inboxes) = Controls::inboxes().unwrap();
    let session = [9; 16];
    let mut sender = ReturnTx::new(Packet::default(), session, 1);
    let mut packets = VecDeque::from([b"CUFBtest".to_vec()]);
    for value in 1..=8 {
        let mut msg = CuMsg::default();
        msg.set_payload(value);
        msg.tov = Tov::Time(CuTime(2));
        Controls::operator(&mut sender)
            .stick()
            .try_send_msg(&msg)
            .unwrap();
        packets.push_back(sender.transport().0.clone());
    }
    let mut feedback = FeedbackInbox::default();
    let (clock, _) = RobotClock::mock();
    let mut link = with_teleops(
        Idle,
        Packets(packets),
        publish,
        clock.clone(),
        ReturnPolicy {
            robot_clock: clock,
            epoch: 1,
            max_age: CuDuration::from_millis(20),
            feedback: Some(feedback.clone()),
        },
        |_| None,
        |r| {
            r.unwrap();
        },
    );
    link.send(CuTelemetryRecord::Session(&CuTelemetrySession {
        metadata: None,
        identity: StreamIdentity {
            session_id: session,
            sender_id: 0,
        },
        context: Default::default(),
        schema: cu29::logstream::ApplicationSchema {
            outputs: vec![],
            reconstruction: vec![],
        },
    }))
    .unwrap();
    assert!(link.poll(TxMode::Running).unwrap().is_some());
    assert_eq!(inboxes.stick.take().unwrap().2, 7);
    let mut bytes = [0; 8];
    assert_eq!(feedback.try_recv_feedback(&mut bytes).unwrap(), Some(8));
    assert_eq!(&bytes, b"CUFBtest");
    link.poll(TxMode::Running).unwrap();
    assert_eq!(inboxes.stick.take().unwrap().2, 8);
}
