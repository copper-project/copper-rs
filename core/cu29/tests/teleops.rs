//! Exercise a schema through the same import path used by robot and twin applications.
#![cfg(feature = "teleops")]
use cu29::logstream::CuStreamTxError;
use cu29::prelude::*;
use cu29::teleops::{Channel, Submit};

#[derive(Teleops)]
#[allow(dead_code)]
struct Controls {
    stick: Latest<u8>,
    mode: Latched<bool>,
    click: Event<u8, 2>,
}

#[derive(Default)]
struct Recorder(Vec<(u16, u64)>, Vec<Vec<u8>>);
impl Submit for Recorder {
    fn submit<C: Channel>(
        &mut self,
        tag: u16,
        frame: u64,
        _: cu29::teleops::Validity,
        value: &C::Payload,
    ) -> Result<(), CuStreamTxError> {
        self.1
            .push(bincode::encode_to_vec(value, bincode::config::standard()).unwrap());
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
    operator.observing(14).click().try_send(&7).unwrap();
    assert_eq!(recorder.0, [(1, 14), (2, 14)]);
    let (mut pubs, inboxes) = Controls::inboxes().unwrap();
    for value in 0..3 {
        assert_eq!(
            pubs.click.publish(u64::from(value), Tov::None, value),
            value < 2
        );
        assert!(pubs.stick.publish(u64::from(value), Tov::None, value));
    }
    assert_eq!(inboxes.click.take(2, 2).unwrap().2, 0);
    assert_eq!(inboxes.click.take(2, 2).unwrap().2, 1);
    assert!(inboxes.click.take(2, 2).is_none());
    assert_eq!(inboxes.stick.take(2, 2).unwrap().2, 2);
}

#[test]
fn select_latest_once_per_frame_and_drop_expired_samples() {
    use cu29::teleops::Teleops;
    let mut recorder = Recorder::default();
    let mut op = Controls::operator(&mut recorder);
    op.observing(20)
        .stick()
        .try_send_latest([1, 2, 3].iter())
        .unwrap();
    op.observing(20).stick().try_send(&4).unwrap();
    op.observing(19).stick().try_send(&5).unwrap();
    op.observing(21).stick().try_send(&6).unwrap();
    assert_eq!(recorder.0, [(0, 20), (0, 21)]);
    assert_eq!(recorder.1, [vec![3], vec![6]]);
    let (mut pubs, ins) = Controls::inboxes().unwrap();
    assert!(pubs.stick.publish(20, Tov::None, 3));
    assert!(!pubs.stick.publish(20, Tov::None, 4));
    assert!(ins.stick.take(24, 3).is_none());
    assert!(!pubs.stick.publish(19, Tov::None, 1));
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
fn duplicated_or_reordered_frames_cannot_renew_control() {
    use cu29::teleops::{Admissions, Rejected, ReturnTx, Teleops};
    let (publish, inboxes) = Controls::inboxes().unwrap();
    let mut rx = Admissions::new(publish, [77; 16]);
    let mut tx = ReturnTx::new(Packet::default(), [77; 16]);
    for (cl, accept) in [(12, true), (12, false), (11, false), (13, true)] {
        Controls::operator(&mut tx)
            .observing(cl)
            .stick()
            .try_send(&3)
            .unwrap();
        let result = rx.receive(&tx.transport().0, |_| Some(CuTime(7)));
        if accept {
            assert_eq!(result, Ok(cl));
        } else {
            assert_eq!(result, Err((Some(cl), Rejected::OutOfOrder)));
        }
    }
    assert_eq!(
        inboxes.stick.take(14, 1),
        Some((13, Tov::Time(CuTime(7)), 3))
    );
    assert!(inboxes.stick.take(14, 1).is_none());
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
    let mut sender = ReturnTx::new(Packet::default(), session);
    let mut packets = VecDeque::from([b"CUFBtest".to_vec()]);
    for value in 1..=8 {
        let mut msg = CuMsg::default();
        msg.set_payload(value);
        msg.tov = Tov::Time(CuTime(2));
        Controls::operator(&mut sender)
            .observing(u64::from(value))
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
            feedback: Some(feedback.clone()),
        },
        |_| Some(CuTime(2)),
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
    for id in 1..=8 {
        link.send(CuTelemetryRecord::CopperList { id, capture: &[] })
            .unwrap();
    }
    assert!(link.poll(TxMode::Running).unwrap().is_some());
    assert_eq!(inboxes.stick.take(8, 2).unwrap().2, 7);
    let mut bytes = [0; 8];
    assert_eq!(feedback.try_recv_feedback(&mut bytes).unwrap(), Some(8));
    assert_eq!(&bytes, b"CUFBtest");
    link.poll(TxMode::Running).unwrap();
    assert_eq!(inboxes.stick.take(8, 2).unwrap().2, 8);
}
