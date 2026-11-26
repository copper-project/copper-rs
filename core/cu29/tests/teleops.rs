//! Exercise a schema through the same import path used by robot and twin applications.
#![cfg(feature = "teleops")]
use cu29::logstream::CuStreamTxError;
use cu29::prelude::*;
use cu29::teleops::Submit;

#[derive(Teleops)]
#[allow(dead_code)]
struct Controls {
    stick: Latest<u8>,
    mode: Latched<bool>,
    click: Event<u8, 2>,
}

#[derive(Default)]
struct Recorder(Vec<(u64, Vec<u8>)>);
impl Submit for Recorder {
    fn submit<U: bincode::Encode>(
        &mut self,
        observed: u64,
        update: &U,
    ) -> Result<(), CuStreamTxError> {
        self.0.push((
            observed,
            bincode::encode_to_vec(update, bincode::config::standard()).unwrap(),
        ));
        Ok(())
    }
}

#[test]
fn displayed_frame_and_channel_policies() {
    use cu29::teleops::Teleops;
    let mut recorder = Recorder::default();
    let mut operator = Controls::operator(&mut recorder, None);
    let mut update = controls::Update {
        mode: Some(operator.sampler.sample(CuLatchedStateUpdate::Clear)),
        ..Default::default()
    };
    assert!(update.click.try_push(operator.sampler.sample(7)).is_ok());
    assert!(update.click.try_push(operator.sampler.sample(8)).is_ok());
    operator.submit(14, &mut update).unwrap();
    assert_eq!(recorder.0.len(), 1);
    let (frame, bytes) = &recorder.0[0];
    let (update, _): (controls::Update, _) =
        bincode::decode_from_slice(bytes, bincode::config::standard()).unwrap();
    assert_eq!(*frame, 14);
    assert!(update.stick.is_none());
    assert_eq!(update.mode.as_ref().unwrap().tov(), Tov::None);
    assert_eq!(update.mode.unwrap().value, CuLatchedStateUpdate::Clear);
    assert_eq!(
        update
            .click
            .into_iter()
            .map(|s| s.value)
            .collect::<Vec<_>>(),
        [7, 8]
    );
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
fn forwarded_input_uses_its_original_tov_only_with_synchronized_clocks() {
    use cu29::teleops::Teleops;
    let mut recorder = Recorder::default();
    let mut msg = CuMsg::new(Some(7u8));
    msg.tov = Tov::Time(CuTime(12));
    let unsynced = Controls::operator(&mut recorder, None);
    assert_eq!(
        unsynced.sampler.from_msg(msg.clone()).unwrap().tov(),
        Tov::None
    );
    let synced = Controls::operator(&mut recorder, Some(RobotClock::mock().0));
    assert_eq!(
        synced.sampler.from_msg(msg).unwrap().tov(),
        Tov::Time(CuTime(12))
    );
}

#[test]
fn select_latest_once_per_frame_and_drop_expired_samples() {
    use cu29::teleops::Teleops;
    let mut recorder = Recorder::default();
    let (clock, mock) = RobotClock::mock();
    let mut op = Controls::operator(&mut recorder, Some(clock));
    for (cl, value) in [(20, 3), (20, 4), (19, 5), (21, 6)] {
        mock.set_value(42);
        let sample = op.sampler.sample(value);
        mock.set_value(72);
        assert_eq!(sample.tov(), Tov::Time(CuTime(42)));
        op.submit(
            cl,
            &mut controls::Update {
                stick: Some(sample),
                ..Default::default()
            },
        )
        .unwrap();
    }
    assert_eq!(
        recorder.0.iter().map(|(id, _)| *id).collect::<Vec<_>>(),
        [20, 21]
    );
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
        let mut op = Controls::operator(&mut tx, None);
        op.submit(
            cl,
            &mut controls::Update {
                stick: Some(op.sampler.sample(3)),
                ..Default::default()
            },
        )
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
        msg.tov = Tov::Time(CuTime(1));
        let mut op = Controls::operator(&mut sender, Some(RobotClock::mock().0));
        op.submit(
            u64::from(value),
            &mut controls::Update {
                stick: op.sampler.from_msg(msg),
                ..Default::default()
            },
        )
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
    assert_eq!(inboxes.stick.take(8, 2), Some((7, Tov::Time(CuTime(1)), 7)));
    let mut bytes = [0; 8];
    assert_eq!(feedback.try_recv_feedback(&mut bytes).unwrap(), Some(8));
    assert_eq!(&bytes, b"CUFBtest");
    link.poll(TxMode::Running).unwrap();
    assert_eq!(inboxes.stick.take(8, 2).unwrap().2, 8);
}

#[test]
fn event_overflow_rejects_the_whole_update_and_allows_retry() {
    use cu29::teleops::{Admissions, Rejected, ReturnTx, Teleops};
    let (mut publish, inboxes) = Controls::inboxes().unwrap();
    assert!(publish.click.publish(3, Tov::None, 1));
    let mut rx = Admissions::new(publish, [1; 16]);
    let mut tx = ReturnTx::new(Packet::default(), [1; 16]);
    let mut op = Controls::operator(&mut tx, None);
    let mut update = controls::Update {
        stick: Some(op.sampler.sample(5)),
        ..Default::default()
    };
    assert!(update.click.try_push(op.sampler.sample(2)).is_ok());
    assert!(update.click.try_push(op.sampler.sample(3)).is_ok());
    op.submit(4, &mut update).unwrap();
    let packet = &tx.transport().0;
    assert_eq!(
        rx.receive(packet, |_| Some(CuTime(5))),
        Err((Some(4), Rejected::Overflow))
    );
    assert!(inboxes.stick.take(4, 0).is_none());
    assert_eq!(inboxes.click.take(4, 1).unwrap().2, 1);
    assert_eq!(rx.receive(packet, |_| Some(CuTime(5))), Ok(4));
    assert_eq!(inboxes.stick.take(4, 0).unwrap().2, 5);
    assert_eq!(inboxes.click.take(4, 0).unwrap().2, 2);
    assert_eq!(inboxes.click.take(4, 0).unwrap().2, 3);
}
