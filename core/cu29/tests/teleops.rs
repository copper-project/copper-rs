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
