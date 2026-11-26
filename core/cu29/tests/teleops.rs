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
