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
    let mut operator = Controls::operator(&mut recorder);
    let mut update = controls::Update {
        mode: Some(CuLatchedStateUpdate::Clear.into()),
        ..Default::default()
    };
    assert!(update.click.try_push(7.into()).is_ok());
    assert!(update.click.try_push(8.into()).is_ok());
    operator.submit(14, &mut update).unwrap();
    assert_eq!(recorder.0.len(), 1);
    let (frame, bytes) = &recorder.0[0];
    let (update, _): (controls::Update, _) =
        bincode::decode_from_slice(bytes, bincode::config::standard()).unwrap();
    assert_eq!(*frame, 14);
    assert!(update.stick.is_none());
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
fn select_latest_once_per_frame_and_drop_expired_samples() {
    use cu29::teleops::Teleops;
    let mut recorder = Recorder::default();
    let mut op = Controls::operator(&mut recorder);
    for (cl, value) in [(20, 3), (20, 4), (19, 5), (21, 6)] {
        op.submit(
            cl,
            &mut controls::Update {
                stick: Some(value.into()),
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
