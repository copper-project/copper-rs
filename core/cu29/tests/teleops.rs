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
