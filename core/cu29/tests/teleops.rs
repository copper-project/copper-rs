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
