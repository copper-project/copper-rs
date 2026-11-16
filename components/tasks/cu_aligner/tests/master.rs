use cu_aligner::{define_master_task, define_window_task};
use cu29::prelude::*;

define_master_task!(Master, master: 0, 0 => { 2, u32 }, 1 => { 4, i32 });
define_master_task!(SecondMaster, master: 1, 0 => { 4, u32 }, 1 => { 2, i32 });
define_window_task!(Window, 0 => { 4, 2, u32 }, 1 => { 4, 2, i32 });
define_window_task!(OtherWindow, 0 => { 4, 2, u32 }, 1 => { 4, 2, i32 });

fn config(wait: u32) -> ComponentConfig {
    let mut cfg = ComponentConfig::default();
    cfg.set("max_wait_ms", wait);
    cfg.set("max_offset_ms", 20);
    cfg.set("stale_data_horizon_ms", 500);
    cfg
}
fn msg<P: CuMsgPayload>(value: P, ms: u64) -> CuMsg<P> {
    let mut msg = CuMsg::new(Some(value));
    msg.tov = Tov::Time(CuTime::from_millis(ms));
    msg
}
fn bytes<T: cu29::bincode::Encode>(value: &T) -> Vec<u8> {
    cu29::bincode::encode_to_vec(value, cu29::bincode::config::standard()).unwrap()
}

#[test]
fn delayed_nearest_preserves_metadata_and_emits_once() {
    let mut task = Master::new(Some(&config(10)), ()).unwrap();
    let (ctx, clock) = CuContext::new_mock_clock();
    clock.set_value(100_000_000);
    let mut camera = msg(1, 100);
    camera.metadata.set_status("camera");
    let mut sensor = msg(2, 95);
    sensor.metadata.set_origin(CuMsgOrigin {
        subsystem_code: 1,
        instance_id: 2,
        cl_id: 3,
    });
    sensor.metadata.process_time.start = CuTime::from_millis(96).into();
    let mut out = CuMsg::default();
    task.process(&ctx, &(&camera, &sensor), &mut out).unwrap();
    assert!(out.payload().is_none());
    clock.set_value(105_000_000);
    let later = msg(3, 103);
    task.process(&ctx, &(&camera, &later), &mut out).unwrap();
    assert!(out.payload().is_none());
    clock.set_value(110_000_000);
    task.process(&ctx, &(&camera, &later), &mut out).unwrap();
    let (actual_camera, actual_sensor) = out.payload().unwrap();
    assert_eq!(bytes(actual_camera), bytes(&camera));
    assert_eq!(bytes(actual_sensor), bytes(&later));
    assert_eq!(out.tov, camera.tov);
    task.process(&ctx, &(&camera, &later), &mut out).unwrap();
    assert!(out.payload().is_none());
    assert_eq!(out.tov, Tov::None);
}

#[test]
fn nearest_tie_prefers_earlier_and_sensor_can_be_reused() {
    let mut task = Master::new(Some(&config(0)), ()).unwrap();
    let (ctx, clock) = CuContext::new_mock_clock();
    clock.set_value(110_000_000);
    let empty = CuMsg::default();
    let earlier = msg(1, 95);
    let later = msg(2, 105);
    let mut out = CuMsg::default();
    task.process(&ctx, &(&empty, &earlier), &mut out).unwrap();
    task.process(&ctx, &(&msg(3, 100), &later), &mut out)
        .unwrap();
    assert_eq!(bytes(&out.payload().unwrap().1), bytes(&earlier));
    task.process(&ctx, &(&msg(4, 101), &later), &mut out)
        .unwrap();
    assert_eq!(bytes(&out.payload().unwrap().1), bytes(&later));
    task.process(&ctx, &(&msg(5, 102), &later), &mut out)
        .unwrap();
    assert_eq!(bytes(&out.payload().unwrap().1), bytes(&later));
}

#[test]
fn missing_or_distant_sensor_drops_master_permanently() {
    let mut task = Master::new(Some(&config(0)), ()).unwrap();
    let ctx = CuContext::new_with_clock();
    let mut out = CuMsg::default();
    let camera = msg(1, 100);
    task.process(&ctx, &(&camera, &msg(2, 50)), &mut out)
        .unwrap();
    assert!(out.payload().is_none());
    task.process(&ctx, &(&camera, &msg(3, 100)), &mut out)
        .unwrap();
    assert!(out.payload().is_none());
    task.process(&ctx, &(&msg(4, 110), &CuMsg::default()), &mut out)
        .unwrap();
    assert!(out.payload().is_some());
}

#[test]
fn nonzero_master_and_offset_boundary() {
    let mut task = SecondMaster::new(Some(&config(0)), ()).unwrap();
    let ctx = CuContext::new_with_clock();
    let mut out = CuMsg::default();
    task.process(&ctx, &(&msg(1, 80), &msg(2, 100)), &mut out)
        .unwrap();
    assert_eq!(out.tov, Tov::Time(CuTime::from_millis(100)));
    assert_eq!(
        out.payload().unwrap().0.tov,
        Tov::Time(CuTime::from_millis(80))
    );
}

#[test]
fn invalid_tov_and_regression_do_not_partially_ingest() {
    let mut task = Master::new(Some(&config(0)), ()).unwrap();
    let ctx = CuContext::new_with_clock();
    let mut out = CuMsg::default();
    let camera = msg(1, 100);
    let mut invalid = msg(2, 99);
    invalid.tov = Tov::None;
    assert!(task.process(&ctx, &(&camera, &invalid), &mut out).is_err());
    invalid.tov = Tov::Range(CuTimeRange {
        start: CuTime::from_millis(98),
        end: CuTime::from_millis(99),
    });
    assert!(task.process(&ctx, &(&camera, &invalid), &mut out).is_err());
    task.process(&ctx, &(&camera, &msg(2, 99)), &mut out)
        .unwrap();
    assert!(out.payload().is_some());
    assert!(
        task.process(&ctx, &(&msg(1, 101), &msg(2, 98)), &mut out)
            .is_err()
    );
    task.process(&ctx, &(&msg(1, 101), &msg(2, 100)), &mut out)
        .unwrap();
    assert!(out.payload().is_some());
}

#[test]
fn pending_capacity_drops_oldest_master() {
    let mut task = Master::new(Some(&config(10)), ()).unwrap();
    let (ctx, clock) = CuContext::new_mock_clock();
    let mut out = CuMsg::default();
    for ms in 100..103 {
        clock.set_value(ms * 1_000_000);
        task.process(&ctx, &(&msg(1, ms), &msg(2, ms)), &mut out)
            .unwrap();
    }
    clock.set_value(112_000_000);
    task.process(&ctx, &(&CuMsg::default(), &CuMsg::default()), &mut out)
        .unwrap();
    assert_eq!(out.tov, Tov::Time(CuTime::from_millis(101)));
    task.process(&ctx, &(&CuMsg::default(), &CuMsg::default()), &mut out)
        .unwrap();
    assert_eq!(out.tov, Tov::Time(CuTime::from_millis(102)));
}

#[test]
fn latest_before_and_invalid_policy() {
    let mut cfg = config(0);
    cfg.set("selection", "latest_before".to_string());
    let mut task = Master::new(Some(&cfg), ()).unwrap();
    let ctx = CuContext::new_with_clock();
    let mut out = CuMsg::default();
    task.process(&ctx, &(&CuMsg::default(), &msg(1, 95)), &mut out)
        .unwrap();
    task.process(&ctx, &(&msg(1, 100), &msg(2, 101)), &mut out)
        .unwrap();
    assert_eq!(out.payload().unwrap().1.payload(), Some(&1));
    cfg.set("selection", "typo".to_string());
    assert!(Master::new(Some(&cfg), ()).is_err());
    cfg.set("selection", "nearest".to_string());
    cfg.set("stale_data_horizon_ms", 19);
    assert!(Master::new(Some(&cfg), ()).is_err());
}

#[test]
fn retention_expires_pending_master_and_sensor_history() {
    let mut task = Master::new(Some(&config(10)), ()).unwrap();
    let (ctx, clock) = CuContext::new_mock_clock();
    clock.set_value(100_000_000);
    let mut out = CuMsg::default();
    task.process(&ctx, &(&msg(1, 100), &msg(2, 100)), &mut out)
        .unwrap();
    clock.set_value(700_000_000);
    task.process(&ctx, &(&CuMsg::default(), &CuMsg::default()), &mut out)
        .unwrap();
    assert!(out.payload().is_none());
    task.process(&ctx, &(&msg(3, 700), &CuMsg::default()), &mut out)
        .unwrap();
    clock.set_value(710_000_000);
    task.process(&ctx, &(&CuMsg::default(), &CuMsg::default()), &mut out)
        .unwrap();
    assert!(out.payload().is_none());
}

#[test]
fn freeze_thaw_restores_pending_deadlines_history_and_deduplication() {
    use cu29::bincode::{
        config::standard,
        de::{DecoderImpl, read::SliceReader},
    };
    use cu29::cutask::BincodeAdapter;
    let cfg = config(10);
    let mut original = Master::new(Some(&cfg), ()).unwrap();
    let (ctx, clock) = CuContext::new_mock_clock();
    clock.set_value(100_000_000);
    let camera = msg(1, 100);
    let mut sensor = msg(2, 99);
    sensor.metadata.set_status("preserve through snapshot");
    sensor.metadata.set_origin(CuMsgOrigin {
        subsystem_code: 3,
        instance_id: 4,
        cl_id: 5,
    });
    let mut out = CuMsg::default();
    original
        .process(&ctx, &(&camera, &sensor), &mut out)
        .unwrap();
    let snapshot = bytes(&BincodeAdapter(&original));
    let mut restored = Master::new(Some(&cfg), ()).unwrap();
    let mut decoder = DecoderImpl::new(SliceReader::new(&snapshot), standard(), ());
    restored.thaw(&mut decoder).unwrap();
    assert_eq!(snapshot, bytes(&BincodeAdapter(&restored)));
    for time in [109_000_000, 110_000_000, 111_000_000] {
        clock.set_value(time);
        let mut replay = CuMsg::default();
        original
            .process(&ctx, &(&camera, &sensor), &mut out)
            .unwrap();
        restored
            .process(&ctx, &(&camera, &sensor), &mut replay)
            .unwrap();
        assert_eq!(bytes(&out), bytes(&replay));
        assert_eq!(out.payload().is_some(), time == 110_000_000);
        if let Some((_, selected)) = out.payload() {
            assert_eq!(bytes(selected), bytes(&sensor));
        }
    }
}
