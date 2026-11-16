//! Record the real graph, then exercise the same CBOR RPCs used by Timetraveler.

use super::*;
use cu_rerun::rerun;
use cu29::remote_debug::{
    DebugRpcResponse, RemoteDebugPaths, RemoteDebugShmConfig, RemoteDebugZenohClient,
    RemoteDebugZenohServer, WireCodec,
};
use rerun::external::arrow::array::ArrayRef;
use rerun::external::re_log_encoding::Decoder;
use rerun::external::re_log_msg::LogMsg;
use serde_json::{Value, json};

mod live {
    use cu29::prelude::*;
    #[copper_runtime(config = "copperconfig.ron")]
    struct RecordingApp {}
    pub use default::RecordingApp as App;
}

fn call(
    client: &RemoteDebugZenohClient,
    session: Option<&str>,
    method: &str,
    params: Value,
) -> DebugRpcResponse {
    let response = client.call(session, method, params).unwrap();
    assert!(response.ok, "{method}: {:?}", response.error);
    response
}

fn geometry(messages: &[LogMsg]) -> ArrayRef {
    messages
        .iter()
        .find_map(|message| {
            let LogMsg::ArrowMsg(_, chunk) = message else {
                return None;
            };
            chunk
                .batch
                .column_by_name(rerun::LineStrips3D::descriptor_strips().component.as_str())
                .cloned()
        })
        .expect("soft-arm geometry")
}

fn decode_geometry(bytes: &[u8]) -> ArrayRef {
    let messages = Decoder::<LogMsg>::decode_lazy(bytes)
        .collect::<Result<Vec<_>, _>>()
        .unwrap();
    geometry(&messages)
}

fn expected_geometry(frame: u64) -> ArrayRef {
    let (rec, storage) = rerun::RecordingStreamBuilder::new("expected")
        .memory()
        .unwrap();
    let arm = cu_rerun_soft_arm::SoftArm::at_phase(std::f32::consts::TAU * frame as f32 / 360.0);
    rec.log("arm", &arm).unwrap();
    geometry(&storage.take())
}

#[test]
fn copper_recording_supports_logged_and_replayed_representations_over_cbor() {
    let dir = tempfile::tempdir().unwrap();
    let log = dir.path().join("recorded.copper");
    let replay = dir.path().join("replay.copper");
    let (clock, mock) = RobotClock::mock();
    let (recording, _storage) = rerun::RecordingStreamBuilder::new("live-test")
        .memory()
        .unwrap();
    {
        let mut app = live::App::builder()
            .with_clock(clock)
            .with_log_path(&log, Some(16 * 1024 * 1024))
            .unwrap()
            .with_resources(move |_| cu_rerun_soft_arm::tasks::viewer_resources(recording))
            .build()
            .unwrap()
            .start()
            .unwrap();
        for _ in 0..72 {
            mock.increment(CuDuration::from_nanos(33_333_333));
            app.run_one_iteration().unwrap();
        }
        app.stop().unwrap();
    }

    let paths = RemoteDebugPaths::new(&format!(
        "copper/tests/soft_arm/{}/debug/v1",
        std::process::id()
    ));
    let server_paths = paths.clone();
    let shm = RemoteDebugShmConfig::new(4 * 1024 * 1024, 256 * 1024);
    let (ready_tx, ready_rx) = std::sync::mpsc::channel();
    let server = std::thread::spawn(move || {
        let mut server = RemoteDebugZenohServer::<
            default::SoftArmReplay,
            default::CuStampedDataSet,
            Callback,
            TimeOf,
            MmapSectionStorage,
            MmapUnifiedLoggerWrite,
            _,
        >::new_with_shm_config(
            server_paths,
            move |params| app_factory(params, &replay),
            build_callback,
            cu29::simulation::recorded_copperlist_timestamp,
            shm,
        )
        .unwrap();
        ready_tx.send(()).unwrap();
        server.serve_until_stopped().unwrap();
    });
    ready_rx
        .recv_timeout(std::time::Duration::from_secs(15))
        .unwrap();
    let client = RemoteDebugZenohClient::builder(paths, "soft-arm-test")
        .codec(WireCodec::Cbor)
        .shm_config(shm)
        .build()
        .unwrap();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let open = call(
            &client,
            None,
            "session.open",
            json!({ "log_base": log, "role": "controller", "codecs": ["cbor"] }),
        );
        let session = open.result.as_ref().unwrap()["session_id"]
            .as_str()
            .unwrap();
        let descriptors = call(&client, Some(session), "representation.list", json!({}));
        let descriptors = &descriptors.result.as_ref().unwrap()["representations"];
        assert_eq!(descriptors.as_array().unwrap().len(), 1);
        assert_eq!(descriptors[0]["task_id"], "motion");
        assert_eq!(
            descriptors[0]["target"],
            json!({ "kind": "message", "output_index": 0 })
        );
        let target = json!({ "kind": "message", "output_index": 0 });
        // Seek across a keyframe, then backwards. Both paths must return the logged shape.
        for frame in [45, 5] {
            let response = call(
                &client,
                Some(session),
                "representation.read",
                json!({
                    "target": target, "at": {"kind": "cl", "cl": frame}, "entity_path": "arm"
                }),
            );
            let logged = decode_geometry(&response.representation.unwrap().bytes);
            assert_eq!(logged.to_data(), expected_geometry(frame).to_data());
            let replay = call(
                &client,
                Some(session),
                "nav.replay",
                json!({
                    "at": {"target": {"kind": "cl", "cl": frame}, "mutate_cursor": true, "resolve": "exact"},
                    "limit": 1, "include_replayed_cl": true, "representation_targets": [target]
                }),
            );
            assert_eq!(replay.representation_frames.len(), 1);
            let replayed = &replay.representation_frames[0];
            assert_eq!(replayed.culistid, frame);
            assert_eq!(
                decode_geometry(&replayed.payload.as_ref().unwrap().bytes).to_data(),
                logged.to_data()
            );
        }
        assert_ne!(
            expected_geometry(45).to_data(),
            expected_geometry(5).to_data()
        );
        let logs = call(
            &client,
            Some(session),
            "logs.list",
            json!({ "offset": 0, "limit": 10 }),
        );
        assert!(
            !logs.result.as_ref().unwrap()["entries"]
                .as_array()
                .unwrap()
                .is_empty()
        );
        call(&client, Some(session), "session.close", json!({}));
    }));
    call(&client, None, "admin.stop", json!({}));
    server.join().unwrap();
    result.unwrap();
}
