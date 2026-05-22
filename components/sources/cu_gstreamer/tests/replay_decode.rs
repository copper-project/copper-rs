#![cfg(feature = "gst")]

use std::fs;
use std::process::Command;
use std::sync::atomic::Ordering;

use bincode::config::standard;
use bincode::{Decode, Encode};
use cu_gstreamer::CuGstBuffer;
use cu29::prelude::*;
use gstreamer::Buffer;
use serde::Serialize;

const HELPER_ENV: &str = "CU_GSTREAMER_REPLAY_DECODE_HELPER";
const PAYLOAD_PATH_ENV: &str = "CU_GSTREAMER_REPLAY_DECODE_PATH";
const EXPECTED_BYTES: &[u8] = &[1, 2, 3, 4, 5, 6];

#[derive(Debug, Default, Encode, Decode, Serialize)]
struct RecordedMsgSet(CuMsg<CuGstBuffer>);

impl ErasedCuStampedDataSet for RecordedMsgSet {
    fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
        vec![&self.0]
    }
}

impl MatchingTasks for RecordedMsgSet {
    fn get_all_task_ids() -> &'static [&'static str] {
        &["camera"]
    }
}

fn encoded_log_entry() -> Vec<u8> {
    gstreamer::init().expect("parent helper should initialize gstreamer");

    let mut msg = CuMsg::<CuGstBuffer>::default();
    msg.set_payload(CuGstBuffer::from(Buffer::from_slice(
        EXPECTED_BYTES.to_vec(),
    )));

    let entry = CopperList::new(7, RecordedMsgSet(msg));
    bincode::encode_to_vec(entry, standard()).expect("encode recorded CopperList")
}

fn decode_like_resim(encoded_path: &str) {
    assert!(
        !gstreamer::INITIALIZED.load(Ordering::SeqCst),
        "decode helper should start without gst init"
    );

    let encoded = fs::read(encoded_path).expect("read encoded CopperList");
    let (entry, read): (CopperList<RecordedMsgSet>, usize) =
        bincode::decode_from_slice(&encoded, standard()).expect("decode recorded CopperList");
    assert_eq!(read, encoded.len());
    assert!(
        !gstreamer::INITIALIZED.load(Ordering::SeqCst),
        "replay decode should not initialize gstreamer"
    );

    let payload = entry.msgs.0.payload().expect("payload should be present");
    assert!(matches!(payload, CuGstBuffer::Replay(_)));
    assert!(payload.as_live().is_none());

    let bytes = payload
        .map_readable()
        .expect("replay payload should be readable");
    assert_eq!(bytes.as_slice(), EXPECTED_BYTES);
}

fn run_self(test_name: &str, encoded_path: &str) -> std::process::Output {
    Command::new(std::env::current_exe().expect("current_exe should succeed"))
        .arg("--exact")
        .arg(test_name)
        .arg("--nocapture")
        .arg("--test-threads=1")
        .env(HELPER_ENV, "decode")
        .env(PAYLOAD_PATH_ENV, encoded_path)
        .output()
        .expect("child test process should launch")
}

#[test]
fn replay_decode_does_not_require_gstreamer_init() {
    if std::env::var(HELPER_ENV).ok().as_deref() == Some("decode") {
        let encoded_path =
            std::env::var(PAYLOAD_PATH_ENV).expect("decode helper should receive payload path");
        decode_like_resim(&encoded_path);
        return;
    }

    let tmp_dir = tempfile::TempDir::new().expect("create tempdir");
    let encoded_path = tmp_dir.path().join("recorded_gst_message.bin");
    fs::write(&encoded_path, encoded_log_entry()).expect("write encoded CopperList");

    let output = run_self(
        "replay_decode_does_not_require_gstreamer_init",
        encoded_path
            .to_str()
            .expect("temp path should be valid utf-8"),
    );
    assert!(
        output.status.success(),
        "decode helper failed\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}
