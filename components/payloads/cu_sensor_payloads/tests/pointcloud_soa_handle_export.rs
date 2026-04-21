use std::process::Command;
use std::thread;

use bincode::de::Decoder;
use bincode::error::DecodeError;
use bincode::{Decode, Encode};
use cu_sensor_payloads::PointCloudSoaHandle;
use cu29::prelude::{
    CopperList, CuMsgMetadata, CuStampedData, ErasedCuStampedData, ErasedCuStampedDataSet,
    MatchingTasks,
};
use serde::{Deserialize, Serialize};

const HELPER_ENV: &str = "CU_SENSOR_POINTCLOUD_EXPORT_STACK_HELPER";
const LARGE_POINTS: usize = 100_000;
const SMALL_STACK_BYTES: usize = 256 * 1024;

#[derive(Debug, Default, Encode, Serialize, Deserialize)]
struct PointCloudMsgs(CuStampedData<PointCloudSoaHandle<LARGE_POINTS>, CuMsgMetadata>);

impl Decode<()> for PointCloudMsgs {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self(Decode::decode(decoder)?))
    }
}

impl ErasedCuStampedDataSet for PointCloudMsgs {
    fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
        vec![&self.0]
    }
}

impl MatchingTasks for PointCloudMsgs {
    fn get_all_task_ids() -> &'static [&'static str] {
        &["points"]
    }
}

fn encoded_copperlist() -> Vec<u8> {
    let mut payload = PointCloudSoaHandle::<LARGE_POINTS>::default();
    payload.with_inner_mut(|cloud| {
        cloud.len = LARGE_POINTS;
        cloud.return_order[0] = 1;
        cloud.return_order[LARGE_POINTS - 1] = 2;
    });

    let mut msgs = PointCloudMsgs::default();
    msgs.0.set_payload(payload);
    let cl = CopperList::new(7, msgs);

    bincode::encode_to_vec(cl, bincode::config::standard()).expect("encode CopperList")
}

fn run_json_export_on_small_stack() {
    let encoded = encoded_copperlist();
    thread::Builder::new()
        .stack_size(SMALL_STACK_BYTES)
        .spawn(move || {
            let (entry, read): (CopperList<PointCloudMsgs>, usize) =
                bincode::decode_from_slice(&encoded, bincode::config::standard())
                    .expect("decode CopperList");
            assert_eq!(read, encoded.len());

            let json = serde_json::to_vec(&entry).expect("serialize CopperList JSON");
            assert!(!json.is_empty());
        })
        .expect("thread spawn should succeed")
        .join()
        .expect("JSON export thread should not panic");
}

fn run_self(test_name: &str, helper_mode: &str) -> std::process::ExitStatus {
    Command::new(std::env::current_exe().expect("current_exe should succeed"))
        .arg("--exact")
        .arg(test_name)
        .env(HELPER_ENV, helper_mode)
        .status()
        .expect("child test process should launch")
}

#[cfg(target_os = "linux")]
#[test]
fn pointcloud_handle_json_export_survives_small_stack() {
    if std::env::var(HELPER_ENV).ok().as_deref() == Some("json") {
        run_json_export_on_small_stack();
        return;
    }

    let status = run_self("pointcloud_handle_json_export_survives_small_stack", "json");
    assert!(
        status.success(),
        "JSON export child should succeed, got {status:?}"
    );
}
