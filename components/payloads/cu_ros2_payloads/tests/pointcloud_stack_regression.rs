use std::process::Command;
use std::thread;

use cu_ros2_payloads::RosBridgeAdapter;
use cu_ros2_payloads::builtin::{Header, Time};
use cu_ros2_payloads::sensor_msgs::{PointCloud2, PointField};
use cu_sensor_payloads::{PointCloudSoa, PointCloudSoaHandle};

const HELPER_ENV: &str = "CU_ROS2_POINTCLOUD_STACK_HELPER";
const LARGE_POINTS: usize = 200_000;
const SMALL_STACK_BYTES: usize = 256 * 1024;

fn large_pointcloud2() -> PointCloud2 {
    let width = LARGE_POINTS as u32;
    let point_step = 24u32;
    let row_step = width * point_step;
    PointCloud2 {
        header: Header {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: "".into(),
        },
        height: 1,
        width,
        fields: vec![
            PointField {
                name: "x".into(),
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointField {
                name: "y".into(),
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointField {
                name: "z".into(),
                offset: 8,
                datatype: 7,
                count: 1,
            },
            PointField {
                name: "intensity".into(),
                offset: 12,
                datatype: 7,
                count: 1,
            },
            PointField {
                name: "tov_sec".into(),
                offset: 16,
                datatype: 6,
                count: 1,
            },
            PointField {
                name: "tov_nsec".into(),
                offset: 20,
                datatype: 6,
                count: 1,
            },
        ],
        is_bigendian: false,
        point_step,
        row_step,
        data: vec![0u8; row_step as usize],
        is_dense: true,
    }
}

fn run_handle_decode_on_small_stack() {
    let ros = large_pointcloud2();
    thread::Builder::new()
        .stack_size(SMALL_STACK_BYTES)
        .spawn(move || {
            let payload = PointCloudSoaHandle::<LARGE_POINTS>::from_ros_message(ros)
                .expect("handle decode should succeed");
            payload.with_inner(|inner| assert_eq!(inner.len, LARGE_POINTS));
        })
        .expect("thread spawn should succeed")
        .join()
        .expect("handle decode thread should not panic");
}

fn run_soa_decode_on_small_stack() {
    let ros = large_pointcloud2();
    thread::Builder::new()
        .stack_size(SMALL_STACK_BYTES)
        .spawn(move || {
            let _ = PointCloudSoa::<LARGE_POINTS>::from_ros_message(ros)
                .expect("SoA decode should run");
        })
        .expect("thread spawn should succeed")
        .join()
        .expect("SoA decode thread should not panic");
}

fn run_self(test_name: &str, helper_mode: &str) -> std::process::ExitStatus {
    Command::new(std::env::current_exe().expect("current_exe should succeed"))
        .arg("--exact")
        .arg(test_name)
        .env(HELPER_ENV, helper_mode)
        .status()
        .expect("child test process should launch")
}

#[test]
fn handled_pointcloud_decode_survives_small_stack() {
    if std::env::var(HELPER_ENV).ok().as_deref() == Some("handled") {
        run_handle_decode_on_small_stack();
        return;
    }

    let status = run_self("handled_pointcloud_decode_survives_small_stack", "handled");
    assert!(
        status.success(),
        "handled child should succeed, got {status:?}"
    );
}

#[cfg(target_os = "linux")]
#[test]
fn soa_pointcloud_decode_overflows_small_stack_subprocess() {
    if std::env::var(HELPER_ENV).ok().as_deref() == Some("soa") {
        run_soa_decode_on_small_stack();
        return;
    }

    let status = run_self(
        "soa_pointcloud_decode_overflows_small_stack_subprocess",
        "soa",
    );
    assert!(
        !status.success(),
        "expected SoA child decode to fail on a small stack, got {status:?}"
    );
}
