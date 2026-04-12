#[path = "common/usb_common.rs"]
mod usb_common;

use clap::Parser;
use cu29::prelude::*;
use std::path::PathBuf;
use std::sync::{
    Arc,
    atomic::{AtomicBool, AtomicI64, AtomicU64, Ordering},
};
use std::thread;
use std::time::Duration;
use usb_common::{change_to_manifest_dir, override_serial_resource, resolve_usb_serial_port};

#[copper_runtime(config = "examples/usb_probe.ron")]
struct UsbProbeApp {}

#[derive(Parser, Debug)]
#[command(name = "usb_probe")]
struct Args {
    #[arg(long)]
    port: Option<PathBuf>,
    #[arg(long, default_value_t = 4)]
    duration_s: u64,
    #[arg(long, default_value_t = 0.75)]
    distance_tolerance_m: f32,
    #[arg(long)]
    expected_distance_m: Option<f32>,
}

pub mod probe {
    use super::*;
    use cu_sensor_payloads::PointCloudSoa;

    const MM_PER_M: f32 = 1000.0;

    pub mod state {
        use super::*;

        static FRAMES: AtomicU64 = AtomicU64::new(0);
        static LAST_POINTS: AtomicU64 = AtomicU64::new(0);
        static LAST_MEDIAN_DISTANCE_MM: AtomicU64 = AtomicU64::new(0);
        static LAST_MEDIAN_X_MM: AtomicI64 = AtomicI64::new(0);
        static LAST_MEDIAN_Y_MM: AtomicI64 = AtomicI64::new(0);
        static LAST_MEDIAN_Z_MM: AtomicI64 = AtomicI64::new(0);
        static LAST_MIN_DISTANCE_MM: AtomicU64 = AtomicU64::new(0);
        static LAST_MAX_DISTANCE_MM: AtomicU64 = AtomicU64::new(0);

        #[derive(Clone, Copy, Debug, Default)]
        pub struct Snapshot {
            pub frames: u64,
            pub last_points: usize,
            pub last_median_distance_m: f32,
            pub last_median_x_m: f32,
            pub last_median_y_m: f32,
            pub last_median_z_m: f32,
            pub last_min_distance_m: f32,
            pub last_max_distance_m: f32,
        }

        pub fn mark(
            points: usize,
            median_distance_m: f32,
            median_x_m: f32,
            median_y_m: f32,
            median_z_m: f32,
            min_distance_m: f32,
            max_distance_m: f32,
        ) {
            FRAMES.fetch_add(1, Ordering::Relaxed);
            LAST_POINTS.store(points as u64, Ordering::Relaxed);
            LAST_MEDIAN_DISTANCE_MM.store(to_mm_u64(median_distance_m), Ordering::Relaxed);
            LAST_MEDIAN_X_MM.store(to_mm_i64(median_x_m), Ordering::Relaxed);
            LAST_MEDIAN_Y_MM.store(to_mm_i64(median_y_m), Ordering::Relaxed);
            LAST_MEDIAN_Z_MM.store(to_mm_i64(median_z_m), Ordering::Relaxed);
            LAST_MIN_DISTANCE_MM.store(to_mm_u64(min_distance_m), Ordering::Relaxed);
            LAST_MAX_DISTANCE_MM.store(to_mm_u64(max_distance_m), Ordering::Relaxed);
        }

        pub fn snapshot() -> Snapshot {
            Snapshot {
                frames: FRAMES.load(Ordering::Relaxed),
                last_points: LAST_POINTS.load(Ordering::Relaxed) as usize,
                last_median_distance_m: from_mm_u64(
                    LAST_MEDIAN_DISTANCE_MM.load(Ordering::Relaxed),
                ),
                last_median_x_m: from_mm_i64(LAST_MEDIAN_X_MM.load(Ordering::Relaxed)),
                last_median_y_m: from_mm_i64(LAST_MEDIAN_Y_MM.load(Ordering::Relaxed)),
                last_median_z_m: from_mm_i64(LAST_MEDIAN_Z_MM.load(Ordering::Relaxed)),
                last_min_distance_m: from_mm_u64(LAST_MIN_DISTANCE_MM.load(Ordering::Relaxed)),
                last_max_distance_m: from_mm_u64(LAST_MAX_DISTANCE_MM.load(Ordering::Relaxed)),
            }
        }

        pub fn summary() -> String {
            let snapshot = snapshot();
            format!(
                "frames={} points={} median_r={:.2}m median_xyz=({:.2}, {:.2}, {:.2})m range=[{:.2}, {:.2}]m",
                snapshot.frames,
                snapshot.last_points,
                snapshot.last_median_distance_m,
                snapshot.last_median_x_m,
                snapshot.last_median_y_m,
                snapshot.last_median_z_m,
                snapshot.last_min_distance_m,
                snapshot.last_max_distance_m,
            )
        }

        fn to_mm_u64(value_m: f32) -> u64 {
            (value_m.max(0.0) * MM_PER_M).round() as u64
        }

        fn to_mm_i64(value_m: f32) -> i64 {
            (value_m * MM_PER_M).round() as i64
        }

        fn from_mm_u64(value_mm: u64) -> f32 {
            value_mm as f32 / MM_PER_M
        }

        fn from_mm_i64(value_mm: i64) -> f32 {
            value_mm as f32 / MM_PER_M
        }
    }

    #[derive(Default, Reflect)]
    pub struct CeilingProbeSink {
        frame_index: u64,
        xs: Vec<f32>,
        ys: Vec<f32>,
        zs: Vec<f32>,
        rs: Vec<f32>,
    }

    impl Freezable for CeilingProbeSink {}

    impl CuSinkTask for CeilingProbeSink {
        type Resources<'r> = ();
        type Input<'m> = input_msg!(PointCloudSoa<512>);

        fn new(
            _config: Option<&ComponentConfig>,
            _resources: Self::Resources<'_>,
        ) -> CuResult<Self> {
            Ok(Self {
                frame_index: 0,
                xs: Vec::with_capacity(512),
                ys: Vec::with_capacity(512),
                zs: Vec::with_capacity(512),
                rs: Vec::with_capacity(512),
            })
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            let Some(cloud) = input.payload() else {
                return Ok(());
            };

            self.frame_index = self.frame_index.wrapping_add(1);
            self.xs.clear();
            self.ys.clear();
            self.zs.clear();
            self.rs.clear();

            for idx in 0..cloud.len {
                let x = cloud.x[idx].value;
                let y = cloud.y[idx].value;
                let z = cloud.z[idx].value;
                self.xs.push(x);
                self.ys.push(y);
                self.zs.push(z);
                // Radial distance is the most stable sanity check because USB
                // test rigs rarely preserve the datasheet's exact board axes.
                self.rs.push((x * x + y * y + z * z).sqrt());
            }

            if self.rs.is_empty() {
                return Ok(());
            }

            let median_distance_m = median(&mut self.rs);
            let median_x_m = median(&mut self.xs);
            let median_y_m = median(&mut self.ys);
            let median_z_m = median(&mut self.zs);
            let min_distance_m = self
                .rs
                .iter()
                .copied()
                .fold(f32::INFINITY, |acc, value| acc.min(value));
            let max_distance_m = self
                .rs
                .iter()
                .copied()
                .fold(0.0f32, |acc, value| acc.max(value));

            state::mark(
                cloud.len,
                median_distance_m,
                median_x_m,
                median_y_m,
                median_z_m,
                min_distance_m,
                max_distance_m,
            );

            if self.frame_index <= 3 || self.frame_index.is_multiple_of(10) {
                info!("[sen0682/probe] {}", state::summary());
            }

            Ok(())
        }
    }

    fn median(values: &mut [f32]) -> f32 {
        values.sort_by(f32::total_cmp);
        let mid = values.len() / 2;
        if values.len().is_multiple_of(2) {
            (values[mid - 1] + values[mid]) * 0.5
        } else {
            values[mid]
        }
    }
}

fn main() {
    if let Err(err) = run() {
        eprintln!("usb_probe failed: {err}");
        std::process::exit(1);
    }
}

fn run() -> CuResult<()> {
    change_to_manifest_dir()?;

    let args = Args::parse();
    let port = resolve_usb_serial_port(args.port)?;

    let mut config = cu29::read_configuration("examples/usb_probe.ron")?;
    override_serial_resource(&mut config, &port);

    let log_dir = tempfile::TempDir::new()
        .map_err(|err| CuError::new_with_cause("failed to create temporary log directory", err))?;
    let log_path = log_dir.path().join("cu_sen0682_usb_probe.copper");

    let mut app = UsbProbeApp::builder()
        .with_config(config)
        .with_log_path(&log_path, Some(32 * 1024 * 1024))?
        .build()?;
    let clock = app.clock();
    app.start_all_tasks()?;

    let running = Arc::new(AtomicBool::new(true));
    let running_for_signal = Arc::clone(&running);
    ctrlc::set_handler(move || {
        running_for_signal.store(false, Ordering::Relaxed);
    })
    .map_err(|err| CuError::new_with_cause("failed to install Ctrl-C handler", err))?;

    info!(
        "sen0682 usb probe started on {} for {}s",
        &port, args.duration_s
    );

    let deadline_ns = clock
        .now()
        .as_nanos()
        .saturating_add(CuDuration::from_secs(args.duration_s).as_nanos());
    let heartbeat_period_ns = CuDuration::from_secs(1).as_nanos();
    let mut next_heartbeat_ns = clock.now().as_nanos().saturating_add(heartbeat_period_ns);

    let mut run_error: Option<CuError> = None;
    while running.load(Ordering::Relaxed) && clock.now().as_nanos() < deadline_ns {
        if let Err(err) = app.run_one_iteration() {
            run_error = Some(err);
            break;
        }

        let now_ns = clock.now().as_nanos();
        if now_ns >= next_heartbeat_ns {
            info!("[sen0682/heartbeat] {}", probe::state::summary());
            next_heartbeat_ns = now_ns.saturating_add(heartbeat_period_ns);
        }

        thread::sleep(Duration::from_millis(5));
    }

    app.stop_all_tasks()?;

    if let Some(err) = run_error {
        return Err(err);
    }

    let snapshot = probe::state::snapshot();
    if snapshot.frames == 0 {
        return Err(CuError::from(
            "no point clouds were received; check the USB serial mapping and sensor power",
        ));
    }
    if snapshot.last_points == 0 {
        return Err(CuError::from(
            "frames arrived but all points were filtered; lower min_range_m or check the stream format",
        ));
    }
    if snapshot.last_median_distance_m < 0.05 || snapshot.last_median_distance_m > 10.5 {
        return Err(CuError::from(format!(
            "median radial distance looks absurd: {:.2}m",
            snapshot.last_median_distance_m
        )));
    }

    let final_summary = probe::state::summary();
    info!("[sen0682/final] {}", &final_summary);
    println!("{final_summary}");

    if let Some(expected_distance_m) = args.expected_distance_m {
        let delta = (snapshot.last_median_distance_m - expected_distance_m).abs();
        if delta > args.distance_tolerance_m {
            return Err(CuError::from(format!(
                "median radial distance {:.2}m is outside {:.2}m ± {:.2}m",
                snapshot.last_median_distance_m, expected_distance_m, args.distance_tolerance_m
            )));
        }
    }

    Ok(())
}
