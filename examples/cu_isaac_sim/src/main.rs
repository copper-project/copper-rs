//! Native Copper graph driven once per shared-memory observation by Isaac simulation time.

mod tasks;
mod transport;

use clap::Parser;
use clap::ValueEnum;
use cu29::prelude::*;
use cu29::units::si::angular_velocity::radian_per_second;
use cu29::units::si::length::meter;
use std::fs;
use std::os::unix::net::UnixListener;
use std::path::PathBuf;
use std::time::Duration;
use std::time::Instant;
use transport::Buffers;
use transport::Channel;
use transport::Request;
use transport::Response;
use transport::io_error;

#[derive(Clone, Copy, ValueEnum)]
enum ControllerOverride {
    Stop,
}

#[derive(Parser)]
struct Options {
    #[arg(long)]
    session: PathBuf,
    #[arg(long, default_value = "logs/isaac.copper")]
    log: PathBuf,
    #[arg(long, value_enum)]
    override_controller: Option<ControllerOverride>,
}

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct IsaacApp {}

fn lifecycle(step: default::SimStep<'_>) -> SimOverride {
    use default::SimStep::*;
    match step {
        Camera(_) | Lidar(_) | Motor(_) | Telemetry(_) => SimOverride::ExecutedBySim,
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn main() -> CuResult<()> {
    let options = Options::parse();
    let session = options.session.canonicalize().map_err(io_error)?;
    let listener = UnixListener::bind(session.join("control.sock")).map_err(io_error)?;
    listener.set_nonblocking(true).map_err(io_error)?;
    let mut buffers = Buffers::new(&session)?;
    let (clock, mock) = RobotClock::mock();
    let log = if options.log.is_absolute() {
        options.log
    } else {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(options.log)
    };
    fs::create_dir_all(
        log.parent()
            .ok_or(CuError::from("Log needs a parent directory"))?,
    )
    .map_err(io_error)?;
    let app = IsaacApp::builder()
        .with_clock(clock.clone())
        .with_log_path(&log, Some(256 * 1024 * 1024))?
        .with_sim_callback(&mut lifecycle)
        .build()?;
    let mut app = app.start(&mut lifecycle)?;
    let deadline = Instant::now() + Duration::from_secs(600);
    let stream = loop {
        match listener.accept() {
            Ok((stream, _)) => break stream,
            Err(error)
                if error.kind() == std::io::ErrorKind::WouldBlock && Instant::now() < deadline =>
            {
                std::thread::sleep(Duration::from_millis(20))
            }
            Err(error) => return Err(io_error(error)),
        }
    };
    let mut channel = Channel::new(stream)?;
    let result = (|| -> CuResult<()> {
        let Request::Hello {
            version: 1,
            observations,
            scenario,
        } = channel.receive()?
        else {
            return Err("Expected Isaac protocol version 1 hello".into());
        };
        if observations == 0 || observations > 10_000 {
            return Err("Observation count must be between 1 and 10000".into());
        }
        fs::write(
            log.with_extension("scenario.json"),
            serde_json::to_vec_pretty(&scenario)
                .map_err(|e| CuError::new_with_cause("Encode scenario", e))?,
        )
        .map_err(io_error)?;
        let mut previous = None;
        for sequence in 0..u64::from(observations) {
            let lease = buffers.acquire()?;
            channel.send(&Response::Lease {
                sequence,
                camera: &lease.camera_descriptor,
                lidar: &lease.lidar_descriptor,
                max_points: tasks::MAX_POINTS,
                point_bytes: transport::POINT_BYTES,
            })?;
            let Request::Frame(frame) = channel.receive()? else {
                return Err("Expected published sensor frame".into());
            };
            frame.validate(sequence, previous.as_ref())?;
            let (image, cloud) = buffers.observation(&lease, &frame)?;
            mock.set_value(frame.simulation_time_ns);
            let mut command = None;
            let mut clearance = None;
            let mut stats = None;
            let mut camera_calls = 0;
            let mut lidar_calls = 0;
            let mut callback = |step: default::SimStep<'_>| {
                use CuTaskCallbackState::*;
                use default::SimStep::*;
                match step {
                    Camera(Process(_, output)) => {
                        camera_calls += 1;
                        output.set_payload(image.clone());
                        output.tov = Tov::Time(CuTime::from_nanos(frame.camera_time_ns));
                        SimOverride::ExecutedBySim
                    }
                    Lidar(Process(_, output)) => {
                        lidar_calls += 1;
                        output.set_payload(cloud.clone());
                        output.tov = transport::lidar_tov(&frame);
                        SimOverride::ExecutedBySim
                    }
                    Controller(Process(input, output)) => {
                        clearance = input
                            .payload()
                            .and_then(|v| v.distance)
                            .map(|v| v.get::<meter>());
                        if options.override_controller.is_some() {
                            output.set_payload(tasks::MotorCommand::default());
                            output.tov = input.tov;
                            SimOverride::ExecutedBySim
                        } else {
                            SimOverride::ExecuteByRuntime
                        }
                    }
                    Motor(Process(input, _)) => {
                        command = input.payload().cloned();
                        SimOverride::ExecutedBySim
                    }
                    Telemetry(Process(input, _)) => {
                        stats = input.payload().map(|v| v.mean_rgb);
                        SimOverride::ExecutedBySim
                    }
                    other => lifecycle(other),
                }
            };
            app.run_one_iteration(&mut callback)?;
            if camera_calls != 1
                || lidar_calls != 1
                || clock.now().as_nanos() != frame.simulation_time_ns
            {
                return Err("One observation must produce exactly one sensor/control iteration at simulation time".into());
            }
            let command = command.ok_or(CuError::from("Motor callback produced no command"))?;
            let mean_rgb = stats.ok_or(CuError::from("Image statistics did not execute"))?;
            channel.send(&Response::Command {
                sequence,
                simulation_time_ns: clock.now().as_nanos(),
                left_rad_s: command.left.get::<radian_per_second>(),
                right_rad_s: command.right.get::<radian_per_second>(),
                clearance_m: clearance,
                mean_rgb,
            })?;
            previous = Some(frame);
        }
        if !matches!(channel.receive()?, Request::Finish) {
            return Err("Expected end of simulator session".into());
        }
        channel.send(&Response::Finished {
            observations,
            log: &log,
        })?;
        info!("Isaac completed {} observations", observations);
        Ok(())
    })();
    app.stop(&mut lifecycle)?;
    result
}
