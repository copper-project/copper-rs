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
    // This offline adapter owns stepping and must gate each iteration on recording capacity.
    // Keep access to the generated runtime so that backpressure stays outside task execution.
    let mut app = app.start(&mut lifecycle)?.into_inner();
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
    let mut conversion_time = Duration::ZERO;
    let mut iteration_time = Duration::ZERO;
    let mut recording_wait_time = Duration::ZERO;
    let mut completed = 0_u32;
    let result = (|| -> CuResult<u32> {
        let Request::Hello {
            version: 1,
            observations,
            scenario,
        } = channel.receive()?
        else {
            return Err("Expected Isaac protocol version 1 hello".into());
        };
        if observations > 10_000 {
            return Err("Observation count must be between 0 (continuous) and 10000".into());
        }
        fs::write(
            log.with_extension("scenario.json"),
            serde_json::to_vec_pretty(&scenario)
                .map_err(|e| CuError::new_with_cause("Encode scenario", e))?,
        )
        .map_err(io_error)?;
        let mut previous = None;
        while observations == 0 || completed < observations {
            let sequence = u64::from(completed);
            let started = Instant::now();
            let manager = &mut app.copper_runtime_mut().copperlists_manager;
            // Async logging reserves one slot for the next iteration. Wait before granting
            // another simulator lease, rather than overrunning its bounded recording queue.
            while manager.available_copper_lists()? < 2 {
                let slot = manager.wait_reclaim_slot()?;
                manager.recycle_slot(slot);
            }
            recording_wait_time += started.elapsed();
            let lease = buffers.acquire()?;
            channel.send(&Response::Lease {
                sequence,
                camera: &lease.camera_descriptor,
                lidar: &lease.lidar_descriptor,
                max_points: tasks::MAX_POINTS,
                point_bytes: transport::POINT_BYTES,
            })?;
            let frame = match channel.receive()? {
                Request::Frame(frame) => frame,
                Request::Finish if observations == 0 => break,
                _ => return Err("Expected published sensor frame".into()),
            };
            frame.validate(sequence, previous.as_ref())?;
            let started = Instant::now();
            let (image, cloud) = buffers.observation(&lease, &frame)?;
            conversion_time += started.elapsed();
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
                        output
                            .metadata
                            .set_status(format_args!("320×240 #{}", frame.sequence));
                        SimOverride::ExecutedBySim
                    }
                    Lidar(Process(_, output)) => {
                        lidar_calls += 1;
                        output.set_payload(cloud.clone());
                        output.tov = transport::lidar_tov(&frame);
                        output
                            .metadata
                            .set_status(format_args!("{} pts", frame.points));
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
                            output.metadata.set_status("STOP override");
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
            let started = Instant::now();
            #[allow(deprecated)] // The adapter owns lifecycle and the off-graph recording gate.
            app.run_one_iteration(&mut callback)?;
            iteration_time += started.elapsed();
            completed = completed
                .checked_add(1)
                .ok_or(CuError::from("Observation counter overflow"))?;
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
        if observations != 0 && !matches!(channel.receive()?, Request::Finish) {
            return Err("Expected end of simulator session".into());
        }
        info!("Isaac completed {} observations", completed);
        Ok(completed)
    })();
    let started = Instant::now();
    #[allow(deprecated)] // Paired with the typed start before taking ownership of stepping.
    app.stop_all_tasks(&mut lifecycle)?;
    app.log_shutdown_completed()?;
    let shutdown_time = started.elapsed();
    let runtime = app.copper_runtime_mut();
    let dropped_copperlists = runtime.copperlists_manager.dropped_copperlists_total();
    let dropped_keyframes = runtime.keyframes_manager.dropped_keyframes_total();
    fs::write(
        log.with_extension("performance.json"),
        serde_json::to_vec_pretty(&serde_json::json!({
            "observations": completed,
            "conversion_s": conversion_time.as_secs_f64(),
            "iteration_s": iteration_time.as_secs_f64(),
            "recording_wait_s": recording_wait_time.as_secs_f64(),
            "shutdown_s": shutdown_time.as_secs_f64(),
            "dropped_copperlists": dropped_copperlists,
            "dropped_keyframes": dropped_keyframes,
        }))
        .map_err(|e| CuError::new_with_cause("Encode native performance", e))?,
    )
    .map_err(io_error)?;
    let observations = result?;
    if dropped_copperlists != 0 || dropped_keyframes != 0 {
        return Err(
            "Incomplete sensor recording; see native performance.json for dropped records".into(),
        );
    }
    channel.send(&Response::Finished {
        observations,
        log: &log,
    })?;
    Ok(())
}
