//! A red button over UDP, with the robot's actual transition returned over FEC.
mod link;
mod policy;
use cu29::logstream::{FecTelemetryRx, TelemetryBufferConfig};
use cu29::prelude::*;
use cu29::teleops::ReturnTx;

pub use policy::{Deadman, Safety, Tick};
use std::error::Error;
use std::time::{Duration, Instant};

#[allow(dead_code)]
#[derive(cu29::prelude::Teleops)]
pub struct Controls {
    pub button: Latest<bool>,
}
pub struct Ops;
bundle_resources!(Ops: Button);
impl ResourceBundle for Ops {
    fn build(
        _: BundleContext<Self>,
        _: Option<&ComponentConfig>,
        _: &mut ResourceManager,
    ) -> CuResult<()> {
        Ok(())
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct Robot {}
mod ground {
    use cu29::prelude::*;
    #[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
    struct Replay {}
    pub type Twin = default::Replay;
}

fn main() -> Result<(), Box<dyn Error>> {
    let automatic = std::env::args().any(|arg| arg == "--auto");
    let dir = std::env::current_dir()?.join("logs").join(format!(
        "deadman-{}",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir)?;
    let (data_tx, data_rx) = link::loopback()?;
    let (return_tx, return_rx) = link::loopback()?;
    let fec = FecTelemetryRx::new(data_rx, Default::default(), None)?;
    let clock = RobotClock::new();
    let (twin, mut control) = ground::Twin::from_telemetry(fec)
        .with_teleops::<Controls>(Some(clock.clone()), move |session| {
            ReturnTx::<_, { link::RETURN_PACKET_BYTES }>::new(return_tx, session)
        })?;
    let (mut twin, mut frames) = twin.with_log_path(dir.join("twin.copper")).spawn()?;
    let (publish, inboxes) = Controls::inboxes()?;
    let robot = Robot::builder()
        .with_clock(clock.clone())
        .with_log_path(dir.join("robot.copper"), None)?
        .with_resources(move |cfg| {
            let mut manager = default::resources_instanciator(cfg)?;
            manager.add_owned(
                default::resources::bundles::OPS.key(OpsId::Button),
                inboxes.button,
            )?;
            Ok(manager)
        })
        .with_telemetry_tx(
            move |clock| link::robot_tx(data_tx, return_rx, publish, clock),
            TelemetryBufferConfig {
                capture_count: 64,
                keyframe_count: 8,
                ..TelemetryBufferConfig::new(512, 512)
            },
        )
        .build()?;
    let mut robot = robot.start()?;
    let (key_tx, key_rx) = std::sync::mpsc::channel();
    let mut key_tx = Some(key_tx);
    let mut pressed: Option<(CuTime, Instant)> = None;
    let mut deadline = Instant::now() + Duration::from_secs(5);
    'measure: loop {
        robot.run_one_iteration()?;
        while let Some(read) = frames.try_read() {
            let frame = &read.frame;
            let list = &frame.copperlist;
            let safe = list.msgs.get_safety_output();
            if let Some((start, instant)) = pressed {
                if let Some(Safety::ButtonTrip { pressed, flipped }) = safe.payload()
                    && *pressed == start
                {
                    let outbound = (*flipped - start).as_nanos();
                    let total = frame.received_at.duration_since(instant).as_nanos();
                    let inbound = total
                        .checked_sub(u128::from(outbound))
                        .ok_or("inconsistent clock measurements")?;
                    println!(
                        "Button → robot safe: {:.3} ms\nRobot safe → twin reception: {:.3} ms\nTotal: {:.3} ms",
                        outbound as f64 / 1e6,
                        inbound as f64 / 1e6,
                        total as f64 / 1e6
                    );
                    break 'measure;
                }
                continue;
            }
            if safe.payload() == Some(&Safety::Armed)
                && let Some(tx) = key_tx.take()
            {
                if automatic {
                    tx.send((control.sampler.sample(true), Instant::now()))?
                } else {
                    println!("Robot armed. 🔴 Press Enter to trip the deadman.");
                    let sampler = control.sampler.clone();
                    std::thread::spawn(move || {
                        let mut line = String::new();
                        if matches!(std::io::stdin().read_line(&mut line),Ok(n) if n>0) {
                            let _ = tx.send((sampler.sample(true), Instant::now()));
                        }
                    });
                    deadline = Instant::now() + Duration::from_secs(300);
                }
            }
            let button = if let Ok((sample, instant)) = key_rx.try_recv() {
                let Tov::Time(time) = sample.tov() else {
                    return Err("shared clock must timestamp the button".into());
                };
                pressed = Some((time, instant));
                deadline = Instant::now() + Duration::from_secs(5);
                sample
            } else {
                control.sampler.sample(false)
            };
            control
                .submit(
                    list.id,
                    &mut controls::Update {
                        button: Some(button),
                    },
                )
                .map_err(|err| format!("button submit: {err:?}"))?;
        }
        if Instant::now() >= deadline {
            return Err(format!(
                "deadline waiting for button transition: {:?}",
                frames.status()
            )
            .into());
        }
        std::thread::sleep(Duration::from_millis(1));
    }
    drop(robot.stop()?);
    twin.stop()?;
    Ok(())
}
