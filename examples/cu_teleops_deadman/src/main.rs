//! A red button over UDP, with the robot's actual transition returned over FEC.
mod link;
mod policy;
use cu29::logstream::{FecTelemetryRx, TelemetryBufferConfig};
use cu29::prelude::*;
use cu29::teleops::ReturnTx;

pub use policy::{Deadman, Tick};
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
    let (twin, mut control) = ground::Twin::from_telemetry(fec)
        .with_teleops::<Controls>(move |session| ReturnTx::new(return_tx, session, 1))?;
    let (mut twin, mut frames) = twin.with_log_path(dir.join("twin.copper")).spawn()?;
    let clock = RobotClock::new();
    let (publish, inboxes) = Controls::inboxes()?;
    let robot_clock = clock.clone();
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
            move |clock| link::robot_tx(data_tx, return_rx, publish, clock, robot_clock),
            TelemetryBufferConfig {
                capture_count: 64,
                keyframe_count: 8,
                ..TelemetryBufferConfig::new(512, 512)
            },
        )
        .build()?;
    let mut robot = robot.start()?;
    let mut previous = None;
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
            let was_armed = list
                .id
                .checked_sub(1)
                .is_some_and(|id| previous == Some((id, false)));
            previous = safe.payload().map(|safe| (list.id, *safe));
            if let Some((start, instant)) = pressed {
                if list.msgs.get_button_output().payload() == Some(&true)
                    && safe.payload() == Some(&true)
                {
                    if !was_armed {
                        return Err(
                            "missing preceding armed frame or watchdog already expired".into()
                        );
                    }
                    let flipped = safe.metadata.process_time.end.unwrap();
                    let outbound = (flipped - start).as_nanos();
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
            if safe.payload() == Some(&false)
                && let Some(tx) = key_tx.take()
            {
                if automatic {
                    tx.send((clock.now(), Instant::now()))?
                } else {
                    println!("Robot armed. 🔴 Press Enter to trip the deadman.");
                    let key_clock = clock.clone();
                    std::thread::spawn(move || {
                        let mut line = String::new();
                        if matches!(std::io::stdin().read_line(&mut line),Ok(n) if n>0) {
                            let _ = tx.send((key_clock.now(), Instant::now()));
                        }
                    });
                    deadline = Instant::now() + Duration::from_secs(300);
                }
            }
            let submission = if let Ok(sample) = key_rx.try_recv() {
                pressed = Some(sample);
                deadline = Instant::now() + Duration::from_secs(5);
                let mut msg = CuMsg::default();
                msg.set_payload(true);
                msg.tov = Tov::Time(sample.0);
                control.button().try_send_msg(&msg)
            } else {
                control.observing(list.id).button().try_send(&false)
            };
            submission.map_err(|err| format!("button submit: {err:?}"))?;
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
