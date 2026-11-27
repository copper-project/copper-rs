//! Replay robot telemetry and measure a button press on the operator's clock.
mod latency;
mod ui;
use cu_teleops_deadman::*;
use cu29::logstream::FecTelemetryRx;
use cu29::prelude::*;
use cu29::teleops::{InputSampler, ReturnTx, Sample};
use latency::{Latency, OperatorClock};
use std::error::Error;

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct Replay {}

const CONFIRM_TIMEOUT: CuDuration = CuDuration(5_000_000_000);

#[derive(Default)]
enum Measurement {
    #[default]
    Ready,
    Pressed {
        start: CuInstant,
        stamp: CuTime,
        before_send: CuDuration,
        synced: bool,
        pending: Option<Sample<Command>>,
    },
    Confirmed(Latency),
}

impl Measurement {
    fn press(&mut self, clock: &RobotClock, sampler: &InputSampler, synced: bool) -> CuResult<()> {
        if matches!(self, Self::Ready) {
            let start = clock.raw_now();
            let sample = sampler.sample(Command::Stop);
            let Tov::Time(stamp) = sample.tov() else {
                return Err("missing operator clock".into());
            };
            *self = Self::Pressed {
                start,
                stamp,
                synced,
                before_send: CuDuration::MIN,
                pending: Some(sample),
            };
        }
        Ok(())
    }

    fn status(&self, armed: bool, last_frame: Option<CuInstant>, now: CuInstant) -> &'static str {
        match self {
            Self::Confirmed(_) => "Trip confirmed",
            Self::Pressed { start, .. } if now - *start >= CONFIRM_TIMEOUT => {
                "No confirmation received"
            }
            Self::Pressed { .. } => "Waiting for trip confirmation",
            Self::Ready if last_frame.is_none() => "Waiting for telemetry",
            Self::Ready if last_frame.is_some_and(|at| now - at >= PERMIT_LIFETIME) => {
                "Telemetry stale"
            }
            Self::Ready if armed => "Robot armed",
            Self::Ready => "Actuation inhibited",
        }
    }

    fn latency(&self) -> Option<Latency> {
        match self {
            Self::Confirmed(result) => Some(*result),
            _ => None,
        }
    }

    fn confirm(&mut self, state: Option<&Safety>, observed: CuInstant) {
        if let Self::Pressed {
            start,
            stamp,
            synced,
            before_send,
            pending: None,
        } = self
            && let Some(Safety::ButtonTrip { pressed, flipped }) = state
            && pressed == stamp
            && observed >= *start
        {
            *self = Self::Confirmed(Latency {
                pressed: *pressed,
                flipped: *flipped,
                synced: *synced,
                confirmation: observed - *start,
                before_send: *before_send,
            });
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1).peekable();
    let bind = args
        .next_if(|a| a != "--auto")
        .unwrap_or_else(|| "127.0.0.1:7442".into());
    let peer = args
        .next_if(|a| a != "--auto")
        .unwrap_or_else(|| "127.0.0.1:7441".into());
    let auto = args.next_if(|a| a == "--auto").is_some();
    if args.next().is_some() {
        return Err("usage: twin [bind_addr [robot_addr]] [--auto]".into());
    }
    let (return_tx, data_rx) = endpoints(&bind, &peer)?;
    let fec = FecTelemetryRx::new(data_rx, Default::default(), None)?;
    let clock = RobotClock::new();
    let (builder, mut control) = Replay::from_telemetry(fec)
        .with_teleops::<Controls>(Some(clock.clone()), move |session| {
            ReturnTx::<_, RETURN_PACKET_BYTES>::new(return_tx, session)
        })?;
    let (path, _lock) = lock_log("twin")?;
    // The twin requires a fresh archive. Its writer removes surplus slabs on creation.
    for entry in [cu29::replay::first_slab_path(&path)?, path.clone()] {
        match std::fs::remove_file(entry) {
            Err(err) if err.kind() != std::io::ErrorKind::NotFound => return Err(err.into()),
            _ => {}
        }
    }
    let (mut twin, mut frames) = builder.with_log_path(path).spawn()?;
    let mut screen = if auto {
        None
    } else {
        Some(ui::Screen::open()?)
    };
    let mut last_frame = None;
    let mut twinned_cl = None;
    let mut armed = false;
    let mut latch = None;
    let mut resume = None;
    let mut timing: Option<OperatorClock> = None;
    let permits = InputSampler::new(None);
    let mut measurement = Measurement::Ready;
    let mut deadline = clock.raw_now() + CONFIRM_TIMEOUT;
    loop {
        if let Some(screen) = &mut screen {
            let now = clock.raw_now();
            screen.draw(
                now,
                twinned_cl,
                timing
                    .as_ref()
                    .map(|t| t.link_status(now))
                    .unwrap_or_default(),
                if latch.is_some() && last_frame.is_some_and(|at| now - at < PERMIT_LIFETIME) {
                    "SAFEMODE · actuation off"
                } else {
                    measurement.status(armed, last_frame, now)
                },
                latch.is_some(),
                measurement.latency(),
            )?;
            match screen.input()? {
                ui::Action::Quit => break,
                ui::Action::Resume if last_frame.is_some_and(|at| now - at < PERMIT_LIFETIME) => {
                    resume = latch
                }
                ui::Action::Trip if armed => measurement.press(
                    &clock,
                    &control.sampler,
                    timing.as_ref().is_some_and(OperatorClock::synced),
                )?,
                _ => {}
            }
        }
        while let Some(read) = frames.try_read() {
            let frame = &read.frame;
            let list = &frame.copperlist;
            twinned_cl = Some(list.id);
            let state = list.msgs.get_watchdog_output().payload();
            let observed = clock.raw_now();
            last_frame = Some(observed);
            let timing = match &mut timing {
                Some(existing) => existing,
                slot @ None => slot.insert(OperatorClock::new(
                    clock.clone(),
                    frame.identity.session_id,
                )?),
            };
            let tick = *list
                .msgs
                .get_tick_output()
                .payload()
                .ok_or("missing robot tick")?;
            timing.observe(
                list.msgs.get_button_output().payload(),
                list.msgs.get_watchdog_output().tov,
                observed,
            )?;
            armed = state == Some(&Safety::Armed)
                && (!timing.synced() || clock.now() - tick < PERMIT_LIFETIME);
            let previous_latch = latch;
            latch = state.copied().and_then(Safety::latch);
            if previous_latch.is_some() && armed {
                measurement = Measurement::Ready;
            }
            if auto && armed && matches!(measurement, Measurement::Ready) && timing.synced() {
                measurement.press(&clock, &control.sampler, timing.synced())?;
                deadline = clock.raw_now() + CONFIRM_TIMEOUT;
            }
            measurement.confirm(state, observed);
            let button = if let Some(id) = resume.take().filter(|id| Some(*id) == latch) {
                permits.sample(Command::Resume(id))
            } else if let Measurement::Pressed {
                pending,
                start,
                before_send,
                ..
            } = &mut measurement
                && let Some(stop) = pending.take()
            {
                *before_send = clock.raw_now() - *start;
                stop
            } else if latch.is_none() && matches!(measurement, Measurement::Pressed { .. }) {
                continue;
            } else {
                let permit = latch.is_none() && matches!(measurement, Measurement::Ready);
                match timing.probe(permit) {
                    Some(probe) => permits.sample(probe),
                    None if permit => permits.sample(Command::Permit),
                    None => continue,
                }
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
        if last_frame.is_some_and(|at| clock.raw_now() - at >= PERMIT_LIFETIME) {
            armed = false;
        }
        if auto {
            if let Some(elapsed) = measurement.latency() {
                println!("{elapsed}");
                break;
            }
            if clock.raw_now() >= deadline {
                return Err(format!(
                    "Timed out waiting for armed robot or confirmed trip: {:?}",
                    frames.status()
                )
                .into());
            }
        }
        std::thread::sleep(CuDuration::from_millis(1).into());
    }
    twin.stop()?;
    Ok(())
}
