#![cfg(feature = "sim")]

mod rc_joystick;

use rc_joystick::RcJoystick;
use std::env;
use std::io;
use std::process;
use std::thread;
use std::time::Duration;

fn main() -> io::Result<()> {
    if cfg!(not(feature = "sim")) {
        // Should not happen because the bin is gated, but keep a clear message.
        eprintln!(
            "rc-tester requires the `sim` feature (cargo run --features sim --bin rc-tester)"
        );
        process::exit(1);
    }

    let preferred = env::args().nth(1);
    let mut joystick = RcJoystick::open(preferred.as_deref()).map_err(|e| {
        eprintln!("Failed to open RC joystick: {e}");
        e
    })?;

    let bindings = joystick.axis_bindings();
    println!("Using RC joystick: {}", joystick.device_name());
    println!(
        "Axis bindings: roll={:?} pitch={:?} yaw={:?} throttle={:?} sa={:?} sb={:?} sc={:?}",
        bindings.roll,
        bindings.pitch,
        bindings.yaw,
        bindings.throttle,
        bindings.knob_sa,
        bindings.knob_sb,
        bindings.knob_sc
    );
    let frame = joystick.current_frame();
    println!(
        "Initial frame: roll {:+.2} pitch {:+.2} yaw {:+.2} throttle {:+.2} sa {:+.2} sb {:+.2} sc {:+.2}",
        frame.roll,
        frame.pitch,
        frame.yaw,
        frame.throttle,
        frame.knob_sa,
        frame.knob_sb,
        frame.knob_sc
    );
    println!("Waiting for RC joystick events...");
    loop {
        match joystick.next_frame()? {
            Some(frame) => {
                let aux_values = frame
                    .aux
                    .iter()
                    .enumerate()
                    .map(|(i, a)| format!("aux{}:{:+.2}", i + 1, a.value))
                    .collect::<Vec<_>>()
                    .join(" ");
                let switch_values = frame
                    .switches
                    .iter()
                    .map(|s| format!("{}:{}", s.name, if s.on { "on" } else { "off" }))
                    .collect::<Vec<_>>()
                    .join(" ");
                println!(
                    "roll {:+.2} pitch {:+.2} yaw {:+.2} throttle {:+.2} sa {:+.2} sb {:+.2} sc {:+.2} | {} | {}",
                    frame.roll,
                    frame.pitch,
                    frame.yaw,
                    frame.throttle,
                    frame.knob_sa,
                    frame.knob_sb,
                    frame.knob_sc,
                    aux_values,
                    switch_values
                );
            }
            None => thread::sleep(Duration::from_millis(10)),
        }
    }
}
