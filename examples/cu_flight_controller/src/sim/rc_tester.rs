#![cfg(feature = "sim")]

mod rc_joystick;

use rc_joystick::RcJoystick;
use std::env;
use std::io;
use std::process;

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

    joystick.print_loop()
}
