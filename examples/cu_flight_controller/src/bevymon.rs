extern crate alloc;

mod messages;
#[cfg(not(target_arch = "wasm32"))]
#[path = "sim/rc_joystick.rs"]
mod rc_joystick;
mod sim_support;
mod tasks;

include!("sim.rs");

fn main() {
    run_bevymon();
}
