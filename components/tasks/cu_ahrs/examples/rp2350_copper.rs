#![cfg_attr(all(feature = "rp2350-demo", target_arch = "arm"), no_std)]
#![cfg_attr(all(feature = "rp2350-demo", target_arch = "arm"), no_main)]

#[cfg(all(feature = "rp2350-demo", target_arch = "arm"))]
mod firmware {
    use cortex_m_rt::entry;
    use cu29::prelude::*;
    use defmt::{info, warn, Debug2Format};
    use defmt_rtt as _;
    use panic_probe as _;
    use rp235x_hal as hal;

    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    pub mod tasks {
        use super::*;

        pub struct RpySink;

        impl Freezable for RpySink {}

        impl CuSinkTask for RpySink {
            type Input<'m> = input_msg!(cu_ahrs::AhrsPose);

            fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
            where
                Self: Sized,
            {
                Ok(Self)
            }

            fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
                if let Some(pose) = input.payload() {
                    info!(
                        "AHRS RPY [rad]: roll={:.3} pitch={:.3} yaw={:.3}",
                        pose.roll, pose.pitch, pose.yaw
                    );
                }
                Ok(())
            }
        }
    }

    #[copper_runtime(config = "examples/rp_copperconfig.ron")]
    struct Rp2350CopperApp {}

    #[entry]
    fn main() -> ! {
        let mut app = Rp2350CopperAppBuilder::new().build().expect("build app");
        if let Err(e) = app.run() {
            warn!("runtime exited: {:?}", Debug2Format(&e));
        }
        loop {
            cortex_m::asm::wfi();
        }
    }
}

#[cfg(not(all(feature = "rp2350-demo", target_arch = "arm")))]
fn main() {
    println!("rp2350_copper requires --no-default-features --features rp2350-demo and the RP2350 target config.");
}
