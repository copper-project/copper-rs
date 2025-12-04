#![cfg_attr(all(feature = "rp2350-demo", target_arch = "arm"), no_std)]
#![cfg_attr(all(feature = "rp2350-demo", target_arch = "arm"), no_main)]

#[cfg(all(feature = "rp2350-demo", target_arch = "arm"))]
mod firmware {
    use cortex_m_rt::entry;
    use cu29::prelude::*;
    use cu_embedded_registry as reg;
    use cu_mpu9250::embedded_hal::EmbeddedHalDriver;
    use cu_mpu9250::Mpu9250Source;
    use defmt::{info, warn, Debug2Format};
    use defmt_rtt as _;
    use hal::fugit::RateExtU32;
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

    pub mod registry {
        use super::*;

        type Mosi =
            hal::gpio::Pin<hal::gpio::bank0::Gpio11, hal::gpio::FunctionSpi, hal::gpio::PullDown>;
        type Miso =
            hal::gpio::Pin<hal::gpio::bank0::Gpio12, hal::gpio::FunctionSpi, hal::gpio::PullDown>;
        type Sck =
            hal::gpio::Pin<hal::gpio::bank0::Gpio10, hal::gpio::FunctionSpi, hal::gpio::PullDown>;
        type SpiPins = (Mosi, Miso, Sck);
        type SpiBus = hal::Spi<hal::spi::Enabled, hal::pac::SPI1, SpiPins>;
        pub type CsPin = hal::gpio::Pin<
            hal::gpio::bank0::Gpio13,
            hal::gpio::FunctionSio<hal::gpio::SioOutput>,
            hal::gpio::PullDown,
        >;
        pub type DelayTimer = hal::Timer<hal::timer::CopyableTimer0>;

        /// Registry-backed MPU9250 source alias for RP2350 SPI1 wiring (GPIO10-13).
        pub type RpMpu9250Source = Mpu9250Source<EmbeddedHalDriver<SpiBus, CsPin>>;

        pub fn register_spi_cs_delay() -> CuResult<()> {
            let mut p = hal::pac::Peripherals::take().ok_or("RP235x peripherals unavailable")?;
            let mut watchdog = hal::watchdog::Watchdog::new(p.WATCHDOG);
            let clocks = hal::clocks::init_clocks_and_plls(
                12_000_000,
                p.XOSC,
                p.CLOCKS,
                p.PLL_SYS,
                p.PLL_USB,
                &mut p.RESETS,
                &mut watchdog,
            )
            .map_err(|_| CuError::from("clock init failed"))?;

            let timer: DelayTimer = hal::Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);

            let sio = hal::sio::Sio::new(p.SIO);
            let pins =
                hal::gpio::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

            let sck: Sck = pins.gpio10.into_function();
            let mosi: Mosi = pins.gpio11.into_function();
            let miso: Miso = pins.gpio12.into_function();
            let mut cs: CsPin = pins.gpio13.into_push_pull_output();
            let _ = cs.set_high();

            let spi: SpiBus = hal::Spi::<_, _, _, 8>::new(p.SPI1, (mosi, miso, sck)).init(
                &mut p.RESETS,
                clocks.peripheral_clock.freq(),
                1_000_000u32.Hz(),
                hal::spi::FrameFormat::MotorolaSpi(hal::spi::MODE_0),
            );

            reg::register_spi(0, spi)?;
            reg::register_cs(0, cs)?;
            reg::register_delay(0, timer)?;
            Ok(())
        }
    }

    #[copper_runtime(config = "examples/rp_copperconfig.ron")]
    struct Rp2350CopperApp {}

    #[entry]
    fn main() -> ! {
        if let Err(e) = registry::register_spi_cs_delay() {
            warn!("registry setup failed: {:?}", Debug2Format(&e));
            loop {
                cortex_m::asm::wfi();
            }
        }
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
