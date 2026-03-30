#![cfg_attr(all(feature = "rp2350-demo", target_arch = "arm"), no_std)]
#![cfg_attr(all(feature = "rp2350-demo", target_arch = "arm"), no_main)]

#[cfg(all(feature = "rp2350-demo", target_arch = "arm"))]
extern crate alloc;

#[cfg(all(feature = "rp2350-demo", target_arch = "arm"))]
mod firmware {
    use alloc::vec;
    use buddy_system_allocator::LockedHeap as Heap;
    use cortex_m_rt::entry;
    use cu_mpu9250::Mpu9250Source;
    use cu29::prelude::*;
    use cu29::resource::{ResourceBundle, ResourceManager};
    use cu29::units::si::angle::radian;
    use cu29::{CuError, CuResult, bundle_resources};
    use defmt::{Debug2Format, info, warn};
    use defmt_rtt as _;
    use embedded_hal::spi::MODE_0;
    use hal::fugit::RateExtU32;
    use panic_probe as _;
    use rp235x_hal as hal;
    use rp235x_hal::Clock;

    const XOSC_HZ: u32 = 12_000_000;
    const IMU_SPI_HZ: u32 = 1_000_000;
    const HEAP_SIZE: usize = 64 * 1024;

    // SAFETY: The RP2350 boot ROM expects the image definition in .start_block.
    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    #[global_allocator]
    static ALLOC: Heap<32> = Heap::empty();
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    fn init_heap() {
        // SAFETY: HEAP_MEM is a dedicated static buffer for this allocator instance.
        unsafe {
            let start = core::ptr::addr_of_mut!(HEAP_MEM) as usize;
            ALLOC.lock().init(start, HEAP_SIZE);
        }
    }

    pub mod tasks {
        use super::*;
        use cu_sensor_payloads::MagnetometerPayload;

        #[derive(Reflect)]
        pub struct RpySink;

        impl Freezable for RpySink {}

        #[derive(Reflect)]
        pub struct NullMagSource;

        impl Freezable for NullMagSource {}

        impl CuSrcTask for NullMagSource {
            type Resources<'r> = ();
            type Output<'m> = output_msg!(MagnetometerPayload);

            fn new(
                _config: Option<&ComponentConfig>,
                _resources: Self::Resources<'_>,
            ) -> CuResult<Self>
            where
                Self: Sized,
            {
                Ok(Self)
            }

            fn process<'o>(
                &mut self,
                _ctx: &CuContext,
                output: &mut Self::Output<'o>,
            ) -> CuResult<()> {
                output.clear_payload();
                output.tov = Tov::None;
                Ok(())
            }
        }

        impl CuSinkTask for RpySink {
            type Resources<'r> = ();
            type Input<'m> = input_msg!(cu_ahrs::AhrsPose);

            fn new(
                _config: Option<&ComponentConfig>,
                _resources: Self::Resources<'_>,
            ) -> CuResult<Self>
            where
                Self: Sized,
            {
                Ok(Self)
            }

            fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
                if let Some(pose) = input.payload() {
                    info!(
                        "AHRS RPY [rad]: roll={} pitch={} yaw={}",
                        pose.roll.get::<radian>(),
                        pose.pitch.get::<radian>(),
                        pose.yaw.get::<radian>()
                    );
                }
                Ok(())
            }
        }
    }

    pub mod resources {
        use super::*;
        use spin::Mutex;

        type Mosi =
            hal::gpio::Pin<hal::gpio::bank0::Gpio11, hal::gpio::FunctionSpi, hal::gpio::PullDown>;
        type Miso =
            hal::gpio::Pin<hal::gpio::bank0::Gpio12, hal::gpio::FunctionSpi, hal::gpio::PullDown>;
        type Sck =
            hal::gpio::Pin<hal::gpio::bank0::Gpio10, hal::gpio::FunctionSpi, hal::gpio::PullDown>;
        type SpiPins = (Mosi, Miso, Sck);

        pub type SpiBusInner = hal::Spi<hal::spi::Enabled, hal::pac::SPI1, SpiPins>;
        pub type CsPinInner = hal::gpio::Pin<
            hal::gpio::bank0::Gpio13,
            hal::gpio::FunctionSio<hal::gpio::SioOutput>,
            hal::gpio::PullDown,
        >;
        pub type DelayTimerInner = hal::Timer<hal::timer::CopyableTimer0>;

        pub struct SingleThreaded<T>(T);

        impl<T> SingleThreaded<T> {
            fn new(inner: T) -> Self {
                Self(inner)
            }
        }

        // SAFETY: The RP2350 demo runtime is single-threaded.
        unsafe impl<T> Send for SingleThreaded<T> {}
        // SAFETY: The RP2350 demo runtime is single-threaded.
        unsafe impl<T> Sync for SingleThreaded<T> {}

        impl<T> embedded_hal::spi::ErrorType for SingleThreaded<T>
        where
            T: embedded_hal::spi::ErrorType,
        {
            type Error = T::Error;
        }

        impl<T> embedded_hal::spi::SpiBus<u8> for SingleThreaded<T>
        where
            T: embedded_hal::spi::SpiBus<u8>,
        {
            fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                self.0.read(words)
            }

            fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
                self.0.write(words)
            }

            fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
                self.0.transfer(read, write)
            }

            fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                self.0.transfer_in_place(words)
            }

            fn flush(&mut self) -> Result<(), Self::Error> {
                self.0.flush()
            }
        }

        impl<T> embedded_hal::digital::ErrorType for SingleThreaded<T>
        where
            T: embedded_hal::digital::ErrorType,
        {
            type Error = T::Error;
        }

        impl<T> embedded_hal::digital::OutputPin for SingleThreaded<T>
        where
            T: embedded_hal::digital::OutputPin,
        {
            fn set_low(&mut self) -> Result<(), Self::Error> {
                self.0.set_low()
            }

            fn set_high(&mut self) -> Result<(), Self::Error> {
                self.0.set_high()
            }
        }

        impl<T> embedded_hal::delay::DelayNs for SingleThreaded<T>
        where
            T: embedded_hal::delay::DelayNs,
        {
            fn delay_ns(&mut self, ns: u32) {
                self.0.delay_ns(ns);
            }
        }

        pub type SpiBus = SingleThreaded<SpiBusInner>;
        pub type CsPin = SingleThreaded<CsPinInner>;
        pub type DelayTimer = SingleThreaded<DelayTimerInner>;

        /// Resource-backed MPU9250 source alias for RP2350 SPI1 wiring (GPIO10-13).
        pub type RpMpu9250Source = Mpu9250Source<SpiBus, CsPin, DelayTimer>;

        static SPI: Mutex<Option<SpiBusInner>> = Mutex::new(None);
        static CS: Mutex<Option<CsPinInner>> = Mutex::new(None);
        static DELAY: Mutex<Option<DelayTimerInner>> = Mutex::new(None);

        pub fn stash_imu_resources(
            spi: SpiBusInner,
            cs: CsPinInner,
            delay: DelayTimerInner,
        ) -> CuResult<()> {
            if SPI.lock().replace(spi).is_some() {
                return Err(CuError::from("RP2350 SPI resource already initialized"));
            }
            if CS.lock().replace(cs).is_some() {
                return Err(CuError::from("RP2350 CS resource already initialized"));
            }
            if DELAY.lock().replace(delay).is_some() {
                return Err(CuError::from("RP2350 delay resource already initialized"));
            }
            Ok(())
        }

        pub struct Rp2350ImuBundle;

        bundle_resources!(Rp2350ImuBundle: Spi, Cs, Delay);

        impl ResourceBundle for Rp2350ImuBundle {
            fn build(
                bundle: cu29::resource::BundleContext<Self>,
                _config: Option<&cu29::config::ComponentConfig>,
                manager: &mut ResourceManager,
            ) -> CuResult<()> {
                let spi = SPI
                    .lock()
                    .take()
                    .ok_or_else(|| CuError::from("RP2350 SPI resource not initialized"))?;
                let cs = CS
                    .lock()
                    .take()
                    .ok_or_else(|| CuError::from("RP2350 CS resource not initialized"))?;
                let delay = DELAY
                    .lock()
                    .take()
                    .ok_or_else(|| CuError::from("RP2350 delay resource not initialized"))?;

                manager.add_owned(bundle.key(Rp2350ImuBundleId::Spi), SingleThreaded::new(spi))?;
                manager.add_owned(bundle.key(Rp2350ImuBundleId::Cs), SingleThreaded::new(cs))?;
                manager.add_owned(
                    bundle.key(Rp2350ImuBundleId::Delay),
                    SingleThreaded::new(delay),
                )?;
                Ok(())
            }
        }
    }

    fn build_clock(timer: resources::DelayTimerInner) -> RobotClock {
        RobotClock::new_with_rtc(
            {
                let timer_now = timer;
                move || timer_now.get_counter().ticks() * 1_000
            },
            {
                let timer_wait = timer;
                move |ns| {
                    let start = timer_wait.get_counter().ticks();
                    let wait_us = ns / 1_000;
                    while timer_wait.get_counter().ticks().wrapping_sub(start) < wait_us {
                        core::hint::spin_loop();
                    }
                }
            },
        )
    }

    #[copper_runtime(config = "examples/rp_copperconfig.ron")]
    struct Rp2350CopperApp {}

    #[entry]
    fn main() -> ! {
        init_heap();

        let mut p = match hal::pac::Peripherals::take() {
            Some(p) => p,
            None => {
                warn!("RP235x peripherals unavailable");
                loop {
                    cortex_m::asm::wfi();
                }
            }
        };
        let mut watchdog = hal::watchdog::Watchdog::new(p.WATCHDOG);
        let clocks = match hal::clocks::init_clocks_and_plls(
            XOSC_HZ,
            p.XOSC,
            p.CLOCKS,
            p.PLL_SYS,
            p.PLL_USB,
            &mut p.RESETS,
            &mut watchdog,
        ) {
            Ok(clocks) => clocks,
            Err(_) => {
                warn!("clock init failed");
                loop {
                    cortex_m::asm::wfi();
                }
            }
        };

        let delay: resources::DelayTimerInner =
            hal::Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);
        let clock = build_clock(delay);

        let sio = hal::sio::Sio::new(p.SIO);
        let pins = hal::gpio::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

        let sck = pins.gpio10.into_function();
        let mosi = pins.gpio11.into_function();
        let miso = pins.gpio12.into_function();
        let mut cs = pins.gpio13.into_push_pull_output();
        let _ = embedded_hal::digital::OutputPin::set_high(&mut cs);

        let spi = hal::Spi::<_, _, _, 8>::new(p.SPI1, (mosi, miso, sck)).init(
            &mut p.RESETS,
            clocks.peripheral_clock.freq(),
            IMU_SPI_HZ.Hz(),
            hal::spi::FrameFormat::MotorolaSpi(MODE_0),
        );

        if let Err(e) = resources::stash_imu_resources(spi, cs, delay) {
            warn!("resource init failed: {:?}", Debug2Format(&e));
            loop {
                cortex_m::asm::wfi();
            }
        }

        let mut app = match Rp2350CopperApp::builder().with_clock(clock).build() {
            Ok(app) => app,
            Err(e) => {
                warn!("app build failed: {:?}", Debug2Format(&e));
                loop {
                    cortex_m::asm::wfi();
                }
            }
        };
        if let Err(e) =
            <Rp2350CopperApp as CuApplication<NoopSectionStorage, NoopLogger>>::run(&mut app)
        {
            warn!("runtime exited: {:?}", Debug2Format(&e));
        }
        loop {
            cortex_m::asm::wfi();
        }
    }
}

#[cfg(not(all(feature = "rp2350-demo", target_arch = "arm")))]
fn main() {
    println!(
        "rp2350_copper requires --no-default-features --features rp2350-demo and the RP2350 target config."
    );
}
