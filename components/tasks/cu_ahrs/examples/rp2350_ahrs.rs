#![cfg_attr(all(feature = "rp2350-demo", target_arch = "arm"), no_std)]
#![cfg_attr(all(feature = "rp2350-demo", target_arch = "arm"), no_main)]

#[cfg(all(feature = "rp2350-demo", target_arch = "arm"))]
mod firmware {
    use cortex_m_rt::entry;
    use cu29::clock::{CuDuration, RobotClock};
    use cu29::cutask::CuMsg;
    use cu29::prelude::CuTask;
    use cu_ahrs::CuAhrs;
    use cu_sensor_payloads::ImuPayload;
    use defmt::{info, warn, Debug2Format};
    use defmt_rtt as _;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::digital::OutputPin;
    use embedded_hal::spi::MODE_0;
    use mpu9250::Mpu9250;
    use panic_probe as _;
    use rp235x_hal as hal;

    use hal::clocks::init_clocks_and_plls;
    use hal::fugit::RateExtU32;
    use hal::gpio::bank0::{Gpio10, Gpio11, Gpio12, Gpio13};
    use hal::gpio::{FunctionSpi, Pins, PullDown};
    use hal::pac;
    use hal::pac::SPI1;
    use hal::sio::Sio;
    use hal::spi::{Enabled, FrameFormat};
    use hal::timer::CopyableTimer0;
    use hal::watchdog::Watchdog;
    use hal::{Clock, Spi, Timer};
    use uom::si::angular_velocity::radian_per_second;
    use uom::si::f32::AngularVelocity as UomAngVel;

// Place image definition so the ROM knows how to boot the firmware.
    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
const US_TO_SEC: f32 = 1.0 / 1_000_000.0;

    type Mosi = hal::gpio::Pin<Gpio11, FunctionSpi, PullDown>;
    type Miso = hal::gpio::Pin<Gpio12, FunctionSpi, PullDown>;
    type Sck = hal::gpio::Pin<Gpio10, FunctionSpi, PullDown>;
    type SpiPins = (Mosi, Miso, Sck);
    type SpiBus = Spi<Enabled, SPI1, SpiPins>;
    type DelayTimer = Timer<CopyableTimer0>;
    type CsPin = hal::gpio::Pin<Gpio13, hal::gpio::FunctionSio<hal::gpio::SioOutput>, PullDown>;
    type CsError = <CsPin as embedded_hal::digital::ErrorType>::Error;
    type SpiErr = <SpiBus as embedded_hal::spi::ErrorType>::Error;
    type MpuSpiError = mpu9250::SpiError<SpiErr, CsError>;
    type MpuError = mpu9250::Error<MpuSpiError>;

    type MpuMarg = Mpu9250<mpu9250::SpiDevice<SpiBus, CsPin>, mpu9250::Marg>;
    type MpuImu = Mpu9250<mpu9250::SpiDevice<SpiBus, CsPin>, mpu9250::Imu>;

    enum Mpu {
        Marg(MpuMarg),
        Imu(MpuImu),
    }

    impl Mpu {
        fn who_am_i(&mut self) -> Result<u8, MpuError> {
            match self {
                Mpu::Marg(m) => Ok(m.who_am_i()?),
                Mpu::Imu(m) => Ok(m.who_am_i()?),
            }
        }

        fn read_sample(&mut self) -> Result<ImuPayload, MpuError> {
            let (accel, gyro, temp) = match self {
                Mpu::Marg(m) => {
                    let data = m.all::<[f32; 3]>()?;
                    (data.accel, data.gyro, data.temp)
                }
                Mpu::Imu(m) => {
                    let data = m.all::<[f32; 3]>()?;
                    (data.accel, data.gyro, data.temp)
                }
            };
            Ok(ImuPayload::from_raw(accel, gyro, temp))
        }
    }

    fn init_mpu(spi: SpiBus, cs: CsPin, timer: &mut DelayTimer) -> Mpu {
        info!("Initializing MPU9250 over SPI1 (imu)...");
        let mut imu = Mpu9250::imu_default(spi, cs, timer).unwrap_or_else(|e| {
            warn!("imu init failed: {:?}", Debug2Format(&e));
            panic!();
        });

        let id = imu.who_am_i().unwrap_or(0);
        let mag_supported = matches!(id, 0x71 | 0x73);

        if mag_supported {
            info!("WHO_AM_I=0x{:02X} -> trying marg (mag enabled)", id);
            let (spi, cs) = imu.release();
            match Mpu9250::marg_default(spi, cs, timer) {
                Ok(dev) => {
                    info!("Using marg mode (gyro+accel+mag)");
                    Mpu::Marg(dev)
                }
                Err(e) => {
                    panic!("marg init failed: {:?}", Debug2Format(&e));
                }
            }
        } else {
            info!("WHO_AM_I=0x{:02X} (mag not supported) -> imu-only", id);
            Mpu::Imu(imu)
        }
    }

    #[entry]
    fn main() -> ! {
    let mut p = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        p.XOSC,
        p.CLOCKS,
        p.PLL_SYS,
        p.PLL_USB,
        &mut p.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer: DelayTimer = Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);

    let sio = Sio::new(p.SIO);
    let pins = Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    let sck: Sck = pins.gpio10.into_function();
    let mosi: Mosi = pins.gpio11.into_function();
    let miso: Miso = pins.gpio12.into_function();
    let mut cs: CsPin = pins.gpio13.into_push_pull_output();
    let _ = cs.set_high(); // idle high

    let spi: SpiBus = Spi::<_, _, _, 8>::new(p.SPI1, (mosi, miso, sck)).init(
        &mut p.RESETS,
        clocks.peripheral_clock.freq(),
        1_000_000u32.Hz(),
        FrameFormat::MotorolaSpi(MODE_0),
    );

    let mut mpu = init_mpu(spi, cs, &mut timer);

    match mpu.who_am_i() {
        Ok(id) => info!("WHO_AM_I=0x{:02X}", id),
        Err(e) => warn!("who_am_i error: {:?}", Debug2Format(&e)),
    }

    let gyro_bias = calibrate_gyro(&mut mpu, &mut timer);

    let mut ahrs = CuAhrs::new_filter();
    let mut last_ticks = timer.get_counter().ticks();

    info!("Starting AHRS loop (500 Hz raw read, 20 ms log)...");
    loop {
        let now = timer.get_counter().ticks();
        let dt_us = now.wrapping_sub(last_ticks);
        last_ticks = now;
        let _dt_s = (dt_us as f32) * US_TO_SEC;

        let sample = match mpu.read_sample() {
            Ok(mut payload) => {
                apply_bias(&mut payload, gyro_bias);
                payload
            }
            Err(e) => {
                warn!("imu read error: {:?}", Debug2Format(&e));
                timer.delay_ms(20u32);
                continue;
            }
        };

        // The AHRS task expects TOV in CuTime; we approximate with timer ticks.
        let pose = {
            // feed with a fake TOV containing elapsed nanoseconds
            let mut msg = CuMsg::new(Some(sample));
            msg.tov = CuDuration::from((now as u64) * 1_000).into();
            let mut output = CuMsg::new(None);
            let _ = ahrs.process(&RobotClock::default(), &msg, &mut output);
            output.payload().cloned().unwrap_or_default()
        };

        info!(
            "pose rad: roll={=f32} pitch={=f32} yaw={=f32}",
            pose.roll, pose.pitch, pose.yaw
        );

        timer.delay_ms(20u32);
    }
}

    fn apply_bias(payload: &mut ImuPayload, bias: [f32; 3]) {
        let bx = UomAngVel::new::<radian_per_second>(bias[0]);
        let by = UomAngVel::new::<radian_per_second>(bias[1]);
        let bz = UomAngVel::new::<radian_per_second>(bias[2]);
        payload.gyro_x -= bx;
        payload.gyro_y -= by;
        payload.gyro_z -= bz;
    }

    fn calibrate_gyro(mpu: &mut Mpu, timer: &mut DelayTimer) -> [f32; 3] {
        info!("=== Gyro bias calibration: keep the board still ===");
        const CAL_MS: u32 = 3_000;
        const SAMPLE_DELAY_MS: u32 = 10;
        let start = timer.get_counter().ticks();
        let mut sum = [0.0f32; 3];
        let mut count: u32 = 0;

        while timer
            .get_counter()
            .ticks()
            .wrapping_sub(start)
            .saturating_div(1_000)
            < (CAL_MS as u64)
        {
            match mpu.read_sample() {
                Ok(payload) => {
                    sum[0] += payload.gyro_x.value;
                    sum[1] += payload.gyro_y.value;
                    sum[2] += payload.gyro_z.value;
                    count += 1;
                }
                Err(e) => warn!("calib read error: {:?}", Debug2Format(&e)),
            }
            timer.delay_ms(SAMPLE_DELAY_MS);
        }

        if count == 0 {
            warn!("calibration produced no samples, defaulting to zero bias");
            return [0.0; 3];
        }

        let inv = 1.0 / (count as f32);
        [sum[0] * inv, sum[1] * inv, sum[2] * inv]
    }
}

#[cfg(not(all(feature = "rp2350-demo", target_arch = "arm")))]
fn main() {
    // Host/dummy entry to keep clippy happy without cross-building.
    println!("rp2350_ahrs example requires --features rp2350-demo and a thumbv8m target.");
}
