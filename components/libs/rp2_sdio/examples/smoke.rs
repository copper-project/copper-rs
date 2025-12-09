#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use hal::dma::DMAExt;
use hal::gpio::bank0::{Gpio34, Gpio35, Gpio36, Gpio37, Gpio38, Gpio39};
use hal::gpio::{FunctionPio0, Pin, PullNone};
use hal::pac;
use hal::pio::PIOExt;
use hal::sio::Sio;
use hal::watchdog::Watchdog;
use panic_probe as _;
use rp235x_hal as hal;
use rp2_sdio::sdio::{Sdio4bit, SD_BLOCK_LEN};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    let mut p = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        12_000_000,
        p.XOSC,
        p.CLOCKS,
        p.PLL_SYS,
        p.PLL_USB,
        &mut p.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let sio = Sio::new(p.SIO);
    // PIO on RP2350B only reaches GPIO32+ when the GPIO base bit is set.
    // Shift the PIO window to 16-47 before handing PIO0 to the HAL.
    p.PIO0.gpiobase().write(|w| w.gpiobase().set_bit());
    info!("PIO0 GPIOBASE set to {}", p.PIO0.gpiobase().read().gpiobase().bit());

    let pins = hal::gpio::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);
    let (mut pio, sm0, sm1, _sm2, _sm3) = p.PIO0.split(&mut p.RESETS);
    let dma = p.DMA.split(&mut p.RESETS);

    // SDIO pin mapping: CLK 34, CMD 35, D0-3: 36-39.
    // PIO base was shifted to 16 above, so we subtract 16 to get the PIO-local pin numbers.
    // Disable all internal pulls; rely on the external 10k pull-ups on CMD/D0..D3 and leave CLK floating.
    let _clk: Pin<Gpio34, FunctionPio0, PullNone> = pins.gpio34.into_floating_input().into_function();
    let _cmd: Pin<Gpio35, FunctionPio0, PullNone> = pins.gpio35.into_floating_input().into_function();
    let _d0: Pin<Gpio36, FunctionPio0, PullNone> = pins.gpio36.into_floating_input().into_function();
    let _d1: Pin<Gpio37, FunctionPio0, PullNone> = pins.gpio37.into_floating_input().into_function();
    let _d2: Pin<Gpio38, FunctionPio0, PullNone> = pins.gpio38.into_floating_input().into_function();
    let _d3: Pin<Gpio39, FunctionPio0, PullNone> = pins.gpio39.into_floating_input().into_function();

    const GPIO_BASE: u8 = 16;
    let sd_clk = 34 - GPIO_BASE;
    let sd_cmd = 35 - GPIO_BASE;
    let sd_d0 = 36 - GPIO_BASE;

    let mut timer = hal::Timer::new_timer0(p.TIMER0, &mut p.RESETS, &clocks);

    info!("Init SDIO...");
    let mut sdio = Sdio4bit::new(&mut pio, dma.ch0, &mut timer, sm0, sm1, sd_clk, sd_cmd, sd_d0, 3);

    match sdio.init_card() {
        Ok(_) => info!("Card initialized"),
        Err(e) => {
            warn!("Card init failed: {}", defmt::Debug2Format(&e));
            loop {}
        }
    }

    let mut buf = [0u8; SD_BLOCK_LEN];
    match sdio.read_block(0, &mut buf) {
        Ok(_) => info!("Read block0 ok"),
        Err(e) => warn!("Read block0 failed: {}", defmt::Debug2Format(&e)),
    }
    match sdio.write_block(8, &buf) {
        Ok(_) => info!("Write block8 ok"),
        Err(e) => warn!("Write block8 failed: {}", defmt::Debug2Format(&e)),
    }

    loop {
        cortex_m::asm::wfi();
    }
}
