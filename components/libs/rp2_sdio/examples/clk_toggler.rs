#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use hal::gpio::FunctionPio0;
use hal::pio::{PIOBuilder, PIOExt, PinDir};
use hal::sio::Sio;
use hal::watchdog::Watchdog;
use panic_probe as _;
use pio::pio_asm;
use rp235x_hal as hal;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    let mut p = hal::pac::Peripherals::take().unwrap();

    let mut watchdog = Watchdog::new(p.WATCHDOG);
    let _clocks = hal::clocks::init_clocks_and_plls(
        12_000_000,
        p.XOSC,
        p.CLOCKS,
        p.PLL_SYS,
        p.PLL_USB,
        &mut p.RESETS,
        &mut watchdog,
    )
    .unwrap();

    info!("PIO clk toggler starting");

    let sio = Sio::new(p.SIO);
    let pins = hal::gpio::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    // Place GPIO34 (SDIO CLK on FC3) under PIO0 control; drive strength via push-pull.
    const GPIO_BASE: u8 = 16;
    let pio_pin_id = 34 - GPIO_BASE;
    let _clk = pins.gpio34.into_push_pull_output().into_function::<FunctionPio0>();

    // Simple PIO program: drive pin high with a delay loop, then low with a delay loop, forever.
    let blink_prog = pio_asm!(
        "
         set pindirs, 1
         .wrap_target
           set pins, 1
           set x, 31
         high_delay:
           nop [31]
           jmp x-- high_delay
           set pins, 0
           set x, 31
         low_delay:
           nop [31]
           jmp x-- low_delay
         .wrap"
    );

    let (mut pio, sm0, _, _, _) = p.PIO0.split(&mut p.RESETS);
    // After split the PIO was reset, so set GPIOBASE here.
    let pio0_block = unsafe { &*hal::pac::PIO0::ptr() };
    pio0_block.gpiobase().write(|w| w.gpiobase().set_bit());
    info!("PIO0 GPIOBASE set to {}", pio0_block.gpiobase().read().gpiobase().bit());
    let installed = pio.install(&blink_prog.program).unwrap();

    let (mut sm, _, _) = PIOBuilder::from_installed_program(installed)
        .set_pins(pio_pin_id, 1)
        .out_pins(pio_pin_id, 1)
        // With a large divider each delay loop ~50ms -> ~10 Hz toggle.
        .clock_divisor_fixed_point(6200, 0)
        .build(sm0);

    sm.set_pindirs([(pio_pin_id, PinDir::Output)]);
    sm.start();

    // Keep the core alive; the PIO is doing the work.
    loop {
        cortex_m::asm::wfi();
    }
}
