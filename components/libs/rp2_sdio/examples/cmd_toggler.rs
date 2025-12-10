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

    info!("PIO CMD toggler starting");

    let sio = Sio::new(p.SIO);
    let pins = hal::gpio::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    // Place GPIO35 (CMD) under PIO0 control.
    const SHIFT_GPIO_BASE: bool = true; // set to false to try raw GPIO numbering (no base shift)
    let pio_pin_id = if SHIFT_GPIO_BASE { 35 - 16 } else { 35 };
    let _cmd = pins.gpio35.into_push_pull_output().into_function::<FunctionPio0>();

    let (mut pio, sm0, _, _, _) = p.PIO0.split(&mut p.RESETS);
    // split() resets PIO; set GPIOBASE after.
    let pio0_block = unsafe { &*hal::pac::PIO0::ptr() };
    if SHIFT_GPIO_BASE {
        pio0_block.gpiobase().write(|w| w.gpiobase().set_bit());
    } else {
        pio0_block.gpiobase().write(|w| w.gpiobase().clear_bit());
    }
    info!(
        "PIO0 GPIOBASE set to {}, targeting PIO-local pin {} for GPIO35",
        pio0_block.gpiobase().read().gpiobase().bit(),
        pio_pin_id
    );

    // Simple PIO program: toggle CMD with long delays.
    let toggle_prog = pio_asm!(
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

    let installed = pio.install(&toggle_prog.program).unwrap();
    let (mut sm, _, _) = PIOBuilder::from_installed_program(installed)
        .set_pins(pio_pin_id, 1)
        .out_pins(pio_pin_id, 1)
        // Divider for faster ~1 kHz-ish toggle.
        .clock_divisor_fixed_point(60, 0)
        .build(sm0);

    sm.set_pindirs([(pio_pin_id, PinDir::Output)]);
    sm.start();

    loop {
        cortex_m::asm::wfi();
    }
}
