//! Read-only SDMMC1 mailbox for a debugger on the MicoAir H743.
#![no_std]
#![no_main]

use core::ptr::{addr_of, addr_of_mut};
use cortex_m_rt::entry;
use stm32h7xx_hal::{gpio::Speed, pac, prelude::*, sdmmc::SdCard};

const BLOCK_BYTES: usize = 512;
const MAX_BLOCKS: usize = 128;
const READY: u32 = 0x43555231;

// The host resolves these symbols from the ELF it loaded in SRAM. Write READ_STATE last.
#[no_mangle]
static mut READ_STATE: u32 = 0;
#[no_mangle]
static mut READ_LBA: u32 = 0;
#[no_mangle]
static mut READ_COUNT: u32 = 0;
#[no_mangle]
static mut READ_DATA: [[u8; BLOCK_BYTES]; MAX_BLOCKS] = [[0; BLOCK_BYTES]; MAX_BLOCKS];
#[no_mangle]
static mut READ_CRC: u32 = 0;
#[no_mangle]
static mut CARD_BLOCKS: u32 = 0;
#[no_mangle]
static mut READER_READY: u32 = 0;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let vos = dp.PWR.constrain().freeze();
    let ccdr = dp
        .RCC
        .constrain()
        .sys_ck(400.MHz())
        .pll1_q_ck(100.MHz())
        .freeze(vos, &dp.SYSCFG);
    let c = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let d = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let pins = (
        c.pc12.into_alternate::<12>().speed(Speed::VeryHigh),
        d.pd2.into_alternate::<12>().speed(Speed::VeryHigh),
        c.pc8.into_alternate::<12>().speed(Speed::VeryHigh),
        c.pc9.into_alternate::<12>().speed(Speed::VeryHigh),
        c.pc10.into_alternate::<12>().speed(Speed::VeryHigh),
        c.pc11.into_alternate::<12>().speed(Speed::VeryHigh),
    );
    let mut sd: stm32h7xx_hal::sdmmc::Sdmmc<_, SdCard> =
        dp.SDMMC1.sdmmc(pins, ccdr.peripheral.SDMMC1, &ccdr.clocks);
    sd.init(25.MHz()).unwrap();
    let capacity = u32::try_from(sd.card().unwrap().size() / BLOCK_BYTES as u64).unwrap();
    // SAFETY: The debugger reads capacity and signature after SD initialization.
    unsafe {
        addr_of_mut!(CARD_BLOCKS).write_volatile(capacity);
        cortex_m::asm::dmb();
        addr_of_mut!(READER_READY).write_volatile(READY);
    }
    loop {
        // SAFETY: A single host owns the mailbox. While state is 1 the MCU owns the buffer.
        unsafe {
            if addr_of!(READ_STATE).read_volatile() != 1 {
                continue;
            }
            cortex_m::asm::dmb();
            let lba = addr_of!(READ_LBA).read_volatile();
            let count = addr_of!(READ_COUNT).read_volatile();
            let mut ok = (1..=MAX_BLOCKS as u32).contains(&count)
                && lba.checked_add(count).is_some_and(|end| end <= capacity);
            let mut crc = crc32fast::Hasher::new();
            if ok {
                for index in 0..count {
                    let block = &mut *addr_of_mut!(READ_DATA[index as usize]);
                    if sd.read_block(lba + index, block).is_err() {
                        ok = false;
                        break;
                    }
                    crc.update(block);
                }
            }
            addr_of_mut!(READ_CRC).write_volatile(crc.finalize());
            cortex_m::asm::dmb();
            addr_of_mut!(READ_STATE).write_volatile(if ok { 2 } else { 3 });
        }
    }
}
