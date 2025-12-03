#![no_std]
#![no_main]

extern crate alloc;

#[allow(unused_imports)]
use defmt_rtt as _;
#[allow(unused_imports)]
use panic_probe as _;

use buddy_system_allocator::LockedHeap as Heap;
use rp235x_hal as hal;

#[unsafe(link_section = ".start_block")]
#[used]
static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[global_allocator]
static ALLOC: Heap<32> = Heap::empty();
