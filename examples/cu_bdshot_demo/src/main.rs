#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod tasks;

use cu29::prelude::*;

#[cfg(not(feature = "std"))]
use core::panic::PanicInfo;

#[cfg(not(feature = "std"))]
mod platform {
    use super::*;
    use alloc::sync::Arc;
    use bincode::error::EncodeError;
    use bincode::Encode;
    use embedded_alloc::LlffHeap as Heap;
    use panic_probe as _;
    use spin::Mutex;

    pub struct DemoSectionStorage;
    pub struct DemoLogger;

    impl SectionStorage for DemoSectionStorage {
        fn initialize<E: Encode>(&mut self, _header: &E) -> Result<usize, EncodeError> {
            Ok(0)
        }

        fn post_update_header<E: Encode>(&mut self, _header: &E) -> Result<usize, EncodeError> {
            Ok(0)
        }

        fn append<E: Encode>(&mut self, _entry: &E) -> Result<usize, EncodeError> {
            Ok(0)
        }

        fn flush(&mut self) -> CuResult<usize> {
            Ok(0)
        }
    }

    impl UnifiedLogWrite<DemoSectionStorage> for DemoLogger {
        fn add_section(
            &mut self,
            entry_type: UnifiedLogType,
            _requested_section_size: usize,
        ) -> CuResult<SectionHandle<DemoSectionStorage>> {
            let section_header = SectionHeader {
                magic: SECTION_MAGIC,
                block_size: 512,
                entry_type,
                offset_to_next_section: 0,
                used: 0,
            };
            let storage = DemoSectionStorage;
            SectionHandle::create(section_header, storage)
                .map_err(|_| CuError::from("section create"))
        }

        fn flush_section(&mut self, _section: &mut SectionHandle<DemoSectionStorage>) {}

        fn status(&self) -> UnifiedLogStatus {
            UnifiedLogStatus {
                total_used_space: 0,
                total_allocated_space: 0,
            }
        }
    }

    #[cfg_attr(not(feature = "std"), global_allocator)]
    static ALLOC: Heap = Heap::empty();

    pub fn init_heap() {
        use core::ptr::addr_of_mut;
        const HEAP_SIZE: usize = 128 * 1024;
        #[link_section = ".bss.heap"]
        static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
        unsafe {
            ALLOC.init(addr_of_mut!(HEAP) as usize, HEAP_SIZE);
        }
    }

    pub fn run_app() {
        init_heap();
        let logger = Arc::new(Mutex::new(DemoLogger));
        let clock = RobotClock::default();
        let mut app = BdshotDemoApp::new(clock, logger.clone()).expect("app");
        let _ = <BdshotDemoApp as CuApplication<DemoSectionStorage, DemoLogger>>::run(&mut app);
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct BdshotDemoApp {}

#[cfg(not(feature = "std"))]
#[no_mangle]
pub extern "C" fn main() {
    platform::run_app();
}

#[cfg(not(feature = "std"))]
#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {
        core::hint::spin_loop()
    }
}

#[cfg(feature = "std")]
fn main() {
    panic!("cu-bdshot-demo is intended for no_std RP2350 builds");
}
