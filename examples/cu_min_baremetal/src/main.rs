#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod tasks;

use cu29::prelude::*;

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::sync::Arc;
    pub use alloc::vec;
    pub use bincode::error::EncodeError;
    pub use bincode::Encode;
    pub use core::ptr::addr_of_mut;
    pub use spin::Mutex;
}

#[cfg(not(feature = "std"))]
use imp::*;

#[cfg(not(feature = "std"))]
use embedded_alloc::LlffHeap as Heap;

#[cfg(not(feature = "std"))]
#[global_allocator]
static ALLOC: Heap = Heap::empty();

#[copper_runtime(config = "copperconfig.ron")]
struct MinimalNoStdApp {}

// This needs to be implemented depending on the embedded platform (emmc, sdcard, etc ...)
#[cfg(not(feature = "std"))]
struct MyEmbeddedLogger {}

#[cfg(not(feature = "std"))]
use core::panic::PanicInfo;

#[cfg(not(feature = "std"))]
#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {
        core::hint::spin_loop()
    }
}

#[cfg(not(feature = "std"))]
struct MySectionStorage;

#[cfg(not(feature = "std"))]
impl SectionStorage for MySectionStorage {
    // Just mock the behavior for now.
    fn initialize<E: Encode>(&mut self, _header: &E) -> Result<usize, EncodeError> {
        Ok(80)
    }

    fn post_update_header<E: Encode>(&mut self, _header: &E) -> Result<usize, EncodeError> {
        Ok(80)
    }

    fn append<E: Encode>(&mut self, _entry: &E) -> Result<usize, EncodeError> {
        Ok(300)
    }

    fn flush(&mut self) -> CuResult<usize> {
        Ok(1000)
    }
}

#[cfg(not(feature = "std"))]
impl UnifiedLogWrite<MySectionStorage> for MyEmbeddedLogger {
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        _requested_section_size: usize,
    ) -> CuResult<SectionHandle<MySectionStorage>> {
        // Just mock the behavior for now.
        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            block_size: 512,
            entry_type,
            offset_to_next_section: 10000,
            used: 0,
            is_open: true,
        };

        let mut storage: MySectionStorage = MySectionStorage {};
        storage
            .initialize(&section_header)
            .map_err(|_| CuError::from("Error initializing storage"))?;
        Ok(SectionHandle::create(section_header, storage).unwrap())
    }

    fn flush_section(&mut self, _section: &mut SectionHandle<MySectionStorage>) {
        // no op for now
    }

    fn status(&self) -> UnifiedLogStatus {
        // no op for now
        UnifiedLogStatus {
            total_used_space: 0,
            total_allocated_space: 0,
        }
    }
}

#[cfg(not(feature = "std"))]
const HEAP_SIZE: usize = 128usize * 1024usize;

#[cfg(not(feature = "std"))]
#[no_mangle]
pub extern "C" fn main() {
    // the no std version

    #[link_section = ".bss.heap"]
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    unsafe {
        ALLOC.init(addr_of_mut!(HEAP) as usize, HEAP_SIZE);
    }

    // this is a kind of platform independent compilation test, we really don't have a RTC here
    let clock = RobotClock::new_with_rtc(
        {
            // fake just use the CPU clock as clock
            move || read_raw_counter()
        },
        {
            move |ns| {
                // fake busy wait
                for _ in 0..ns {
                    core::hint::spin_loop();
                }
            }
        },
    );
    let writer = Arc::new(Mutex::new(MyEmbeddedLogger {}));
    let mut app = MinimalNoStdApp::new(clock, writer).unwrap();
    let _ = <MinimalNoStdApp as CuApplication<MySectionStorage, MyEmbeddedLogger>>::run(&mut app);
}

#[cfg(feature = "std")]
fn main() {
    // the standard version
    use cu29_helpers::basic_copper_setup;
    use std::fs;
    use std::path::{Path, PathBuf};

    const SLAB_SIZE: Option<usize> = Some(4096 * 1024 * 1024);

    let logger_path = "logs/nostd.copper";
    if let Some(parent) = Path::new(logger_path).parent() {
        if !parent.exists() {
            fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }

    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, true, None)
        .expect("Failed to setup logger.");
    let mut application = MinimalNoStdAppBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create application.");

    let outcome = application.run();
    match outcome {
        Ok(_result) => {}
        Err(error) => {
            debug!("Application Ended: {}", error)
        }
    }
}
