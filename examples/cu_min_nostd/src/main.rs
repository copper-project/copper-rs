#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), no_main)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod tasks;

use cu29::prelude::*;

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::boxed::Box;
    pub use alloc::sync::Arc;
    pub use alloc::vec;
    pub use bincode::config::standard;
    pub use bincode::encode_into_slice;
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
use core::ptr::addr_of_mut;

#[cfg(not(feature = "std"))]
#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {
        core::hint::spin_loop()
    }
}

#[cfg(not(feature = "std"))]
impl UnifiedLogWrite for MyEmbeddedLogger {
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> SectionHandle {
        use alloc::vec::Vec;
        // FIXME(gbin): this is just a hack to make it compile, it needs to be correctly implemented.
        // It minimalistically mimic the std implementation over the mmap file but with a memory leak.
        let buf: Vec<u8> = vec![0u8; requested_section_size];
        let boxed: Box<[u8]> = buf.into_boxed_slice();
        let slice_static: &'static mut [u8] = Box::leak(boxed);

        let section_header = SectionHeader {
            magic: SECTION_MAGIC,
            entry_type,
            section_size: requested_section_size as u32,
            filled_size: 0u32,
        };

        encode_into_slice(&section_header, slice_static, standard())
            .expect("Failed to encode section header");

        SectionHandle::create(section_header, slice_static)
    }

    fn flush_section(&mut self, _section: &mut SectionHandle) {
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
const HEAP_SIZE: usize = 128 * 1024;

#[cfg(not(feature = "std"))]
#[no_mangle]
pub extern "C" fn main() {
    // the no std version

    #[link_section = ".bss.heap"]
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    unsafe {
        ALLOC.init(addr_of_mut!(HEAP) as usize, HEAP_SIZE);
    }

    let clock = RobotClock::new();
    let writer = Arc::new(Mutex::new(MyEmbeddedLogger {}));
    let mut app = MinimalNoStdApp::new(clock, writer).unwrap();
    let _ = <MinimalNoStdApp as CuApplication<MyEmbeddedLogger>>::run(&mut app);
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
