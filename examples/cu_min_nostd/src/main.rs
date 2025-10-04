#!no_std

extern crate alloc;

pub mod tasks;

use alloc::sync::Arc;
use bincode::config::standard;
use bincode::encode_into_slice;
use cu29::prelude::*;
use spin::Mutex;

#[copper_runtime(config = "copperconfig.ron")]
struct MinimalNoStdApp {}

// This needs to be implemented depending on the embedded platform (emmc, sdcard, etc ...)
struct MyEmbeddedLogger {}

impl UnifiedLogWrite for MyEmbeddedLogger {
    fn add_section(
        &mut self,
        entry_type: UnifiedLogType,
        requested_section_size: usize,
    ) -> SectionHandle {
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

fn main() {
    let clock = RobotClock::new();
    let writer = Arc::new(Mutex::new(MyEmbeddedLogger {}));
    let mut app = MinimalNoStdApp::new(clock, writer).unwrap();
    let _ = <MinimalNoStdApp as CuApplication<MyEmbeddedLogger>>::run(&mut app);
}
