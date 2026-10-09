#![cfg(feature = "std")]

use cu29_traits::CuError;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::fmt::{self, Write};

struct CountingAllocator;

thread_local! {
    static TRACKING: Cell<bool> = const { Cell::new(false) };
    static ALLOCATIONS: Cell<usize> = const { Cell::new(0) };
}

fn record_allocation() {
    let _ = TRACKING.try_with(|tracking| {
        if tracking.get() {
            let _ = ALLOCATIONS.try_with(|count| count.set(count.get() + 1));
        }
    });
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        record_allocation();
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        record_allocation();
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        record_allocation();
        unsafe { System.realloc(ptr, layout, size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

struct StackBuffer {
    bytes: [u8; 256],
    len: usize,
}

impl Write for StackBuffer {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let end = self.len + value.len();
        let destination = self.bytes.get_mut(self.len..end).ok_or(fmt::Error)?;
        destination.copy_from_slice(value.as_bytes());
        self.len = end;
        Ok(())
    }
}

#[test]
fn display_does_not_allocate_for_plain_or_caused_errors() {
    let cases = [
        (CuError::from("test error"), "test error\n   context:None"),
        (
            CuError::from("test error").add_cause("some context"),
            "test error\n   context:some context",
        ),
    ];
    for (error, expected) in cases {
        let mut output = StackBuffer {
            bytes: [0; 256],
            len: 0,
        };
        ALLOCATIONS.with(|count| count.set(0));
        TRACKING.with(|tracking| tracking.set(true));
        let result = write!(&mut output, "{error}");
        TRACKING.with(|tracking| tracking.set(false));
        let allocations = ALLOCATIONS.with(Cell::get);
        assert!(result.is_ok());
        assert_eq!(
            std::str::from_utf8(&output.bytes[..output.len]).unwrap(),
            expected
        );
        assert_eq!(allocations, 0, "formatting an existing CuError allocated");
    }
}
