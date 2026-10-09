#![cfg(feature = "std")]

use cu29_runtime::monitoring::CountingAlloc;
use cu29_traits::CuError;
use std::alloc::System;
use std::fmt::{self, Write};

#[global_allocator]
static ALLOCATOR: CountingAlloc<System> = CountingAlloc::new(System);

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
        let before = ALLOCATOR.allocated();
        let result = write!(&mut output, "{error}");
        let allocated_bytes = ALLOCATOR.allocated() - before;
        assert!(result.is_ok());
        assert_eq!(
            std::str::from_utf8(&output.bytes[..output.len]).unwrap(),
            expected
        );
        assert_eq!(
            allocated_bytes, 0,
            "formatting an existing CuError allocated"
        );
    }
}
