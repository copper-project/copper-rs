//! Startup serialization must work for external and recursive payloads without reflection or allocation.
#![cfg(all(feature = "decode-catalog", feature = "self-describing-logs"))]

use bincode::Encode;
use bincode::enc::write::SliceWriter;
use bincode::value_decode::ValueDecodeRef;
use cu_gnss_payloads::GnssFixSolution;
use cu29_value::catalog::ValueDecodeCatalog;
use cu29_value::catalog_header::ValueDecodeCatalogLayout;
use cu29_value::catalog_stream::{CatalogDescription, CatalogSlot, write_catalog};
use cu29_value::decode::ValueDecodeLimits;
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

thread_local! {
    static TRACK_ALLOCATIONS: Cell<bool> = const { Cell::new(false) };
    static ALLOCATIONS: Cell<usize> = const { Cell::new(0) };
}

struct CountingAllocator;
fn count_allocation() {
    TRACK_ALLOCATIONS.with(|active| {
        if active.get() {
            ALLOCATIONS.with(|count| count.set(count.get() + 1));
        }
    });
}
// SAFETY: All allocation operations delegate to the system allocator unchanged.
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        count_allocation();
        // SAFETY: The caller supplies a valid allocation layout.
        unsafe { System.alloc(layout) }
    }
    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        count_allocation();
        // SAFETY: The allocation and layout are forwarded unchanged.
        unsafe { System.realloc(pointer, layout, size) }
    }
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        // SAFETY: The allocation and layout are forwarded unchanged.
        unsafe { System.dealloc(pointer, layout) }
    }
}
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

#[derive(Encode)]
#[bincode(crate = "bincode")]
struct Recursive {
    value: u16,
    next: Option<Box<Recursive>>,
}
#[derive(Encode)]
#[bincode(crate = "bincode")]
struct Sample {
    gnss: GnssFixSolution,
    recursive: Recursive,
}

static CATALOG: CatalogDescription = CatalogDescription {
    mission: "default",
    config_ron: "(tasks: [], cnx: [])",
    layout: ValueDecodeCatalogLayout::Compact,
    slots: &[
        CatalogSlot {
            task_id: "source",
            msg_type: "Sample",
            payload: Some(ValueDecodeRef::of::<Sample>()),
        },
        CatalogSlot {
            task_id: "filter",
            msg_type: "Sample",
            payload: Some(ValueDecodeRef::of::<Sample>()),
        },
        CatalogSlot {
            task_id: "uncaptured",
            msg_type: "Opaque",
            payload: None,
        },
    ],
};

fn blob() -> Vec<u8> {
    let mut buffer = [0; 16384];
    let mut writer = SliceWriter::new(&mut buffer);
    write_catalog(&mut writer, &CATALOG).unwrap();
    let len = writer.bytes_written();
    buffer[..len].to_vec()
}

#[test]
fn streams_external_and_recursive_types_without_allocating() {
    let mut buffer = [0; 16384];
    let mut writer = SliceWriter::new(&mut buffer);
    TRACK_ALLOCATIONS.with(|active| active.set(true));
    let result = write_catalog(&mut writer, &CATALOG);
    TRACK_ALLOCATIONS.with(|active| active.set(false));
    result.unwrap();
    assert_eq!(ALLOCATIONS.with(Cell::get), 0);
    let len = writer.bytes_written();
    let catalog = ValueDecodeCatalog::from_blob(&buffer[..len]).unwrap();
    assert_eq!(catalog.slots[0].binding, catalog.slots[1].binding);
    assert_eq!(catalog.slots[2].binding, None);
    assert!(
        catalog
            .description
            .schemas
            .iter()
            .any(|schema| schema.type_path.ends_with("::GnssFixSolution"))
    );
    assert!(catalog.description.schemas.iter().any(|schema| {
        schema.quantity().is_some_and(|quantity| {
            quantity == cu29_value::QuantityMetadata::coherent(cu29_value::Quantity::Length)
        })
    }));
    let sample = Sample {
        gnss: GnssFixSolution::default(),
        recursive: Recursive {
            value: 7,
            next: Some(Box::new(Recursive {
                value: 9,
                next: None,
            })),
        },
    };
    let bytes = bincode::encode_to_vec(sample, bincode::config::standard()).unwrap();
    let (_, used) = catalog
        .description
        .decode_at(
            catalog.slots[0].binding.unwrap(),
            &bytes,
            bincode::config::standard(),
            ValueDecodeLimits::default(),
        )
        .unwrap();
    assert_eq!(used, bytes.len());
}

#[test]
fn rejects_corruption_truncation_and_wrong_lengths() {
    let original = blob();
    for index in [10, original.len() / 2, original.len() - 1] {
        let mut damaged = original.clone();
        damaged[index] ^= 0x40;
        assert!(ValueDecodeCatalog::from_blob(&damaged).is_err());
    }
    for end in [0, 9, 10, original.len() / 2, original.len() - 1] {
        assert!(ValueDecodeCatalog::from_blob(&original[..end]).is_err());
    }
}

#[test]
fn empty_catalog_remains_valid() {
    static EMPTY: CatalogDescription = CatalogDescription {
        mission: "default",
        config_ron: "()",
        layout: ValueDecodeCatalogLayout::Compact,
        slots: &[],
    };
    let mut buffer = [0; 4096];
    let mut writer = SliceWriter::new(&mut buffer);
    write_catalog(&mut writer, &EMPTY).unwrap();
    let len = writer.bytes_written();
    assert!(
        ValueDecodeCatalog::from_blob(&buffer[..len])
            .unwrap()
            .slots
            .is_empty()
    );
}
