#![cfg(feature = "derive")]

extern crate cu_bincode as bincode;
use bincode::{BorrowDecode, Decode, Encode, config, decode_from_slice, encode_into_slice};

// Deliberately implements neither Encode nor Decode nor Default.
#[derive(Debug, PartialEq)]
struct RuntimeState(u8);

fn restored_state() -> RuntimeState {
    RuntimeState(5)
}

#[derive(Debug, PartialEq, Encode, Decode)]
struct Record {
    id: u64,
    #[bincode(skip, default = "restored_state")]
    state: RuntimeState,
    value: u32,
    #[bincode(skip)]
    cache: u64,
}

#[derive(Debug, PartialEq, Encode, Decode)]
enum Event {
    Named {
        id: u8,
        #[bincode(skip, default = "restored_state")]
        state: RuntimeState,
        value: u16,
    },
    Tuple(u8, #[bincode(skip)] u32, u16),
}

#[derive(Debug, PartialEq, Encode, BorrowDecode)]
struct Borrowed<'a>(
    &'a str,
    #[bincode(skip, default = "restored_state")] RuntimeState,
);

#[test]
fn state_is_absent_and_defaults_are_restored() {
    let mut bytes = [0; 32];
    let len = encode_into_slice(
        Record {
            id: 3,
            state: RuntimeState(99),
            value: 7,
            cache: 111,
        },
        &mut bytes,
        config::standard(),
    )
    .unwrap();
    assert_eq!(&bytes[..len], &[3, 7]);
    let expected = Record {
        id: 3,
        state: restored_state(),
        value: 7,
        cache: 0,
    };
    assert_eq!(
        decode_from_slice::<Record, _>(&bytes[..len], config::standard()).unwrap(),
        (expected, len)
    );
    let (borrowed, used) =
        bincode::borrow_decode_from_slice::<Record, _>(&bytes[..len], config::standard()).unwrap();
    assert_eq!(borrowed.state, restored_state());
    assert_eq!(borrowed.cache, 0);
    assert_eq!(used, len);
}

#[test]
fn named_and_tuple_enum_fields() {
    let mut bytes = [0; 32];
    for (event, expected, wire) in [
        (
            Event::Named {
                id: 7,
                state: RuntimeState(99),
                value: 9,
            },
            Event::Named {
                id: 7,
                state: restored_state(),
                value: 9,
            },
            [0, 7, 9],
        ),
        (Event::Tuple(7, 99, 9), Event::Tuple(7, 0, 9), [1, 7, 9]),
    ] {
        let len = encode_into_slice(event, &mut bytes, config::standard()).unwrap();
        assert_eq!(&bytes[..len], wire);
        assert_eq!(
            decode_from_slice::<Event, _>(&bytes[..len], config::standard()).unwrap(),
            (expected, len)
        );
        assert_eq!(
            bincode::borrow_decode_from_slice::<Event, _>(&bytes[..len], config::standard())
                .unwrap()
                .1,
            len
        );
    }
}

#[test]
fn borrowed_tuple_struct() {
    let mut bytes = [0; 32];
    let len = encode_into_slice(
        Borrowed("hello", RuntimeState(99)),
        &mut bytes,
        config::standard(),
    )
    .unwrap();
    let (value, used) =
        bincode::borrow_decode_from_slice::<Borrowed<'_>, _>(&bytes[..len], config::standard())
            .unwrap();
    assert_eq!(value, Borrowed("hello", restored_state()));
    assert_eq!(used, len);
}

#[derive(Default, Debug, PartialEq)]
struct Cache(u8);

#[derive(Debug, PartialEq, Encode, Decode)]
struct GenericRecord<T> {
    id: u8,
    #[bincode(skip)]
    cache: T,
}

#[derive(Debug, PartialEq, Encode, Decode)]
enum GenericEvent<T> {
    Entry(u8, #[bincode(skip)] T),
}

#[derive(Debug, PartialEq, Encode, Decode)]
struct CustomGeneric<T: From<Cache>>(u8, #[bincode(skip, default = "make_cache")] T);

fn make_cache<T: From<Cache>>() -> T {
    Cache(9).into()
}

#[test]
fn skipped_generic_fields_need_no_codec_bounds() {
    let mut bytes = [0; 32];
    let len = encode_into_slice(
        GenericRecord {
            id: 3,
            cache: Cache(7),
        },
        &mut bytes,
        config::standard(),
    )
    .unwrap();
    assert_eq!(&bytes[..len], &[3]);
    assert_eq!(
        decode_from_slice::<GenericRecord<Cache>, _>(&bytes[..len], config::standard())
            .unwrap()
            .0
            .cache,
        Cache(0)
    );
    let len = encode_into_slice(
        GenericEvent::Entry(3, Cache(7)),
        &mut bytes,
        config::standard(),
    )
    .unwrap();
    assert_eq!(&bytes[..len], &[0, 3]);
    assert_eq!(
        bincode::borrow_decode_from_slice::<GenericEvent<Cache>, _>(
            &bytes[..len],
            config::standard()
        )
        .unwrap()
        .0,
        GenericEvent::Entry(3, Cache(0))
    );
}

#[test]
fn custom_default_generic() {
    let (value, used) =
        decode_from_slice::<CustomGeneric<Cache>, _>(&[3], config::standard()).unwrap();
    assert_eq!(value, CustomGeneric(3, Cache(9)));
    assert_eq!(used, 1);
}
