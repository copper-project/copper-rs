use cu_bincode::{Decode, Encode, Uleb128, config, decode_from_slice, encode_into_slice};

fn check<T>(value: T)
where
    T: Copy + core::fmt::Debug + PartialEq,
    Uleb128<T>: Encode + Decode<()> + for<'de> cu_bincode::BorrowDecode<'de, ()>,
{
    let mut bytes = [0; 19];
    let len = encode_into_slice(Uleb128(value), &mut bytes, config::standard()).unwrap();
    let (decoded, used) =
        decode_from_slice::<Uleb128<T>, _>(&bytes[..len], config::standard()).unwrap();
    assert_eq!(decoded.0, value);
    assert_eq!(used, len);
    let (borrowed, used) =
        cu_bincode::borrow_decode_from_slice::<Uleb128<T>, _>(&bytes[..len], config::standard())
            .unwrap();
    assert_eq!(borrowed.0, value);
    assert_eq!(used, len);

    let fixed = config::standard()
        .with_fixed_int_encoding()
        .with_big_endian();
    let mut other = [0; 19];
    let other_len = encode_into_slice(Uleb128(value), &mut other, fixed).unwrap();
    assert_eq!(&other[..other_len], &bytes[..len]);
    assert_eq!(
        decode_from_slice::<Uleb128<T>, _>(&other[..other_len], fixed)
            .unwrap()
            .0
            .0,
        value
    );
}

#[test]
fn all_integer_widths_and_signed_extremes() {
    macro_rules! unsigned {
        ($($ty:ty),*) => {$ (
            for value in [0, 1, 127, 128, <$ty>::MAX] { check::<$ty>(value); }
        )*};
    }
    macro_rules! signed {
        ($($ty:ty),*) => {$ (
            for value in [<$ty>::MIN, -65, -64, -1, 0, 1, 63, 64, <$ty>::MAX] { check::<$ty>(value); }
        )*};
    }
    unsigned!(u8, u16, u32, u64, u128, usize);
    signed!(i8, i16, i32, i64, i128, isize);
    for shift in (7..128).step_by(7) {
        for value in [(1u128 << shift) - 1, 1u128 << shift, (1u128 << shift) + 1] {
            check(value);
        }
    }
}

#[test]
fn known_bytes_and_unwrapped_values_are_unchanged() {
    let mut bytes = [0; 32];
    let len = encode_into_slice(
        (300u32, Uleb128(624485u64), Uleb128(-150i128), 300u32),
        &mut bytes,
        config::standard(),
    )
    .unwrap();
    assert_eq!(
        &bytes[..len],
        &[251, 44, 1, 0xe5, 0x8e, 0x26, 0xab, 2, 251, 44, 1]
    );
}

#[test]
fn copper_timestamp_and_backreference_bytes_match() {
    // Independent reference for the encoding previously used by Copper.
    fn old_encode(mut value: u128) -> Vec<u8> {
        let mut out = Vec::new();
        loop {
            let mut byte = (value & 0x7f) as u8;
            value >>= 7;
            if value != 0 {
                byte |= 0x80;
            }
            out.push(byte);
            if value == 0 {
                return out;
            }
        }
    }
    for delta in [
        -i128::from(u64::MAX),
        -1_000_000,
        -1000,
        -65,
        -1,
        0,
        1,
        63,
        64,
        1000,
        1_000_000,
        i128::from(u64::MAX),
    ] {
        let zigzag = if delta < 0 {
            ((-delta) as u128) * 2 - 1
        } else {
            (delta as u128) << 1
        };
        let mut out = [0; 19];
        let len = encode_into_slice(Uleb128(delta), &mut out, config::standard()).unwrap();
        assert_eq!(&out[..len], old_encode(zigzag));
    }
    for backref in 0..=1024u128 {
        let mut out = [0; 19];
        let len = encode_into_slice(Uleb128(backref), &mut out, config::standard()).unwrap();
        assert_eq!(&out[..len], old_encode(backref));
    }
}

#[test]
fn malformed_input_and_limits() {
    use cu_bincode::error::DecodeError;
    assert!(matches!(
        decode_from_slice::<Uleb128<u64>, _>(&[0x80], config::standard()),
        Err(DecodeError::UnexpectedEnd { .. })
    ));
    assert!(decode_from_slice::<Uleb128<u8>, _>(&[0x80, 2], config::standard()).is_err());
    assert!(decode_from_slice::<Uleb128<i8>, _>(&[0xff, 2], config::standard()).is_err());
    assert!(decode_from_slice::<Uleb128<u128>, _>(&[0x80; 19], config::standard()).is_err());
    let mut overflow = [0xff; 19];
    overflow[18] = 4;
    assert!(decode_from_slice::<Uleb128<u128>, _>(&overflow, config::standard()).is_err());
    assert!(matches!(
        decode_from_slice::<Uleb128<u64>, _>(&[0], config::standard().with_limit::<7>()),
        Err(DecodeError::LimitExceeded)
    ));
    assert_eq!(
        decode_from_slice::<Uleb128<u64>, _>(&[0], config::standard().with_limit::<8>()).unwrap(),
        (Uleb128(0), 1)
    );
    assert!(encode_into_slice(Uleb128(128u16), &mut [0; 1], config::standard()).is_err());
}
