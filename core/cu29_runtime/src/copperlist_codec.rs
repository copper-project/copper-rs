//! Lossless, independently decodable common-metadata encoding for generated CopperLists.

use crate::cutask::{CuMsg, CuMsgMetadata, CuMsgPayload};
use bincode::de::Decoder;
use bincode::de::read::Reader;
use bincode::enc::Encoder;
use bincode::enc::write::Writer;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use core::array;
use cu29_clock::{CuTime, CuTimeRange, PartialCuTimeRange, Tov};
use cu29_traits::{CuCompactString, CuMsgOrigin};

const PLANE_NONE: u8 = 0;
const PLANE_ALL: u8 = 1;
const PLANE_BITMAP: u8 = 2;

/// References to the common metadata of one generated CopperList slot.
#[doc(hidden)]
#[derive(Clone, Copy)]
pub struct CommonMetadataRef<'a> {
    tov: &'a Tov,
    metadata: &'a CuMsgMetadata,
}

impl<'a> CommonMetadataRef<'a> {
    pub const fn new(tov: &'a Tov, metadata: &'a CuMsgMetadata) -> Self {
        Self { tov, metadata }
    }
}

/// Decoded common metadata for one generated CopperList slot.
#[doc(hidden)]
#[derive(Debug, Default)]
pub struct DecodedCommonMetadataSlot {
    pub tov: Tov,
    pub metadata: CuMsgMetadata,
    pub original_payload_present: bool,
    pub captured_payload_present: bool,
}

/// Fixed-capacity decoded metadata used while generated payload fields are decoded.
#[doc(hidden)]
pub struct DecodedCommonMetadata<const N: usize> {
    slots: [DecodedCommonMetadataSlot; N],
}

impl<const N: usize> DecodedCommonMetadata<N> {
    pub fn take_slot(&mut self, index: usize) -> DecodedCommonMetadataSlot {
        core::mem::take(&mut self.slots[index])
    }
}

/// Reassembles one generated message after its payload has been decoded.
#[doc(hidden)]
pub fn restore_msg<T: CuMsgPayload>(
    payload: Option<T>,
    slot: DecodedCommonMetadataSlot,
) -> CuMsg<T> {
    CuMsg::from_parts(payload, slot.tov, slot.metadata)
}

fn write_byte<E: Encoder>(encoder: &mut E, byte: u8) -> Result<(), EncodeError> {
    encoder.writer().write(&[byte])
}

fn read_byte<D: Decoder>(decoder: &mut D) -> Result<u8, DecodeError> {
    let mut byte = [0u8; 1];
    decoder.reader().read(&mut byte)?;
    Ok(byte[0])
}

fn write_u64_le<E: Encoder>(encoder: &mut E, value: u64) -> Result<(), EncodeError> {
    encoder.writer().write(&value.to_le_bytes())
}

fn read_u64_le<D: Decoder>(decoder: &mut D) -> Result<u64, DecodeError> {
    let mut bytes = [0u8; 8];
    decoder.reader().read(&mut bytes)?;
    Ok(u64::from_le_bytes(bytes))
}

fn encode_uleb128<E: Encoder>(encoder: &mut E, mut value: u128) -> Result<(), EncodeError> {
    loop {
        let mut byte = (value & 0x7f) as u8;
        value >>= 7;
        if value != 0 {
            byte |= 0x80;
        }
        write_byte(encoder, byte)?;
        if value == 0 {
            return Ok(());
        }
    }
}

fn decode_uleb128<D: Decoder>(decoder: &mut D) -> Result<u128, DecodeError> {
    let mut value = 0u128;
    for byte_index in 0..19 {
        let shift = byte_index * 7;
        let byte = read_byte(decoder)?;
        let chunk = u128::from(byte & 0x7f);
        if byte_index == 18 && chunk > 0x03 {
            return Err(DecodeError::Other("CopperList ULEB128 overflow"));
        }
        value |= chunk << shift;
        if byte & 0x80 == 0 {
            return Ok(value);
        }
    }
    Err(DecodeError::Other("CopperList ULEB128 is too long"))
}

fn zigzag(delta: i128) -> u128 {
    if delta >= 0 {
        (delta as u128) << 1
    } else {
        ((-delta) as u128) * 2 - 1
    }
}

fn unzigzag(value: u128) -> Result<i128, DecodeError> {
    let magnitude = value >> 1;
    let magnitude = i128::try_from(magnitude)
        .map_err(|_| DecodeError::Other("CopperList timestamp delta overflow"))?;
    Ok(if value & 1 == 0 {
        magnitude
    } else {
        -magnitude - 1
    })
}

fn encode_timestamp<E: Encoder>(
    encoder: &mut E,
    value: CuTime,
    anchor: CuTime,
) -> Result<(), EncodeError> {
    let delta = i128::from(value.as_nanos()) - i128::from(anchor.as_nanos());
    encode_uleb128(encoder, zigzag(delta))
}

fn decode_timestamp<D: Decoder>(decoder: &mut D, anchor: CuTime) -> Result<CuTime, DecodeError> {
    let delta = unzigzag(decode_uleb128(decoder)?)?;
    let nanos = i128::from(anchor.as_nanos())
        .checked_add(delta)
        .and_then(|value| u64::try_from(value).ok())
        .filter(|value| *value <= CuTime::MAX.as_nanos())
        .ok_or(DecodeError::Other(
            "CopperList timestamp is outside CuTime range",
        ))?;
    Ok(CuTime(nanos))
}

fn encode_plane<const N: usize, E: Encoder>(
    flags: &[bool; N],
    encoder: &mut E,
) -> Result<(), EncodeError> {
    let present = flags.iter().filter(|flag| **flag).count();
    if present == 0 {
        return write_byte(encoder, PLANE_NONE);
    }
    if present == N {
        return write_byte(encoder, PLANE_ALL);
    }

    write_byte(encoder, PLANE_BITMAP)?;
    for chunk in flags.chunks(8) {
        let mut byte = 0u8;
        for (bit, present) in chunk.iter().enumerate() {
            if *present {
                byte |= 1 << bit;
            }
        }
        write_byte(encoder, byte)?;
    }
    Ok(())
}

fn decode_plane<const N: usize, D: Decoder>(decoder: &mut D) -> Result<[bool; N], DecodeError> {
    match read_byte(decoder)? {
        PLANE_NONE => Ok([false; N]),
        PLANE_ALL => Ok([true; N]),
        PLANE_BITMAP => {
            let mut flags = [false; N];
            for chunk_start in (0..N).step_by(8) {
                let byte = read_byte(decoder)?;
                let chunk_len = (N - chunk_start).min(8);
                for bit in 0..chunk_len {
                    flags[chunk_start + bit] = byte & (1 << bit) != 0;
                }
                if chunk_len < 8 && byte >> chunk_len != 0 {
                    return Err(DecodeError::Other(
                        "CopperList presence bitmap has nonzero padding",
                    ));
                }
            }
            Ok(flags)
        }
        _ => Err(DecodeError::Other("Invalid CopperList presence-plane mode")),
    }
}

fn first_timestamp(slots: &[CommonMetadataRef<'_>]) -> Option<CuTime> {
    for slot in slots {
        match slot.tov {
            Tov::Time(time) => return Some(*time),
            Tov::Range(range) => return Some(range.start),
            Tov::None => {}
        }
    }
    for slot in slots {
        if !slot.metadata.process_time.start.is_none() {
            return Some(slot.metadata.process_time.start.unwrap());
        }
        if !slot.metadata.process_time.end.is_none() {
            return Some(slot.metadata.process_time.end.unwrap());
        }
    }
    None
}

/// Encodes all common slot metadata before any payload bytes.
#[doc(hidden)]
pub fn encode_common_metadata<const N: usize, E: Encoder>(
    slots: &[CommonMetadataRef<'_>; N],
    original_payload_presence: &[bool; N],
    captured_payload_presence: &[bool; N],
    encoder: &mut E,
) -> Result<(), EncodeError> {
    let tov_presence: [bool; N] = array::from_fn(|index| !matches!(slots[index].tov, Tov::None));
    let tov_range: [bool; N] = array::from_fn(|index| matches!(slots[index].tov, Tov::Range(_)));
    let process_start: [bool; N] =
        array::from_fn(|index| !slots[index].metadata.process_time.start.is_none());
    let process_end: [bool; N] =
        array::from_fn(|index| !slots[index].metadata.process_time.end.is_none());
    let status: [bool; N] = array::from_fn(|index| !slots[index].metadata.status_txt.0.is_empty());
    let origin: [bool; N] = array::from_fn(|index| slots[index].metadata.origin.is_some());

    encode_plane(&tov_presence, encoder)?;
    encode_plane(&tov_range, encoder)?;
    encode_plane(&process_start, encoder)?;
    encode_plane(&process_end, encoder)?;
    encode_plane(&status, encoder)?;
    encode_plane(&origin, encoder)?;
    encode_plane(original_payload_presence, encoder)?;
    encode_plane(captured_payload_presence, encoder)?;

    if captured_payload_presence
        .iter()
        .zip(original_payload_presence.iter())
        .any(|(captured, original)| *captured && !*original)
    {
        return Err(EncodeError::Other(
            "Captured CopperList payload cannot be absent from the original view",
        ));
    }

    let timestamp_base = first_timestamp(slots);
    write_byte(encoder, u8::from(timestamp_base.is_some()))?;
    if let Some(base) = timestamp_base {
        write_u64_le(encoder, base.as_nanos())?;

        for slot in slots {
            match slot.tov {
                Tov::None => {}
                Tov::Time(time) => encode_timestamp(encoder, *time, base)?,
                Tov::Range(range) => {
                    encode_timestamp(encoder, range.start, base)?;
                    encode_timestamp(encoder, range.end, range.start)?;
                }
            }
        }
        for slot in slots {
            let process_time = slot.metadata.process_time;
            if !process_time.start.is_none() {
                encode_timestamp(encoder, process_time.start.unwrap(), base)?;
            }
            if !process_time.end.is_none() {
                let anchor = if process_time.start.is_none() {
                    base
                } else {
                    process_time.start.unwrap()
                };
                encode_timestamp(encoder, process_time.end.unwrap(), anchor)?;
            }
        }
    }

    for (index, slot) in slots.iter().enumerate() {
        if !status[index] {
            continue;
        }
        let backref = slots[..index]
            .iter()
            .position(|previous| previous.metadata.status_txt == slot.metadata.status_txt)
            .map_or(0, |previous| previous + 1);
        encode_uleb128(encoder, backref as u128)?;
        if backref == 0 {
            slot.metadata.status_txt.encode(encoder)?;
        }
    }

    for (index, slot) in slots.iter().enumerate() {
        let Some(origin) = slot.metadata.origin.as_ref() else {
            continue;
        };
        let backref = slots[..index]
            .iter()
            .position(|previous| previous.metadata.origin.as_ref() == Some(origin))
            .map_or(0, |previous| previous + 1);
        encode_uleb128(encoder, backref as u128)?;
        if backref == 0 {
            origin.encode(encoder)?;
        }
    }
    Ok(())
}

/// Decodes the common metadata block that precedes generated payload bytes.
#[doc(hidden)]
pub fn decode_common_metadata<const N: usize, D: Decoder<Context = ()>>(
    decoder: &mut D,
) -> Result<DecodedCommonMetadata<N>, DecodeError> {
    let tov_presence: [bool; N] = decode_plane(decoder)?;
    let tov_range: [bool; N] = decode_plane(decoder)?;
    let process_start: [bool; N] = decode_plane(decoder)?;
    let process_end: [bool; N] = decode_plane(decoder)?;
    let status: [bool; N] = decode_plane(decoder)?;
    let origin: [bool; N] = decode_plane(decoder)?;
    let original_payload_presence: [bool; N] = decode_plane(decoder)?;
    let captured_payload_presence: [bool; N] = decode_plane(decoder)?;

    if tov_range
        .iter()
        .zip(tov_presence.iter())
        .any(|(range, present)| *range && !*present)
    {
        return Err(DecodeError::Other(
            "CopperList TOV range marked without a present TOV",
        ));
    }
    if captured_payload_presence
        .iter()
        .zip(original_payload_presence.iter())
        .any(|(captured, original)| *captured && !*original)
    {
        return Err(DecodeError::Other(
            "Captured CopperList payload absent from original view",
        ));
    }

    let has_timestamps = tov_presence.iter().any(|present| *present)
        || process_start.iter().any(|present| *present)
        || process_end.iter().any(|present| *present);
    let base_present = match read_byte(decoder)? {
        0 => false,
        1 => true,
        _ => return Err(DecodeError::Other("Invalid CopperList timestamp-base tag")),
    };
    if has_timestamps != base_present {
        return Err(DecodeError::Other(
            "CopperList timestamp-base presence does not match timestamp planes",
        ));
    }
    let base = if base_present {
        Some(CuTime(read_u64_le(decoder)?))
    } else {
        None
    };
    if base.is_some_and(|base| base > CuTime::MAX) {
        return Err(DecodeError::Other(
            "CopperList timestamp base is outside CuTime range",
        ));
    }

    let mut slots: [DecodedCommonMetadataSlot; N] = array::from_fn(|_| Default::default());
    if let Some(base) = base {
        for index in 0..N {
            if !tov_presence[index] {
                continue;
            }
            let start = decode_timestamp(decoder, base)?;
            slots[index].tov = if tov_range[index] {
                Tov::Range(CuTimeRange {
                    start,
                    end: decode_timestamp(decoder, start)?,
                })
            } else {
                Tov::Time(start)
            };
        }
        for index in 0..N {
            let start = if process_start[index] {
                Some(decode_timestamp(decoder, base)?)
            } else {
                None
            };
            let end = if process_end[index] {
                Some(decode_timestamp(decoder, start.unwrap_or(base))?)
            } else {
                None
            };
            slots[index].metadata.process_time = PartialCuTimeRange {
                start: start.into(),
                end: end.into(),
            };
        }
    }

    for index in 0..N {
        if !status[index] {
            continue;
        }
        let backref = usize::try_from(decode_uleb128(decoder)?)
            .map_err(|_| DecodeError::Other("CopperList status backreference overflow"))?;
        slots[index].metadata.status_txt = if backref == 0 {
            CuCompactString::decode(decoder)?
        } else if backref <= index && status[backref - 1] {
            slots[backref - 1].metadata.status_txt.clone()
        } else {
            return Err(DecodeError::Other(
                "Invalid CopperList status backreference",
            ));
        };
    }

    for index in 0..N {
        if !origin[index] {
            continue;
        }
        let backref = usize::try_from(decode_uleb128(decoder)?)
            .map_err(|_| DecodeError::Other("CopperList origin backreference overflow"))?;
        slots[index].metadata.origin = if backref == 0 {
            Some(CuMsgOrigin::decode(decoder)?)
        } else if backref <= index && origin[backref - 1] {
            slots[backref - 1].metadata.origin.clone()
        } else {
            return Err(DecodeError::Other(
                "Invalid CopperList origin backreference",
            ));
        };
    }

    for index in 0..N {
        slots[index].original_payload_present = original_payload_presence[index];
        slots[index].captured_payload_present = captured_payload_presence[index];
    }
    Ok(DecodedCommonMetadata { slots })
}

/// Encodes one present payload without repeating its presence or common metadata.
#[doc(hidden)]
pub fn encode_payload<T: CuMsgPayload, E: Encoder>(
    payload: &T,
    encoder: &mut E,
) -> Result<(), EncodeError> {
    let encoded_start = cu29_traits::observed_encode_bytes();
    let handle_start = crate::monitoring::current_payload_handle_bytes();
    payload.encode(encoder)?;
    let encoded_bytes = cu29_traits::observed_encode_bytes().saturating_sub(encoded_start);
    let handle_bytes =
        crate::monitoring::current_payload_handle_bytes().saturating_sub(handle_start);
    crate::monitoring::record_current_slot_payload_io_stats(
        core::mem::size_of::<T>(),
        encoded_bytes,
        handle_bytes,
    );
    Ok(())
}

/// Decodes one payload whose presence was declared in the common metadata block.
#[doc(hidden)]
pub fn decode_payload<T: CuMsgPayload, D: Decoder<Context = ()>>(
    decoder: &mut D,
) -> Result<T, DecodeError> {
    T::decode(decoder)
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config::standard;
    use bincode::encode_to_vec;

    fn metadata(
        start: Option<u64>,
        end: Option<u64>,
        status: &str,
        origin: Option<CuMsgOrigin>,
    ) -> CuMsgMetadata {
        let mut metadata = CuMsgMetadata {
            process_time: PartialCuTimeRange {
                start: start.map(CuTime).into(),
                end: end.map(CuTime).into(),
            },
            origin,
            ..Default::default()
        };
        metadata.set_status(status);
        metadata
    }

    fn assert_metadata_eq(expected: &CuMsgMetadata, actual: &CuMsgMetadata) {
        let expected_start: Option<CuTime> = expected.process_time.start.into();
        let actual_start: Option<CuTime> = actual.process_time.start.into();
        let expected_end: Option<CuTime> = expected.process_time.end.into();
        let actual_end: Option<CuTime> = actual.process_time.end.into();
        assert_eq!(expected_start, actual_start);
        assert_eq!(expected_end, actual_end);
        assert_eq!(expected.status_txt, actual.status_txt);
        assert_eq!(expected.origin, actual.origin);
    }

    #[test]
    fn exact_round_trip_for_mixed_extreme_and_repeated_metadata() {
        let repeated_origin = CuMsgOrigin {
            subsystem_code: u16::MAX,
            instance_id: u32::MAX,
            cl_id: u64::MAX,
        };
        let metadata = [
            metadata(
                Some(u64::MAX - 1),
                Some(0),
                "same",
                Some(repeated_origin.clone()),
            ),
            metadata(None, Some(17), "", None),
            metadata(Some(9), Some(u64::MAX - 1), "same", Some(repeated_origin)),
            metadata(
                None,
                None,
                "different",
                Some(CuMsgOrigin {
                    subsystem_code: 7,
                    instance_id: 8,
                    cl_id: 9,
                }),
            ),
        ];
        let tovs = [
            Tov::Range(CuTimeRange {
                start: CuTime(u64::MAX - 1),
                end: CuTime(0),
            }),
            Tov::None,
            Tov::Time(CuTime(5)),
            Tov::Range(CuTimeRange {
                start: CuTime(99),
                end: CuTime(12),
            }),
        ];
        let refs = array::from_fn(|index| CommonMetadataRef::new(&tovs[index], &metadata[index]));
        let original = [true, false, true, true];
        let captured = [true, false, false, true];

        struct Encoded<'a> {
            refs: &'a [CommonMetadataRef<'a>; 4],
            original: &'a [bool; 4],
            captured: &'a [bool; 4],
        }
        impl Encode for Encoded<'_> {
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                encode_common_metadata(self.refs, self.original, self.captured, encoder)
            }
        }
        let bytes = encode_to_vec(
            Encoded {
                refs: &refs,
                original: &original,
                captured: &captured,
            },
            standard(),
        )
        .unwrap();
        let mut decoder = bincode::de::DecoderImpl::new(
            bincode::de::read::SliceReader::new(&bytes),
            standard(),
            (),
        );
        let mut decoded = decode_common_metadata::<4, _>(&mut decoder).unwrap();
        for index in 0..4 {
            let slot = decoded.take_slot(index);
            assert_eq!(tovs[index], slot.tov);
            assert_metadata_eq(&metadata[index], &slot.metadata);
            assert_eq!(original[index], slot.original_payload_present);
            assert_eq!(captured[index], slot.captured_payload_present);
        }
    }

    #[test]
    fn adaptive_planes_cover_none_all_sparse_and_dense() {
        for flags in [
            [false; 9],
            [true; 9],
            [true, false, false, false, false, false, false, false, false],
            [true, true, true, true, true, true, true, true, false],
        ] {
            struct Plane<'a>(&'a [bool; 9]);
            impl Encode for Plane<'_> {
                fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                    encode_plane(self.0, encoder)
                }
            }
            let bytes = encode_to_vec(Plane(&flags), standard()).unwrap();
            let (decoded, used): ([bool; 9], usize) = {
                let mut decoder = bincode::de::DecoderImpl::new(
                    bincode::de::read::SliceReader::new(&bytes),
                    standard(),
                    (),
                );
                (decode_plane(&mut decoder).unwrap(), bytes.len())
            };
            assert_eq!(flags, decoded);
            assert_eq!(
                used,
                if flags.iter().all(|flag| *flag) || flags.iter().all(|flag| !*flag) {
                    1
                } else {
                    3
                }
            );
        }
    }

    #[test]
    fn metadata_encoding_is_independently_decodable() {
        let metadata = [metadata(Some(100), Some(101), "ok", None)];
        let tovs = [Tov::Time(CuTime(99))];
        let refs = [CommonMetadataRef::new(&tovs[0], &metadata[0])];
        struct Encoded<'a>(&'a [CommonMetadataRef<'a>; 1]);
        impl Encode for Encoded<'_> {
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                encode_common_metadata(self.0, &[true], &[true], encoder)
            }
        }
        let bytes = encode_to_vec(Encoded(&refs), standard()).unwrap();
        let mut decoder = bincode::de::DecoderImpl::new(
            bincode::de::read::SliceReader::new(&bytes),
            standard(),
            (),
        );
        let decoded = decode_common_metadata::<1, _>(&mut decoder).unwrap();
        assert_eq!(decoded.slots[0].tov, tovs[0]);
    }
}
