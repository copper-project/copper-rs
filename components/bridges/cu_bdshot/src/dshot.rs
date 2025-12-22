use crate::decode::{
    decode_telemetry_packet, extract_telemetry_payload_with_crc_test, fold_gcr, gcr_to_16bit,
};
use crate::messages::DShotTelemetry;

/// Compute 4-bit checksum over 3 nibbles, optionally inverted (Betaflight inverts when telemetry is set).
pub(crate) fn dshot_checksum(data: u16, invert: bool) -> u16 {
    let mut csum = 0;
    let mut csum_data = data;
    for _ in 0..3 {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    let csum = csum & 0xF;
    if invert { (!csum) & 0xF } else { csum }
}

/// Build a 16-bit DSHOT frame from value and telemetry flag.
pub(crate) fn dshot_frame(value: u16, telemetry: bool, invert_csum_on_telem: bool) -> u16 {
    let mut v = (value & 0x7ff) << 1;
    if telemetry {
        v |= 1;
    }
    // Betaflight inverts checksum whenever bidirectional telemetry is enabled,
    // regardless of the telemetry bit content.
    let csum = dshot_checksum(v, invert_csum_on_telem);
    (v << 4) | csum
}

/// Decode telemetry frame; returns 11-bit payload if checksum matches.
#[allow(dead_code)]
pub(crate) fn decode_telemetry(raw: u16, invert_csum_on_telem: bool) -> Option<u16> {
    let payload = raw >> 4;
    let csum = raw & 0xF;
    if dshot_checksum(payload, invert_csum_on_telem) == csum {
        Some(payload & 0x7FF)
    } else {
        None
    }
}

/// Slide a 16-bit window over 21 sampled bits and pick the first checksum-valid candidate.
#[allow(dead_code)]
pub(crate) fn try_decode_raw21(raw21: u32, invert_csum_on_telem: bool) -> Option<(u16, u8)> {
    for offset in 0..=5 {
        let candidate = ((raw21 >> (5 - offset)) & 0xFFFF) as u16;
        if decode_telemetry(candidate, invert_csum_on_telem).is_some() {
            return Some((candidate, offset));
        }
    }
    None
}

#[allow(dead_code)]
pub(crate) struct TelemetryDecode {
    pub raw20: u32,
    pub data: u16,
    pub payload: u16,
    pub decoded: DShotTelemetry,
}

pub(crate) fn decode_raw21(raw21: u32) -> Option<TelemetryDecode> {
    let raw20 = fold_gcr(raw21);
    let data = gcr_to_16bit(raw20)?;
    let payload = extract_telemetry_payload_with_crc_test(data)?;
    let decoded = decode_telemetry_packet(payload);
    Some(TelemetryDecode {
        raw20,
        data,
        payload,
        decoded,
    })
}

#[cfg(all(test, not(target_arch = "arm")))]
mod tests {
    use super::*;

    #[test]
    fn zero_frame_matches_betaflight_checksum_invert() {
        // Capture shows Betaflight idle frame as 0000_0000_0000_1111 (telemetry bit clear, checksum inverted).
        assert_eq!(dshot_frame(0, false, true), 0x000f);
        // With telemetry bit set, expect 0x001e.
        assert_eq!(dshot_frame(0, true, true), 0x001e);
    }

    #[test]
    fn checksum_round_trip() {
        for val in [0u16, 1, 7, 42, 0x3ff, 0x7ff] {
            let frame = dshot_frame(val, true, true);
            let payload = frame >> 4;
            let csum = frame & 0xf;
            assert_eq!(dshot_checksum(payload, true), csum);
        }
    }

    #[test]
    fn gcr_decode_known_zero_rpm() {
        // cu_bdshot canonical GCR 20-bit for 0 RPM.
        let gcr20 = 0b01111_01111_01111_11001u32;
        let data = gcr_to_16bit(gcr20).expect("valid GCR");
        assert_eq!(data, 0xFFF0);
        let payload = extract_telemetry_payload_with_crc_test(data).expect("valid CRC");
        assert_eq!(payload, 0x0FFF);
        assert_eq!(decode_telemetry_packet(payload), DShotTelemetry::Erpm(0));
    }
}
