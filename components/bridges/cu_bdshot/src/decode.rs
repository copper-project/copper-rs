use crate::messages::DShotTelemetry;

const INVALID_GCR_ENTRY: u8 = 0xFF;

// GCR is a redundant encoding to detect reading errors.
// It takes 5 bits to encode one quartet
const GCR_LUT: [u8; 32] = [
    INVALID_GCR_ENTRY,
    INVALID_GCR_ENTRY,
    INVALID_GCR_ENTRY,
    INVALID_GCR_ENTRY,
    INVALID_GCR_ENTRY,
    INVALID_GCR_ENTRY,
    INVALID_GCR_ENTRY,
    INVALID_GCR_ENTRY,
    INVALID_GCR_ENTRY,
    0x9,
    0xA,
    0xB,
    INVALID_GCR_ENTRY,
    0xD,
    0xE,
    0xF,
    INVALID_GCR_ENTRY,
    INVALID_GCR_ENTRY,
    0x2,
    0x3,
    INVALID_GCR_ENTRY,
    0x5,
    0x6,
    0x7,
    INVALID_GCR_ENTRY,
    0x0,
    0x8,
    0x1,
    INVALID_GCR_ENTRY,
    0x4,
    0xC,
    INVALID_GCR_ENTRY,
];

/// It is encoded on transitions to flip bits so 21 bits become 20
#[inline]
pub fn fold_gcr(raw21: u32) -> u32 {
    raw21 ^ (raw21 >> 1)
}

#[inline]
pub fn gcr_to_16bit(raw20: u32) -> Option<u16> {
    let s0 = GCR_LUT[((raw20 >> 15) & 0x1F) as usize];
    let s1 = GCR_LUT[((raw20 >> 10) & 0x1F) as usize];
    let s2 = GCR_LUT[((raw20 >> 5) & 0x1F) as usize];
    let s3 = GCR_LUT[(raw20 & 0x1F) as usize];
    if s0 == INVALID_GCR_ENTRY
        || s1 == INVALID_GCR_ENTRY
        || s2 == INVALID_GCR_ENTRY
        || s3 == INVALID_GCR_ENTRY
    {
        return None;
    }
    Some(((s0 as u16) << 12) | ((s1 as u16) << 8) | ((s2 as u16) << 4) | s3 as u16)
}

#[inline]
fn test_crc(data: u16) -> bool {
    let low = (data >> 4) & 0xF;
    let mid = (data >> 8) & 0xF;
    let high = (data >> 12) & 0xF;
    let crc = (!(low ^ mid ^ high)) & 0xF;
    crc == (data & 0xF)
}

#[inline]
pub fn extract_telemetry_payload_with_crc_test(data: u16) -> Option<u16> {
    if !test_crc(data) {
        return None;
    }
    Some(data >> 4)
}

#[inline]
pub fn decode_telemetry_packet(payload: u16) -> DShotTelemetry {
    if payload == 0x0FFF {
        return DShotTelemetry::Erpm(0);
    }

    let t = ((payload >> 8) & 0xF) as u8; // EDT type (if any)
    let value = (payload & 0xFF) as u8;

    match t {
        0x02 => DShotTelemetry::Temp(value),
        0x04 => DShotTelemetry::Voltage(value),
        0x08 => DShotTelemetry::Amps(value),
        0x0a => DShotTelemetry::Debug1(value),
        0x0c => DShotTelemetry::Debug2(value),
        0x0d => DShotTelemetry::Debug3(value),
        0x0e => DShotTelemetry::Event(value),
        _ => {
            // This is a regular eRPM response
            let base = (payload & 0x01FF) as u32; // 9-bit mantissa
            let exp = ((payload >> 9) & 0x07) as u32; // 3-bit exponent
            if base == 0 {
                return DShotTelemetry::EncodingError;
            }
            let period_us = base << exp;
            let Some(period_us) = core::num::NonZeroU32::new(period_us) else {
                return DShotTelemetry::EncodingError;
            };
            let period_us = period_us.get();
            let erpm = (60_000_000u32 + (period_us >> 1)) / period_us;
            DShotTelemetry::Erpm(erpm.min(u32::from(u16::MAX)) as u16)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        DShotTelemetry, decode_telemetry_packet, extract_telemetry_payload_with_crc_test, fold_gcr,
        gcr_to_16bit,
    };

    #[test]
    fn test_canonical_zero_rpm_telemetry() {
        // 01111_01111_01111_11001 (20 data bits), NRZI with 1 = toggle, 0 = hold, starting level = 0:
        // 21-bit wire level sequence: 001010010100101010001

        let raw_on_wire_21bit = 0b001010010100101010001;

        let grc_20bit_encoded = fold_gcr(raw_on_wire_21bit);
        assert_eq!(grc_20bit_encoded, 0b01111_01111_01111_11001); // this is the 0 RPM response packet

        let data = gcr_to_16bit(grc_20bit_encoded);
        assert_eq!(data, Some(0xFFF0));

        let payload = extract_telemetry_payload_with_crc_test(data.unwrap());
        assert_eq!(payload, Some(0xFFF));

        let decoded = decode_telemetry_packet(payload.unwrap());
        assert_eq!(decoded, DShotTelemetry::Erpm(0))
    }
}
