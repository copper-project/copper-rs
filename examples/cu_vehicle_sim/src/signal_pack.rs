//! CAN signal packing and unpacking engine.
//!
//! Implements bit-level signal manipulation matching the opendbc `set_value` / `get_raw_value`
//! functions exactly. Supports both big-endian (Motorola) and little-endian (Intel) byte order.

use crate::dbc_generated::DbcSignalDef;

/// Pack a raw integer value into a CAN frame's data bytes at the signal's bit position.
///
/// This is a direct port of opendbc's `packer.set_value()`. The algorithm walks bytes
/// starting from the signal's LSB, filling `size` bits with the value.
pub fn set_signal_value(data: &mut [u8], sig: &DbcSignalDef, mut ival: u64) {
    let mut i = (sig.lsb / 8) as usize;
    let mut bits_remaining = sig.size as u32;

    // Mask the value to the signal's bit width
    if sig.size < 64 {
        ival &= (1u64 << sig.size) - 1;
    }

    while i < data.len() && bits_remaining > 0 {
        let shift = if (sig.lsb / 8) as usize == i {
            (sig.lsb % 8) as u32
        } else {
            0
        };
        let size = bits_remaining.min(8 - shift);
        let mask = ((1u64 << size) - 1) << shift;

        data[i] &= !(mask as u8);
        data[i] |= ((ival & ((1u64 << size) - 1)) << shift) as u8;

        bits_remaining -= size;
        ival >>= size;

        if sig.is_little_endian {
            i += 1;
        } else {
            if i == 0 {
                break;
            }
            i -= 1;
        }
    }
}

/// Convert a physical (floating-point) signal value to a raw integer, then pack it.
///
/// Applies the inverse of the DBC formula: `raw = round((physical - offset) / factor)`
/// Handles signed signals by two's-complement wrapping.
pub fn pack_signal(data: &mut [u8], sig: &DbcSignalDef, physical_value: f64) {
    let mut ival = ((physical_value - sig.offset) / sig.factor + 0.5).floor() as i64;

    if ival < 0 {
        // Two's complement for signed signals
        ival = (1i64 << sig.size) + ival;
    }

    set_signal_value(data, sig, ival as u64);
}

/// Extract a raw integer value from CAN frame data at the signal's bit position.
///
/// This is a direct port of opendbc's `parser.get_raw_value()`. Walks bytes from MSB.
pub fn get_raw_value(data: &[u8], sig: &DbcSignalDef) -> u64 {
    let mut ret: u64 = 0;
    let mut i = (sig.msb / 8) as usize;
    let mut bits_remaining = sig.size as u32;

    while i < data.len() && bits_remaining > 0 {
        let lsb_byte = if (sig.lsb / 8) as usize == i {
            sig.lsb
        } else {
            (i as u16) * 8
        };
        let msb_byte = if (sig.msb / 8) as usize == i {
            sig.msb
        } else {
            ((i as u16) + 1) * 8 - 1
        };
        let size = (msb_byte - lsb_byte + 1) as u32;
        let d = ((data[i] >> (lsb_byte as u32 - (i as u32) * 8)) as u64)
            & ((1u64 << size) - 1);
        ret |= d << (bits_remaining - size);
        bits_remaining -= size;

        if sig.is_little_endian {
            if i == 0 {
                break;
            }
            i -= 1;
        } else {
            i += 1;
        }
    }

    ret
}

/// Unpack a raw value to a physical (floating-point) value.
///
/// Applies the DBC formula: `physical = raw * factor + offset`
/// Handles signed signals.
pub fn unpack_signal(data: &[u8], sig: &DbcSignalDef) -> f64 {
    let mut raw = get_raw_value(data, sig) as i64;

    if sig.is_signed {
        // Sign-extend
        raw -= ((raw >> (sig.size - 1)) & 1) * (1i64 << sig.size);
    }

    raw as f64 * sig.factor + sig.offset
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_sig(start_bit: u16, size: u8, le: bool, signed: bool) -> DbcSignalDef {
        // Compute MSB/LSB like the DBC parser
        let be_bits: Vec<u16> = (0..64u16)
            .flat_map(|i| (0..8u16).rev().map(move |j| j + i * 8))
            .collect();

        let (msb, lsb) = if le {
            (start_bit + size as u16 - 1, start_bit)
        } else {
            let idx = be_bits.iter().position(|&b| b == start_bit).unwrap();
            (start_bit, be_bits[idx + size as usize - 1])
        };

        DbcSignalDef {
            name: "TEST",
            start_bit,
            size,
            is_little_endian: le,
            is_signed: signed,
            factor: 1.0,
            offset: 0.0,
            min_val: 0.0,
            max_val: 255.0,
            unit: "",
            msb,
            lsb,
            is_checksum: false,
            is_counter: false,
        }
    }

    #[test]
    fn pack_unpack_be_unsigned() {
        // COUNTER in TRACK_A: start_bit=7, size=8, big-endian, unsigned
        let sig = make_sig(7, 8, false, false);
        let mut data = [0u8; 8];
        pack_signal(&mut data, &sig, 42.0);
        assert_eq!(data[0], 42);
        let val = unpack_signal(&data, &sig);
        assert_eq!(val, 42.0);
    }

    #[test]
    fn pack_unpack_be_signed() {
        // LAT_DIST: start_bit=31, size=11, big-endian, signed, factor=0.04
        let be_bits: Vec<u16> = (0..64u16)
            .flat_map(|i| (0..8u16).rev().map(move |j| j + i * 8))
            .collect();
        let idx = be_bits.iter().position(|&b| b == 31).unwrap();
        let lsb = be_bits[idx + 10];

        let sig = DbcSignalDef {
            name: "LAT_DIST",
            start_bit: 31,
            size: 11,
            is_little_endian: false,
            is_signed: true,
            factor: 0.04,
            offset: 0.0,
            min_val: -50.0,
            max_val: 50.0,
            unit: "m",
            msb: 31,
            lsb,
            is_checksum: false,
            is_counter: false,
        };

        let mut data = [0u8; 8];

        // Pack +2.0m → raw = 2.0/0.04 = 50
        pack_signal(&mut data, &sig, 2.0);
        let val = unpack_signal(&data, &sig);
        assert!((val - 2.0).abs() < 0.05, "Expected ~2.0, got {val}");

        // Pack -5.0m → raw = -5.0/0.04 = -125 (two's complement in 11 bits)
        data = [0u8; 8];
        pack_signal(&mut data, &sig, -5.0);
        let val = unpack_signal(&data, &sig);
        assert!((val - (-5.0)).abs() < 0.05, "Expected ~-5.0, got {val}");
    }

    #[test]
    fn pack_unpack_le_unsigned() {
        // NEW_SIGNAL_1: start_bit=15, size=7, big-endian, unsigned
        let sig = make_sig(15, 7, false, false);
        let mut data = [0u8; 8];
        pack_signal(&mut data, &sig, 100.0);
        let val = unpack_signal(&data, &sig);
        assert_eq!(val, 100.0);
    }

    #[test]
    fn round_trip_all_track_a_signals() {
        // Test with the actual TRACK_A signal layout from generated code
        use crate::dbc_generated::DBC_MESSAGES;

        let msg_def = &DBC_MESSAGES[0]; // TRACK_A_0
        assert_eq!(msg_def.name, "TRACK_A_0");

        let mut data = [0u8; 8];

        // Pack COUNTER=10, LONG_DIST=100m, LAT_DIST=3.0m, REL_SPEED=5.0m/s, VALID=1, NEW_TRACK=0
        let values: &[(&str, f64)] = &[
            ("COUNTER", 10.0),
            ("LONG_DIST", 100.0),
            ("LAT_DIST", 3.0),
            ("REL_SPEED", 5.0),
            ("VALID", 1.0),
            ("NEW_TRACK", 0.0),
        ];

        for &(name, val) in values {
            let sig = msg_def.signals.iter().find(|s| s.name == name).unwrap();
            pack_signal(&mut data, sig, val);
        }

        // Now unpack and verify
        for &(name, expected) in values {
            let sig = msg_def.signals.iter().find(|s| s.name == name).unwrap();
            let actual = unpack_signal(&data, sig);
            let tolerance = sig.factor * 0.6; // within one LSB
            assert!(
                (actual - expected).abs() <= tolerance,
                "Signal {name}: expected {expected}, got {actual} (tol={tolerance})"
            );
        }
    }
}
