//! Toyota CAN checksum algorithm.
//!
//! Direct port of `toyota_checksum()` from opendbc/car/toyota/toyotacan.py:
//! ```python
//! def toyota_checksum(address, sig, d):
//!     s = len(d)
//!     addr = address
//!     while addr:
//!         s += addr & 0xFF
//!         addr >>= 8
//!     for i in range(len(d) - 1):
//!         s += d[i]
//!     return s & 0xFF
//! ```

/// Compute the Toyota CAN checksum for a message.
///
/// The checksum is: `(dlc + sum_of_address_bytes + sum_of_data_bytes_except_last) & 0xFF`
/// The checksum itself occupies the last byte of the message (bit position 63:56 in big-endian).
pub fn toyota_checksum(address: u32, data: &[u8]) -> u8 {
    let mut s: u32 = data.len() as u32;

    // Sum address bytes
    let mut addr = address;
    while addr > 0 {
        s += addr & 0xFF;
        addr >>= 8;
    }

    // Sum all data bytes except the last (which holds the checksum)
    for &byte in &data[..data.len() - 1] {
        s += byte as u32;
    }

    (s & 0xFF) as u8
}

/// Apply the Toyota checksum to a CAN frame's data buffer.
///
/// Computes the checksum and writes it to `data[dlc - 1]` (the last byte).
pub fn apply_toyota_checksum(address: u32, data: &mut [u8]) {
    let cksum = toyota_checksum(address, data);
    let last = data.len() - 1;
    data[last] = cksum;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checksum_basic() {
        // Verify against known Toyota checksum behavior
        let mut data = [0u8; 8];
        // Address 0x180 (384), empty data → checksum = (8 + 0x01 + 0x80) & 0xFF = 0x89
        apply_toyota_checksum(0x180, &mut data);
        assert_eq!(data[7], 0x89, "Empty frame checksum");
    }

    #[test]
    fn checksum_with_data() {
        let mut data = [10, 20, 30, 40, 50, 60, 70, 0];
        // sum = 8 (dlc) + 0x01 + 0x80 (addr bytes) + 10+20+30+40+50+60+70 (data[0..7])
        // sum = 8 + 1 + 128 + 280 = 417 = 0x1A1 → 0xA1
        apply_toyota_checksum(0x180, &mut data);
        assert_eq!(data[7], 0xA1);
    }

    #[test]
    fn checksum_idempotent() {
        // Applying twice should give the same result (checksum byte is excluded from sum)
        let mut data = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x00];
        apply_toyota_checksum(0x180, &mut data);
        let first = data[7];
        apply_toyota_checksum(0x180, &mut data);
        assert_eq!(data[7], first, "Checksum should be idempotent");
    }
}
