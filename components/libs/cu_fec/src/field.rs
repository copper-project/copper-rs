//! Finite-field operations used by RFC 8681.

/// The finite field used for repair-symbol coefficients.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Field {
    /// Binary coefficients. Multiplication is logical AND and symbol addition is XOR.
    Gf2,
    /// Byte coefficients in GF(2^8), using the RFC 8681 polynomial `0x11d`.
    Gf256,
}

impl Field {
    /// Returns the RFC 8681 FEC Encoding ID: 9 for GF(2), 10 for GF(2^8).
    pub const fn fec_encoding_id(self) -> u8 {
        match self {
            Self::Gf2 => 9,
            Self::Gf256 => 10,
        }
    }

    pub(crate) fn mul(self, lhs: u8, rhs: u8) -> u8 {
        match self {
            Self::Gf2 => lhs & rhs,
            Self::Gf256 => gf256_mul(lhs, rhs),
        }
    }

    pub(crate) fn inv(self, value: u8) -> u8 {
        debug_assert_ne!(value, 0);
        match self {
            Self::Gf2 => 1,
            Self::Gf256 => gf256_pow(value, 254),
        }
    }
}

fn gf256_mul(mut lhs: u8, mut rhs: u8) -> u8 {
    let mut product = 0_u8;
    for _ in 0..8 {
        if rhs & 1 != 0 {
            product ^= lhs;
        }
        let carry = lhs & 0x80 != 0;
        lhs <<= 1;
        if carry {
            lhs ^= 0x1d;
        }
        rhs >>= 1;
    }
    product
}

fn gf256_pow(mut base: u8, mut exponent: u8) -> u8 {
    let mut result = 1_u8;
    while exponent != 0 {
        if exponent & 1 != 0 {
            result = gf256_mul(result, base);
        }
        base = gf256_mul(base, base);
        exponent >>= 1;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gf256_uses_the_rfc_polynomial() {
        assert_eq!(gf256_mul(0x80, 0x02), 0x1d);
        assert_eq!(gf256_mul(0x57, 0x83), 0x31);
    }

    #[test]
    fn every_nonzero_byte_has_an_inverse() {
        for value in 1..=u8::MAX {
            assert_eq!(gf256_mul(value, Field::Gf256.inv(value)), 1);
        }
    }
}
