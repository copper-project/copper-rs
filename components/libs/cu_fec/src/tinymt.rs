//! RFC 8682 TinyMT32 with the parameter set mandated by that RFC.
//!
//! This code was derived from IETF RFC 8682. Please reproduce this note if possible.

const MAT1: u32 = 0x8f70_11ee;
const MAT2: u32 = 0xfc78_ff1f;
const TMAT: u32 = 0x3793_fdff;
const MASK: u32 = 0x7fff_ffff;

pub(crate) struct TinyMt32 {
    status: [u32; 4],
}

impl TinyMt32 {
    pub(crate) fn new(seed: u32) -> Self {
        let mut generator = Self {
            status: [seed, MAT1, MAT2, TMAT],
        };
        for i in 1_u32..8 {
            let previous = generator.status[((i - 1) & 3) as usize];
            generator.status[(i & 3) as usize] ^=
                i.wrapping_add(1_812_433_253_u32.wrapping_mul(previous ^ (previous >> 30)));
        }
        for _ in 0..8 {
            generator.next_state();
        }
        generator
    }

    pub(crate) fn next_u32(&mut self) -> u32 {
        self.next_state();
        self.temper()
    }

    pub(crate) fn next_u4(&mut self) -> u8 {
        (self.next_u32() & 0x0f) as u8
    }

    pub(crate) fn next_u8(&mut self) -> u8 {
        (self.next_u32() & 0xff) as u8
    }

    fn next_state(&mut self) {
        let mut y = self.status[3];
        let mut x = (self.status[0] & MASK) ^ self.status[1] ^ self.status[2];
        x ^= x << 1;
        y ^= (y >> 1) ^ x;
        self.status[0] = self.status[1];
        self.status[1] = self.status[2];
        self.status[2] = x ^ (y << 10);
        self.status[3] = y;
        if y & 1 != 0 {
            self.status[1] ^= MAT1;
            self.status[2] ^= MAT2;
        }
    }

    fn temper(&self) -> u32 {
        let t1 = self.status[0].wrapping_add(self.status[2] >> 8);
        let mut t0 = self.status[3] ^ t1;
        if t1 & 1 != 0 {
            t0 ^= TMAT;
        }
        t0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_rfc_8682_seed_one_vector() {
        let expected = [
            2_545_341_989,
            981_918_433,
            3_715_302_833,
            2_387_538_352,
            3_591_001_365,
            3_820_442_102,
            2_114_400_566,
            2_196_103_051,
            2_783_359_912,
            764_534_509,
            643_179_475,
            1_822_416_315,
            881_558_334,
            4_207_026_366,
            3_690_273_640,
            3_240_535_687,
            2_921_447_122,
            3_984_931_427,
            4_092_394_160,
            44_209_675,
            2_188_315_343,
            2_908_663_843,
            1_834_519_336,
            3_774_670_961,
            3_019_990_707,
            4_065_554_902,
            1_239_765_502,
            4_035_716_197,
            3_412_127_188,
            552_822_483,
            161_364_450,
            353_727_785,
            140_085_994,
            149_132_008,
            2_547_770_827,
            4_064_042_525,
            4_078_297_538,
            2_057_335_507,
            622_384_752,
            2_041_665_899,
            2_193_913_817,
            1_080_849_512,
            33_160_901,
            662_956_935,
            642_999_063,
            3_384_709_977,
            1_723_175_122,
            3_866_752_252,
            521_822_317,
            2_292_524_454,
        ];
        let mut generator = TinyMt32::new(1);
        for value in expected {
            assert_eq!(generator.next_u32(), value);
        }
    }

    #[test]
    fn matches_rfc_8681_small_integer_vectors() {
        let expected_u8 = [37, 225, 177, 176, 21, 246, 54, 139, 168, 237];
        let expected_u4 = [5, 1, 1, 0, 5, 6, 6, 11, 8, 13];

        let mut bytes = TinyMt32::new(1);
        for value in expected_u8 {
            assert_eq!(bytes.next_u8(), value);
        }

        let mut nibbles = TinyMt32::new(1);
        for value in expected_u4 {
            assert_eq!(nibbles.next_u4(), value);
        }
    }
}
