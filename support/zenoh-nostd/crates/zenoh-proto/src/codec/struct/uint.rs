use crate::*;

impl ZBodyLen for u8 {
    fn z_body_len(&self) -> usize {
        1
    }
}

impl ZBodyEncode for u8 {
    fn z_body_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        w.write_u8(*self)?;
        Ok(())
    }
}

impl<'a> ZBodyDecode<'a> for u8 {
    type Ctx = ();

    fn z_body_decode(
        r: &mut impl crate::ZReadable<'a>,
        _: (),
    ) -> core::result::Result<Self, crate::CodecError> {
        Ok(r.read_u8()?)
    }
}

crate::derive_zstruct_with_body!(u8);

const VLE_LEN_MAX: usize = vle_len(u64::MAX);

const fn vle_len(x: u64) -> usize {
    const B1: u64 = u64::MAX << 7;
    const B2: u64 = u64::MAX << (7 * 2);
    const B3: u64 = u64::MAX << (7 * 3);
    const B4: u64 = u64::MAX << (7 * 4);
    const B5: u64 = u64::MAX << (7 * 5);
    const B6: u64 = u64::MAX << (7 * 6);
    const B7: u64 = u64::MAX << (7 * 7);
    const B8: u64 = u64::MAX << (7 * 8);

    if (x & B1) == 0 {
        1
    } else if (x & B2) == 0 {
        2
    } else if (x & B3) == 0 {
        3
    } else if (x & B4) == 0 {
        4
    } else if (x & B5) == 0 {
        5
    } else if (x & B6) == 0 {
        6
    } else if (x & B7) == 0 {
        7
    } else if (x & B8) == 0 {
        8
    } else {
        9
    }
}

impl ZBodyLen for u64 {
    fn z_body_len(&self) -> usize {
        vle_len(*self)
    }
}

impl ZBodyEncode for u64 {
    fn z_body_encode(
        &self,
        w: &mut impl crate::ZWriteable,
    ) -> core::result::Result<(), crate::CodecError> {
        let mut x = *self;

        w.write_slot(VLE_LEN_MAX, |buffer: &mut [u8]| {
            let mut len = 0;

            while (x & !0x7f_u64) != 0 {
                unsafe {
                    *buffer.get_unchecked_mut(len) = (x as u8) | 0x80_u8;
                }

                len += 1;
                x >>= 7;
            }

            if len != VLE_LEN_MAX {
                unsafe {
                    *buffer.get_unchecked_mut(len) = x as u8;
                }
                len += 1;
            }

            len
        })?;

        Ok(())
    }
}

impl<'a> ZBodyDecode<'a> for u64 {
    type Ctx = ();

    fn z_body_decode(
        r: &mut impl crate::ZReadable<'a>,
        _: (),
    ) -> core::result::Result<Self, crate::CodecError> {
        let mut b = r.read_u8()?;

        let mut v = 0;
        let mut i = 0;

        while (b & 0x80_u8) != 0 && i != 7 * (VLE_LEN_MAX - 1) {
            v |= ((b & 0x7f_u8) as u64) << i;
            b = r.read_u8()?;
            i += 7;
        }

        v |= (b as u64) << i;

        Ok(v)
    }
}

crate::derive_zstruct_with_body!(u64);

macro_rules! zint {
    ($($ty:ty),*) => {
        $(
            impl ZBodyLen for $ty {
                fn z_body_len(&self) -> usize {
                    <u64 as ZBodyLen>::z_body_len(&(*self as u64))
                }
            }

            impl ZBodyEncode for $ty {
                fn z_body_encode(&self, w: &mut impl crate::ZWriteable) -> core::result::Result<(), crate::CodecError> {
                    <u64 as ZBodyEncode>::z_body_encode(&(*self as u64), w)
                }
            }

            impl<'a> ZBodyDecode<'a> for $ty {
                type Ctx = ();

                fn z_body_decode(r: &mut impl crate::ZReadable<'a>, _: ()) -> core::result::Result<Self, crate::CodecError> {
                    let v = <u64 as ZBodyDecode>::z_body_decode(r, ())?;
                    Ok(v as $ty)
                }
            }
        )*
    };
}

zint!(u16, u32, usize);
crate::derive_zstruct_with_body!(u16, u32, usize);
