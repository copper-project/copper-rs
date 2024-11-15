use bincode::de::{BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{BorrowDecode, Decode, Encode};
use cu29_clock::CuTime;
use cu29_soa_derive::Soa;
use derive_more::{Add, Deref, Div, From, Mul, Sub};
use uom::si::f32::{Length, Ratio};
use uom::si::length::meter;
use uom::si::ratio::percent;

#[derive(Default, PartialEq, Debug, Copy, Clone, Add, Deref, Sub, From, Mul, Div)]
pub struct Reflectivity(Ratio);

/// Encode as f32
impl Encode for Reflectivity {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.value, encoder)
    }
}

/// Decode as f32
impl Decode for Reflectivity {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(Reflectivity(Ratio::new::<percent>(value)))
    }
}

/// Decode as f32
impl<'de> BorrowDecode<'de> for Reflectivity {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(Reflectivity(Ratio::new::<percent>(value)))
    }
}

#[derive(Default, PartialEq, Debug, Copy, Clone, Add, Deref, Sub, From, Mul, Div)]
pub struct LidarLength(Length);

/// Encode it as a f32 in m
impl Encode for LidarLength {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.value, encoder)
    }
}

/// Decode it as a f32 in m
impl Decode for LidarLength {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(LidarLength(Length::new::<meter>(value)))
    }
}

/// Decode it as a f32 in m
impl<'de> BorrowDecode<'de> for LidarLength {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(LidarLength(Length::new::<meter>(value)))
    }
}

#[derive(Default, Clone, Encode, Decode, PartialEq, Debug, Soa)]
pub struct LidarPayload {
    tov: CuTime,
    x: LidarLength,
    y: LidarLength,
    z: LidarLength,
    i: Reflectivity,
}

impl LidarPayload {
    pub fn new(tov: CuTime, x: f32, y: f32, z: f32, i: f32) -> Self {
        Self {
            tov,
            x: LidarLength(Length::new::<meter>(x)),
            y: LidarLength(Length::new::<meter>(y)),
            z: LidarLength(Length::new::<meter>(z)),
            i: Reflectivity(Ratio::new::<percent>(i)),
        }
    }

    pub fn new_uom(tov: CuTime, x: Length, y: Length, z: Length, i: Ratio) -> Self {
        Self {
            tov,
            x: LidarLength(x),
            y: LidarLength(y),
            z: LidarLength(z),
            i: Reflectivity(i),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_clock::CuDuration;

    #[test]
    fn test_lidar_payload() {
        let payload = LidarPayload::new(CuDuration(1), 1.0, 2.0, 3.0, 0.0);
        assert_eq!(payload.x.0.value, 1.0);
        assert_eq!(payload.y.0.value, 2.0);
        assert_eq!(payload.z.0.value, 3.0);
    }

    #[test]
    fn test_length_add_sub() {
        let a = LidarLength(Length::new::<meter>(1.0));
        let b = LidarLength(Length::new::<meter>(2.0));
        let c = a + b;
        assert_eq!(c.value, 3.0);
        let d = c - a;
        assert_eq!(d.value, 2.0);
    }

    #[test]
    fn test_encoding_length() {
        let a = LidarLength(Length::new::<meter>(1.0));
        let mut encoded = vec![0u8; 1024]; // Reserve a buffer with sufficient capacity

        let length =
            bincode::encode_into_slice(&a, &mut encoded, bincode::config::standard()).unwrap();
        assert_eq!(length, 4);
    }
}
