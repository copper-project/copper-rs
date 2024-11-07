use bincode::de::{BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{BorrowDecode, Decode, Encode};
use cu29_soa_derive::Soa;
use derive_more::{Add, Deref, Div, From, Mul, Sub};
use uom::si::f32::Length;
use uom::si::f32::Luminance;
use uom::si::length::meter;
use uom::si::luminance::candela_per_square_meter;

#[derive(Default, PartialEq, Debug, Copy, Clone, Add, Deref, Sub, From, Mul, Div)]
pub struct LidarIntensity(Luminance);

/// Encode as f32 in candela per square meter
impl Encode for LidarIntensity {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.value, encoder)
    }
}

/// Decode as f32 in candela per square meter
impl Decode for LidarIntensity {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(LidarIntensity(Luminance::new::<candela_per_square_meter>(
            value,
        )))
    }
}

/// Decode as f32 in candela per square meter for borrowed decoding
impl<'de> BorrowDecode<'de> for LidarIntensity {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(LidarIntensity(Luminance::new::<candela_per_square_meter>(
            value,
        )))
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
    x: LidarLength,
    y: LidarLength,
    z: LidarLength,
    i: LidarIntensity,
}

impl LidarPayload {
    pub fn new(x: f32, y: f32, z: f32, i: f32) -> Self {
        Self {
            x: LidarLength(Length::new::<meter>(x)),
            y: LidarLength(Length::new::<meter>(y)),
            z: LidarLength(Length::new::<meter>(z)),
            i: LidarIntensity(Luminance::new::<candela_per_square_meter>(i)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lidar_payload() {
        let payload = LidarPayload::new(1.0, 2.0, 3.0, 0.0);
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
