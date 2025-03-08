use bincode::de::{BorrowDecode, BorrowDecoder, Decode, Decoder};
use bincode::enc::{Encode, Encoder};
use bincode::error::{DecodeError, EncodeError};
use cu29_clock::CuTime;
use cu29_soa_derive::Soa;
use derive_more::{Add, Deref, Div, From, Mul, Sub};
use uom::si::f32::{Length, Ratio};
use uom::si::length::meter;
use uom::si::ratio::percent;

#[derive(Default, PartialEq, Debug, Copy, Clone, Add, Deref, Sub, From, Mul, Div)]
pub struct Reflectivity(Ratio);

impl From<f32> for Reflectivity {
    fn from(value: f32) -> Self {
        Self(Ratio::new::<percent>(value))
    }
}

/// Encode as f32
impl Encode for Reflectivity {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let Reflectivity(ratio) = self;
        Encode::encode(&ratio.value, encoder)
    }
}

/// Decode as f32
impl Decode<()> for Reflectivity {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(Reflectivity(Ratio::new::<percent>(value)))
    }
}

/// Decode as f32
impl<'de> BorrowDecode<'de, ()> for Reflectivity {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(Reflectivity(Ratio::new::<percent>(value)))
    }
}

#[derive(Default, PartialEq, Debug, Copy, Clone, Add, Deref, Sub, From, Mul, Div)]
pub struct Distance(pub Length);

/// Encode it as a f32 in m
impl Encode for Distance {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let Distance(length) = self;
        Encode::encode(&length.value, encoder)
    }
}

impl From<f32> for Distance {
    fn from(value: f32) -> Self {
        Self(Length::new::<meter>(value))
    }
}

/// Decode it as a f32 in m
impl Decode<()> for Distance {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(Distance(Length::new::<meter>(value)))
    }
}

/// Decode it as a f32 in m
impl<'de> BorrowDecode<'de, ()> for Distance {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(Distance(Length::new::<meter>(value)))
    }
}

/// Standardized PointCloud.
/// note: the derive(Soa) will generate a PointCloudSoa struct that will store the data in a SoA format.
/// The Soa format is appropriate for early pipeline operations like changing their frame of reference.
/// important: The ToV of the points are not assumed to be sorted.
#[derive(Default, Clone, PartialEq, Debug, Soa)]
pub struct PointCloud {
    pub tov: CuTime, // Time of Validity, not sorted.
    pub x: Distance,
    pub y: Distance,
    pub z: Distance,
    pub i: Reflectivity,
    pub return_order: u8, // 0 for first return, 1 for second return, etc.
}

impl Encode for PointCloud {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.tov.encode(encoder)?;
        self.x.encode(encoder)?;
        self.y.encode(encoder)?;
        self.z.encode(encoder)?;
        self.i.encode(encoder)?;
        self.return_order.encode(encoder)
    }
}

impl Decode<()> for PointCloud {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let tov = CuTime::decode(decoder)?;
        let x = Distance::decode(decoder)?;
        let y = Distance::decode(decoder)?;
        let z = Distance::decode(decoder)?;
        let i = Reflectivity::decode(decoder)?;
        let return_order = u8::decode(decoder)?;
        Ok(PointCloud {
            tov,
            x,
            y,
            z,
            i,
            return_order,
        })
    }
}

impl PointCloud {
    pub fn new(tov: CuTime, x: f32, y: f32, z: f32, i: f32, return_order: Option<u8>) -> Self {
        Self {
            tov,
            x: Distance(Length::new::<meter>(x)),
            y: Distance(Length::new::<meter>(y)),
            z: Distance(Length::new::<meter>(z)),
            i: Reflectivity(Ratio::new::<percent>(i)),
            return_order: return_order.unwrap_or(0),
        }
    }

    pub fn new_uom(
        tov: CuTime,
        x: Length,
        y: Length,
        z: Length,
        i: Ratio,
        return_order: Option<u8>,
    ) -> Self {
        Self {
            tov,
            x: Distance(x),
            y: Distance(y),
            z: Distance(z),
            i: Reflectivity(i),
            return_order: return_order.unwrap_or(0),
        }
    }
}

impl<const N: usize> PointCloudSoa<N> {
    /// Sort in place the point cloud so it can be ready for merge sorts for example
    pub fn sort(&mut self) {
        self.quicksort(0, N - 1);
    }

    /// Implementation of the sort
    fn quicksort(&mut self, low: usize, high: usize) {
        if low < high {
            let pivot_index = self.partition(low, high);
            if pivot_index > 0 {
                self.quicksort(low, pivot_index - 1);
            }
            self.quicksort(pivot_index + 1, high);
        }
    }

    /// Used by quicksort.
    fn partition(&mut self, low: usize, high: usize) -> usize {
        let pivot = self.tov[high];
        let mut i = low;
        for j in low..high {
            if self.tov[j] <= pivot {
                self.swap(i, j);
                i += 1;
            }
        }
        self.swap(i, high);
        i
    }

    /// swap the elements at index a and b
    pub fn swap(&mut self, a: usize, b: usize) {
        self.tov.swap(a, b);
        self.x.swap(a, b);
        self.y.swap(a, b);
        self.z.swap(a, b);
        self.i.swap(a, b);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_clock::CuDuration;

    #[test]
    fn test_point_payload() {
        let payload = PointCloud::new(CuDuration(1), 1.0, 2.0, 3.0, 0.0, None);
        assert_eq!(payload.x.0.value, 1.0);
        assert_eq!(payload.y.0.value, 2.0);
        assert_eq!(payload.z.0.value, 3.0);
    }

    #[test]
    fn test_length_add_sub() {
        let a = Distance(Length::new::<meter>(1.0));
        let b = Distance(Length::new::<meter>(2.0));
        let c = a + b;
        assert_eq!(c.value, 3.0);
        let d = c - a;
        assert_eq!(d.value, 2.0);
    }

    #[test]
    fn test_encoding_length() {
        let a = Distance(Length::new::<meter>(1.0));
        let mut encoded = vec![0u8; 1024]; // Reserve a buffer with sufficient capacity

        let length =
            bincode::encode_into_slice(a, &mut encoded, bincode::config::standard()).unwrap();
        assert_eq!(length, 4);
    }
}
