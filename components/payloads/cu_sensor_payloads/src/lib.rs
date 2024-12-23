use std::ops::DerefMut;

use bincode::de::read::Reader;
use bincode::de::{BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{BorrowDecode, Decode, Encode};
use cu29_clock::CuTime;
use cu29_soa_derive::Soa;
use derive_more::{Add, Deref, Div, From, Mul, Sub};
use image::{ColorType, DynamicImage, ImageBuffer};
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
pub struct Distance(pub Length);

/// Encode it as a f32 in m
impl Encode for Distance {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.0.value, encoder)
    }
}

/// Decode it as a f32 in m
impl Decode for Distance {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(Distance(Length::new::<meter>(value)))
    }
}

/// Decode it as a f32 in m
impl<'de> BorrowDecode<'de> for Distance {
    fn borrow_decode<D: BorrowDecoder<'de>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let value: f32 = Decode::decode(decoder)?;
        Ok(Distance(Length::new::<meter>(value)))
    }
}

/// Standardized PointCloud.
/// note: the derive(Soa) will generate a PointCloudSoa struct that will store the data in a SoA format.
/// The Soa format is appropriate for early pipeline operations like changing their frame of reference.
/// important: The ToV of the points are not assumed to be sorted.
#[derive(Default, Clone, Encode, Decode, PartialEq, Debug, Soa)]
pub struct PointCloud {
    pub tov: CuTime, // Time of Validity, not sorted.
    pub x: Distance,
    pub y: Distance,
    pub z: Distance,
    pub i: Reflectivity,
    pub return_order: u8, // 0 for first return, 1 for second return, etc.
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

fn color_type_to_num(color_type: ColorType) -> u16 {
    match color_type {
        ColorType::L8 => 0,
        ColorType::La8 => 1,
        ColorType::Rgb8 => 2,
        ColorType::Rgba8 => 3,
        ColorType::L16 => 4,
        ColorType::La16 => 5,
        ColorType::Rgb16 => 6,
        ColorType::Rgba16 => 7,
        ColorType::Rgb32F => 8,
        ColorType::Rgba32F => 9,
        _ => unreachable!("color_type: {:?} is unsupported", color_type),
    }
}
fn color_type_from_num(num: u16) -> ColorType {
    match num {
        0 => ColorType::L8,
        1 => ColorType::La8,
        2 => ColorType::Rgb8,
        3 => ColorType::Rgba8,
        4 => ColorType::L16,
        5 => ColorType::La16,
        6 => ColorType::Rgb16,
        7 => ColorType::Rgba16,
        8 => ColorType::Rgb32F,
        9 => ColorType::Rgba32F,
        _ => unreachable!("color_id: {:?} is unsupported", num),
    }
}

#[derive(Default, Clone, PartialEq, Debug)]
pub struct CuImage(DynamicImage);

impl Deref for CuImage {
    type Target = DynamicImage;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for CuImage {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl CuImage {
    pub fn new(img: DynamicImage) -> Self {
        Self(img)
    }
}

impl Encode for CuImage {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        // println!("enc w: {} h: {}", self.width(), self.height());
        Encode::encode(&self.width(), encoder)?;
        // println!("1");
        Encode::encode(&self.height(), encoder)?;
        // println!("2");
        Encode::encode(&color_type_to_num(self.color()), encoder)?;
        // println!("3");
        Encode::encode(&self.as_bytes(), encoder)?;
        // println!("4");
        Ok(())
    }
}

impl Decode for CuImage {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let w: u32 = Decode::decode(decoder)?;
        let h: u32 = Decode::decode(decoder)?;
        let color_id: u16 = Decode::decode(decoder)?;
        let color_type = color_type_from_num(color_id);
        //TODO maybe without allocation?
        let pixels_count = (w * h) as usize;
        let image = match color_type {
            ColorType::Rgb8 => {
                let mut data: Vec<u8> = Vec::with_capacity(pixels_count);
                let buf = data.as_mut_slice();
                decoder.reader().read(buf)?;
                ImageBuffer::from_raw(w, h, data).map(DynamicImage::ImageRgb8)
            }
            ColorType::Rgba8 => {
                let mut data: Vec<u8> = Vec::with_capacity(pixels_count);
                let buf = data.as_mut_slice();
                decoder.reader().read(buf)?;
                ImageBuffer::from_raw(w, h, data).map(DynamicImage::ImageRgba8)
            }
            ColorType::L8 => {
                let mut data: Vec<u8> = Vec::with_capacity(pixels_count);
                let buf = data.as_mut_slice();
                decoder.reader().read(buf)?;
                ImageBuffer::from_raw(w, h, data).map(DynamicImage::ImageLuma8)
            }
            ColorType::La8 => {
                let mut data: Vec<u8> = Vec::with_capacity(pixels_count);
                let buf = data.as_mut_slice();
                decoder.reader().read(buf)?;
                ImageBuffer::from_raw(w, h, data).map(DynamicImage::ImageLumaA8)
            }
            ColorType::Rgb16 => {
                let mut data: Vec<u16> = Vec::with_capacity(pixels_count);
                let buf = data.as_mut_slice();
                decoder.reader().read(bytemuck::cast_slice_mut(buf))?;
                ImageBuffer::from_raw(w, h, data).map(DynamicImage::ImageRgb16)
            }
            ColorType::Rgba16 => {
                let mut data: Vec<u16> = Vec::with_capacity(pixels_count);
                let buf = data.as_mut_slice();
                decoder.reader().read(bytemuck::cast_slice_mut(buf))?;
                ImageBuffer::from_raw(w, h, data).map(DynamicImage::ImageRgba16)
            }
            ColorType::Rgb32F => {
                let mut data: Vec<f32> = Vec::with_capacity(pixels_count);
                let buf = data.as_mut_slice();
                decoder.reader().read(bytemuck::cast_slice_mut(buf))?;
                ImageBuffer::from_raw(w, h, data).map(DynamicImage::ImageRgb32F)
            }
            ColorType::Rgba32F => {
                let mut data: Vec<f32> = Vec::with_capacity(pixels_count);
                let buf = data.as_mut_slice();
                decoder.reader().read(bytemuck::cast_slice_mut(buf))?;
                ImageBuffer::from_raw(w, h, data).map(DynamicImage::ImageRgba32F)
            }
            ColorType::L16 => {
                let mut data: Vec<u16> = Vec::with_capacity(pixels_count);
                let buf = data.as_mut_slice();
                decoder.reader().read(bytemuck::cast_slice_mut(buf))?;
                ImageBuffer::from_raw(w, h, data).map(DynamicImage::ImageLuma16)
            }
            ColorType::La16 => {
                let mut data: Vec<u16> = Vec::with_capacity(pixels_count);
                let buf = data.as_mut_slice();
                decoder.reader().read(bytemuck::cast_slice_mut(buf))?;
                ImageBuffer::from_raw(w, h, data).map(DynamicImage::ImageLumaA16)
            }
            _ => unimplemented!(),
        };
        Ok(CuImage(image.unwrap()))
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
            bincode::encode_into_slice(&a, &mut encoded, bincode::config::standard()).unwrap();
        assert_eq!(length, 4);
    }
}
