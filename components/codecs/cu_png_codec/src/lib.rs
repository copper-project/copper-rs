use bincode::de::{Decode, Decoder};
use bincode::enc::{Encode, Encoder};
use bincode::error::{DecodeError, EncodeError};
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use cu29::logcodec::CuLogCodec;
use cu29::prelude::{CuError, CuHandle, CuResult};
use image::ColorType;
use image::ImageDecoder;
use image::ImageEncoder;
use image::codecs::png::{CompressionType, FilterType, PngDecoder, PngEncoder};
use serde::{Deserialize, Serialize};
use std::io::Cursor;

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub enum CuPngCompression {
    Default,
    #[default]
    Fast,
    Best,
    Uncompressed,
    Level(u8),
}

impl CuPngCompression {
    fn into_image(self) -> CuResult<CompressionType> {
        match self {
            Self::Default => Ok(CompressionType::Default),
            Self::Fast => Ok(CompressionType::Fast),
            Self::Best => Ok(CompressionType::Best),
            Self::Uncompressed => Ok(CompressionType::Uncompressed),
            Self::Level(level @ 1..=9) => Ok(CompressionType::Level(level)),
            Self::Level(level) => Err(CuError::from(format!(
                "PNG compression level must be in 1..=9, got {level}"
            ))),
        }
    }
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub enum CuPngFilter {
    NoFilter,
    Sub,
    Up,
    Avg,
    Paeth,
    #[default]
    Adaptive,
}

impl From<CuPngFilter> for FilterType {
    fn from(value: CuPngFilter) -> Self {
        match value {
            CuPngFilter::NoFilter => FilterType::NoFilter,
            CuPngFilter::Sub => FilterType::Sub,
            CuPngFilter::Up => FilterType::Up,
            CuPngFilter::Avg => FilterType::Avg,
            CuPngFilter::Paeth => FilterType::Paeth,
            CuPngFilter::Adaptive => FilterType::Adaptive,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CuPngCodecConfig {
    #[serde(default)]
    pub compression: CuPngCompression,
    #[serde(default)]
    pub filter: CuPngFilter,
}

#[derive(Debug)]
pub struct CuPngCodec {
    compression: CompressionType,
    filter: FilterType,
    scratch: Vec<u8>,
}

impl CuPngCodec {
    fn encode_error(message: impl Into<String>) -> EncodeError {
        EncodeError::OtherString(message.into())
    }

    fn decode_error(message: impl Into<String>) -> DecodeError {
        DecodeError::OtherString(message.into())
    }

    fn expected_rgb_stride(format: CuImageBufferFormat) -> Result<u32, String> {
        format
            .width
            .checked_mul(3)
            .ok_or_else(|| "PNG codec image stride overflow".to_string())
    }
}

impl CuLogCodec<CuImage<Vec<u8>>> for CuPngCodec {
    type Config = CuPngCodecConfig;

    fn new(config: Self::Config) -> CuResult<Self> {
        Ok(Self {
            compression: config.compression.into_image()?,
            filter: config.filter.into(),
            scratch: Vec::new(),
        })
    }

    fn encode_payload<E: Encoder>(
        &mut self,
        payload: &CuImage<Vec<u8>>,
        encoder: &mut E,
    ) -> Result<(), EncodeError> {
        let expected_stride =
            Self::expected_rgb_stride(payload.format).map_err(Self::encode_error)?;
        if payload.format.pixel_format != *b"RGB3" {
            return Err(Self::encode_error(format!(
                "PNG codec only supports RGB3 images, got {:?}",
                payload.format.pixel_format
            )));
        }
        if payload.format.stride != expected_stride {
            return Err(Self::encode_error(format!(
                "PNG codec requires tightly packed RGB images (expected stride {}, got {})",
                expected_stride, payload.format.stride
            )));
        }

        let byte_size = payload.format.byte_size();
        self.scratch.clear();
        payload.buffer_handle.with_inner(|inner| {
            let image_bytes: &[u8] = inner;
            if image_bytes.len() < byte_size {
                return Err(Self::encode_error(format!(
                    "PNG codec expected at least {} image bytes, got {}",
                    byte_size,
                    image_bytes.len()
                )));
            }

            PngEncoder::new_with_quality(&mut self.scratch, self.compression, self.filter)
                .write_image(
                    &image_bytes[..byte_size],
                    payload.format.width,
                    payload.format.height,
                    ColorType::Rgb8.into(),
                )
                .map_err(|err| Self::encode_error(err.to_string()))
        })?;

        payload.seq.encode(encoder)?;
        self.scratch.encode(encoder)?;
        Ok(())
    }

    fn decode_payload<D: Decoder<Context = ()>>(
        &mut self,
        decoder: &mut D,
    ) -> Result<CuImage<Vec<u8>>, DecodeError> {
        let seq: u64 = Decode::decode(decoder)?;
        let png_bytes: Vec<u8> = Decode::decode(decoder)?;

        let png = PngDecoder::new(Cursor::new(&png_bytes))
            .map_err(|err| Self::decode_error(err.to_string()))?;
        let color_type = png.color_type();
        if color_type != ColorType::Rgb8.into() {
            return Err(Self::decode_error(format!(
                "PNG codec expected RGB8 decode output, got {color_type:?}"
            )));
        }

        let (width, height) = png.dimensions();
        let mut buffer = vec![0u8; png.total_bytes() as usize];
        png.read_image(&mut buffer)
            .map_err(|err| Self::decode_error(err.to_string()))?;

        let stride = width
            .checked_mul(3)
            .ok_or_else(|| Self::decode_error("PNG codec image stride overflow"))?;
        Ok(CuImage {
            seq,
            format: CuImageBufferFormat {
                width,
                height,
                stride,
                pixel_format: *b"RGB3",
            },
            buffer_handle: CuHandle::new_detached(buffer),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::config::standard;
    use bincode::{decode_from_slice, encode_to_vec};
    use cu29::prelude::CuMsg;

    struct EncodedWithCodec<'a> {
        image: &'a CuImage<Vec<u8>>,
        codec: std::cell::RefCell<CuPngCodec>,
    }

    impl Encode for EncodedWithCodec<'_> {
        fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
            cu29::logcodec::encode_msg_with_codec(
                &CuMsg::new(Some(self.image.clone())),
                &mut *self.codec.borrow_mut(),
                encoder,
            )
        }
    }

    struct DecodedWithCodec(CuImage<Vec<u8>>);

    impl Decode<()> for DecodedWithCodec {
        fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
            let mut codec = CuPngCodec::new(CuPngCodecConfig::default())
                .map_err(|err| DecodeError::OtherString(err.to_string()))?;
            let msg = cu29::logcodec::decode_msg_with_codec::<CuImage<Vec<u8>>, _, _>(
                decoder, &mut codec,
            )?;
            Ok(Self(msg.payload().cloned().ok_or_else(|| {
                DecodeError::OtherString("missing image payload".to_string())
            })?))
        }
    }

    fn sample_image() -> CuImage<Vec<u8>> {
        let width = 16;
        let height = 12;
        let stride = width * 3;
        let mut buffer = vec![0u8; (stride * height) as usize];
        for y in 0..height as usize {
            for x in 0..width as usize {
                let idx = y * stride as usize + x * 3;
                buffer[idx] = x as u8;
                buffer[idx + 1] = y as u8;
                buffer[idx + 2] = (x + y) as u8;
            }
        }
        CuImage {
            seq: 7,
            format: CuImageBufferFormat {
                width,
                height,
                stride,
                pixel_format: *b"RGB3",
            },
            buffer_handle: CuHandle::new_detached(buffer),
        }
    }

    #[test]
    fn png_codec_validates_compression_level() {
        let err = CuPngCodec::new(CuPngCodecConfig {
            compression: CuPngCompression::Level(10),
            filter: CuPngFilter::Adaptive,
        })
        .expect_err("expected error");
        assert!(err.to_string().contains("1..=9"));
    }

    #[test]
    fn png_codec_roundtrip_preserves_pixels() {
        let original = sample_image();
        let encoded = encode_to_vec(
            EncodedWithCodec {
                image: &original,
                codec: std::cell::RefCell::new(
                    CuPngCodec::new(CuPngCodecConfig::default()).expect("codec"),
                ),
            },
            standard(),
        )
        .expect("encode");

        let (decoded, consumed): (DecodedWithCodec, usize) =
            decode_from_slice(&encoded, standard()).expect("decode");
        assert_eq!(consumed, encoded.len());
        assert_eq!(decoded.0.seq, original.seq);
        assert_eq!(decoded.0.format.width, original.format.width);
        assert_eq!(decoded.0.format.height, original.format.height);
        assert_eq!(decoded.0.format.stride, original.format.stride);
        assert_eq!(decoded.0.format.pixel_format, original.format.pixel_format);
        let original_bytes = original.buffer_handle.with_inner(|inner| {
            let bytes: &[u8] = inner;
            bytes.to_vec()
        });
        let decoded_bytes = decoded.0.buffer_handle.with_inner(|inner| {
            let bytes: &[u8] = inner;
            bytes.to_vec()
        });
        assert_eq!(decoded_bytes, original_bytes);
    }
}
