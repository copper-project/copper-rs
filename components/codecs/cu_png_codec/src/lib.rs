use bincode::de::{Decode, Decoder};
use bincode::enc::{Encode, Encoder};
use bincode::error::{DecodeError, EncodeError};
use cu_sensor_payloads::{CuImage, CuImageBufferFormat};
use cu29::logcodec::CuLogCodec;
use cu29::prelude::{CuError, CuHandle, CuResult};
use png::{
    BitDepth, ColorType, Compression, Decoder as PngDecoder, DecodingError as PngDecodingError,
    DeflateCompression, Encoder as PngEncoder, Filter,
};
use serde::{Deserialize, Serialize};
use std::io::{self, Cursor};

#[derive(Debug, Clone, Copy)]
enum PngCompressionSetting {
    Simple(Compression),
    DeflateLevel(u8),
}

impl PngCompressionSetting {
    fn apply<W: std::io::Write>(self, encoder: &mut PngEncoder<'_, W>) {
        match self {
            Self::Simple(compression) => encoder.set_compression(compression),
            Self::DeflateLevel(level) => {
                encoder.set_compression(Compression::Fast);
                encoder.set_deflate_compression(DeflateCompression::Level(level));
            }
        }
    }
}

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
    fn into_png(self) -> CuResult<PngCompressionSetting> {
        match self {
            Self::Default => Ok(PngCompressionSetting::Simple(Compression::Balanced)),
            Self::Fast => Ok(PngCompressionSetting::Simple(Compression::Fast)),
            Self::Best => Ok(PngCompressionSetting::Simple(Compression::High)),
            Self::Uncompressed => Ok(PngCompressionSetting::Simple(Compression::NoCompression)),
            Self::Level(level @ 1..=9) => Ok(PngCompressionSetting::DeflateLevel(level)),
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

impl From<CuPngFilter> for Filter {
    fn from(value: CuPngFilter) -> Self {
        match value {
            CuPngFilter::NoFilter => Filter::NoFilter,
            CuPngFilter::Sub => Filter::Sub,
            CuPngFilter::Up => Filter::Up,
            CuPngFilter::Avg => Filter::Avg,
            CuPngFilter::Paeth => Filter::Paeth,
            CuPngFilter::Adaptive => Filter::Adaptive,
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
    compression: PngCompressionSetting,
    filter: Filter,
    // PNG must be length-delimited before entering the CopperList stream: a
    // streaming PNG decoder may otherwise read into the following record.
    encoded_scratch: Vec<u8>,
}

impl CuPngCodec {
    fn encode_error(message: impl Into<String>) -> EncodeError {
        EncodeError::OtherString(message.into())
    }

    fn decode_error(message: impl Into<String>) -> DecodeError {
        DecodeError::OtherString(message.into())
    }

    fn png_decode_error(err: PngDecodingError) -> DecodeError {
        match err {
            PngDecodingError::IoError(inner) if inner.kind() == io::ErrorKind::UnexpectedEof => {
                DecodeError::UnexpectedEnd { additional: 1 }
            }
            other => Self::decode_error(other.to_string()),
        }
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
            compression: config.compression.into_png()?,
            filter: config.filter.into(),
            encoded_scratch: Vec::new(),
        })
    }

    fn source_payload_handle_bytes(&self, payload: &CuImage<Vec<u8>>) -> usize {
        payload.format.byte_size()
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
        payload.seq.encode(encoder)?;
        self.encoded_scratch.clear();
        let compression = self.compression;
        let filter = self.filter;
        let encoded_scratch = &mut self.encoded_scratch;
        payload.buffer_handle.with_inner(|inner| {
            let image_bytes: &[u8] = inner;
            if image_bytes.len() < byte_size {
                return Err(Self::encode_error(format!(
                    "PNG codec expected at least {} image bytes, got {}",
                    byte_size,
                    image_bytes.len()
                )));
            }

            let mut png_encoder =
                PngEncoder::new(encoded_scratch, payload.format.width, payload.format.height);
            png_encoder.set_color(ColorType::Rgb);
            png_encoder.set_depth(BitDepth::Eight);
            compression.apply(&mut png_encoder);
            png_encoder.set_filter(filter);

            let mut png_writer = png_encoder
                .write_header()
                .map_err(|err| Self::encode_error(err.to_string()))?;
            png_writer
                .write_image_data(&image_bytes[..byte_size])
                .map_err(|err| Self::encode_error(err.to_string()))
        })?;
        self.encoded_scratch.encode(encoder)?;
        Ok(())
    }

    fn decode_payload<D: Decoder<Context = ()>>(
        &mut self,
        decoder: &mut D,
    ) -> Result<CuImage<Vec<u8>>, DecodeError> {
        let seq: u64 = Decode::decode(decoder)?;
        let encoded_png: Vec<u8> = Decode::decode(decoder)?;
        let mut png = PngDecoder::new(Cursor::new(encoded_png))
            .read_info()
            .map_err(Self::png_decode_error)?;
        let (color_type, bit_depth) = png.output_color_type();
        if color_type != ColorType::Rgb || bit_depth != BitDepth::Eight {
            return Err(Self::decode_error(format!(
                "PNG codec expected RGB8 decode output, got {color_type:?} / {bit_depth:?}"
            )));
        }

        let output_size = png
            .output_buffer_size()
            .ok_or_else(|| Self::decode_error("PNG codec output buffer size overflow"))?;
        let mut buffer = vec![0u8; output_size];
        let info = png
            .next_frame(&mut buffer)
            .map_err(Self::png_decode_error)?;
        png.finish().map_err(Self::png_decode_error)?;
        buffer.truncate(info.buffer_size());

        let stride = info
            .width
            .checked_mul(3)
            .ok_or_else(|| Self::decode_error("PNG codec image stride overflow"))?;
        Ok(CuImage {
            seq,
            format: CuImageBufferFormat {
                width: info.width,
                height: info.height,
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

    #[derive(Debug)]
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

    #[test]
    fn png_codec_frames_are_length_delimited() {
        let first = sample_image();
        let mut second = sample_image();
        second.seq = 8;
        let encoded = encode_to_vec(
            (
                EncodedWithCodec {
                    image: &first,
                    codec: std::cell::RefCell::new(
                        CuPngCodec::new(CuPngCodecConfig::default()).expect("codec"),
                    ),
                },
                EncodedWithCodec {
                    image: &second,
                    codec: std::cell::RefCell::new(
                        CuPngCodec::new(CuPngCodecConfig::default()).expect("codec"),
                    ),
                },
            ),
            standard(),
        )
        .expect("encode");

        let ((first_decoded, second_decoded), consumed): (
            (DecodedWithCodec, DecodedWithCodec),
            usize,
        ) = decode_from_slice(&encoded, standard()).expect("decode");
        assert_eq!(consumed, encoded.len());
        assert_eq!(first_decoded.0.seq, first.seq);
        assert_eq!(second_decoded.0.seq, second.seq);
    }

    #[test]
    fn png_codec_reports_handle_backed_source_bytes_to_monitoring() {
        let image = sample_image();
        let cache = cu29::monitoring::CuMsgIoCache::<1>::default();

        {
            let capture = cu29::monitoring::start_copperlist_io_capture(&cache);
            capture.select_slot(0);
            let _ = encode_to_vec(
                EncodedWithCodec {
                    image: &image,
                    codec: std::cell::RefCell::new(
                        CuPngCodec::new(CuPngCodecConfig::default()).expect("codec"),
                    ),
                },
                standard(),
            )
            .expect("encode");
        }

        let io = cache.get(0);
        assert!(io.present);
        assert_eq!(io.handle_bytes, image.format.byte_size() as u64);
    }

    #[test]
    fn png_codec_preserves_unexpected_end() {
        let original = sample_image();
        let mut buffer = vec![0u8; 32];
        let err = bincode::encode_into_slice(
            EncodedWithCodec {
                image: &original,
                codec: std::cell::RefCell::new(
                    CuPngCodec::new(CuPngCodecConfig::default()).expect("codec"),
                ),
            },
            &mut buffer,
            standard(),
        )
        .expect_err("expected short buffer failure");

        assert!(matches!(err, EncodeError::UnexpectedEnd));
    }

    #[test]
    fn png_codec_decode_truncation_is_unexpected_end() {
        let original = sample_image();
        let mut encoded = encode_to_vec(
            EncodedWithCodec {
                image: &original,
                codec: std::cell::RefCell::new(
                    CuPngCodec::new(CuPngCodecConfig::default()).expect("codec"),
                ),
            },
            standard(),
        )
        .expect("encode");
        encoded.pop().expect("encoded payload should not be empty");

        let err = decode_from_slice::<DecodedWithCodec, _>(&encoded, standard())
            .expect_err("expected truncated decode failure");

        assert!(matches!(err, DecodeError::UnexpectedEnd { .. }));
    }
}
