use bincode::de::read::Reader as BincodeReader;
use bincode::de::{Decode, Decoder};
use bincode::enc::write::Writer as BincodeWriter;
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
use std::io::{self, BufRead, Read, Seek, SeekFrom, Write};

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

struct BincodeWriterAdapter<'a, W> {
    inner: &'a mut W,
    last_error: Option<EncodeError>,
}

impl<'a, W> BincodeWriterAdapter<'a, W> {
    fn new(inner: &'a mut W) -> Self {
        Self {
            inner,
            last_error: None,
        }
    }

    fn take_encode_error(&mut self, fallback: impl Into<String>) -> EncodeError {
        self.last_error
            .take()
            .unwrap_or_else(|| EncodeError::OtherString(fallback.into()))
    }
}

impl<W: BincodeWriter> Write for BincodeWriterAdapter<'_, W> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.inner.write(buf).map_err(|err| {
            if self.last_error.is_none() {
                self.last_error = Some(err);
            }
            io::Error::other("bincode writer failure")
        })?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

struct BincodeReaderAdapter<'a, R> {
    inner: &'a mut R,
    position: u64,
}

impl<'a, R> BincodeReaderAdapter<'a, R> {
    fn new(inner: &'a mut R) -> Self {
        Self { inner, position: 0 }
    }

    fn peek_window_len(&mut self) -> usize
    where
        R: BincodeReader,
    {
        if self.inner.peek_read(8192).is_some() {
            8192
        } else if self.inner.peek_read(4096).is_some() {
            4096
        } else if self.inner.peek_read(2048).is_some() {
            2048
        } else if self.inner.peek_read(1024).is_some() {
            1024
        } else if self.inner.peek_read(512).is_some() {
            512
        } else if self.inner.peek_read(256).is_some() {
            256
        } else if self.inner.peek_read(128).is_some() {
            128
        } else if self.inner.peek_read(64).is_some() {
            64
        } else if self.inner.peek_read(32).is_some() {
            32
        } else if self.inner.peek_read(16).is_some() {
            16
        } else if self.inner.peek_read(8).is_some() {
            8
        } else if self.inner.peek_read(4).is_some() {
            4
        } else if self.inner.peek_read(2).is_some() {
            2
        } else if self.inner.peek_read(1).is_some() {
            1
        } else {
            0
        }
    }

    fn fill_from_peek(&mut self) -> io::Result<&[u8]>
    where
        R: BincodeReader,
    {
        let peek_len = self.peek_window_len();
        if peek_len == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "PNG codec expected more bytes from the bincode decoder",
            ));
        }

        self.inner.peek_read(peek_len).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "PNG codec expected more bytes from the bincode decoder",
            )
        })
    }
}

impl<R: BincodeReader> Read for BincodeReaderAdapter<'_, R> {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }

        let available = self.fill_from_peek()?;
        let count = available.len().min(buf.len());
        buf[..count].copy_from_slice(&available[..count]);
        self.consume(count);
        Ok(count)
    }
}

impl<R: BincodeReader> BufRead for BincodeReaderAdapter<'_, R> {
    fn fill_buf(&mut self) -> io::Result<&[u8]> {
        self.fill_from_peek()
    }

    fn consume(&mut self, amt: usize) {
        self.inner.consume(amt);
        self.position = self.position.saturating_add(amt as u64);
    }
}

impl<R> Seek for BincodeReaderAdapter<'_, R> {
    fn seek(&mut self, pos: SeekFrom) -> io::Result<u64> {
        match pos {
            SeekFrom::Current(0) => Ok(self.position),
            SeekFrom::Start(target) if target == self.position => Ok(self.position),
            _ => Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "PNG codec only supports forward streaming decode",
            )),
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
        payload.seq.encode(encoder)?;
        payload.buffer_handle.with_inner(|inner| {
            let image_bytes: &[u8] = inner;
            if image_bytes.len() < byte_size {
                return Err(Self::encode_error(format!(
                    "PNG codec expected at least {} image bytes, got {}",
                    byte_size,
                    image_bytes.len()
                )));
            }

            let mut writer = BincodeWriterAdapter::new(encoder.writer());
            let png_result = {
                let mut png_encoder =
                    PngEncoder::new(&mut writer, payload.format.width, payload.format.height);
                png_encoder.set_color(ColorType::Rgb);
                png_encoder.set_depth(BitDepth::Eight);
                self.compression.apply(&mut png_encoder);
                png_encoder.set_filter(self.filter);

                match png_encoder.write_header() {
                    Ok(mut png_writer) => png_writer.write_image_data(&image_bytes[..byte_size]),
                    Err(err) => Err(err),
                }
            };
            png_result.map_err(|err| writer.take_encode_error(err.to_string()))
        })?;
        Ok(())
    }

    fn decode_payload<D: Decoder<Context = ()>>(
        &mut self,
        decoder: &mut D,
    ) -> Result<CuImage<Vec<u8>>, DecodeError> {
        let seq: u64 = Decode::decode(decoder)?;
        let mut reader = BincodeReaderAdapter::new(decoder.reader());
        let mut png = PngDecoder::new(&mut reader)
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
