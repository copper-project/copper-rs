use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use cu_sensor_payloads::CuImage;
use cu29::logcodec::CuLogCodec;
use cu29::prelude::{CuError, CuResult};
use serde::{Deserialize, Serialize};

#[cfg(feature = "ffmpeg")]
use bincode::{Decode, Encode};
#[cfg(feature = "ffmpeg")]
use cu_sensor_payloads::{CuImageBufferFormat, CuImagePlaneLayout};
#[cfg(feature = "ffmpeg")]
use ffmpeg_next as ffmpeg;

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub enum CuFfv1Coder {
    Rice,
    #[default]
    RangeDef,
    RangeTab,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub enum CuFfv1SliceCrc {
    #[default]
    Auto,
    Disabled,
    Enabled,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub enum CuFfv1RemapMode {
    #[default]
    Auto,
    Off,
    DualRle,
    FlipDualRle,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CuFfv1CodecConfig {
    #[serde(default)]
    pub coder: CuFfv1Coder,
    #[serde(default)]
    pub slice_crc: CuFfv1SliceCrc,
    #[serde(default)]
    pub remap_mode: CuFfv1RemapMode,
    #[serde(default = "default_remap_optimizer")]
    pub remap_optimizer: u8,
    #[serde(default = "default_threads")]
    pub threads: usize,
    #[serde(default)]
    pub slices: Option<usize>,
}

const fn default_remap_optimizer() -> u8 {
    3
}

const fn default_threads() -> usize {
    1
}

impl Default for CuFfv1CodecConfig {
    fn default() -> Self {
        Self {
            coder: CuFfv1Coder::default(),
            slice_crc: CuFfv1SliceCrc::default(),
            remap_mode: CuFfv1RemapMode::default(),
            remap_optimizer: default_remap_optimizer(),
            threads: default_threads(),
            slices: None,
        }
    }
}

pub struct CuFfv1Codec {
    #[cfg(feature = "ffmpeg")]
    config: CuFfv1CodecConfig,
    #[cfg(feature = "ffmpeg")]
    encoder: Option<EncoderState>,
    #[cfg(feature = "ffmpeg")]
    decoder: Option<DecoderState>,
}

#[cfg(feature = "ffmpeg")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, bincode::Encode, bincode::Decode)]
enum CuFfv1WireFormat {
    Gray8,
    Yuv420P,
    Bgra,
}

#[cfg(feature = "ffmpeg")]
#[derive(Debug, Clone, PartialEq, Eq)]
struct EncoderKey {
    width: u32,
    height: u32,
    wire_format: CuFfv1WireFormat,
}

#[cfg(feature = "ffmpeg")]
struct EncoderState {
    key: EncoderKey,
    encoder: ffmpeg::encoder::Video,
    frame: ffmpeg::frame::Video,
}

#[cfg(feature = "ffmpeg")]
#[derive(Debug, Clone, PartialEq, Eq)]
struct DecoderKey {
    width: u32,
    height: u32,
    wire_format: CuFfv1WireFormat,
    extradata: Vec<u8>,
}

#[cfg(feature = "ffmpeg")]
struct DecoderState {
    key: DecoderKey,
    decoder: ffmpeg::decoder::Video,
    frame: ffmpeg::frame::Video,
}

#[cfg(feature = "ffmpeg")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct WireSpec {
    width: u32,
    height: u32,
    pixel: ffmpeg::format::Pixel,
    wire_format: CuFfv1WireFormat,
}

impl CuFfv1Coder {
    #[cfg(feature = "ffmpeg")]
    fn as_ffmpeg(self) -> &'static str {
        match self {
            Self::Rice => "rice",
            Self::RangeDef => "range_def",
            Self::RangeTab => "range_tab",
        }
    }
}

impl CuFfv1SliceCrc {
    #[cfg(feature = "ffmpeg")]
    fn as_ffmpeg(self) -> &'static str {
        match self {
            Self::Auto => "-1",
            Self::Disabled => "0",
            Self::Enabled => "1",
        }
    }
}

impl CuFfv1RemapMode {
    #[cfg(feature = "ffmpeg")]
    fn as_ffmpeg(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Off => "off",
            Self::DualRle => "dualrle",
            Self::FlipDualRle => "flipdualrle",
        }
    }
}

impl CuFfv1Codec {
    fn encode_error(message: impl Into<String>) -> EncodeError {
        EncodeError::OtherString(message.into())
    }

    fn decode_error(message: impl Into<String>) -> DecodeError {
        DecodeError::OtherString(message.into())
    }

    fn validate_config(config: &CuFfv1CodecConfig) -> CuResult<()> {
        if config.threads == 0 {
            return Err(CuError::from("FFV1 codec threads must be at least 1"));
        }
        if matches!(config.slices, Some(0)) {
            return Err(CuError::from(
                "FFV1 codec slices must be at least 1 when set",
            ));
        }
        if config.remap_optimizer > 5 {
            return Err(CuError::from(format!(
                "FFV1 remap_optimizer must be in 0..=5, got {}",
                config.remap_optimizer
            )));
        }
        Ok(())
    }
}

impl CuLogCodec<CuImage<Vec<u8>>> for CuFfv1Codec {
    type Config = CuFfv1CodecConfig;

    fn new(config: Self::Config) -> CuResult<Self> {
        Self::validate_config(&config)?;

        #[cfg(feature = "ffmpeg")]
        {
            ffmpeg::init().map_err(|err| {
                CuError::new_with_cause("Failed to initialize FFmpeg for FFV1 codec", err)
            })?;

            Ok(Self {
                config,
                encoder: None,
                decoder: None,
            })
        }

        #[cfg(not(feature = "ffmpeg"))]
        {
            let _ = config;
            Err(CuError::from(
                "FFV1 codec support requires enabling the `cu-ffv1-codec/ffmpeg` feature",
            ))
        }
    }

    fn encode_payload<E: Encoder>(
        &mut self,
        payload: &CuImage<Vec<u8>>,
        encoder: &mut E,
    ) -> Result<(), EncodeError> {
        #[cfg(feature = "ffmpeg")]
        {
            return self.encode_payload_ffmpeg(payload, encoder);
        }

        #[cfg(not(feature = "ffmpeg"))]
        {
            let _ = payload;
            let _ = encoder;
            Err(Self::encode_error(
                "FFV1 codec support requires enabling the `cu-ffv1-codec/ffmpeg` feature",
            ))
        }
    }

    fn decode_payload<D: Decoder<Context = ()>>(
        &mut self,
        decoder: &mut D,
    ) -> Result<CuImage<Vec<u8>>, DecodeError> {
        #[cfg(feature = "ffmpeg")]
        {
            return self.decode_payload_ffmpeg(decoder);
        }

        #[cfg(not(feature = "ffmpeg"))]
        {
            let _ = decoder;
            Err(Self::decode_error(
                "FFV1 codec support requires enabling the `cu-ffv1-codec/ffmpeg` feature",
            ))
        }
    }
}

#[cfg(feature = "ffmpeg")]
impl CuFfv1Codec {
    fn encode_payload_ffmpeg<E: Encoder>(
        &mut self,
        payload: &CuImage<Vec<u8>>,
        encoder: &mut E,
    ) -> Result<(), EncodeError> {
        let wire = encoded_frame_from_payload(payload, &self.config, &mut self.encoder)
            .map_err(|err| Self::encode_error(err.to_string()))?;

        payload.seq.encode(encoder)?;
        payload.format.encode(encoder)?;
        wire.wire_format.encode(encoder)?;
        wire.padding.encode(encoder)?;
        wire.extradata.encode(encoder)?;
        wire.packet.encode(encoder)?;
        Ok(())
    }

    fn decode_payload_ffmpeg<D: Decoder<Context = ()>>(
        &mut self,
        decoder: &mut D,
    ) -> Result<CuImage<Vec<u8>>, DecodeError> {
        let seq: u64 = Decode::decode(decoder)?;
        let original_format: CuImageBufferFormat = Decode::decode(decoder)?;
        let wire_format: CuFfv1WireFormat = Decode::decode(decoder)?;
        let padding: Vec<u8> = Decode::decode(decoder)?;
        let extradata: Vec<u8> = Decode::decode(decoder)?;
        let packet: Vec<u8> = Decode::decode(decoder)?;

        let buffer = decoded_frame_to_buffer(
            original_format,
            wire_format,
            &padding,
            &extradata,
            &packet,
            &mut self.decoder,
        )
        .map_err(|err| Self::decode_error(err.to_string()))?;

        Ok(CuImage {
            seq,
            format: original_format,
            buffer_handle: cu29::prelude::CuHandle::new_detached(buffer),
        })
    }
}

#[cfg(feature = "ffmpeg")]
#[derive(Debug)]
struct EncodedFrame {
    wire_format: CuFfv1WireFormat,
    padding: Vec<u8>,
    extradata: Vec<u8>,
    packet: Vec<u8>,
}

#[cfg(feature = "ffmpeg")]
fn encoded_frame_from_payload(
    payload: &CuImage<Vec<u8>>,
    config: &CuFfv1CodecConfig,
    state: &mut Option<EncoderState>,
) -> CuResult<EncodedFrame> {
    validate_image_format(payload.format)?;
    let wire_spec = wire_spec(payload.format)?;
    let padding = payload.buffer_handle.with_inner(|inner| {
        let bytes: &[u8] = inner;
        let required = payload.format.byte_size();
        if bytes.len() < required {
            return Err(CuError::from(format!(
                "FFV1 codec expected at least {required} image bytes, got {}",
                bytes.len()
            )));
        }
        extract_padding(payload.format, &bytes[..required])
    })?;

    let key = EncoderKey {
        width: wire_spec.width,
        height: wire_spec.height,
        wire_format: wire_spec.wire_format,
    };
    let needs_reopen = state.as_ref().is_none_or(|state| state.key != key);
    if needs_reopen {
        *state = Some(build_encoder_state(wire_spec, config)?);
    }
    let state = state
        .as_mut()
        .expect("FFV1 encoder state should be initialized");

    payload.buffer_handle.with_inner(|inner| {
        let bytes: &[u8] = inner;
        fill_frame_from_payload(bytes, payload.format, wire_spec, &mut state.frame)
    })?;

    state.frame.set_pts(Some(0));
    state
        .encoder
        .send_frame(&state.frame)
        .map_err(|err| CuError::new_with_cause("Failed to send frame to FFV1 encoder", err))?;

    let mut packet = ffmpeg::Packet::empty();
    state
        .encoder
        .receive_packet(&mut packet)
        .map_err(|err| CuError::new_with_cause("FFV1 encoder did not produce a packet", err))?;

    let mut extra_packet = ffmpeg::Packet::empty();
    match state.encoder.receive_packet(&mut extra_packet) {
        Err(ffmpeg::Error::Other { errno }) if errno == ffmpeg::error::EAGAIN => {}
        Err(ffmpeg::Error::Eof) => {}
        Ok(()) => {
            return Err(CuError::from(
                "FFV1 encoder produced more than one packet for a single image",
            ));
        }
        Err(err) => {
            return Err(CuError::new_with_cause(
                "Failed while draining FFV1 encoder packet queue",
                err,
            ));
        }
    }

    let packet_bytes = packet
        .data()
        .ok_or_else(|| CuError::from("FFV1 encoder returned an empty packet"))?
        .to_vec();
    let extradata = copy_context_extradata(state.encoder.as_ref());

    Ok(EncodedFrame {
        wire_format: wire_spec.wire_format,
        padding,
        extradata,
        packet: packet_bytes,
    })
}

#[cfg(feature = "ffmpeg")]
fn decoded_frame_to_buffer(
    original_format: CuImageBufferFormat,
    wire_format: CuFfv1WireFormat,
    padding: &[u8],
    extradata: &[u8],
    packet: &[u8],
    state: &mut Option<DecoderState>,
) -> CuResult<Vec<u8>> {
    validate_image_format(original_format)?;
    let wire_spec = wire_spec(original_format)?;
    if wire_spec.wire_format != wire_format {
        return Err(CuError::from(format!(
            "FFV1 wire format mismatch: payload declared {:?} but original format {:?} maps to {:?}",
            wire_format, original_format.pixel_format, wire_spec.wire_format
        )));
    }

    let key = DecoderKey {
        width: wire_spec.width,
        height: wire_spec.height,
        wire_format,
        extradata: extradata.to_vec(),
    };
    let needs_reopen = state.as_ref().is_none_or(|state| state.key != key);
    if needs_reopen {
        *state = Some(build_decoder_state(wire_spec, extradata)?);
    }
    let state = state
        .as_mut()
        .expect("FFV1 decoder state should be initialized");

    let packet = ffmpeg::Packet::copy(packet);
    state
        .decoder
        .send_packet(&packet)
        .map_err(|err| CuError::new_with_cause("Failed to send packet to FFV1 decoder", err))?;
    state
        .decoder
        .receive_frame(&mut state.frame)
        .map_err(|err| CuError::new_with_cause("FFV1 decoder did not produce a frame", err))?;
    state.decoder.flush();

    if state.frame.width() != wire_spec.width || state.frame.height() != wire_spec.height {
        return Err(CuError::from(format!(
            "FFV1 decoder returned {}x{} for expected {}x{} frame",
            state.frame.width(),
            state.frame.height(),
            wire_spec.width,
            wire_spec.height
        )));
    }
    if state.frame.format() != wire_spec.pixel {
        return Err(CuError::from(format!(
            "FFV1 decoder returned pixel format {:?}, expected {:?}",
            state.frame.format(),
            wire_spec.pixel
        )));
    }

    let mut buffer = vec![0u8; original_format.byte_size()];
    fill_output_from_frame(&state.frame, original_format, &mut buffer)?;
    restore_padding(original_format, &mut buffer, padding)?;
    Ok(buffer)
}

#[cfg(feature = "ffmpeg")]
fn build_encoder_state(wire_spec: WireSpec, config: &CuFfv1CodecConfig) -> CuResult<EncoderState> {
    let codec = ffmpeg::encoder::find_by_name("ffv1")
        .ok_or_else(|| CuError::from("Could not find FFV1 encoder in FFmpeg"))?;
    let mut context = ffmpeg::codec::Context::new_with_codec(codec);
    context.set_threading(threading_config(config.threads));

    let mut encoder = context
        .encoder()
        .video()
        .map_err(|err| CuError::new_with_cause("Failed to build FFV1 encoder context", err))?;
    encoder.set_width(wire_spec.width);
    encoder.set_height(wire_spec.height);
    encoder.set_format(wire_spec.pixel);
    encoder.set_time_base((1, 1));
    encoder.set_frame_rate(Some((1, 1)));
    encoder.set_gop(1);
    encoder.set_max_b_frames(0);

    let mut options = ffmpeg::Dictionary::new();
    options.set("coder", config.coder.as_ffmpeg());
    options.set("slicecrc", config.slice_crc.as_ffmpeg());
    options.set("remap_mode", config.remap_mode.as_ffmpeg());
    options.set("remap_optimizer", &config.remap_optimizer.to_string());
    if let Some(slices) = config.slices {
        options.set("slices", &slices.to_string());
    }

    let encoder = encoder
        .open_as_with(codec, options)
        .map_err(|err| CuError::new_with_cause("Failed to open FFV1 encoder", err))?;
    let frame = ffmpeg::frame::Video::new(wire_spec.pixel, wire_spec.width, wire_spec.height);

    Ok(EncoderState {
        key: EncoderKey {
            width: wire_spec.width,
            height: wire_spec.height,
            wire_format: wire_spec.wire_format,
        },
        encoder,
        frame,
    })
}

#[cfg(feature = "ffmpeg")]
fn build_decoder_state(wire_spec: WireSpec, extradata: &[u8]) -> CuResult<DecoderState> {
    let decoder = match open_ffv1_decoder(wire_spec, extradata) {
        Ok(decoder) => decoder,
        Err(primary_err) if !extradata.is_empty() => open_ffv1_decoder(wire_spec, &[]).map_err(
            |fallback_err| {
                CuError::from(format!(
                    "Failed to open FFV1 decoder with extradata ({primary_err}); retry without extradata also failed ({fallback_err})"
                ))
            },
        )?,
        Err(err) => return Err(err),
    };
    let frame = ffmpeg::frame::Video::empty();

    Ok(DecoderState {
        key: DecoderKey {
            width: wire_spec.width,
            height: wire_spec.height,
            wire_format: wire_spec.wire_format,
            extradata: extradata.to_vec(),
        },
        decoder,
        frame,
    })
}

#[cfg(feature = "ffmpeg")]
fn open_ffv1_decoder(wire_spec: WireSpec, extradata: &[u8]) -> CuResult<ffmpeg::decoder::Video> {
    let codec = ffmpeg::decoder::find_by_name("ffv1")
        .ok_or_else(|| CuError::from("Could not find FFV1 decoder in FFmpeg"))?;
    let mut context = ffmpeg::codec::Context::new();
    context.set_threading(threading_config(1));
    unsafe {
        (*context.as_mut_ptr()).width = wire_spec.width as i32;
        (*context.as_mut_ptr()).height = wire_spec.height as i32;
        (*context.as_mut_ptr()).pix_fmt = wire_spec.pixel.into();
        set_context_extradata(&mut context, extradata)?;
    }
    let mut decoder = context.decoder();
    decoder.set_packet_time_base((1, 1));
    let decoder = decoder
        .open_as(codec)
        .and_then(|decoder| decoder.video())
        .map_err(|err| CuError::new_with_cause("Failed to open FFV1 decoder", err))?;
    Ok(decoder)
}

#[cfg(feature = "ffmpeg")]
fn threading_config(count: usize) -> ffmpeg::threading::Config {
    let mut config = ffmpeg::threading::Config::count(count);
    config.kind = if count > 1 {
        ffmpeg::threading::Type::Slice
    } else {
        ffmpeg::threading::Type::None
    };
    config
}

#[cfg(feature = "ffmpeg")]
fn wire_spec(format: CuImageBufferFormat) -> CuResult<WireSpec> {
    let pixel = match &format.pixel_format {
        b"GRAY" | b"Y800" => ffmpeg::format::Pixel::GRAY8,
        b"I420" | b"YV12" | b"NV12" | b"NV21" => ffmpeg::format::Pixel::YUV420P,
        b"RGB3" | b"BGR3" | b"RGB " | b"BGR " | b"RGBA" | b"BGRA" => ffmpeg::format::Pixel::BGRA,
        _ => {
            return Err(CuError::from(format!(
                "FFV1 codec does not support Copper pixel format {:?}",
                format.pixel_format
            )));
        }
    };

    let wire_format = match pixel {
        ffmpeg::format::Pixel::GRAY8 => CuFfv1WireFormat::Gray8,
        ffmpeg::format::Pixel::YUV420P => CuFfv1WireFormat::Yuv420P,
        ffmpeg::format::Pixel::BGRA => CuFfv1WireFormat::Bgra,
        _ => unreachable!("unexpected FFV1 wire pixel format"),
    };

    Ok(WireSpec {
        width: format.width,
        height: format.height,
        pixel,
        wire_format,
    })
}

#[cfg(feature = "ffmpeg")]
fn validate_image_format(format: CuImageBufferFormat) -> CuResult<()> {
    if !format.is_valid() {
        return Err(CuError::from(format!(
            "FFV1 codec received invalid image format {:?}",
            format
        )));
    }
    Ok(())
}

#[cfg(feature = "ffmpeg")]
fn extract_padding(format: CuImageBufferFormat, bytes: &[u8]) -> CuResult<Vec<u8>> {
    let mut padding = Vec::with_capacity(expected_padding_len(format)?);
    for plane_index in 0..format.plane_count() {
        let plane = plane_layout(format, plane_index)?;
        for row in 0..plane.height as usize {
            let row_start = plane.offset_bytes + row * plane.stride_bytes as usize;
            let pad_start = row_start + plane.row_bytes as usize;
            let pad_end = row_start + plane.stride_bytes as usize;
            padding.extend_from_slice(&bytes[pad_start..pad_end]);
        }
    }
    Ok(padding)
}

#[cfg(feature = "ffmpeg")]
fn restore_padding(format: CuImageBufferFormat, bytes: &mut [u8], padding: &[u8]) -> CuResult<()> {
    let mut cursor = 0usize;
    for plane_index in 0..format.plane_count() {
        let plane = plane_layout(format, plane_index)?;
        for row in 0..plane.height as usize {
            let row_start = plane.offset_bytes + row * plane.stride_bytes as usize;
            let pad_start = row_start + plane.row_bytes as usize;
            let pad_end = row_start + plane.stride_bytes as usize;
            let pad_len = pad_end.saturating_sub(pad_start);
            if cursor + pad_len > padding.len() {
                return Err(CuError::from(
                    "FFV1 padding sideband is shorter than expected",
                ));
            }
            bytes[pad_start..pad_end].copy_from_slice(&padding[cursor..cursor + pad_len]);
            cursor += pad_len;
        }
    }

    if cursor != padding.len() {
        return Err(CuError::from(
            "FFV1 padding sideband is longer than expected for the output image layout",
        ));
    }
    Ok(())
}

#[cfg(feature = "ffmpeg")]
fn expected_padding_len(format: CuImageBufferFormat) -> CuResult<usize> {
    let mut total = 0usize;
    for plane_index in 0..format.plane_count() {
        let plane = plane_layout(format, plane_index)?;
        let pad_len = ((plane.stride_bytes - plane.row_bytes) as usize)
            .checked_mul(plane.height as usize)
            .ok_or_else(|| CuError::from("FFV1 padding length overflow"))?;
        total = total
            .checked_add(pad_len)
            .ok_or_else(|| CuError::from("FFV1 padding length overflow"))?;
    }
    Ok(total)
}

#[cfg(feature = "ffmpeg")]
fn plane_layout(format: CuImageBufferFormat, plane_index: usize) -> CuResult<CuImagePlaneLayout> {
    format.plane(plane_index).ok_or_else(|| {
        CuError::from(format!(
            "FFV1 codec could not resolve plane {plane_index} for image format {:?}",
            format
        ))
    })
}

#[cfg(feature = "ffmpeg")]
fn fill_frame_from_payload(
    bytes: &[u8],
    format: CuImageBufferFormat,
    wire_spec: WireSpec,
    frame: &mut ffmpeg::frame::Video,
) -> CuResult<()> {
    match wire_spec.wire_format {
        CuFfv1WireFormat::Gray8 => copy_single_plane(bytes, plane_layout(format, 0)?, frame, 0),
        CuFfv1WireFormat::Yuv420P => match &format.pixel_format {
            b"I420" => copy_planar_420(bytes, format, frame, false),
            b"YV12" => copy_planar_420(bytes, format, frame, true),
            b"NV12" => copy_nv12(bytes, format, frame, false),
            b"NV21" => copy_nv12(bytes, format, frame, true),
            _ => Err(CuError::from(format!(
                "FFV1 YUV420P mapping does not support input format {:?}",
                format.pixel_format
            ))),
        },
        CuFfv1WireFormat::Bgra => copy_to_bgra(bytes, format, frame),
    }
}

#[cfg(feature = "ffmpeg")]
fn fill_output_from_frame(
    frame: &ffmpeg::frame::Video,
    original_format: CuImageBufferFormat,
    bytes: &mut [u8],
) -> CuResult<()> {
    match &original_format.pixel_format {
        b"GRAY" | b"Y800" => copy_frame_plane(frame, 0, bytes, plane_layout(original_format, 0)?),
        b"I420" => copy_frame_planar_420(frame, original_format, bytes, false),
        b"YV12" => copy_frame_planar_420(frame, original_format, bytes, true),
        b"NV12" => copy_frame_nv12(frame, original_format, bytes, false),
        b"NV21" => copy_frame_nv12(frame, original_format, bytes, true),
        b"RGB3" | b"BGR3" | b"RGB " | b"BGR " | b"RGBA" | b"BGRA" => {
            copy_from_bgra(frame, original_format, bytes)
        }
        _ => Err(CuError::from(format!(
            "FFV1 output mapping does not support Copper pixel format {:?}",
            original_format.pixel_format
        ))),
    }
}

#[cfg(feature = "ffmpeg")]
fn copy_single_plane(
    bytes: &[u8],
    plane: CuImagePlaneLayout,
    frame: &mut ffmpeg::frame::Video,
    frame_plane: usize,
) -> CuResult<()> {
    let dst_stride = frame.stride(frame_plane);
    let dst = frame.data_mut(frame_plane);
    for row in 0..plane.height as usize {
        let src_start = plane.offset_bytes + row * plane.stride_bytes as usize;
        let dst_start = row * dst_stride;
        let row_bytes = plane.row_bytes as usize;
        dst[dst_start..dst_start + row_bytes]
            .copy_from_slice(&bytes[src_start..src_start + row_bytes]);
    }
    Ok(())
}

#[cfg(feature = "ffmpeg")]
fn copy_frame_plane(
    frame: &ffmpeg::frame::Video,
    frame_plane: usize,
    bytes: &mut [u8],
    plane: CuImagePlaneLayout,
) -> CuResult<()> {
    let src_stride = frame.stride(frame_plane);
    let src = frame.data(frame_plane);
    for row in 0..plane.height as usize {
        let src_start = row * src_stride;
        let dst_start = plane.offset_bytes + row * plane.stride_bytes as usize;
        let row_bytes = plane.row_bytes as usize;
        bytes[dst_start..dst_start + row_bytes]
            .copy_from_slice(&src[src_start..src_start + row_bytes]);
    }
    Ok(())
}

#[cfg(feature = "ffmpeg")]
fn copy_planar_420(
    bytes: &[u8],
    format: CuImageBufferFormat,
    frame: &mut ffmpeg::frame::Video,
    swap_chroma: bool,
) -> CuResult<()> {
    copy_single_plane(bytes, plane_layout(format, 0)?, frame, 0)?;
    let u_plane = if swap_chroma { 2 } else { 1 };
    let v_plane = if swap_chroma { 1 } else { 2 };
    copy_single_plane(bytes, plane_layout(format, 1)?, frame, u_plane)?;
    copy_single_plane(bytes, plane_layout(format, 2)?, frame, v_plane)?;
    Ok(())
}

#[cfg(feature = "ffmpeg")]
fn copy_frame_planar_420(
    frame: &ffmpeg::frame::Video,
    format: CuImageBufferFormat,
    bytes: &mut [u8],
    swap_chroma: bool,
) -> CuResult<()> {
    copy_frame_plane(frame, 0, bytes, plane_layout(format, 0)?)?;
    let u_plane = if swap_chroma { 2 } else { 1 };
    let v_plane = if swap_chroma { 1 } else { 2 };
    copy_frame_plane(frame, u_plane, bytes, plane_layout(format, 1)?)?;
    copy_frame_plane(frame, v_plane, bytes, plane_layout(format, 2)?)?;
    Ok(())
}

#[cfg(feature = "ffmpeg")]
fn copy_nv12(
    bytes: &[u8],
    format: CuImageBufferFormat,
    frame: &mut ffmpeg::frame::Video,
    swap_chroma: bool,
) -> CuResult<()> {
    copy_single_plane(bytes, plane_layout(format, 0)?, frame, 0)?;
    let chroma = plane_layout(format, 1)?;
    let dst_stride_u = frame.stride(1);
    let dst_stride_v = frame.stride(2);
    let (dst_u_ptr, dst_u_len) = {
        let dst = frame.data_mut(1);
        (dst.as_mut_ptr(), dst.len())
    };
    let (dst_v_ptr, dst_v_len) = {
        let dst = frame.data_mut(2);
        (dst.as_mut_ptr(), dst.len())
    };
    let dst_u = unsafe { std::slice::from_raw_parts_mut(dst_u_ptr, dst_u_len) };
    let dst_v = unsafe { std::slice::from_raw_parts_mut(dst_v_ptr, dst_v_len) };

    for row in 0..chroma.height as usize {
        let src_start = chroma.offset_bytes + row * chroma.stride_bytes as usize;
        let row_bytes = chroma.row_bytes as usize;
        let src_row = &bytes[src_start..src_start + row_bytes];
        let u_row_start = row * dst_stride_u;
        let v_row_start = row * dst_stride_v;
        let chroma_width = format.width.div_ceil(2) as usize;
        for col in 0..chroma_width {
            let pair = col * 2;
            let first = src_row[pair];
            let second = src_row[pair + 1];
            if swap_chroma {
                dst_u[u_row_start + col] = second;
                dst_v[v_row_start + col] = first;
            } else {
                dst_u[u_row_start + col] = first;
                dst_v[v_row_start + col] = second;
            }
        }
    }
    Ok(())
}

#[cfg(feature = "ffmpeg")]
fn copy_frame_nv12(
    frame: &ffmpeg::frame::Video,
    format: CuImageBufferFormat,
    bytes: &mut [u8],
    swap_chroma: bool,
) -> CuResult<()> {
    copy_frame_plane(frame, 0, bytes, plane_layout(format, 0)?)?;
    let chroma = plane_layout(format, 1)?;
    let src_u = frame.data(1);
    let src_v = frame.data(2);
    let stride_u = frame.stride(1);
    let stride_v = frame.stride(2);

    for row in 0..chroma.height as usize {
        let dst_start = chroma.offset_bytes + row * chroma.stride_bytes as usize;
        let chroma_width = format.width.div_ceil(2) as usize;
        for col in 0..chroma_width {
            let u = src_u[row * stride_u + col];
            let v = src_v[row * stride_v + col];
            let pair = dst_start + col * 2;
            if swap_chroma {
                bytes[pair] = v;
                bytes[pair + 1] = u;
            } else {
                bytes[pair] = u;
                bytes[pair + 1] = v;
            }
        }
    }
    Ok(())
}

#[cfg(feature = "ffmpeg")]
fn copy_to_bgra(
    bytes: &[u8],
    format: CuImageBufferFormat,
    frame: &mut ffmpeg::frame::Video,
) -> CuResult<()> {
    let plane = plane_layout(format, 0)?;
    let dst_stride = frame.stride(0);
    let dst = frame.data_mut(0);

    for row in 0..plane.height as usize {
        let src_start = plane.offset_bytes + row * plane.stride_bytes as usize;
        let dst_start = row * dst_stride;
        let src_row = &bytes[src_start..src_start + plane.row_bytes as usize];
        let dst_row = &mut dst[dst_start..dst_start + format.width as usize * 4];
        match &format.pixel_format {
            b"BGRA" => dst_row.copy_from_slice(src_row),
            b"RGBA" => {
                for col in 0..format.width as usize {
                    let src_px = col * 4;
                    let dst_px = col * 4;
                    dst_row[dst_px] = src_row[src_px + 2];
                    dst_row[dst_px + 1] = src_row[src_px + 1];
                    dst_row[dst_px + 2] = src_row[src_px];
                    dst_row[dst_px + 3] = src_row[src_px + 3];
                }
            }
            b"RGB3" | b"RGB " => {
                for col in 0..format.width as usize {
                    let src_px = col * 3;
                    let dst_px = col * 4;
                    dst_row[dst_px] = src_row[src_px + 2];
                    dst_row[dst_px + 1] = src_row[src_px + 1];
                    dst_row[dst_px + 2] = src_row[src_px];
                    dst_row[dst_px + 3] = 0xff;
                }
            }
            b"BGR3" | b"BGR " => {
                for col in 0..format.width as usize {
                    let src_px = col * 3;
                    let dst_px = col * 4;
                    dst_row[dst_px] = src_row[src_px];
                    dst_row[dst_px + 1] = src_row[src_px + 1];
                    dst_row[dst_px + 2] = src_row[src_px + 2];
                    dst_row[dst_px + 3] = 0xff;
                }
            }
            _ => {
                return Err(CuError::from(format!(
                    "FFV1 BGRA mapping does not support input format {:?}",
                    format.pixel_format
                )));
            }
        }
    }
    Ok(())
}

#[cfg(feature = "ffmpeg")]
fn copy_from_bgra(
    frame: &ffmpeg::frame::Video,
    format: CuImageBufferFormat,
    bytes: &mut [u8],
) -> CuResult<()> {
    let plane = plane_layout(format, 0)?;
    let src_stride = frame.stride(0);
    let src = frame.data(0);

    for row in 0..plane.height as usize {
        let src_start = row * src_stride;
        let dst_start = plane.offset_bytes + row * plane.stride_bytes as usize;
        let src_row = &src[src_start..src_start + format.width as usize * 4];
        let dst_row = &mut bytes[dst_start..dst_start + plane.row_bytes as usize];
        match &format.pixel_format {
            b"BGRA" => dst_row.copy_from_slice(src_row),
            b"RGBA" => {
                for col in 0..format.width as usize {
                    let src_px = col * 4;
                    let dst_px = col * 4;
                    dst_row[dst_px] = src_row[src_px + 2];
                    dst_row[dst_px + 1] = src_row[src_px + 1];
                    dst_row[dst_px + 2] = src_row[src_px];
                    dst_row[dst_px + 3] = src_row[src_px + 3];
                }
            }
            b"RGB3" | b"RGB " => {
                for col in 0..format.width as usize {
                    let src_px = col * 4;
                    let dst_px = col * 3;
                    dst_row[dst_px] = src_row[src_px + 2];
                    dst_row[dst_px + 1] = src_row[src_px + 1];
                    dst_row[dst_px + 2] = src_row[src_px];
                }
            }
            b"BGR3" | b"BGR " => {
                for col in 0..format.width as usize {
                    let src_px = col * 4;
                    let dst_px = col * 3;
                    dst_row[dst_px] = src_row[src_px];
                    dst_row[dst_px + 1] = src_row[src_px + 1];
                    dst_row[dst_px + 2] = src_row[src_px + 2];
                }
            }
            _ => {
                return Err(CuError::from(format!(
                    "FFV1 BGRA output mapping does not support target format {:?}",
                    format.pixel_format
                )));
            }
        }
    }
    Ok(())
}

#[cfg(feature = "ffmpeg")]
fn copy_context_extradata(context: &ffmpeg::codec::Context) -> Vec<u8> {
    unsafe {
        let ptr = (*context.as_ptr()).extradata;
        let size = (*context.as_ptr()).extradata_size;
        if ptr.is_null() || size <= 0 {
            Vec::new()
        } else {
            std::slice::from_raw_parts(ptr, size as usize).to_vec()
        }
    }
}

#[cfg(feature = "ffmpeg")]
unsafe fn set_context_extradata(
    context: &mut ffmpeg::codec::Context,
    extradata: &[u8],
) -> CuResult<()> {
    if extradata.is_empty() {
        return Ok(());
    }

    let padded = extradata
        .len()
        .checked_add(ffmpeg::ffi::AV_INPUT_BUFFER_PADDING_SIZE as usize)
        .ok_or_else(|| CuError::from("FFV1 extradata length overflow"))?;
    let ptr = unsafe { ffmpeg::ffi::av_mallocz(padded) } as *mut u8;
    if ptr.is_null() {
        return Err(CuError::from("Failed to allocate FFV1 decoder extradata"));
    }

    unsafe {
        std::ptr::copy_nonoverlapping(extradata.as_ptr(), ptr, extradata.len());
        (*context.as_mut_ptr()).extradata = ptr;
        (*context.as_mut_ptr()).extradata_size = extradata.len() as i32;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ffv1_codec_validates_config() {
        let err = CuFfv1Codec::validate_config(&CuFfv1CodecConfig {
            threads: 0,
            ..CuFfv1CodecConfig::default()
        })
        .expect_err("expected invalid thread count");
        assert!(err.to_string().contains("threads"));
    }

    #[cfg(not(feature = "ffmpeg"))]
    #[test]
    fn ffv1_codec_reports_missing_backend() {
        let err = match CuFfv1Codec::new(CuFfv1CodecConfig::default()) {
            Ok(_) => panic!("expected backend error"),
            Err(err) => err,
        };
        assert!(err.to_string().contains("cu-ffv1-codec/ffmpeg"));
    }

    #[cfg(feature = "ffmpeg")]
    mod ffmpeg_roundtrip {
        use super::*;
        use bincode::config::standard;
        use bincode::{decode_from_slice, encode_to_vec};
        use cu29::prelude::CuMsg;

        struct EncodedWithCodec<'a> {
            image: &'a CuImage<Vec<u8>>,
            codec: std::cell::RefCell<CuFfv1Codec>,
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
                let mut codec = CuFfv1Codec::new(CuFfv1CodecConfig::default())
                    .map_err(|err| DecodeError::OtherString(err.to_string()))?;
                let msg = cu29::logcodec::decode_msg_with_codec::<CuImage<Vec<u8>>, _, _>(
                    decoder, &mut codec,
                )?;
                Ok(Self(msg.payload().cloned().ok_or_else(|| {
                    DecodeError::OtherString("missing image payload".to_string())
                })?))
            }
        }

        fn roundtrip(image: &CuImage<Vec<u8>>) -> CuImage<Vec<u8>> {
            let encoded = encode_to_vec(
                EncodedWithCodec {
                    image,
                    codec: std::cell::RefCell::new(
                        CuFfv1Codec::new(CuFfv1CodecConfig::default()).expect("codec"),
                    ),
                },
                standard(),
            )
            .expect("encode");

            let (decoded, consumed): (DecodedWithCodec, usize) =
                decode_from_slice(&encoded, standard()).expect("decode");
            assert_eq!(consumed, encoded.len());
            decoded.0
        }

        fn sample_rgb3_padded() -> CuImage<Vec<u8>> {
            let width = 5;
            let height = 4;
            let stride = 20;
            let mut bytes = vec![0u8; stride as usize * height as usize];
            for y in 0..height as usize {
                for x in 0..width as usize {
                    let idx = y * stride as usize + x * 3;
                    bytes[idx] = x as u8;
                    bytes[idx + 1] = y as u8;
                    bytes[idx + 2] = (x + y * 3) as u8;
                }
                bytes[y * stride as usize + width as usize * 3] = 0xaa;
                bytes[y * stride as usize + width as usize * 3 + 1] = 0x55;
                bytes[y * stride as usize + width as usize * 3 + 2] = y as u8;
                bytes[y * stride as usize + width as usize * 3 + 3] = 0xff;
                bytes[y * stride as usize + width as usize * 3 + 4] = xorshift(y as u8);
            }

            CuImage {
                seq: 9,
                format: CuImageBufferFormat {
                    width,
                    height,
                    stride,
                    pixel_format: *b"RGB3",
                },
                buffer_handle: cu29::prelude::CuHandle::new_detached(bytes),
            }
        }

        fn sample_nv12() -> CuImage<Vec<u8>> {
            let format = CuImageBufferFormat {
                width: 6,
                height: 4,
                stride: 8,
                pixel_format: *b"NV12",
            };
            let mut bytes = vec![0u8; format.byte_size()];
            let y = format.plane(0).expect("y plane");
            let uv = format.plane(1).expect("uv plane");

            for row in 0..y.height as usize {
                let base = y.offset_bytes + row * y.stride_bytes as usize;
                for col in 0..y.row_bytes as usize {
                    bytes[base + col] = (row * 11 + col) as u8;
                }
                bytes[base + y.row_bytes as usize] = 0xee;
                bytes[base + y.row_bytes as usize + 1] = 0xdd;
            }

            for row in 0..uv.height as usize {
                let base = uv.offset_bytes + row * uv.stride_bytes as usize;
                for col in (0..uv.row_bytes as usize).step_by(2) {
                    bytes[base + col] = (100 + row + col) as u8;
                    bytes[base + col + 1] = (150 + row + col) as u8;
                }
                bytes[base + uv.row_bytes as usize] = 0xaa;
                bytes[base + uv.row_bytes as usize + 1] = 0xbb;
            }

            CuImage {
                seq: 3,
                format,
                buffer_handle: cu29::prelude::CuHandle::new_detached(bytes),
            }
        }

        fn xorshift(mut x: u8) -> u8 {
            x ^= x << 3;
            x ^= x >> 5;
            x ^ (x << 1)
        }

        fn assert_same_format(lhs: CuImageBufferFormat, rhs: CuImageBufferFormat) {
            assert_eq!(lhs.width, rhs.width);
            assert_eq!(lhs.height, rhs.height);
            assert_eq!(lhs.stride, rhs.stride);
            assert_eq!(lhs.pixel_format, rhs.pixel_format);
        }

        #[test]
        fn ffv1_roundtrip_preserves_rgb3_with_padding() {
            let original = sample_rgb3_padded();
            let decoded = roundtrip(&original);
            assert_eq!(decoded.seq, original.seq);
            assert_same_format(decoded.format, original.format);
            let original_bytes = original.buffer_handle.with_inner(|inner| {
                let bytes: &[u8] = inner;
                bytes.to_vec()
            });
            let decoded_bytes = decoded.buffer_handle.with_inner(|inner| {
                let bytes: &[u8] = inner;
                bytes.to_vec()
            });
            assert_eq!(decoded_bytes, original_bytes);
        }

        #[test]
        fn ffv1_roundtrip_preserves_nv12_with_padding() {
            let original = sample_nv12();
            let decoded = roundtrip(&original);
            assert_eq!(decoded.seq, original.seq);
            assert_same_format(decoded.format, original.format);
            let original_bytes = original.buffer_handle.with_inner(|inner| {
                let bytes: &[u8] = inner;
                bytes.to_vec()
            });
            let decoded_bytes = decoded.buffer_handle.with_inner(|inner| {
                let bytes: &[u8] = inner;
                bytes.to_vec()
            });
            assert_eq!(decoded_bytes, original_bytes);
        }
    }
}
