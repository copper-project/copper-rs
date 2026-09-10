//! Typed payload codecs with Copper-owned framing in the log stream.

#[cfg(not(feature = "std"))]
extern crate alloc;

use crate::config::{ComponentConfig, CuConfig, LoggingCodecSpec};
use crate::cutask::{CuMsg, CuMsgMetadata, CuMsgPayload};
use crate::sync_compat::{Mutex, OnceLock, lock as lock_mutex, once_get_or_init};
use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
#[cfg(feature = "std")]
use bincode::config::standard;
use bincode::de::read::Reader;
use bincode::de::{Decode, Decoder, DecoderImpl};
#[cfg(feature = "std")]
use bincode::decode_from_std_read;
use bincode::enc::write::Writer;
use bincode::enc::{Encode, Encoder};
use bincode::error::{DecodeError, EncodeError};
use core::any::TypeId;
use cu29_clock::Tov;
use cu29_traits::{CuError, CuResult, observed_encode_bytes};
use hashbrown::HashMap;
use portable_atomic::{AtomicU64, Ordering};
use serde::de::DeserializeOwned;
#[cfg(feature = "std")]
use std::io::Read;
#[cfg(feature = "std")]
use std::path::Path;

#[cfg(feature = "std")]
use crate::curuntime::{RuntimeLifecycleEvent, RuntimeLifecycleRecord};
#[cfg(feature = "std")]
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};

/// Encodes a payload inside a boundary managed by Copper.
///
/// Copper prefixes each present codec payload with its encoded byte length (a
/// fixed four-byte little-endian integer). Encoding writes directly to storage;
/// decoding receives a reader restricted to that payload and must consume it fully.
/// Codecs encode their own representation without adding Copper's frame header.
/// The destination writer must support position queries and overwrites.
pub trait CuLogCodec<P: CuMsgPayload>: 'static {
    type Config: DeserializeOwned + Default;

    fn new(config: Self::Config) -> CuResult<Self>
    where
        Self: Sized;

    /// Returns handle-backed source bytes read directly by the codec.
    ///
    /// This reports only extra handle-backed residency beyond the payload's
    /// fixed `size_of::<P>()` footprint already accounted by runtime
    /// monitoring. Codecs must implement this explicitly so they opt into the
    /// correct accounting model for their payload type.
    fn source_payload_handle_bytes(&self, payload: &P) -> usize;

    fn encode_payload<E: Encoder>(
        &mut self,
        payload: &P,
        encoder: &mut E,
    ) -> Result<(), EncodeError>;

    fn decode_payload<D: Decoder<Context = ()>>(
        &mut self,
        decoder: &mut D,
    ) -> Result<P, DecodeError>;
}

pub struct CodecState<C> {
    inner: Mutex<Option<(u64, C)>>,
}

impl<C> CodecState<C> {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(None),
        }
    }
}

impl<C> Default for CodecState<C> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct EffectiveConfigEntry {
    version: AtomicU64,
    ron: Mutex<String>,
}

impl EffectiveConfigEntry {
    fn new(ron: &str) -> Self {
        Self {
            version: AtomicU64::new(1),
            ron: Mutex::new(ron.to_string()),
        }
    }

    pub fn version(&self) -> u64 {
        self.version.load(Ordering::Acquire)
    }

    pub fn ron(&self) -> String {
        lock_mutex(&self.ron).clone()
    }

    fn set(&self, ron: &str) {
        *lock_mutex(&self.ron) = ron.to_string();
        self.version.fetch_add(1, Ordering::AcqRel);
    }
}

type EffectiveConfigRegistry = HashMap<TypeId, &'static EffectiveConfigEntry>;

static EFFECTIVE_CONFIGS: OnceLock<Mutex<EffectiveConfigRegistry>> = OnceLock::new();

fn effective_config_registry() -> &'static Mutex<EffectiveConfigRegistry> {
    once_get_or_init(&EFFECTIVE_CONFIGS, || Mutex::new(HashMap::new()))
}

pub fn effective_config_entry<T: 'static>(default_ron: &str) -> &'static EffectiveConfigEntry {
    let registry = effective_config_registry();
    let mut registry = lock_mutex(registry);
    if let Some(entry) = registry.get(&TypeId::of::<T>()) {
        return entry;
    }

    let entry = Box::leak(Box::new(EffectiveConfigEntry::new(default_ron)));
    registry.insert(TypeId::of::<T>(), entry);
    entry
}

pub fn set_effective_config_ron<T: 'static>(ron: &str) {
    effective_config_entry::<T>(ron).set(ron);
}

pub fn with_codec_for_encode<C, R, B, F>(
    state: &'static CodecState<C>,
    config_entry: &EffectiveConfigEntry,
    build: B,
    f: F,
) -> Result<R, EncodeError>
where
    B: FnOnce(&str) -> CuResult<C>,
    F: FnOnce(&mut C) -> Result<R, EncodeError>,
{
    let version = config_entry.version();
    let mut guard = lock_mutex(&state.inner);
    if guard
        .as_ref()
        .is_none_or(|(cached_version, _)| *cached_version != version)
    {
        let effective_config_ron = config_entry.ron();
        let codec = build(&effective_config_ron)
            .map_err(|err| EncodeError::OtherString(err.to_string()))?;
        *guard = Some((version, codec));
    }
    let (_, codec) = guard
        .as_mut()
        .expect("codec state must be initialized after build");
    f(codec)
}

pub fn with_codec_for_decode<C, R, B, F>(
    state: &'static CodecState<C>,
    config_entry: &EffectiveConfigEntry,
    build: B,
    f: F,
) -> Result<R, DecodeError>
where
    B: FnOnce(&str) -> CuResult<C>,
    F: FnOnce(&mut C) -> Result<R, DecodeError>,
{
    let version = config_entry.version();
    let mut guard = lock_mutex(&state.inner);
    if guard
        .as_ref()
        .is_none_or(|(cached_version, _)| *cached_version != version)
    {
        let effective_config_ron = config_entry.ron();
        let codec = build(&effective_config_ron)
            .map_err(|err| DecodeError::OtherString(err.to_string()))?;
        *guard = Some((version, codec));
    }
    let (_, codec) = guard
        .as_mut()
        .expect("codec state must be initialized after build");
    f(codec)
}

pub fn resolve_task_output_codec<'a>(
    config: &'a CuConfig,
    mission_id: Option<&str>,
    task_id: &str,
    msg_type: &str,
) -> CuResult<Option<&'a LoggingCodecSpec>> {
    let node = config.find_task_node(mission_id, task_id).ok_or_else(|| {
        CuError::from(format!(
            "Could not find task '{task_id}' while resolving log codec for '{msg_type}'."
        ))
    })?;

    let codec_id = node
        .get_logging()
        .and_then(|logging| logging.codec_for_msg_type(msg_type));
    let Some(codec_id) = codec_id else {
        return Ok(None);
    };

    config
        .find_logging_codec_spec(codec_id)
        .map(Some)
        .ok_or_else(|| {
            CuError::from(format!(
                "Task '{task_id}' binds output '{msg_type}' to unknown logging codec '{codec_id}'."
            ))
        })
}

pub fn instantiate_codec<C, P>(
    effective_config_ron: &str,
    mission_id: Option<&str>,
    task_id: &str,
    msg_type: &str,
    expected_type_path: &str,
) -> CuResult<C>
where
    C: CuLogCodec<P>,
    P: CuMsgPayload,
{
    let config = CuConfig::deserialize_ron(effective_config_ron)?;
    let spec = resolve_task_output_codec(&config, mission_id, task_id, msg_type)?.ok_or_else(
        || {
            CuError::from(format!(
                "Task '{task_id}' output '{msg_type}' has no configured logging codec in the effective config."
            ))
        },
    )?;

    if spec.type_ != expected_type_path {
        return Err(CuError::from(format!(
            "Task '{task_id}' output '{msg_type}' resolved logging codec type '{}' but '{}' was compiled for this slot.",
            spec.type_, expected_type_path
        )));
    }

    let codec_config = deserialize_codec_config::<C, P>(spec.config.as_ref())?;
    C::new(codec_config)
}

pub fn deserialize_codec_config<C, P>(config: Option<&ComponentConfig>) -> CuResult<C::Config>
where
    C: CuLogCodec<P>,
    P: CuMsgPayload,
{
    match config {
        Some(config) => config.deserialize_into::<C::Config>().map_err(|err| {
            CuError::from(format!(
                "Failed to deserialize logging codec config for payload '{}': {err}",
                core::any::type_name::<P>()
            ))
        }),
        None => Ok(C::Config::default()),
    }
}

pub fn encode_msg_with_codec<T, C, E>(
    msg: &CuMsg<T>,
    codec: &mut C,
    encoder: &mut E,
) -> Result<(), EncodeError>
where
    T: CuMsgPayload,
    C: CuLogCodec<T>,
    E: Encoder,
{
    // NOTE: like `Encode for CuStampedData` in cutask.rs, this is generic over the
    // payload type, so it cannot itself consult `HandleContent` policy via the
    // autoref-specialization pattern. The codegen-emitted per-slot encoder calls
    // `cu29::cutask::encode_metadata_only` instead when bytes should be skipped.
    match msg.payload() {
        None => {
            0u8.encode(encoder)?;
        }
        Some(payload) => {
            1u8.encode(encoder)?;
            encode_payload_with_codec(payload, codec, encoder)?;
        }
    }
    msg.tov.encode(encoder)?;
    msg.metadata.encode(encoder)?;
    Ok(())
}

pub fn decode_msg_with_codec<T, C, D>(
    decoder: &mut D,
    codec: &mut C,
) -> Result<CuMsg<T>, DecodeError>
where
    T: CuMsgPayload,
    C: CuLogCodec<T>,
    D: Decoder<Context = ()>,
{
    let present: u8 = Decode::decode(decoder)?;
    let payload = match present {
        0 => None,
        1 => Some(decode_framed_payload(decoder, codec)?),
        value => {
            return Err(DecodeError::OtherString(format!(
                "Invalid CuMsg presence tag {value} for payload '{}'",
                core::any::type_name::<T>()
            )));
        }
    };
    let tov: Tov = Decode::decode(decoder)?;
    let metadata: CuMsgMetadata = Decode::decode(decoder)?;
    Ok(CuMsg::from_parts(payload, tov, metadata))
}

/// Encodes one present payload with its configured codec. Presence and common
/// metadata are carried by the generated CopperList metadata block.
#[doc(hidden)]
pub fn encode_payload_with_codec<T, C, E>(
    payload: &T,
    codec: &mut C,
    encoder: &mut E,
) -> Result<(), EncodeError>
where
    T: CuMsgPayload,
    C: CuLogCodec<T>,
    E: Encoder,
{
    let encoded_start = observed_encode_bytes();
    let handle_start = crate::monitoring::current_payload_handle_bytes();
    let source_handle_bytes = codec.source_payload_handle_bytes(payload);
    if source_handle_bytes > 0 {
        crate::monitoring::record_payload_handle_bytes(source_handle_bytes);
    }
    encode_framed_payload(payload, codec, encoder)?;
    let encoded_bytes = observed_encode_bytes().saturating_sub(encoded_start);
    let handle_bytes =
        crate::monitoring::current_payload_handle_bytes().saturating_sub(handle_start);
    crate::monitoring::record_current_slot_payload_io_stats(
        core::mem::size_of::<T>(),
        encoded_bytes,
        handle_bytes,
    );
    Ok(())
}

/// Decodes one payload with its configured codec after generated metadata has
/// declared that the payload bytes are present.
#[doc(hidden)]
pub fn decode_payload_with_codec<T, C, D>(decoder: &mut D, codec: &mut C) -> Result<T, DecodeError>
where
    T: CuMsgPayload,
    C: CuLogCodec<T>,
    D: Decoder<Context = ()>,
{
    decode_framed_payload(decoder, codec)
}

const CODEC_LENGTH_BYTES: usize = 4;

fn encode_framed_payload<T, C, E>(
    payload: &T,
    codec: &mut C,
    encoder: &mut E,
) -> Result<(), EncodeError>
where
    T: CuMsgPayload,
    C: CuLogCodec<T>,
    E: Encoder,
{
    let header = encoder.writer().position()?;
    encoder.writer().write(&[0; CODEC_LENGTH_BYTES])?;
    let start = encoder.writer().position()?;
    codec.encode_payload(payload, encoder)?;
    let len = encoder
        .writer()
        .position()?
        .checked_sub(start)
        .and_then(|len| u32::try_from(len).ok())
        .ok_or(EncodeError::Other(
            "Logging codec payload exceeds its frame length",
        ))?;
    encoder.writer().overwrite(header, &len.to_le_bytes())
}

/// Restricts every reader operation, including lookahead, to one codec payload.
struct PayloadReader<'a, R> {
    inner: &'a mut R,
    remaining: usize,
    overconsumed: bool,
}

impl<R: Reader> Reader for PayloadReader<'_, R> {
    fn read_some(&mut self, bytes: &mut [u8]) -> Result<usize, DecodeError> {
        let len = bytes.len().min(self.remaining);
        if len == 0 {
            return Ok(0);
        }
        let read = self.inner.read_some(&mut bytes[..len])?;
        self.remaining -= read;
        Ok(read)
    }

    fn read(&mut self, bytes: &mut [u8]) -> Result<(), DecodeError> {
        if bytes.len() > self.remaining {
            return Err(DecodeError::UnexpectedEnd {
                additional: bytes.len() - self.remaining,
            });
        }
        self.inner.read(bytes)?;
        self.remaining -= bytes.len();
        Ok(())
    }

    fn peek_read(&mut self, len: usize) -> Option<&[u8]> {
        if len > self.remaining {
            return None;
        }
        self.inner.peek_read(len)
    }

    fn consume(&mut self, len: usize) {
        if len > self.remaining {
            self.overconsumed = true;
        }
        let len = len.min(self.remaining);
        self.inner.consume(len);
        self.remaining -= len;
    }
}

fn decode_framed_payload<T, C, D>(decoder: &mut D, codec: &mut C) -> Result<T, DecodeError>
where
    T: CuMsgPayload,
    C: CuLogCodec<T>,
    D: Decoder<Context = ()>,
{
    let mut length = [0; CODEC_LENGTH_BYTES];
    decoder.claim_bytes_read(CODEC_LENGTH_BYTES)?;
    decoder.reader().read(&mut length)?;
    let len =
        usize::try_from(u32::from_le_bytes(length)).map_err(|_| DecodeError::LimitExceeded)?;
    // Charge the enclosing decoder so successive frames share its byte limit.
    decoder.claim_bytes_read(len)?;
    let config = *decoder.config();
    let mut reader = PayloadReader {
        inner: decoder.reader(),
        remaining: len,
        overconsumed: false,
    };
    let payload = codec.decode_payload(&mut DecoderImpl::new(&mut reader, config, ()))?;
    if reader.overconsumed {
        return Err(DecodeError::Other(
            "Logging codec consumed beyond its payload frame",
        ));
    }
    if reader.remaining != 0 {
        return Err(DecodeError::Other(
            "Logging codec did not consume its entire payload frame",
        ));
    }
    Ok(payload)
}

#[cfg(feature = "std")]
fn read_next_entry<T: Decode<()>>(src: &mut impl Read) -> CuResult<Option<T>> {
    match decode_from_std_read::<T, _, _>(src, standard()) {
        Ok(entry) => Ok(Some(entry)),
        Err(DecodeError::UnexpectedEnd { .. }) => Ok(None),
        Err(DecodeError::Io { inner, .. }) if inner.kind() == std::io::ErrorKind::UnexpectedEof => {
            Ok(None)
        }
        Err(err) => Err(CuError::new_with_cause(
            "Failed to decode runtime lifecycle entry while loading effective log config",
            err,
        )),
    }
}

#[cfg(feature = "std")]
pub fn read_effective_config_ron_from_log(log_base: &Path) -> CuResult<Option<String>> {
    let logger = UnifiedLoggerBuilder::new()
        .file_base_name(log_base)
        .build()
        .map_err(|err| {
            CuError::new_with_cause(
                &format!(
                    "Failed to open Copper log '{}' while loading effective log config",
                    log_base.display()
                ),
                err,
            )
        })?;
    let UnifiedLogger::Read(read_logger) = logger else {
        return Err(CuError::from(
            "Expected readable unified logger while loading effective log config",
        ));
    };

    let mut reader =
        UnifiedLoggerIOReader::new(read_logger, cu29_traits::UnifiedLogType::RuntimeLifecycle);
    while let Some(record) = read_next_entry::<RuntimeLifecycleRecord>(&mut reader)? {
        if let RuntimeLifecycleEvent::Instantiated {
            effective_config_ron,
            ..
        } = record.event
        {
            return Ok(Some(effective_config_ron));
        }
    }

    Ok(None)
}

#[cfg(feature = "std")]
pub fn seed_effective_config_from_log<T: 'static>(log_base: &Path) -> CuResult<Option<String>> {
    let effective_config_ron = read_effective_config_ron_from_log(log_base)?;
    if let Some(ref ron) = effective_config_ron {
        set_effective_config_ron::<T>(ron);
    }
    Ok(effective_config_ron)
}

#[cfg(test)]
mod framing_tests {
    use super::*;
    use bincode::config::standard;
    use bincode::de::read::SliceReader;
    use bincode::enc::EncoderImpl;
    use bincode::enc::write::SliceWriter;
    use cu29_traits::ObservedWriter;

    #[derive(Default)]
    struct BytesCodec {
        encodes: usize,
    }

    impl CuLogCodec<u8> for BytesCodec {
        type Config = ();

        fn new(_: ()) -> CuResult<Self> {
            Ok(Self::default())
        }

        fn source_payload_handle_bytes(&self, _: &u8) -> usize {
            0
        }

        fn encode_payload<E: Encoder>(
            &mut self,
            payload: &u8,
            encoder: &mut E,
        ) -> Result<(), EncodeError> {
            self.encodes += 1;
            encoder.writer().write(&[*payload; 3])
        }

        fn decode_payload<D: Decoder<Context = ()>>(
            &mut self,
            decoder: &mut D,
        ) -> Result<u8, DecodeError> {
            // A codec may look ahead, but cannot see the adjacent payload.
            assert!(decoder.reader().peek_read(4).is_none());
            assert!(matches!(
                decoder.reader().read(&mut [0; 4]),
                Err(DecodeError::UnexpectedEnd { .. })
            ));
            let mut bytes = [0; 3];
            decoder.reader().read(&mut bytes)?;
            assert!(decoder.reader().peek_read(1).is_none());
            Ok(bytes[0])
        }
    }

    #[test]
    fn codec_frames_encode_once_in_fixed_storage_and_count_header_once() {
        let mut storage = [0; 15];
        let mut codec = BytesCodec::default();
        cu29_traits::begin_observed_encode();
        let mut encoder = EncoderImpl::new(
            ObservedWriter::new(SliceWriter::new(&mut storage)),
            standard(),
        );
        encode_payload_with_codec(&7, &mut codec, &mut encoder).unwrap();
        encode_payload_with_codec(&9, &mut codec, &mut encoder).unwrap();
        42u8.encode(&mut encoder).unwrap();
        assert_eq!(encoder.writer().position().unwrap(), 15);
        assert_eq!(cu29_traits::finish_observed_encode(), 15);
        assert_eq!(codec.encodes, 2);
        assert_eq!(storage, [3, 0, 0, 0, 7, 7, 7, 3, 0, 0, 0, 9, 9, 9, 42]);
        let mut decoder = DecoderImpl::new(SliceReader::new(&storage), standard(), ());
        assert_eq!(
            decode_payload_with_codec(&mut decoder, &mut codec).unwrap(),
            7
        );
        assert_eq!(
            decode_payload_with_codec(&mut decoder, &mut codec).unwrap(),
            9
        );
        assert_eq!(u8::decode(&mut decoder).unwrap(), 42);
    }

    #[test]
    fn codec_frames_preserve_message_metadata_and_absent_payloads() {
        let mut storage = [0; 256];
        let mut codec = BytesCodec::default();
        let mut present = CuMsg::new(Some(7u8));
        present.tov = Tov::Time(cu29_clock::CuTime::from_nanos(123));
        let absent = CuMsg::<u8>::default();
        let mut encoder = EncoderImpl::new(SliceWriter::new(&mut storage), standard());
        encode_msg_with_codec(&present, &mut codec, &mut encoder).unwrap();
        encode_msg_with_codec(&absent, &mut codec, &mut encoder).unwrap();
        let len = encoder.writer().position().unwrap();
        assert_eq!(codec.encodes, 1);
        let mut decoder = DecoderImpl::new(SliceReader::new(&storage[..len]), standard(), ());
        let decoded = decode_msg_with_codec(&mut decoder, &mut codec).unwrap();
        assert_eq!(decoded.payload(), Some(&7));
        assert_eq!(decoded.tov, present.tov);
        assert!(
            decode_msg_with_codec(&mut decoder, &mut codec)
                .unwrap()
                .payload()
                .is_none()
        );
        assert!(decoder.reader().peek_read(1).is_none());
    }

    #[test]
    fn codec_frame_limits_accumulate_in_outer_decoder() {
        let bytes = [3, 0, 0, 0, 7, 7, 7, 3, 0, 0, 0, 9, 9, 9];
        let mut codec = BytesCodec::default();
        let mut decoder =
            DecoderImpl::new(SliceReader::new(&bytes), standard().with_limit::<13>(), ());
        assert_eq!(
            decode_payload_with_codec(&mut decoder, &mut codec).unwrap(),
            7
        );
        assert!(matches!(
            decode_payload_with_codec(&mut decoder, &mut codec),
            Err(DecodeError::LimitExceeded)
        ));
    }

    #[test]
    fn codec_frame_truncation_and_short_destination_are_errors() {
        let bytes = [3, 0, 0, 0, 7, 7, 7];
        let mut codec = BytesCodec::default();
        for len in 0..bytes.len() {
            let mut decoder = DecoderImpl::new(SliceReader::new(&bytes[..len]), standard(), ());
            assert!(matches!(
                decode_payload_with_codec(&mut decoder, &mut codec),
                Err(DecodeError::UnexpectedEnd { .. })
            ));
        }
        let mut storage = [0; 6];
        let mut encoder = EncoderImpl::new(SliceWriter::new(&mut storage), standard());
        assert!(matches!(
            encode_payload_with_codec(&7, &mut codec, &mut encoder),
            Err(EncodeError::UnexpectedEnd)
        ));
        assert_eq!(&storage[..4], &[0; 4]);
    }

    #[test]
    fn payload_reader_caps_consume_and_preserves_following_bytes() {
        let mut source = SliceReader::new(&[7, 8, 9]);
        let mut reader = PayloadReader {
            inner: &mut source,
            remaining: 2,
            overconsumed: false,
        };
        assert_eq!(reader.peek_read(2), Some(&[7, 8][..]));
        reader.consume(usize::MAX);
        assert!(reader.overconsumed);
        assert_eq!(reader.remaining, 0);
        assert_eq!(source.peek_read(1), Some(&[9][..]));
    }

    #[test]
    fn codec_frame_rejects_unconsumed_payload_bytes() {
        // BytesCodec consumes three bytes; the fourth belongs to this frame.
        let mut source = SliceReader::new(&[4, 0, 0, 0, 7, 7, 7, 8, 9]);
        struct ShortCodec;
        impl CuLogCodec<u8> for ShortCodec {
            type Config = ();
            fn new(_: ()) -> CuResult<Self> {
                Ok(Self)
            }
            fn source_payload_handle_bytes(&self, _: &u8) -> usize {
                0
            }
            fn encode_payload<E: Encoder>(&mut self, _: &u8, _: &mut E) -> Result<(), EncodeError> {
                Ok(())
            }
            fn decode_payload<D: Decoder<Context = ()>>(
                &mut self,
                decoder: &mut D,
            ) -> Result<u8, DecodeError> {
                let mut bytes = [0; 3];
                decoder.reader().read(&mut bytes)?;
                Ok(bytes[0])
            }
        }
        let mut decoder = DecoderImpl::new(&mut source, standard(), ());
        assert!(matches!(
            decode_payload_with_codec(&mut decoder, &mut ShortCodec),
            Err(DecodeError::Other(
                "Logging codec did not consume its entire payload frame"
            ))
        ));
        assert_eq!(source.peek_read(2), Some(&[8, 9][..]));
    }

    #[cfg(feature = "std")]
    #[test]
    fn codec_frames_decode_from_non_peekable_reader() {
        let bytes = [3, 0, 0, 0, 7, 7, 7, 42];
        let mut source = &bytes[..];
        struct Frame;
        impl Decode<()> for Frame {
            fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
                assert_eq!(
                    decode_payload_with_codec(decoder, &mut BytesCodec::default())?,
                    7
                );
                Ok(Self)
            }
        }
        bincode::decode_from_std_read::<Frame, _, _>(&mut source, standard()).unwrap();
        assert_eq!(source, &[42]);
    }
}
