#[cfg(not(feature = "std"))]
extern crate alloc;

use crate::config::{ComponentConfig, CuConfig, LoggingCodecSpec};
use crate::cutask::{CuMsg, CuMsgMetadata, CuMsgPayload};
use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
#[cfg(feature = "std")]
use bincode::config::standard;
use bincode::de::{Decode, Decoder};
#[cfg(feature = "std")]
use bincode::decode_from_std_read;
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

#[cfg(not(feature = "std"))]
mod imp {
    pub use spin::Mutex;
    pub use spin::once::Once as OnceLock;
}

#[cfg(feature = "std")]
mod imp {
    pub use std::sync::{Mutex, OnceLock};
}

use imp::*;

#[cfg(feature = "std")]
use crate::curuntime::{RuntimeLifecycleEvent, RuntimeLifecycleRecord};
#[cfg(feature = "std")]
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};

#[cfg(feature = "std")]
fn lock_mutex<T>(m: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

#[cfg(not(feature = "std"))]
fn lock_mutex<T>(m: &Mutex<T>) -> spin::MutexGuard<'_, T> {
    m.lock()
}

pub trait CuLogCodec<P: CuMsgPayload>: 'static {
    type Config: DeserializeOwned + Default;

    fn new(config: Self::Config) -> CuResult<Self>
    where
        Self: Sized;

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

#[cfg(feature = "std")]
fn effective_config_registry() -> &'static Mutex<EffectiveConfigRegistry> {
    EFFECTIVE_CONFIGS.get_or_init(|| Mutex::new(HashMap::new()))
}

#[cfg(not(feature = "std"))]
fn effective_config_registry() -> &'static Mutex<EffectiveConfigRegistry> {
    EFFECTIVE_CONFIGS.call_once(|| Mutex::new(HashMap::new()))
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
    match msg.payload() {
        None => {
            0u8.encode(encoder)?;
        }
        Some(payload) => {
            1u8.encode(encoder)?;
            let encoded_start = observed_encode_bytes();
            let handle_start = crate::monitoring::current_payload_handle_bytes();
            codec.encode_payload(payload, encoder)?;
            let encoded_bytes = observed_encode_bytes().saturating_sub(encoded_start);
            let handle_bytes =
                crate::monitoring::current_payload_handle_bytes().saturating_sub(handle_start);
            crate::monitoring::record_current_slot_payload_io_stats(
                core::mem::size_of::<T>(),
                encoded_bytes,
                handle_bytes,
            );
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
        1 => Some(codec.decode_payload(decoder)?),
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
