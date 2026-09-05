//! Resolved stream plans and self-describing session manifests.

use crate::{
    ContinuousSenderConfig, DensityThreshold, EncodingSymbolId, Error, Field,
    FiniteObjectSenderConfig, Lane, LogStreamSenderConfig, PACKET_HEADER_LEN, RecordKind,
    RecoverySenderConfig, Result, RlcConfig, StreamIdentity,
    decode_record as decode_semantic_record, encode_record,
};
use alloc::{
    string::{String, ToString},
    vec::Vec,
};
use bincode::{Decode, Encode};
use cu29_runtime::config::{LogStreamDestinationConfig, LogStreamRepairDensity, LogStreamRlcField};
use cu29_traits::TaskOutputSpec;

pub const SESSION_MANIFEST_VERSION: u16 = 2;

/// Link and codec policy after RON validation and MTU resolution.
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode)]
pub struct LogStreamPlan {
    pub destination_id: String,
    pub mtu_bytes: u16,
    pub symbol_size: u16,
    pub bitrate_bps: u64,
    pub memory_budget_kib: u32,
    pub max_latency_ms: u32,
    pub burst_packets: u32,
    pub continuous: ResolvedContinuousFec,
    pub objects: ResolvedObjectFec,
    pub content: ResolvedContentPolicy,
    pub max_record_bytes: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub struct ResolvedContinuousFec {
    pub field: ResolvedRlcField,
    pub window_symbols: u16,
    pub repair_every_source_symbols: u16,
    pub repair_density: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub enum ResolvedRlcField {
    Gf2,
    Gf256,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub struct ResolvedObjectFec {
    pub max_object_bytes: u64,
    pub repair_symbols_per_block: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Encode, Decode)]
pub struct ResolvedContentPolicy {
    pub archive: bool,
    pub live_viz: bool,
    pub anchor_interval: u32,
}

/// Minimal application schema required to interpret recovered CopperList slots.
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode)]
pub struct ApplicationSchema {
    pub outputs: Vec<ApplicationOutputSchema>,
    /// Per-output deterministic ABI; empty means full capture.
    pub reconstruction: Vec<Option<u32>>,
}

#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode)]
pub struct ApplicationOutputSchema {
    pub task_id: String,
    pub message_type: String,
    pub payload_type: String,
}

impl ApplicationSchema {
    pub fn from_output_specs(specs: &[TaskOutputSpec]) -> Self {
        Self {
            reconstruction: Vec::new(),
            outputs: specs
                .iter()
                .map(|spec| ApplicationOutputSchema {
                    task_id: spec.task_id.into(),
                    message_type: spec.msg_type.into(),
                    payload_type: spec.payload_type_path().into(),
                })
                .collect(),
        }
    }
}

/// Versioned control object from which a receiver constructs its decoders.
#[derive(Clone, Debug, PartialEq, Eq, Encode, Decode)]
pub struct SessionManifest {
    pub version: u16,
    pub identity: StreamIdentity,
    pub plan: LogStreamPlan,
    pub application_schema: ApplicationSchema,
}

impl SessionManifest {
    pub fn new(
        identity: StreamIdentity,
        plan: LogStreamPlan,
        application_schema: ApplicationSchema,
    ) -> Self {
        Self {
            version: SESSION_MANIFEST_VERSION,
            identity,
            plan,
            application_schema,
        }
    }

    pub fn encode_record(&self) -> Result<Vec<u8>> {
        let payload = bincode::encode_to_vec(self, bincode::config::standard())
            .map_err(|error| Error::Codec(error.to_string()))?;
        encode_record(RecordKind::Manifest, 0, &payload)
    }

    pub fn decode_record(record: &[u8]) -> Result<Self> {
        let record = decode_semantic_record(record)?;
        if record.kind != RecordKind::Manifest {
            return Err(Error::InvalidConfig("record is not a session manifest"));
        }
        let (manifest, consumed): (Self, usize) =
            bincode::decode_from_slice(record.payload, bincode::config::standard())
                .map_err(|error| Error::Codec(error.to_string()))?;
        if consumed != record.payload.len() {
            return Err(Error::Codec("session manifest has trailing bytes".into()));
        }
        if manifest.version != SESSION_MANIFEST_VERSION {
            return Err(Error::UnsupportedManifestVersion(manifest.version));
        }
        manifest.plan.validate()?;
        Ok(manifest)
    }
}

impl LogStreamPlan {
    /// Resolves one RON destination into the exact values carried by the manifest.
    pub fn resolve(config: &LogStreamDestinationConfig) -> Result<Self> {
        let symbol_size = config
            .link
            .mtu_bytes
            .checked_sub(PACKET_HEADER_LEN as u16)
            .ok_or(Error::InvalidConfig("MTU does not fit the packet header"))?;
        if usize::from(symbol_size) > crate::DEFAULT_MAX_SYMBOL_SIZE {
            return Err(Error::InvalidConfig(
                "MTU exceeds the generated sender symbol capacity",
            ));
        }
        if usize::from(config.fec.continuous.window_symbols) > crate::DEFAULT_MAX_WINDOW_SYMBOLS {
            return Err(Error::InvalidConfig(
                "RLC window exceeds the generated sender capacity",
            ));
        }
        let field = match config.fec.continuous.field {
            LogStreamRlcField::Gf2 => ResolvedRlcField::Gf2,
            LogStreamRlcField::Gf256 => ResolvedRlcField::Gf256,
        };
        let repair_density = match config.fec.continuous.repair_density {
            LogStreamRepairDensity::Full => DensityThreshold::FULL.get(),
            LogStreamRepairDensity::Threshold(value) => value,
        };
        DensityThreshold::new(repair_density)?;
        let plan = Self {
            destination_id: config.id.clone(),
            mtu_bytes: config.link.mtu_bytes,
            symbol_size,
            bitrate_bps: config.link.bitrate_bps,
            memory_budget_kib: config.link.memory_budget_kib,
            max_latency_ms: config.link.max_latency_ms,
            burst_packets: config.link.burst_packets,
            continuous: ResolvedContinuousFec {
                field,
                window_symbols: config.fec.continuous.window_symbols,
                repair_every_source_symbols: config.fec.continuous.repair_every_source_symbols,
                repair_density,
            },
            objects: ResolvedObjectFec {
                max_object_bytes: config.fec.objects.max_object_bytes,
                repair_symbols_per_block: config.fec.objects.repair_symbols_per_block,
            },
            content: ResolvedContentPolicy {
                archive: config.content.archive,
                live_viz: config.content.live_viz,
                anchor_interval: config.content.anchor_interval,
            },
            max_record_bytes: config.max_record_bytes,
        };
        plan.validate()?;
        Ok(plan)
    }

    /// Validates a resolved plan, including plans decoded from an untrusted manifest.
    pub fn validate(&self) -> Result<()> {
        let expected_mtu = PACKET_HEADER_LEN
            .checked_add(usize::from(self.symbol_size))
            .ok_or(Error::InvalidConfig("resolved MTU overflow"))?;
        if usize::from(self.mtu_bytes) != expected_mtu {
            return Err(Error::InvalidConfig(
                "resolved symbol size does not match the MTU",
            ));
        }
        if usize::from(self.symbol_size) <= crate::rlc::FRAGMENT_HEADER_LEN {
            return Err(Error::InvalidConfig(
                "MTU leaves no room for continuous record bytes",
            ));
        }
        if usize::from(self.symbol_size) > crate::DEFAULT_MAX_SYMBOL_SIZE
            || usize::from(self.continuous.window_symbols) > crate::DEFAULT_MAX_WINDOW_SYMBOLS
        {
            return Err(Error::InvalidConfig(
                "resolved FEC values exceed generated sender capacities",
            ));
        }
        if self.bitrate_bps == 0
            || self.memory_budget_kib == 0
            || self.max_latency_ms == 0
            || self.burst_packets == 0
            || self.continuous.window_symbols == 0
            || self.continuous.repair_every_source_symbols == 0
            || self.objects.max_object_bytes == 0
            || self.objects.repair_symbols_per_block == 0
            || self.content.anchor_interval == 0
            || self.max_record_bytes == 0
        {
            return Err(Error::InvalidConfig(
                "resolved stream plan contains a zero bound",
            ));
        }
        if !self.content.archive && !self.content.live_viz {
            return Err(Error::InvalidConfig(
                "resolved stream plan selects no content consumer",
            ));
        }
        DensityThreshold::new(self.continuous.repair_density)?;
        let max_record_bytes = usize::try_from(self.max_record_bytes)
            .map_err(|_| Error::InvalidConfig("record bound exceeds usize"))?;
        let rlc = self.rlc_config()?;
        let continuous_bytes = rlc
            .symbol_size()
            .checked_mul(rlc.window_symbols())
            .and_then(|bytes| bytes.checked_add(max_record_bytes))
            .and_then(|bytes| bytes.checked_add(usize::from(self.mtu_bytes)))
            .ok_or(Error::InvalidConfig("stream memory estimate overflow"))?;
        let memory_budget_bytes = usize::try_from(self.memory_budget_kib)
            .ok()
            .and_then(|kib| kib.checked_mul(1024))
            .ok_or(Error::InvalidConfig("stream memory budget overflow"))?;
        if continuous_bytes > memory_budget_bytes {
            return Err(Error::InvalidConfig(
                "continuous sender working set exceeds memory budget",
            ));
        }
        Ok(())
    }

    pub fn rlc_config(&self) -> Result<RlcConfig> {
        RlcConfig::new(
            usize::from(self.symbol_size),
            usize::from(self.continuous.window_symbols),
            self.continuous.field.into(),
        )
        .map_err(Into::into)
    }

    /// Builds both sender lanes and their canonical repeated manifest.
    pub fn sender_config(
        &self,
        identity: StreamIdentity,
        schema: ApplicationSchema,
    ) -> Result<LogStreamSenderConfig> {
        self.validate()?;
        let max_record_bytes = usize::try_from(self.max_record_bytes)
            .map_err(|_| Error::InvalidConfig("record bound exceeds usize"))?;
        let manifest = SessionManifest::new(identity, self.clone(), schema).encode_record()?;
        if manifest.len() as u64 > self.objects.max_object_bytes {
            return Err(Error::ObjectTooLarge {
                actual: manifest.len() as u64,
                maximum: self.objects.max_object_bytes,
            });
        }
        Ok(LogStreamSenderConfig {
            pacing: crate::PacingConfig {
                bitrate_bps: self.bitrate_bps,
                burst_packets: self.burst_packets,
                max_latency: cu29_clock::CuDuration::from_millis(u64::from(self.max_latency_ms)),
                memory_budget_bytes: self.memory_budget_kib as usize * 1024,
            },
            continuous: ContinuousSenderConfig {
                identity,
                first_packet_sequence: 0,
                lane: Lane::ReplayCritical,
                fec: self.rlc_config()?,
                max_record_bytes,
                initial_esi: EncodingSymbolId::new(0),
                repair_every_source_symbols: usize::from(
                    self.continuous.repair_every_source_symbols,
                ),
                first_repair_key: 0,
                repair_density: DensityThreshold::new(self.continuous.repair_density)?,
            },
            recovery: RecoverySenderConfig {
                finite: FiniteObjectSenderConfig {
                    identity,
                    first_packet_sequence: 0,
                    lane: Lane::Control,
                    symbol_size: self.symbol_size,
                    max_object_bytes: self.objects.max_object_bytes,
                    repair_symbols_per_block: self.objects.repair_symbols_per_block,
                },
                manifest_record: manifest,
                anchor_interval: self.content.anchor_interval,
            },
        })
    }
}

impl From<ResolvedRlcField> for Field {
    fn from(field: ResolvedRlcField) -> Self {
        match field {
            ResolvedRlcField::Gf2 => Self::Gf2,
            ResolvedRlcField::Gf256 => Self::Gf256,
        }
    }
}

/// Creates a process-local session identity without adding configuration or environment inputs.
#[cfg(feature = "std")]
pub fn new_session_id() -> [u8; 16] {
    use core::sync::atomic::{AtomicU64, Ordering};
    use std::time::{SystemTime, UNIX_EPOCH};

    static NEXT_SESSION: AtomicU64 = AtomicU64::new(0);
    let counter = NEXT_SESSION.fetch_add(1, Ordering::Relaxed);
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let mut hasher = blake3::Hasher::new();
    hasher.update(&now.to_le_bytes());
    hasher.update(&counter.to_le_bytes());
    hasher.update(&std::process::id().to_le_bytes());
    hasher.finalize().as_bytes()[..16].try_into().unwrap()
}
