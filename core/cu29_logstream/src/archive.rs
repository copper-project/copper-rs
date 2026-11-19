//! Application-typed native archival. All work here is receiver-side.

use crate::capture::CapturedList;
use crate::{
    ApplicationSchema, Error, ReceivedManifest, RecordKind, Result, SessionEvent, SessionEventRef,
    StreamIdentity,
};
use bincode::{
    Encode,
    enc::{Encoder, write::Writer},
    error::EncodeError,
};
use cu29_runtime::{
    continuity::{SourceGapReason, StreamContinuityRecord},
    copperlist::CopperList,
};
use cu29_traits::{CopperListTuple, UnifiedLogType, WriteStream};
use cu29_unifiedlog::{
    LogStream, UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerWrite, memmap::MmapSectionStorage,
};
use std::{
    marker::PhantomData,
    path::Path,
    sync::{Arc, Mutex},
};

type ArchiveDecoder<P> = fn(&[u8]) -> Result<(CapturedList<P>, &[u8])>;

type NativeStream = LogStream<MmapSectionStorage, UnifiedLoggerWrite>;

/// One application-typed archive per `(session id, sender id)`. Pass all ordered
/// router events for that sender to `accept`. The expected schema comes from
/// `P`'s generated output specs, independently of the remote manifest.
/// `CaptureArchive` applies the generated selective codec for a live twin.
/// A write failure poisons the writer: do not retry a partially committed event.
pub struct NativeArchive<P: CopperListTuple> {
    copperlists: NativeStream,
    keyframes: NativeStream,
    continuity: NativeStream,
    structured: NativeStream,
    last_structured: Option<cu29_log::CuLogEntry>,
    identity: StreamIdentity,
    manifest_digest: [u8; 32],
    manifest_object_id: u64,
    next_id: u64,
    poisoned: bool,
    payload: PhantomData<P>,
    decode: ArchiveDecoder<P>,
}

struct CanonicalEntry<'a>(&'a [u8]);
impl Encode for CanonicalEntry<'_> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> core::result::Result<(), EncodeError> {
        encoder.writer().write(self.0)
    }
}

impl<P: CopperListTuple> NativeArchive<P> {
    /// Creates native message, task state, continuity and structured log sections.
    /// The section size must fit the largest accepted canonical entry.
    pub fn new(
        path: &Path,
        received: &ReceivedManifest,
        slab_bytes: usize,
        section_bytes: usize,
    ) -> Result<Self> {
        let expected = ApplicationSchema::from_output_specs(P::get_output_specs());
        Self::new_checked(
            path,
            received,
            slab_bytes,
            section_bytes,
            expected,
            |bytes| Ok((CapturedList::new(crate::decode_copperlist(bytes)?), bytes)),
        )
    }

    fn new_checked(
        path: &Path,
        received: &ReceivedManifest,
        slab_bytes: usize,
        section_bytes: usize,
        expected_schema: ApplicationSchema,
        decode: ArchiveDecoder<P>,
    ) -> Result<Self> {
        let manifest = received.manifest();
        manifest.requirements.validate()?;
        if manifest.application_schema != expected_schema {
            return Err(Error::InvalidConfig(
                "archive requires the matching application schema",
            ));
        }
        if section_bytes == 0 || section_bytes > slab_bytes {
            return Err(Error::InvalidConfig("archive section must fit in its slab"));
        }
        let logger = UnifiedLoggerBuilder::new()
            .file_base_name(path)
            .preallocated_size(slab_bytes)
            .write(true)
            .create(true)
            .build()
            .map_err(io_error)?;
        let UnifiedLogger::Write(logger) = logger else {
            unreachable!()
        };
        Self::new_with_logger(received, section_bytes, expected_schema, decode, logger)
    }

    fn new_with_logger(
        received: &ReceivedManifest,
        section_bytes: usize,
        expected_schema: ApplicationSchema,
        decode: ArchiveDecoder<P>,
        logger: UnifiedLoggerWrite,
    ) -> Result<Self> {
        let manifest = received.manifest();
        manifest.requirements.validate()?;
        if manifest.application_schema != expected_schema {
            return Err(Error::InvalidConfig(
                "archive requires the matching application schema",
            ));
        }
        let mut archive = Self::open_streams(
            logger,
            section_bytes,
            manifest.context,
            manifest.identity,
            decode,
        )?;
        archive.manifest_digest = received.record().decoded().digest;
        archive.manifest_object_id = received.record().decoded().object_id;
        archive
            .continuity
            .log(&StreamContinuityRecord::Manifest {
                record: received.record().bytes(),
            })
            .map_err(io_error)?;
        Ok(archive)
    }

    fn open_streams(
        logger: UnifiedLoggerWrite,
        section_bytes: usize,
        context: cu29_unifiedlog::SectionContext,
        identity: StreamIdentity,
        decode: ArchiveDecoder<P>,
    ) -> Result<Self> {
        let logger = Arc::new(Mutex::new(logger));
        Ok(Self {
            last_structured: None,
            copperlists: NativeStream::with_context(
                UnifiedLogType::CopperList,
                logger.clone(),
                section_bytes,
                context,
            )
            .map_err(io_error)?,
            keyframes: NativeStream::with_context(
                UnifiedLogType::FrozenTasks,
                logger.clone(),
                section_bytes,
                context,
            )
            .map_err(io_error)?,
            structured: NativeStream::with_context(
                UnifiedLogType::StructuredLogLine,
                logger.clone(),
                section_bytes,
                context,
            )
            .map_err(io_error)?,
            continuity: NativeStream::with_context(
                UnifiedLogType::StreamContinuity,
                logger,
                section_bytes,
                context,
            )
            .map_err(io_error)?,
            identity,
            manifest_digest: [0; 32],
            manifest_object_id: 0,
            next_id: 0,
            poisoned: false,
            payload: PhantomData,
            decode,
        })
    }

    /// Validates a typed CopperList once, appends its original canonical bytes,
    /// and returns the typed value for an in-process consumer. Keyframes are
    /// archived only after the router has verified their recovery point references.
    pub fn accept(&mut self, event: &SessionEventRef<'_>) -> Result<Option<CopperList<P>>> {
        self.accept_capture(event)
            .map(|capture| capture.map(|c| c.copperlist))
    }

    fn accept_capture(&mut self, event: &SessionEventRef<'_>) -> Result<Option<CapturedList<P>>> {
        self.last_structured = None;
        if self.poisoned {
            return Err(Error::InvalidConfig(
                "archive failed; close it before continuing",
            ));
        }
        let result = self.accept_inner(event);
        if result.is_err() {
            self.poisoned = true;
        }
        result
    }

    fn accept_inner(&mut self, event: &SessionEventRef<'_>) -> Result<Option<CapturedList<P>>> {
        let identity = match event {
            SessionEvent::Manifest(manifest) => manifest.manifest().identity,
            SessionEvent::ContinuousRecord { identity, .. }
            | SessionEvent::Gap { identity, .. }
            | SessionEvent::Object { identity, .. }
            | SessionEvent::VerifiedRecoveryPoint { identity, .. } => *identity,
        };
        if identity != self.identity {
            return Err(Error::InconsistentObject);
        }
        match event {
            SessionEvent::Manifest(manifest) => {
                if manifest.record().decoded().digest != self.manifest_digest
                    || manifest.record().decoded().object_id != self.manifest_object_id
                {
                    return Err(Error::InconsistentObject);
                }
            }
            SessionEvent::ContinuousRecord { record, .. } => {
                let decoded = record.decoded();
                if decoded.kind != RecordKind::CopperList {
                    return Err(Error::InconsistentObject);
                }
                return self
                    .append_list(decoded.object_id, decoded.payload)
                    .map(Some);
            }
            SessionEvent::Gap { gap, .. } => {
                let reason = match gap.reason {
                    crate::GapReason::RlcWindowExpired => SourceGapReason::RlcWindowExpired,
                    crate::GapReason::SessionEnded => SourceGapReason::SessionEnded,
                    crate::GapReason::LateJoin => SourceGapReason::LateJoin,
                    crate::GapReason::RecoveryPoint => SourceGapReason::RecoveryPoint,
                };
                self.append_gap(gap.first_id, gap.last_id, reason)?;
            }
            SessionEvent::VerifiedRecoveryPoint {
                recovery_point,
                recovery_record,
                keyframe,
                ..
            } => {
                let decoded = keyframe.decoded();
                if recovery_point.manifest_record_digest != self.manifest_digest
                    || recovery_point.manifest_object_id != self.manifest_object_id
                    || !recovery_point.references_keyframe(decoded)
                    || keyframe.keyframe_id() != Some(recovery_point.copperlist_id)
                    || recovery_record.decoded().kind != RecordKind::RecoveryPoint
                    || recovery_record.decoded().object_id != recovery_point.copperlist_id
                    || crate::decode_recovery_point(recovery_record.decoded().payload)?
                        != *recovery_point
                {
                    return Err(Error::InconsistentObject);
                }
                self.keyframes
                    .log(&CanonicalEntry(decoded.payload))
                    .map_err(io_error)?;
                self.continuity
                    .log(&StreamContinuityRecord::RecoveryPoint {
                        copperlist_id: recovery_point.copperlist_id,
                        record: recovery_record.bytes(),
                    })
                    .map_err(io_error)?;
            }
            SessionEvent::Object { record, .. } => {
                let decoded = record.decoded();
                if decoded.kind == RecordKind::StructuredLog {
                    self.last_structured = Some(self.append_log(decoded.payload)?);
                }
            }
        }
        Ok(None)
    }

    fn append_list(&mut self, id: u64, payload: &[u8]) -> Result<CapturedList<P>> {
        if id != self.next_id {
            return Err(Error::InconsistentObject);
        }
        let (capture, native) = (self.decode)(payload)?;
        let copperlist = &capture.copperlist;
        if copperlist.id != id {
            return Err(Error::InconsistentObject);
        }
        let next = self
            .next_id
            .checked_add(1)
            .ok_or(Error::InconsistentObject)?;
        self.copperlists
            .log(&CanonicalEntry(native))
            .map_err(io_error)?;
        self.next_id = next;
        Ok(capture)
    }
    fn append_log(&mut self, payload: &[u8]) -> Result<cu29_log::CuLogEntry> {
        let (entry, used): (cu29_log::CuLogEntry, usize) = bincode::decode_from_slice(
            payload,
            bincode::config::standard().with_limit::<4_194_304>(),
        )
        .map_err(|_| Error::InvalidConfig("invalid structured log entry"))?;
        if used != payload.len() {
            return Err(Error::InvalidConfig("trailing structured log bytes"));
        }
        self.structured
            .log(&CanonicalEntry(payload))
            .map_err(io_error)?;
        Ok(entry)
    }
    fn append_gap(&mut self, first_id: u64, last_id: u64, reason: SourceGapReason) -> Result<()> {
        if first_id != self.next_id || last_id < first_id {
            return Err(Error::InconsistentObject);
        }
        let next = last_id.checked_add(1).ok_or(Error::InconsistentObject)?;
        self.continuity
            .log(&StreamContinuityRecord::<&[u8]>::Gap {
                first_id,
                last_id,
                reason,
            })
            .map_err(io_error)?;
        self.next_id = next;
        Ok(())
    }
    /// Move the structured entry decoded during the most recent successful accept.
    /// Call before accepting another event; publication can reuse this owned value.
    pub fn take_structured_log(&mut self) -> Option<cu29_log::CuLogEntry> {
        self.last_structured.take()
    }

    /// Records the receiver's final known boundary and closes native sections.
    /// The router must first be finalized through any caller-known sender tail.
    pub fn finish(mut self) -> Result<()> {
        if self.poisoned {
            return Err(Error::InvalidConfig("archive failed before finalization"));
        }
        self.continuity
            .log(&StreamContinuityRecord::<&[u8]>::Finished {
                next_copperlist_id: self.next_id,
            })
            .map_err(io_error)
    }
}

fn io_error(error: impl core::fmt::Display) -> Error {
    Error::Codec(error.to_string())
}

/// Capture-aware native archive. Synthesized payloads never enter this writer.
/// Native CopperList metadata retains both original and captured presence planes;
/// StreamContinuity retains the reconstruction contract and source gaps.
pub struct CaptureArchive<P: crate::capture::CaptureDataSet>(NativeArchive<P>);
impl<P: crate::capture::CaptureDataSet> CaptureArchive<P> {
    pub(crate) fn from_session(
        session: &crate::protocol::CuTelemetrySession,
        section_bytes: usize,
        logger: UnifiedLoggerWrite,
    ) -> Result<Self> {
        if session.schema != P::stream_schema()
            || session.context.instance_id != session.identity.sender_id
        {
            return Err(Error::InvalidConfig(
                "telemetry session requires matching sender identity and application schema",
            ));
        }
        let mut archive = NativeArchive::open_streams(
            logger,
            section_bytes,
            session.context,
            session.identity,
            crate::capture::decode_capture,
        )?;
        let record =
            bincode::encode_to_vec(session, bincode::config::standard()).map_err(io_error)?;
        archive
            .continuity
            .log(&StreamContinuityRecord::TelemetrySession { record })
            .map_err(io_error)?;
        Ok(Self(archive))
    }

    pub(crate) fn list(&mut self, id: u64, capture: &[u8]) -> Result<CapturedList<P>> {
        self.0.append_list(id, capture)
    }
    pub(crate) fn log_entry(&mut self, entry: &[u8]) -> Result<cu29_log::CuLogEntry> {
        self.0.append_log(entry)
    }
    pub(crate) fn gap(&mut self, first: u64, last: u64, reason: SourceGapReason) -> Result<()> {
        self.0.append_gap(first, last, reason)
    }
    pub(crate) fn keyframe(
        &mut self,
        id: u64,
        state: &[u8],
    ) -> Result<cu29_runtime::curuntime::KeyFrame> {
        let frame = crate::decode_keyframe(state)?;
        if frame.culistid != id {
            return Err(Error::InconsistentObject);
        }
        self.0
            .keyframes
            .log(&CanonicalEntry(state))
            .map_err(io_error)?;
        self.0
            .continuity
            .log(&StreamContinuityRecord::<&[u8]>::TelemetryKeyFrame { copperlist_id: id })
            .map_err(io_error)?;
        Ok(frame)
    }

    /// Open capture streams after the application has sealed static metadata.
    #[doc(hidden)]
    pub fn new_with_logger(
        received: &ReceivedManifest,
        section_bytes: usize,
        logger: UnifiedLoggerWrite,
    ) -> Result<Self> {
        Ok(Self(NativeArchive::new_with_logger(
            received,
            section_bytes,
            P::stream_schema(),
            crate::capture::decode_capture,
            logger,
        )?))
    }

    pub fn new(
        path: &Path,
        received: &ReceivedManifest,
        slab_bytes: usize,
        section_bytes: usize,
    ) -> Result<Self> {
        Ok(Self(NativeArchive::new_checked(
            path,
            received,
            slab_bytes,
            section_bytes,
            P::stream_schema(),
            crate::capture::decode_capture,
        )?))
    }
    pub fn accept(
        &mut self,
        event: &SessionEventRef<'_>,
    ) -> Result<Option<crate::capture::CapturedList<P>>> {
        self.0.accept_capture(event)
    }

    /// Move the structured entry from the most recent successful accept.
    pub fn take_structured_log(&mut self) -> Option<cu29_log::CuLogEntry> {
        self.0.take_structured_log()
    }

    pub fn finish(self) -> Result<()> {
        self.0.finish()
    }
}
