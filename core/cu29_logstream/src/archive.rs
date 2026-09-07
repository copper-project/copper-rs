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
    /// Creates native CopperList, FrozenTasks and StreamContinuity sections.
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
        manifest.plan.validate()?;
        if manifest.version != crate::SESSION_MANIFEST_VERSION {
            return Err(Error::UnsupportedManifestVersion(manifest.version));
        }
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
        let logger = Arc::new(Mutex::new(logger));
        let mut archive = Self {
            copperlists: NativeStream::new(
                UnifiedLogType::CopperList,
                logger.clone(),
                section_bytes,
            )
            .map_err(io_error)?,
            keyframes: NativeStream::new(
                UnifiedLogType::FrozenTasks,
                logger.clone(),
                section_bytes,
            )
            .map_err(io_error)?,
            continuity: NativeStream::new(UnifiedLogType::StreamContinuity, logger, section_bytes)
                .map_err(io_error)?,
            identity: manifest.identity,
            manifest_digest: received.record().decoded().digest,
            manifest_object_id: received.record().decoded().object_id,
            next_id: 0,
            poisoned: false,
            payload: PhantomData,
            decode,
        };
        archive
            .continuity
            .log(&StreamContinuityRecord::Manifest {
                record: received.record().bytes(),
            })
            .map_err(io_error)?;
        Ok(archive)
    }

    /// Validates a typed CopperList once, appends its original canonical bytes,
    /// and returns the typed value for an in-process consumer. Keyframes are
    /// archived only after the router has verified their recovery point references.
    pub fn accept(&mut self, event: &SessionEventRef<'_>) -> Result<Option<CopperList<P>>> {
        self.accept_capture(event)
            .map(|capture| capture.map(|c| c.copperlist))
    }

    fn accept_capture(&mut self, event: &SessionEventRef<'_>) -> Result<Option<CapturedList<P>>> {
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
                if decoded.kind != RecordKind::CopperList || decoded.object_id != self.next_id {
                    return Err(Error::InconsistentObject);
                }
                let (capture, native) = (self.decode)(decoded.payload)?;
                let copperlist = &capture.copperlist;
                if copperlist.id != decoded.object_id {
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
                return Ok(Some(capture));
            }
            SessionEvent::Gap { gap, .. } => {
                if gap.first_id != self.next_id || gap.last_id < gap.first_id {
                    return Err(Error::InconsistentObject);
                }
                let next = gap
                    .last_id
                    .checked_add(1)
                    .ok_or(Error::InconsistentObject)?;
                let reason = match gap.reason {
                    crate::GapReason::RlcWindowExpired => SourceGapReason::RlcWindowExpired,
                    crate::GapReason::SessionEnded => SourceGapReason::SessionEnded,
                    crate::GapReason::LateJoin => SourceGapReason::LateJoin,
                    crate::GapReason::RecoveryPoint => SourceGapReason::RecoveryPoint,
                };
                self.continuity
                    .log(&StreamContinuityRecord::<&[u8]>::Gap {
                        first_id: gap.first_id,
                        last_id: gap.last_id,
                        reason,
                    })
                    .map_err(io_error)?;
                self.next_id = next;
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
            SessionEvent::Object { .. } => {}
        }
        Ok(None)
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

    pub fn finish(self) -> Result<()> {
        self.0.finish()
    }
}
