//! Application-typed native archival. All work here is receiver-side.

use crate::{
    ApplicationSchema, Error, RecordKind, Result, SessionEvent, SessionManifest, StreamIdentity,
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

type ArchiveDecoder<P> =
    fn(&[u8]) -> Result<(CopperList<P>, &[u8], Option<crate::capture::CaptureProof>)>;

type NativeStream = LogStream<MmapSectionStorage, UnifiedLoggerWrite>;

/// One application-typed archive per `(session id, sender id)`. Pass all ordered
/// router events for that sender to `accept`. The expected schema comes from
/// `P`'s generated output specs, independently of the remote manifest.
/// Only the full captured native codec view is supported; hybrid views are deferred.
/// A write failure poisons the writer: do not retry a partially committed event.
pub struct NativeArchive<P: CopperListTuple> {
    copperlists: NativeStream,
    keyframes: NativeStream,
    continuity: NativeStream,
    identity: StreamIdentity,
    manifest: SessionManifest,
    next_id: u64,
    poisoned: bool,
    payload: PhantomData<P>,
    decode: ArchiveDecoder<P>,
    last_proof: Option<crate::capture::CaptureProof>,
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
        manifest: SessionManifest,
        slab_bytes: usize,
        section_bytes: usize,
    ) -> Result<Self> {
        let expected = ApplicationSchema::from_output_specs(P::get_output_specs());
        Self::new_checked(
            path,
            manifest,
            slab_bytes,
            section_bytes,
            expected,
            |bytes| Ok((crate::decode_copperlist(bytes)?, bytes, None)),
        )
    }

    fn new_checked(
        path: &Path,
        manifest: SessionManifest,
        slab_bytes: usize,
        section_bytes: usize,
        expected_schema: ApplicationSchema,
        decode: ArchiveDecoder<P>,
    ) -> Result<Self> {
        manifest.plan.validate()?;
        if manifest.version != crate::SESSION_MANIFEST_VERSION {
            return Err(Error::UnsupportedManifestVersion(manifest.version));
        }
        if manifest.application_schema != expected_schema || !manifest.plan.content.archive {
            return Err(Error::InvalidConfig(
                "archive requires the matching application schema and archive content policy",
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
            manifest,
            next_id: 0,
            poisoned: false,
            payload: PhantomData,
            decode,
            last_proof: None,
        };
        archive
            .continuity
            .log(&StreamContinuityRecord::Manifest {
                record: archive.manifest.encode_record()?,
            })
            .map_err(io_error)?;
        Ok(archive)
    }

    /// Validates a typed CopperList once, appends its original canonical bytes,
    /// and returns the typed value for an in-process consumer. Keyframes are
    /// archived only after the router has verified their anchor references.
    pub fn accept(&mut self, event: &SessionEvent) -> Result<Option<CopperList<P>>> {
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

    fn accept_inner(&mut self, event: &SessionEvent) -> Result<Option<CopperList<P>>> {
        let identity = match event {
            SessionEvent::Manifest(manifest) => manifest.identity,
            SessionEvent::ContinuousRecord { identity, .. }
            | SessionEvent::Gap { identity, .. }
            | SessionEvent::Object { identity, .. }
            | SessionEvent::VerifiedAnchor { identity, .. } => *identity,
        };
        if identity != self.identity {
            return Err(Error::InconsistentObject);
        }
        match event {
            SessionEvent::Manifest(manifest) => {
                if manifest != &self.manifest {
                    return Err(Error::InconsistentObject);
                }
            }
            SessionEvent::ContinuousRecord { record, .. } => {
                let decoded = record.decoded()?;
                if decoded.kind != RecordKind::CopperList || decoded.object_id != self.next_id {
                    return Err(Error::InconsistentObject);
                }
                let (copperlist, native, proof) = (self.decode)(decoded.payload)?;
                if copperlist.id != decoded.object_id {
                    return Err(Error::InconsistentObject);
                }
                let next = self
                    .next_id
                    .checked_add(1)
                    .ok_or(Error::InconsistentObject)?;
                if let Some(proof) = &proof {
                    self.continuity
                        .log(&StreamContinuityRecord::Capture {
                            copperlist_id: copperlist.id,
                            proof: bincode::encode_to_vec(proof, bincode::config::standard())
                                .map_err(io_error)?,
                        })
                        .map_err(io_error)?;
                }
                self.last_proof = proof;
                self.copperlists
                    .log(&CanonicalEntry(native))
                    .map_err(io_error)?;
                self.next_id = next;
                return Ok(Some(copperlist));
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
                    crate::GapReason::AnchorRecovery => SourceGapReason::AnchorRecovery,
                };
                self.continuity
                    .log(&StreamContinuityRecord::Gap {
                        first_id: gap.first_id,
                        last_id: gap.last_id,
                        reason,
                    })
                    .map_err(io_error)?;
                self.next_id = next;
            }
            SessionEvent::VerifiedAnchor {
                anchor, keyframe, ..
            } => {
                let decoded = keyframe.decoded()?;
                let manifest_record = self.manifest.encode_record()?;
                if !anchor.references_manifest(crate::decode_record(&manifest_record)?)
                    || !anchor.references_keyframe(decoded)
                    || crate::decode_keyframe(decoded.payload)?.culistid != anchor.copperlist_id
                {
                    return Err(Error::InconsistentObject);
                }
                self.keyframes
                    .log(&CanonicalEntry(decoded.payload))
                    .map_err(io_error)?;
                let record = crate::encode_record(
                    RecordKind::Anchor,
                    anchor.copperlist_id,
                    &crate::encode_anchor(anchor)?,
                )?;
                self.continuity
                    .log(&StreamContinuityRecord::Anchor {
                        copperlist_id: anchor.copperlist_id,
                        record,
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
            .log(&StreamContinuityRecord::Finished {
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
/// StreamContinuity retains each proof and the complete reconstruction contract.
pub struct CaptureArchive<P: crate::capture::CaptureDataSet>(NativeArchive<P>);
impl<P: crate::capture::CaptureDataSet> CaptureArchive<P> {
    pub fn new(
        path: &Path,
        manifest: SessionManifest,
        slab_bytes: usize,
        section_bytes: usize,
    ) -> Result<Self> {
        Ok(Self(NativeArchive::new_checked(
            path,
            manifest,
            slab_bytes,
            section_bytes,
            P::stream_schema(),
            |bytes| {
                let (capture, native) = crate::capture::decode_capture(bytes)?;
                Ok((capture.copperlist, native, Some(capture.proof)))
            },
        )?))
    }
    pub fn accept(
        &mut self,
        event: &SessionEvent,
    ) -> Result<Option<crate::capture::CapturedList<P>>> {
        Ok(self
            .0
            .accept(event)?
            .map(|copperlist| crate::capture::CapturedList {
                copperlist,
                proof: self
                    .0
                    .last_proof
                    .take()
                    .expect("capture decoder supplies proof"),
            }))
    }
    pub fn finish(self) -> Result<()> {
        self.0.finish()
    }
}
