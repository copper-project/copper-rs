//! Shared byte addressing, append validation and section reclamation for storage adapters.

use crate::{
    ApplicationMetadata, CapacityPolicy, MAIN_MAGIC, MainHeader, SECTION_HEADER_COMPACT_SIZE,
    SECTION_MAGIC, SectionContext, SectionHandle, SectionHeader, SectionStorage,
    UNIFIED_LOG_FORMAT_VERSION, UnifiedLogStatus, UnifiedLogWrite,
};
use alloc::string::ToString;
use alloc::sync::Arc;
use bincode::config::standard;
use bincode::enc::EncoderImpl;
use bincode::enc::write::{SizeWriter, Writer};
use bincode::error::EncodeError;
use bincode::{Encode, decode_from_slice, encode_into_slice};
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use cu29_traits::{
    CuError, CuResult, ObservedWriter, UnifiedLogType, abort_observed_encode,
    begin_observed_encode, finish_observed_encode,
};

const HEADER: u64 = SECTION_HEADER_COMPACT_SIZE as u64;
const LEASES: usize = 64;
const IDLE: usize = 1;
const BUSY: usize = 2;
const SEALED: usize = 3;

struct Lease {
    state: AtomicUsize,
    next: UnsafeCell<u64>,
    offset: UnsafeCell<u64>,
}
// SAFETY: `next` is accessed only while the corresponding state is exclusively BUSY.
unsafe impl Sync for Lease {}
impl Lease {
    fn load(&self, ordering: Ordering) -> usize {
        self.state.load(ordering)
    }
    fn store(&self, state: usize, ordering: Ordering) {
        self.state.store(state, ordering);
    }
    fn compare_exchange(
        &self,
        current: usize,
        new: usize,
        success: Ordering,
        failure: Ordering,
    ) -> Result<usize, usize> {
        self.state.compare_exchange(current, new, success, failure)
    }
    fn offset(&self) -> u64 {
        // SAFETY: Only the owning allocator accesses this field; it serializes allocation.
        unsafe { *self.offset.get() }
    }
    fn set_offset(&self, offset: u64) {
        // SAFETY: Only the owning allocator accesses this field, through exclusive allocation.
        unsafe {
            *self.offset.get() = offset;
        }
    }
    fn next(&self) -> u64 {
        // SAFETY: The caller owns this lease in BUSY state.
        unsafe { *self.next.get() }
    }
    fn set_next(&self, offset: u64) {
        // SAFETY: The caller owns BUSY, or initializes a generation before publishing IDLE.
        unsafe {
            *self.next.get() = offset;
        }
    }
}

struct LeaseTable {
    slots: [Lease; LEASES],
    poisoned: AtomicBool,
}
impl core::ops::Deref for LeaseTable {
    type Target = [Lease; LEASES];
    fn deref(&self) -> &Self::Target {
        &self.slots
    }
}

struct Claims {
    leases: Arc<LeaseTable>,
    held: [Option<(usize, usize)>; LEASES],
}
impl Claims {
    fn new(leases: Arc<LeaseTable>) -> Self {
        Self {
            leases,
            held: [None; LEASES],
        }
    }
    fn claim(&mut self, slot: usize) -> CuResult<()> {
        if self.held.iter().flatten().any(|&(s, _)| s == slot) {
            return Ok(());
        }
        let state = self.leases[slot].load(Ordering::Acquire);
        if state % 4 == SEALED || state == 0 {
            return Ok(());
        }
        if state % 4 != IDLE {
            return Err(CuError::from("Space is blocked by a write in progress"));
        }
        self.leases[slot]
            .compare_exchange(
                state,
                state / 4 * 4 + BUSY,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .map_err(|_| CuError::from("Space is blocked by a write in progress"))?;
        *self
            .held
            .iter_mut()
            .find(|s| s.is_none())
            .ok_or_else(|| CuError::from("Lease claim table is full"))? = Some((slot, state));
        Ok(())
    }
    fn seal(&mut self, slot: usize) {
        if let Some(held) = self
            .held
            .iter_mut()
            .find(|s| s.is_some_and(|(s, _)| s == slot))
        {
            let (_, old) = held.take().unwrap();
            self.leases[slot].store(old / 4 * 4 + SEALED, Ordering::Release);
        }
    }
}
impl Drop for Claims {
    fn drop(&mut self) {
        for &(slot, old) in self.held.iter().flatten() {
            self.leases[slot].store(old, Ordering::Release);
        }
    }
}

/// A bounded byte range owned by a section. Adapters write native encoder output directly.
pub trait ByteRegion: Send + Sync {
    fn len(&self) -> u64;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    fn read(&self, offset: u64, bytes: &mut [u8]) -> CuResult<()>;
    fn write(&mut self, offset: u64, bytes: &[u8]) -> CuResult<()>;
    fn flush(&mut self) -> CuResult<()>;
    fn commit(&mut self) -> CuResult<()> {
        self.flush()
    }
    /// Discard a failed write's pending block state. Committed bytes remain readable.
    fn abort(&mut self) {}
}

/// Maps byte ranges to backing allocations or partition blocks.
pub trait ByteStorage: Send + Sync {
    type Region: ByteRegion;
    fn len(&self) -> u64;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    fn grow(&mut self, end: u64) -> CuResult<()>;
    fn region(&self, offset: u64, len: u64) -> CuResult<Self::Region>;
}

fn encode_error(_: CuError) -> EncodeError {
    EncodeError::Other("Storage write failed")
}

struct RegionWriter<'a, R> {
    region: &'a mut R,
    position: u64,
    limit: u64,
}
impl<R: ByteRegion> Writer for RegionWriter<'_, R> {
    fn write(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        let end = self
            .position
            .checked_add(bytes.len() as u64)
            .ok_or(EncodeError::UnexpectedEnd)?;
        if end > self.limit {
            return Err(EncodeError::UnexpectedEnd);
        }
        self.region
            .write(self.position, bytes)
            .map_err(encode_error)?;
        self.position = end;
        Ok(())
    }
}

struct CompareWriter<'a, R> {
    region: &'a R,
    position: u64,
    limit: u64,
}
impl<R: ByteRegion> Writer for CompareWriter<'_, R> {
    fn write(&mut self, mut bytes: &[u8]) -> Result<(), EncodeError> {
        if self
            .position
            .checked_add(bytes.len() as u64)
            .is_none_or(|end| end > self.limit)
        {
            return Err(EncodeError::Other("Static metadata differs"));
        }
        let mut scratch = [0u8; 512];
        while !bytes.is_empty() {
            let count = bytes.len().min(scratch.len());
            self.region
                .read(self.position, &mut scratch[..count])
                .map_err(encode_error)?;
            if scratch[..count] != bytes[..count] {
                return Err(EncodeError::Other("Static metadata differs"));
            }
            self.position += count as u64;
            bytes = &bytes[count..];
        }
        Ok(())
    }
}

pub fn encoded_size<E: Encode>(value: &E) -> CuResult<u64> {
    let mut encoder = EncoderImpl::new(SizeWriter::default(), standard());
    value
        .encode(&mut encoder)
        .map_err(|e| CuError::from("Cannot size metadata").add_cause(&e.to_string()))?;
    Ok(encoder.into_writer().bytes_written as u64)
}

fn write_header<R: ByteRegion, E: Encode>(region: &mut R, value: &E) -> CuResult<()> {
    let mut bytes = [0u8; 512];
    let count = encode_into_slice(value, &mut bytes, standard())
        .map_err(|e| CuError::from("Cannot encode log header").add_cause(&e.to_string()))?;
    region.write(0, &bytes[..count])?;
    region.commit()
}

pub fn read_header<B: ByteStorage>(storage: &B, offset: u64) -> CuResult<SectionHeader> {
    let mut bytes = [0u8; 512];
    storage.region(offset, HEADER)?.read(0, &mut bytes)?;
    let (header, _) = decode_from_slice::<SectionHeader, _>(&bytes, standard())
        .map_err(|e| CuError::from("Invalid section header").add_cause(&e.to_string()))?;
    if header.magic != SECTION_MAGIC
        || header.block_size != SECTION_HEADER_COMPACT_SIZE
        || header.allocated < HEADER
        || header.used as u64 > header.allocated - HEADER
        || !header.allocated.is_multiple_of(HEADER)
    {
        return Err(CuError::from("Invalid section size or committed length"));
    }
    Ok(header)
}

pub fn read_main<B: ByteStorage>(storage: &B) -> CuResult<MainHeader> {
    let mut bytes = [0u8; 512];
    storage.region(0, HEADER)?.read(0, &mut bytes)?;
    let (header, _) = decode_from_slice::<MainHeader, _>(&bytes, standard())
        .map_err(|e| CuError::from("Invalid main header").add_cause(&e.to_string()))?;
    if header.magic != MAIN_MAGIC
        || header.format_version != UNIFIED_LOG_FORMAT_VERSION
        || header.page_size == 0
        || !(header.page_size as u64).is_multiple_of(HEADER)
        || header.sections_begin < header.page_size as u64
        || header.sections_begin > header.sections_end
        || header.sections_end > storage.len()
        || !header.sections_begin.is_multiple_of(HEADER)
        || !header.sections_end.is_multiple_of(HEADER)
        || (header.head_section == 0) != (header.tail_section == 0)
    {
        return Err(CuError::from(
            "Invalid log version, alignment or byte bounds",
        ));
    }
    Ok(header)
}

/// A section has a generation lease in a fixed startup-allocated table.
/// Reclamation seals idle leases; concurrent encoding holds BUSY until commit.
pub struct ByteSection<R: ByteRegion> {
    region: R,
    position: u64,
    header: SectionHeader,
    leases: Arc<LeaseTable>,
    slot: usize,
    generation: usize,
}
impl<R: ByteRegion> ByteSection<R> {
    fn token(&self, state: usize) -> usize {
        self.generation * 4 + state
    }
}
impl<R: ByteRegion> SectionStorage for ByteSection<R> {
    fn initialize<E: Encode>(&mut self, _: &E) -> Result<usize, EncodeError> {
        Ok(HEADER as usize)
    }
    fn post_update_header<E: Encode>(&mut self, _: &E) -> Result<usize, EncodeError> {
        // The authoritative header is committed with the payload while holding the lease.
        Ok(HEADER as usize)
    }
    fn is_sealed(&self) -> bool {
        self.leases[self.slot].load(Ordering::Acquire) != self.token(IDLE)
    }
    fn append<E: Encode>(&mut self, entry: &E) -> Result<usize, EncodeError> {
        if self.leases.poisoned.load(Ordering::Acquire) {
            return Err(EncodeError::Other("Logger faulted after a storage failure"));
        }
        self.leases[self.slot]
            .compare_exchange(
                self.token(IDLE),
                self.token(BUSY),
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .map_err(|_| EncodeError::Other("Section lease is sealed or busy"))?;
        begin_observed_encode();
        let old_position = self.position;
        let result = (|| {
            let writer = RegionWriter {
                region: &mut self.region,
                position: old_position,
                limit: self.header.allocated,
            };
            let mut encoder = EncoderImpl::new(ObservedWriter::new(writer), standard());
            entry.encode(&mut encoder)?;
            let end = encoder.into_writer().into_inner().position;
            self.region.commit().map_err(encode_error)?;
            let mut committed = self.header.clone();
            committed.next_section = self.leases[self.slot].next();
            committed.used = u32::try_from(end - HEADER).map_err(|_| EncodeError::UnexpectedEnd)?;
            if let Err(error) = write_header(&mut self.region, &committed) {
                self.leases.poisoned.store(true, Ordering::Release);
                return Err(encode_error(error));
            }
            self.header = committed;
            self.position = end;
            Ok((end - old_position) as usize)
        })();
        match &result {
            Ok(count) => {
                debug_assert_eq!(*count, finish_observed_encode());
            }
            Err(_) => {
                abort_observed_encode();
                self.region.abort();
            }
        }
        self.leases[self.slot].store(self.token(IDLE), Ordering::Release);
        result
    }
    fn flush(&mut self) -> CuResult<usize> {
        if self.leases[self.slot]
            .compare_exchange(
                self.token(IDLE),
                self.token(BUSY),
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok()
        {
            let mut header = self.header.clone();
            header.next_section = self.leases[self.slot].next();
            header.is_open = false;
            let result = write_header(&mut self.region, &header).and_then(|()| self.region.flush());
            if result.is_ok() {
                self.header = header;
                self.leases[self.slot].store(self.token(SEALED), Ordering::Release);
            } else {
                self.leases.poisoned.store(true, Ordering::Release);
                self.leases[self.slot].store(self.token(IDLE), Ordering::Release);
                result?;
            }
        }
        Ok(self.position as usize)
    }
}

/// One allocator implements both backing adapters. Section bookkeeping is fixed-capacity.
pub struct ByteLogger<B: ByteStorage> {
    storage: B,
    header: MainHeader,
    policy: CapacityPolicy,
    capacity: u64,
    leases: Arc<LeaseTable>,
    generation: usize,
    next_run_id: u64,
    retained_space: u64,
    append_pending: bool,
    faulted: bool,
}
impl<B: ByteStorage> ByteLogger<B> {
    pub fn new(
        mut storage: B,
        alignment: u16,
        policy: CapacityPolicy,
        capacity: u64,
    ) -> CuResult<Self> {
        if alignment == 0
            || !(alignment as u64).is_multiple_of(HEADER)
            || capacity <= alignment as u64
            || !capacity.is_multiple_of(HEADER)
        {
            return Err(CuError::from(
                "Invalid log capacity or allocation alignment",
            ));
        }
        storage.grow(capacity)?;
        let header = MainHeader {
            magic: MAIN_MAGIC,
            format_version: UNIFIED_LOG_FORMAT_VERSION,
            page_size: alignment,
            metadata_offset: 0,
            sections_begin: alignment as u64,
            sections_end: capacity,
            head_section: 0,
            tail_section: 0,
            clean_close: false,
        };
        let mut logger = Self::from_parts(storage, header, policy, capacity, false);
        logger.publish()?;
        Ok(logger)
    }
    fn from_parts(
        storage: B,
        header: MainHeader,
        policy: CapacityPolicy,
        capacity: u64,
        append_pending: bool,
    ) -> Self {
        Self {
            storage,
            header,
            policy,
            capacity,
            leases: Arc::new(LeaseTable {
                slots: core::array::from_fn(|_| Lease {
                    state: AtomicUsize::new(0),
                    next: UnsafeCell::new(0),
                    offset: UnsafeCell::new(0),
                }),
                poisoned: AtomicBool::new(false),
            }),
            generation: 0,
            next_run_id: 1,
            retained_space: 0,
            append_pending,
            faulted: false,
        }
    }
    /// Read-only validation. No storage extension or header write precedes metadata comparison.
    pub fn append(storage: B, policy: CapacityPolicy, capacity: u64) -> CuResult<Self> {
        let header = read_main(&storage)?;
        if !header.clean_close {
            return Err(CuError::from("Cannot append: log was not cleanly closed"));
        }
        if !capacity.is_multiple_of(HEADER) || capacity <= header.sections_begin {
            return Err(CuError::from("Invalid append capacity"));
        }
        let mut logger = Self::from_parts(storage, header, policy, capacity, true);
        let mut cursor = logger.header.head_section;
        let mut remaining = (logger.header.sections_end - logger.header.sections_begin) / HEADER;
        while cursor != 0 {
            if remaining == 0 {
                return Err(CuError::from("Cyclic section links"));
            }
            remaining -= 1;
            let section = logger.data_header(cursor)?;
            if cursor + section.allocated > capacity {
                return Err(CuError::from(
                    "Cannot shrink storage beneath retained sections",
                ));
            }
            if section.is_open {
                return Err(CuError::from("Cannot append: retained section is open"));
            }
            logger.retained_space = logger
                .retained_space
                .checked_add(section.allocated)
                .filter(|used| *used <= logger.header.sections_end - logger.header.sections_begin)
                .ok_or_else(|| CuError::from("Retained sections exceed data capacity"))?;
            logger.next_run_id = logger.next_run_id.max(
                section
                    .context
                    .run_id
                    .checked_add(1)
                    .ok_or_else(|| CuError::from("Run ID exhausted"))?,
            );
            if section.next_section == 0 && cursor != logger.header.tail_section {
                return Err(CuError::from("Invalid tail section"));
            }
            if cursor == logger.header.tail_section && section.next_section != 0 {
                return Err(CuError::from("Invalid tail link"));
            }
            cursor = section.next_section;
        }
        logger.validate_metadata()?;
        Ok(logger)
    }
    fn validate_metadata(&self) -> CuResult<()> {
        if self.header.metadata_offset == 0 {
            return Ok(());
        }
        let offset = self.header.metadata_offset;
        if offset != self.header.page_size as u64 {
            return Err(CuError::from("Invalid metadata offset"));
        }
        let h = read_header(&self.storage, offset)?;
        if h.is_open
            || h.entry_type != UnifiedLogType::ApplicationMetadata
            || offset
                .checked_add(h.allocated)
                .is_none_or(|end| end > self.header.sections_begin)
        {
            return Err(CuError::from("Invalid application metadata section"));
        }
        if h.next_section != 0 {
            let c = read_header(&self.storage, h.next_section)?;
            if c.is_open
                || c.entry_type != UnifiedLogType::ValueDecodeCatalog
                || c.next_section != 0
                || h.next_section != offset + h.allocated
                || h.next_section.checked_add(c.allocated) != Some(self.header.sections_begin)
            {
                return Err(CuError::from("Invalid catalog metadata section"));
            }
        } else if offset + h.allocated != self.header.sections_begin {
            return Err(CuError::from("Invalid metadata bounds"));
        }
        Ok(())
    }
    fn data_header(&self, offset: u64) -> CuResult<SectionHeader> {
        let mut claims = Claims::new(self.leases.clone());
        if let Some(slot) = self
            .leases
            .iter()
            .position(|lease| lease.offset() == offset)
        {
            claims.claim(slot)?;
        }
        self.data_header_unlocked(offset)
    }
    fn data_header_unlocked(&self, offset: u64) -> CuResult<SectionHeader> {
        if offset < self.header.sections_begin
            || offset >= self.header.sections_end
            || !offset.is_multiple_of(HEADER)
        {
            return Err(CuError::from("Section offset is outside the data region"));
        }
        let h = read_header(&self.storage, offset)?;
        if offset
            .checked_add(h.allocated)
            .is_none_or(|end| end > self.header.sections_end)
            || (h.next_section != 0
                && (h.next_section < self.header.sections_begin
                    || h.next_section >= self.header.sections_end
                    || !h.next_section.is_multiple_of(HEADER)))
        {
            return Err(CuError::from("Invalid section bounds or next link"));
        }
        Ok(h)
    }
    fn publish(&mut self) -> CuResult<()> {
        let result = (|| {
            let mut region = self.storage.region(0, HEADER)?;
            write_header(&mut region, &self.header)?;
            region.flush()
        })();
        if result.is_err() {
            self.faulted = true;
        }
        result
    }
    fn activate(&mut self) -> CuResult<()> {
        if self.faulted || self.leases.poisoned.load(Ordering::Acquire) {
            return Err(CuError::from("Logger faulted after a storage failure"));
        }
        if self.append_pending {
            self.storage.grow(self.capacity)?;
            self.header.sections_end = self.capacity;
            self.header.clean_close = false;
            self.publish()?;
            self.append_pending = false;
        }
        Ok(())
    }
    fn close_idle(&mut self, offset: u64) -> CuResult<()> {
        if let Some(slot) = self
            .leases
            .iter()
            .position(|lease| lease.offset() == offset)
        {
            let state = self.leases[slot].load(Ordering::Acquire);
            if state % 4 == BUSY {
                return Err(CuError::from("Space is blocked by a write in progress"));
            }
            if state % 4 == IDLE {
                self.leases[slot]
                    .compare_exchange(
                        state,
                        state / 4 * 4 + SEALED,
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    )
                    .map_err(|_| CuError::from("Space is blocked by a write in progress"))?;
                let mut h = self.data_header(offset)?;
                h.is_open = false;
                write_header(&mut self.storage.region(offset, HEADER)?, &h)?;
                self.storage.region(offset, h.allocated)?.flush()?;
            }
        }
        if self.data_header(offset)?.is_open {
            return Err(CuError::from("Space is blocked by an open section"));
        }
        Ok(())
    }
    fn compare<E: Encode>(&self, offset: u64, kind: UnifiedLogType, value: &E) -> CuResult<()> {
        let h = read_header(&self.storage, offset)?;
        if h.entry_type != kind {
            return Err(CuError::from("Static metadata section type differs"));
        }
        let region = self.storage.region(offset + HEADER, h.used as u64)?;
        let writer = CompareWriter {
            region: &region,
            position: 0,
            limit: h.used as u64,
        };
        let mut encoder = EncoderImpl::new(writer, standard());
        value.encode(&mut encoder).map_err(|_| CuError::from("Cannot append: static metadata does not match; appending different applications or versions to the same log is not supported."))?;
        if encoder.into_writer().position != h.used as u64 {
            return Err(CuError::from(
                "Cannot append: static metadata does not match; appending different applications or versions to the same log is not supported.",
            ));
        }
        Ok(())
    }
    fn write_static<E: Encode>(
        &mut self,
        offset: u64,
        allocated: u64,
        next: u64,
        kind: UnifiedLogType,
        value: &E,
    ) -> CuResult<()> {
        let mut region = self.storage.region(offset, allocated)?;
        let writer = RegionWriter {
            region: &mut region,
            position: HEADER,
            limit: allocated,
        };
        let mut encoder = EncoderImpl::new(writer, standard());
        value.encode(&mut encoder).map_err(|e| {
            CuError::from("Cannot encode static metadata").add_cause(&e.to_string())
        })?;
        let used = encoder.into_writer().position - HEADER;
        region.flush()?;
        write_header(
            &mut region,
            &SectionHeader {
                entry_type: kind,
                allocated,
                next_section: next,
                used: u32::try_from(used)
                    .map_err(|_| CuError::from("Metadata exceeds section length limit"))?,
                is_open: false,
                ..SectionHeader::default()
            },
        )
    }
    pub fn close(&mut self) -> CuResult<()> {
        if self.append_pending {
            return Ok(());
        }
        if self.faulted || self.leases.poisoned.load(Ordering::Acquire) {
            return Err(CuError::from("Cannot cleanly close a faulted logger"));
        }
        self.faulted = true;
        let mut cursor = self.header.head_section;
        let mut remaining = (self.header.sections_end - self.header.sections_begin) / HEADER;
        while cursor != 0 {
            if remaining == 0 {
                return Err(CuError::from("Cyclic section links prevent clean close"));
            }
            remaining -= 1;
            self.close_idle(cursor)?;
            cursor = self.data_header(cursor)?.next_section;
        }
        self.header.clean_close = true;
        self.publish()?;
        self.faulted = false;
        Ok(())
    }
}
fn aligned(size: u64) -> CuResult<u64> {
    size.checked_add(HEADER - 1)
        .map(|s| s / HEADER * HEADER)
        .ok_or_else(|| CuError::from("Section size overflow"))
}
impl<B: ByteStorage> UnifiedLogWrite<ByteSection<B::Region>> for ByteLogger<B> {
    fn seal_metadata<C: Encode>(
        &mut self,
        metadata: &ApplicationMetadata,
        catalog: Option<&C>,
    ) -> CuResult<()> {
        #[derive(Encode)]
        struct MetadataBody<'a> {
            app_type: &'a str,
            app_name: &'a str,
            app_version: &'a str,
            git_commit: Option<&'a str>,
            git_dirty: Option<bool>,
            subsystem_id: Option<&'a str>,
            subsystem_code: u16,
            effective_config_ron: &'a str,
            missions: &'a [alloc::string::String],
            catalog_offset: u64,
        }
        let mut metadata = MetadataBody {
            app_type: &metadata.app_type,
            app_name: &metadata.app_name,
            app_version: &metadata.app_version,
            git_commit: metadata.git_commit.as_deref(),
            git_dirty: metadata.git_dirty,
            subsystem_id: metadata.subsystem_id.as_deref(),
            subsystem_code: metadata.subsystem_code,
            effective_config_ron: &metadata.effective_config_ron,
            missions: &metadata.missions,
            catalog_offset: 0,
        };
        let base = self.header.page_size as u64;
        // The offset changes bincode's varint width; sizing converges monotonically.
        let mut app_size = 0;
        for _ in 0..8 {
            let size = aligned(HEADER + encoded_size(&metadata)?)?;
            let offset = if catalog.is_some() { base + size } else { 0 };
            app_size = size;
            if metadata.catalog_offset == offset {
                break;
            }
            metadata.catalog_offset = offset;
        }
        let catalog_size = catalog
            .map(|c| encoded_size(c).and_then(|s| aligned(HEADER + s)))
            .transpose()?
            .unwrap_or(0);
        let end = base
            .checked_add(app_size)
            .and_then(|s| s.checked_add(catalog_size))
            .ok_or_else(|| CuError::from("Metadata size overflow"))?;
        if self.header.metadata_offset != 0 {
            self.compare(base, UnifiedLogType::ApplicationMetadata, &metadata)?;
            if let Some(catalog) = catalog {
                self.compare(
                    metadata.catalog_offset,
                    UnifiedLogType::ValueDecodeCatalog,
                    catalog,
                )?;
            }
            self.activate()?;
            return Ok(());
        }
        if self.header.head_section != 0 || self.header.sections_begin != base {
            return Err(CuError::from("Static metadata region is already sealed"));
        }
        if end + HEADER > self.header.sections_end && self.policy != CapacityPolicy::Grow {
            return Err(CuError::from("Static metadata exceeds log capacity"));
        }
        self.storage.grow(end + HEADER)?;
        self.activate()?;
        self.faulted = true;
        self.write_static(
            base,
            app_size,
            metadata.catalog_offset,
            UnifiedLogType::ApplicationMetadata,
            &metadata,
        )?;
        if let Some(catalog) = catalog {
            self.write_static(
                metadata.catalog_offset,
                catalog_size,
                0,
                UnifiedLogType::ValueDecodeCatalog,
                catalog,
            )?;
        }
        self.header.metadata_offset = base;
        self.header.sections_begin = end;
        if end + HEADER > self.header.sections_end {
            self.header.sections_end = self.storage.len();
        }
        self.publish()?;
        self.faulted = false;
        Ok(())
    }
    fn construction_context(
        &mut self,
        instance_id: u32,
        mission_index: u32,
    ) -> CuResult<SectionContext> {
        if self.header.metadata_offset == 0 {
            return Err(CuError::from("Static metadata must precede construction"));
        }
        let run_id = self.next_run_id;
        self.next_run_id = run_id
            .checked_add(1)
            .ok_or_else(|| CuError::from("Run ID exhausted"))?;
        Ok(SectionContext {
            run_id,
            instance_id,
            mission_index,
        })
    }
    fn add_section(
        &mut self,
        kind: UnifiedLogType,
        size: usize,
    ) -> CuResult<SectionHandle<ByteSection<B::Region>>> {
        self.add_section_with_context(kind, size, SectionContext::default())
    }
    fn add_section_with_context(
        &mut self,
        kind: UnifiedLogType,
        size: usize,
        context: SectionContext,
    ) -> CuResult<SectionHandle<ByteSection<B::Region>>> {
        if self.append_pending && self.header.metadata_offset != 0 {
            return Err(CuError::from(
                "Compare static metadata before appending data",
            ));
        }
        let size = aligned(size as u64)?;
        if size <= HEADER || size - HEADER > u32::MAX as u64 {
            return Err(CuError::from("Invalid section allocation size"));
        }
        let capacity = self.header.sections_end - self.header.sections_begin;
        if self.policy != CapacityPolicy::Grow && size > capacity {
            return Err(CuError::from("Section exceeds log capacity"));
        }
        let tail = self.header.tail_section;
        let mut position = if tail == 0 {
            self.header.sections_begin
        } else {
            tail + self.data_header(tail)?.allocated
        };
        if self.policy == CapacityPolicy::OverwriteOldest
            && position
                .checked_add(size)
                .is_none_or(|end| end > self.header.sections_end)
        {
            position = self.header.sections_begin;
        }
        let overlaps = |offset: u64, header: &SectionHeader| {
            position < offset + header.allocated && offset < position + size
        };
        let mut last_overlap = 0;
        let mut cursor = self.header.head_section;
        let mut remaining = capacity / HEADER;
        while cursor != 0 {
            if remaining == 0 {
                return Err(CuError::from("Cyclic section links"));
            }
            remaining -= 1;
            let h = self.data_header(cursor)?;
            if overlaps(cursor, &h) {
                last_overlap = cursor;
            }
            cursor = h.next_section;
        }
        if self.policy == CapacityPolicy::Grow
            && (last_overlap != 0 || position + size > self.header.sections_end)
        {
            position = self.header.sections_end;
            last_overlap = 0;
        }
        if self.policy == CapacityPolicy::StopWhenFull
            && (last_overlap != 0 || position + size > self.header.sections_end)
        {
            return Err(CuError::from("Log is full"));
        }
        // Claim every affected encoder before any write. A failed preflight restores all leases.
        let mut claims = Claims::new(self.leases.clone());
        if let Some(slot) = self.leases.iter().position(|lease| lease.offset() == tail) {
            claims.claim(slot)?;
        }
        let mut cursor = self.header.head_section;
        if last_overlap != 0 {
            loop {
                if let Some(slot) = self
                    .leases
                    .iter()
                    .position(|lease| lease.offset() == cursor)
                {
                    claims.claim(slot)?;
                } else if self.data_header_unlocked(cursor)?.is_open {
                    return Err(CuError::from("Space is blocked by an open section"));
                }
                if cursor == last_overlap {
                    break;
                }
                cursor = self.data_header_unlocked(cursor)?.next_section;
            }
        }
        let free_slot = self
            .leases
            .iter()
            .position(|s| matches!(s.load(Ordering::Acquire) % 4, 0 | SEALED));
        if free_slot.is_none() && last_overlap == 0 {
            return Err(CuError::from(
                "Too many concurrent section streams (maximum 64)",
            ));
        }
        // Validate the mapping geometry before reclamation or header changes.
        if position + size <= self.storage.len() {
            let _ = self.storage.region(position, size)?;
        }
        self.activate()?;
        self.faulted = true;
        if position + size > self.header.sections_end {
            self.storage.grow(position + size)?;
            self.header.sections_end = self.storage.len();
        }
        if last_overlap != 0 {
            let mut cursor = self.header.head_section;
            loop {
                let mut h = self.data_header_unlocked(cursor)?;
                h.is_open = false;
                write_header(&mut self.storage.region(cursor, HEADER)?, &h)?;
                self.storage.region(cursor, h.allocated)?.flush()?;
                if let Some(slot) = self
                    .leases
                    .iter()
                    .position(|lease| lease.offset() == cursor)
                {
                    claims.seal(slot);
                }
                self.retained_space -= h.allocated;
                let next = h.next_section;
                if cursor == last_overlap {
                    self.header.head_section = next;
                    if next == 0 {
                        self.header.tail_section = 0;
                    }
                    break;
                }
                cursor = next;
            }
            self.publish()?;
        }
        let slot = self
            .leases
            .iter()
            .position(|s| {
                s.load(Ordering::Acquire) % 4 != IDLE && s.load(Ordering::Acquire) % 4 != BUSY
            })
            .ok_or_else(|| CuError::from("Too many concurrent section writes"))?;
        self.generation = self
            .generation
            .checked_add(1)
            .filter(|g| *g < usize::MAX / 4)
            .ok_or_else(|| CuError::from("Section generation exhausted"))?;
        let header = SectionHeader {
            entry_type: kind,
            allocated: size,
            context,
            ..SectionHeader::default()
        };
        let mut region = self.storage.region(position, size)?;
        write_header(&mut region, &header)?;
        if self.header.tail_section != 0 {
            let mut previous = self.data_header_unlocked(self.header.tail_section)?;
            previous.next_section = position;
            if let Some(slot) = self
                .leases
                .iter()
                .position(|lease| lease.offset() == self.header.tail_section)
            {
                // This lease was claimed before allocation, so its link can be updated.
                if self.leases[slot].load(Ordering::Acquire) % 4 == BUSY {
                    self.leases[slot].set_next(position);
                }
            }
            write_header(
                &mut self.storage.region(self.header.tail_section, HEADER)?,
                &previous,
            )?;
        }
        if self.header.head_section == 0 {
            self.header.head_section = position;
        }
        self.header.tail_section = position;
        self.header.clean_close = false;
        self.publish()?;
        self.retained_space += size;
        for lease in self.leases.iter() {
            if lease.offset() == position {
                lease.set_offset(0);
            }
        }
        self.leases[slot].set_offset(position);
        self.leases[slot].set_next(0);
        self.leases[slot].store(self.generation * 4 + IDLE, Ordering::Release);
        let storage = ByteSection {
            region,
            position: HEADER,
            header: header.clone(),
            leases: self.leases.clone(),
            slot,
            generation: self.generation,
        };
        let handle = SectionHandle::create(header, storage)?;
        self.faulted = false;
        Ok(handle)
    }
    fn flush_section(&mut self, section: &mut SectionHandle<ByteSection<B::Region>>) {
        let _ = self.try_flush_section(section);
    }
    fn try_flush_section(
        &mut self,
        section: &mut SectionHandle<ByteSection<B::Region>>,
    ) -> CuResult<()> {
        if self.faulted || self.leases.poisoned.load(Ordering::Acquire) {
            return Err(CuError::from("Logger faulted after a storage failure"));
        }
        if let Err(error) = section.storage.flush() {
            self.faulted = true;
            return Err(error);
        }
        section.mark_closed();
        Ok(())
    }
    fn status(&self) -> UnifiedLogStatus {
        UnifiedLogStatus {
            total_used_space: usize::try_from(self.header.sections_begin + self.retained_space)
                .unwrap_or(usize::MAX),
            total_allocated_space: usize::try_from(self.header.sections_end).unwrap_or(usize::MAX),
        }
    }
}
impl<B: ByteStorage> Drop for ByteLogger<B> {
    fn drop(&mut self) {
        let _ = self.close();
    }
}
