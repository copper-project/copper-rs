//! Shared byte addressing, append validation and section reclamation for storage adapters.

use crate::{
    ApplicationMetadata, CapacityPolicy, MAIN_MAGIC, MainHeader, SECTION_HEADER_COMPACT_SIZE,
    SECTION_MAGIC, SectionAppendOutcome, SectionContext, SectionHandle, SectionHeader,
    SectionStorage, UNIFIED_LOG_FORMAT_VERSION, UnifiedLogStatus, UnifiedLogWrite,
};
use alloc::string::ToString;
use alloc::sync::Arc;
use bincode::config::standard;
use bincode::enc::EncoderImpl;
use bincode::enc::write::{SizeWriter, Writer};
use bincode::error::EncodeError;
use bincode::{Decode, Encode, decode_from_slice, encode_into_slice};
use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use cu29_traits::{
    CuError, CuResult, ObservedWriter, UnifiedLogType, abort_observed_encode,
    begin_observed_encode, finish_observed_encode,
};

const BLOCK: u64 = 512;
const HEADER: u64 = SECTION_HEADER_COMPACT_SIZE as u64;
const LEASES: usize = 64;
const IDLE: usize = 1;
const BUSY: usize = 2;
const SEALED: usize = 3;
const GENERATION_STRIDE: usize = 8;

struct Lease {
    state: AtomicUsize,
    next: UnsafeCell<u64>,
    allocated: UnsafeCell<u64>,
    offset: UnsafeCell<u64>,
}
// SAFETY: Only the serialized allocator accesses `offset` and `allocated`.
unsafe impl Sync for Lease {}
impl Lease {
    fn load(&self, ordering: Ordering) -> usize {
        let state = self.state.load(ordering);
        #[cfg(all(test, feature = "std"))]
        tests::lease_event(self, tests::LeaseEvent::Loaded(state));
        state
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
        #[cfg(all(test, feature = "std"))]
        tests::lease_event(self, tests::LeaseEvent::BeforeCompareExchange);
        let result = self.state.compare_exchange(current, new, success, failure);
        #[cfg(all(test, feature = "std"))]
        tests::lease_event(self, tests::LeaseEvent::AfterCompareExchange);
        result
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
        // SAFETY: Only the serialized allocator accesses chain geometry.
        unsafe { *self.next.get() }
    }
    fn set_next(&self, offset: u64) {
        // SAFETY: Only the serialized allocator accesses chain geometry.
        unsafe {
            *self.next.get() = offset;
        }
    }
    fn allocated(&self) -> u64 {
        // SAFETY: Only the serialized allocator accesses this field.
        unsafe { *self.allocated.get() }
    }
    fn set_allocated(&self, allocated: u64) {
        // SAFETY: Only the serialized allocator accesses this field.
        unsafe {
            *self.allocated.get() = allocated;
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

// Each owner writes a whole independent 512-byte block, including on SD adapters.
#[derive(Encode, Decode)]
struct SectionDescriptor {
    magic: [u8; 2],
    block_size: u16,
    entry_type: UnifiedLogType,
    allocated: u64,
    next_section: u64,
    context: SectionContext,
    position: u64,
    generation: u64,
    next_generation: u64,
}
#[derive(Encode, Decode)]
struct SectionCommit {
    position: u64,
    generation: u64,
    used: u32,
    is_open: bool,
}
impl SectionDescriptor {
    fn new(header: &SectionHeader, position: u64, generation: u64) -> Self {
        Self {
            magic: header.magic,
            block_size: header.block_size,
            entry_type: header.entry_type,
            allocated: header.allocated,
            next_section: header.next_section,
            context: header.context,
            position,
            generation,
            next_generation: 0,
        }
    }
}
fn read_descriptor<B: ByteStorage>(storage: &B, offset: u64) -> CuResult<SectionDescriptor> {
    let mut bytes = [0u8; 512];
    storage.region(offset, BLOCK)?.read(0, &mut bytes)?;
    let (descriptor, _) = decode_from_slice::<SectionDescriptor, _>(&bytes, standard())
        .map_err(|e| CuError::from("Invalid section descriptor").add_cause(&e.to_string()))?;
    if descriptor.magic != SECTION_MAGIC
        || descriptor.block_size != SECTION_HEADER_COMPACT_SIZE
        || descriptor.position != offset
        || descriptor.allocated < HEADER
        || !descriptor.allocated.is_multiple_of(BLOCK)
        || offset
            .checked_add(descriptor.allocated)
            .is_none_or(|end| end > storage.len())
    {
        return Err(CuError::from(
            "Invalid section descriptor bounds or identity",
        ));
    }
    Ok(descriptor)
}
fn write_commit<R: ByteRegion>(
    region: &mut R,
    header: &SectionHeader,
    position: u64,
    generation: u64,
) -> CuResult<()> {
    let mut bytes = [0u8; 512];
    let count = encode_into_slice(
        SectionCommit {
            position,
            generation,
            used: header.used,
            is_open: header.is_open,
        },
        &mut bytes,
        standard(),
    )
    .map_err(|e| CuError::from("Cannot encode section commit").add_cause(&e.to_string()))?;
    region.write(BLOCK, &bytes[..count])?;
    region.commit()
}
fn initialize_section<R: ByteRegion>(
    region: &mut R,
    header: &SectionHeader,
    position: u64,
    generation: u64,
) -> CuResult<()> {
    write_commit(region, header, position, generation)?;
    write_header(
        region,
        &SectionDescriptor::new(header, position, generation),
    )
}
pub fn read_header<B: ByteStorage>(storage: &B, offset: u64) -> CuResult<SectionHeader> {
    let descriptor = read_descriptor(storage, offset)?;
    let mut bytes = [0u8; 512];
    storage.region(offset + BLOCK, BLOCK)?.read(0, &mut bytes)?;
    let (commit, _) = decode_from_slice::<SectionCommit, _>(&bytes, standard())
        .map_err(|e| CuError::from("Invalid section commit").add_cause(&e.to_string()))?;
    if commit.position != offset
        || commit.generation != descriptor.generation
        || commit.used as u64 > descriptor.allocated - HEADER
    {
        return Err(CuError::from(
            "Invalid committed length or section generation",
        ));
    }
    if descriptor.next_section != 0 {
        let next = read_descriptor(storage, descriptor.next_section)?;
        if next.generation != descriptor.next_generation {
            return Err(CuError::from("Stale section link generation"));
        }
    }
    Ok(SectionHeader {
        magic: descriptor.magic,
        block_size: descriptor.block_size,
        entry_type: descriptor.entry_type,
        allocated: descriptor.allocated,
        next_section: descriptor.next_section,
        context: descriptor.context,
        used: commit.used,
        is_open: commit.is_open,
    })
}

pub fn read_main<B: ByteStorage>(storage: &B) -> CuResult<MainHeader> {
    let mut bytes = [0u8; 512];
    storage.region(0, BLOCK)?.read(0, &mut bytes)?;
    let (header, _) = decode_from_slice::<MainHeader, _>(&bytes, standard())
        .map_err(|e| CuError::from("Invalid main header").add_cause(&e.to_string()))?;
    if header.magic != MAIN_MAGIC
        || header.format_version != UNIFIED_LOG_FORMAT_VERSION
        || header.page_size == 0
        || !(header.page_size as u64).is_multiple_of(BLOCK)
        || header.sections_begin < header.page_size as u64
        || header.sections_begin > header.sections_end
        || header.sections_end > storage.len()
        || !header.sections_begin.is_multiple_of(BLOCK)
        || !header.sections_end.is_multiple_of(BLOCK)
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
    offset: u64,
    write_reserved: bool,
    append_only: bool,
}
impl<R: ByteRegion> ByteSection<R> {
    fn token(&self, state: usize) -> usize {
        self.generation * GENERATION_STRIDE + state
    }
    fn release_write(&mut self, state: usize) -> CuResult<()> {
        // Append-only storage belongs to this handle until flush/drop. There is
        // no reclaimer to compete with between records, including after errors.
        if self.append_only {
            if state == SEALED {
                self.leases[self.slot].store(self.token(SEALED), Ordering::Release);
            }
            return Ok(());
        }
        self.leases[self.slot]
            .compare_exchange(
                self.token(BUSY),
                self.token(state),
                Ordering::Release,
                Ordering::Relaxed,
            )
            .map(|_| ())
            .map_err(|_| {
                self.leases.poisoned.store(true, Ordering::Release);
                CuError::from("Section lease changed during write")
            })
    }
}
impl<R: ByteRegion> Drop for ByteSection<R> {
    fn drop(&mut self) {
        if self.append_only && self.header.is_open {
            let _ = self.flush();
        } else if self.write_reserved {
            self.region.abort();
            let _ = self.release_write(IDLE);
        }
    }
}
struct SectionWrite<'a, R: ByteRegion> {
    section: &'a mut ByteSection<R>,
    released: bool,
    observing: bool,
}
impl<R: ByteRegion> SectionWrite<'_, R> {
    fn finish(mut self, state: usize) -> CuResult<()> {
        self.released = true;
        self.section.release_write(state)
    }
}
impl<R: ByteRegion> Drop for SectionWrite<'_, R> {
    fn drop(&mut self) {
        if !self.released {
            if self.observing {
                abort_observed_encode();
            }
            self.section.region.abort();
            let _ = self.section.release_write(IDLE);
        }
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
    fn reserve_write(&mut self) -> CuResult<()> {
        if self.write_reserved {
            return Err(CuError::from("Section writing rights already reserved"));
        }
        if self.append_only {
            if !self.header.is_open {
                return Err(CuError::from("Cannot reserve a closed section"));
            }
            self.write_reserved = true;
            return Ok(());
        }
        self.leases[self.slot]
            .compare_exchange(
                self.token(IDLE),
                self.token(BUSY),
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .map_err(|_| CuError::from("Cannot reserve revoked section writing rights"))?;
        self.write_reserved = true;
        Ok(())
    }
    fn is_sealed(&self) -> bool {
        if self.append_only {
            return !self.header.is_open;
        }
        self.leases[self.slot].load(Ordering::Acquire) != self.token(IDLE)
    }
    fn append<E: Encode>(&mut self, entry: &E) -> Result<usize, EncodeError> {
        match self.append_or_rebind(entry)? {
            SectionAppendOutcome::Appended(count) => Ok(count),
            SectionAppendOutcome::Rebind => {
                Err(EncodeError::Other("Section lease is sealed or busy"))
            }
        }
    }
    fn append_or_rebind<E: Encode>(
        &mut self,
        entry: &E,
    ) -> Result<SectionAppendOutcome, EncodeError> {
        if self.leases.poisoned.load(Ordering::Acquire) {
            return Err(EncodeError::Other("Logger faulted after a storage failure"));
        }
        if self.append_only && !self.header.is_open {
            return Ok(SectionAppendOutcome::Rebind);
        }
        if !core::mem::take(&mut self.write_reserved)
            && !self.append_only
            && self.leases[self.slot]
                .compare_exchange(
                    self.token(IDLE),
                    self.token(BUSY),
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_err()
        {
            return Ok(SectionAppendOutcome::Rebind);
        }
        let guard = SectionWrite {
            section: self,
            released: false,
            observing: true,
        };
        let section = &mut *guard.section;
        begin_observed_encode();
        let old_position = section.position;
        let result = (|| {
            let writer = RegionWriter {
                region: &mut section.region,
                position: old_position,
                limit: section.header.allocated,
            };
            let mut encoder = EncoderImpl::new(ObservedWriter::new(writer), standard());
            entry.encode(&mut encoder)?;
            let end = encoder.into_writer().into_inner().position;
            section.region.commit().map_err(encode_error)?;
            let mut committed = section.header.clone();
            committed.used = u32::try_from(end - HEADER).map_err(|_| EncodeError::UnexpectedEnd)?;
            if let Err(error) = write_commit(
                &mut section.region,
                &committed,
                section.offset,
                section.generation as u64,
            ) {
                section.leases.poisoned.store(true, Ordering::Release);
                return Err(encode_error(error));
            }
            section.header = committed;
            section.position = end;
            Ok((end - old_position) as usize)
        })();
        match &result {
            Ok(count) => {
                let observed = finish_observed_encode();
                debug_assert_eq!(*count, observed);
            }
            Err(_) => {
                abort_observed_encode();
                section.region.abort();
            }
        }
        let release = guard.finish(IDLE).map_err(encode_error);
        result.and_then(|count| release.map(|()| SectionAppendOutcome::Appended(count)))
    }
    fn flush(&mut self) -> CuResult<usize> {
        if (self.append_only && self.header.is_open)
            || core::mem::take(&mut self.write_reserved)
            || (!self.append_only
                && self.leases[self.slot]
                    .compare_exchange(
                        self.token(IDLE),
                        self.token(BUSY),
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    )
                    .is_ok())
        {
            self.write_reserved = false;
            let guard = SectionWrite {
                section: self,
                released: false,
                observing: false,
            };
            let section = &mut *guard.section;
            let mut header = section.header.clone();
            header.is_open = false;
            let result = write_commit(
                &mut section.region,
                &header,
                section.offset,
                section.generation as u64,
            )
            .and_then(|()| section.region.flush());
            let state = if result.is_ok() {
                section.header = header;
                SEALED
            } else {
                section.leases.poisoned.store(true, Ordering::Release);
                IDLE
            };
            let release = guard.finish(state);
            result.and(release)?;
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
    // First byte beyond every retained extent. Grow never reuses earlier gaps.
    allocation_end: u64,
    #[cfg(test)]
    extent_visits: AtomicUsize,
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
            || !(alignment as u64).is_multiple_of(BLOCK)
            || capacity <= alignment as u64
            || !capacity.is_multiple_of(BLOCK)
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
        let allocation_end = header.sections_begin;
        Self {
            storage,
            header,
            policy,
            capacity,
            leases: Arc::new(LeaseTable {
                slots: core::array::from_fn(|_| Lease {
                    state: AtomicUsize::new(0),
                    next: UnsafeCell::new(0),
                    allocated: UnsafeCell::new(0),
                    offset: UnsafeCell::new(0),
                }),
                poisoned: AtomicBool::new(false),
            }),
            generation: 0,
            next_run_id: 1,
            retained_space: 0,
            allocation_end,
            #[cfg(test)]
            extent_visits: AtomicUsize::new(0),
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
        if !capacity.is_multiple_of(BLOCK) || capacity <= header.sections_begin {
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
            logger.generation = logger.generation.max(
                usize::try_from(read_descriptor(&logger.storage, cursor)?.generation)
                    .map_err(|_| CuError::from("Section generation exceeds platform limits"))?,
            );
            logger.allocation_end = logger.allocation_end.max(cursor + section.allocated);
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
    fn section_extent(&self, offset: u64) -> CuResult<(u64, u64)> {
        #[cfg(test)]
        self.extent_visits.fetch_add(1, Ordering::Relaxed);
        if let Some(lease) = self.leases.iter().find(|lease| lease.offset() == offset) {
            return Ok((lease.allocated(), lease.next()));
        }
        let header = self.data_header_unlocked(offset)?;
        Ok((header.allocated, header.next_section))
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
            || !offset.is_multiple_of(BLOCK)
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
                    || !h.next_section.is_multiple_of(BLOCK)))
        {
            return Err(CuError::from("Invalid section bounds or next link"));
        }
        Ok(h)
    }
    fn publish(&mut self) -> CuResult<()> {
        let result = (|| {
            let mut region = self.storage.region(0, BLOCK)?;
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
                let generation = read_descriptor(&self.storage, offset)?.generation;
                write_commit(
                    &mut self.storage.region(offset, HEADER)?,
                    &h,
                    offset,
                    generation,
                )?;
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
        initialize_section(
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
            offset,
            0,
        )
    }
    pub fn close(&mut self) -> CuResult<()> {
        if self.append_pending {
            return Ok(());
        }
        if self.faulted || self.leases.poisoned.load(Ordering::Acquire) {
            return Err(CuError::from("Cannot cleanly close a faulted logger"));
        }
        // Live append-only handles own their commit block for their lifetime.
        // Close cannot seal them behind an encoder's back; callers flush/drop first.
        if self.policy != CapacityPolicy::OverwriteOldest
            && self
                .leases
                .iter()
                .any(|lease| lease.load(Ordering::Acquire) % 4 == BUSY)
        {
            return Err(CuError::from(
                "Flush or drop live append-only sections before closing the logger",
            ));
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
    size.checked_add(BLOCK - 1)
        .map(|s| s / BLOCK * BLOCK)
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
        self.allocation_end = end;
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
        let mut position = if self.policy == CapacityPolicy::Grow {
            self.allocation_end
        } else if tail == 0 {
            self.header.sections_begin
        } else {
            tail + self.section_extent(tail)?.0
        };
        if self.policy == CapacityPolicy::OverwriteOldest
            && position
                .checked_add(size)
                .is_none_or(|end| end > self.header.sections_end)
        {
            position = self.header.sections_begin;
        }
        let end = position
            .checked_add(size)
            .ok_or_else(|| CuError::from("Section extent overflow"))?;
        let overlaps = |offset: u64, allocated: u64| position < offset + allocated && offset < end;
        let mut last_overlap = 0;
        // Append-only growth starts beyond every occupied extent, including
        // after reopening a wrapped ring. Only reclamation needs a chain walk.
        if self.policy != CapacityPolicy::Grow {
            let mut cursor = self.header.head_section;
            let mut remaining = capacity / HEADER;
            while cursor != 0 {
                if remaining == 0 {
                    return Err(CuError::from("Cyclic section links"));
                }
                remaining -= 1;
                let (allocated, next) = self.section_extent(cursor)?;
                if overlaps(cursor, allocated) {
                    last_overlap = cursor;
                }
                cursor = next;
            }
        }
        if self.policy == CapacityPolicy::StopWhenFull
            && (last_overlap != 0 || end > self.header.sections_end)
        {
            return Err(CuError::from("Log is full"));
        }
        // Claim every affected encoder before any write. A failed preflight restores all leases.
        let mut claims = Claims::new(self.leases.clone());
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
        if end <= self.storage.len() {
            let _ = self.storage.region(position, size)?;
        }
        self.activate()?;
        self.faulted = true;
        if end > self.header.sections_end {
            self.storage.grow(end)?;
            self.header.sections_end = self.storage.len();
        }
        if last_overlap != 0 {
            let mut cursor = self.header.head_section;
            loop {
                let mut h = self.data_header_unlocked(cursor)?;
                h.is_open = false;
                let generation = read_descriptor(&self.storage, cursor)?.generation;
                write_commit(
                    &mut self.storage.region(cursor, HEADER)?,
                    &h,
                    cursor,
                    generation,
                )?;
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
            // A writer may finish between loads. Classify one snapshot and reuse
            // only unassigned or sealed slots; idle streams retain their generation.
            .position(|s| matches!(s.load(Ordering::Acquire) % 4, 0 | SEALED))
            .ok_or_else(|| CuError::from("Too many concurrent section writes"))?;
        self.generation = self
            .generation
            .checked_add(1)
            .filter(|g| *g < usize::MAX / GENERATION_STRIDE)
            .ok_or_else(|| CuError::from("Section generation exhausted"))?;
        let header = SectionHeader {
            entry_type: kind,
            allocated: size,
            context,
            ..SectionHeader::default()
        };
        let mut region = self.storage.region(position, size)?;
        initialize_section(&mut region, &header, position, self.generation as u64)?;
        if self.header.tail_section != 0 {
            let tail = self.header.tail_section;
            let mut previous = read_descriptor(&self.storage, tail)?;
            previous.next_section = position;
            previous.next_generation = self.generation as u64;
            write_header(&mut self.storage.region(tail, BLOCK)?, &previous)?;
            if let Some(slot) = self.leases.iter().position(|lease| lease.offset() == tail) {
                self.leases[slot].set_next(position);
            }
        }
        if self.header.head_section == 0 {
            self.header.head_section = position;
        }
        self.header.tail_section = position;
        self.header.clean_close = false;
        self.publish()?;
        self.retained_space += size;
        self.allocation_end = self.allocation_end.max(end);
        for lease in self.leases.iter() {
            if lease.offset() == position {
                lease.set_offset(0);
            }
        }
        self.leases[slot].set_offset(position);
        self.leases[slot].set_allocated(size);
        self.leases[slot].set_next(0);
        self.leases[slot].store(
            self.generation * GENERATION_STRIDE
                + if self.policy == CapacityPolicy::OverwriteOldest {
                    IDLE
                } else {
                    BUSY
                },
            Ordering::Release,
        );
        let storage = ByteSection {
            region,
            position: HEADER,
            header: header.clone(),
            leases: self.leases.clone(),
            slot,
            generation: self.generation,
            offset: position,
            write_reserved: false,
            append_only: self.policy != CapacityPolicy::OverwriteOldest,
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

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use crate::{
        LogStream, UnifiedLogRead, UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerRead,
    };
    use cu29_traits::WriteStream;
    use std::cell::{Cell, RefCell};
    use std::sync::Mutex;

    pub(super) enum LeaseEvent {
        Loaded(usize),
        BeforeCompareExchange,
        AfterCompareExchange,
    }

    type LeaseHook = Box<dyn FnMut(&Lease, LeaseEvent)>;
    std::thread_local! {
        static LEASE_HOOK: RefCell<Option<LeaseHook>> = RefCell::new(None);
    }

    pub(super) fn lease_event(lease: &Lease, event: LeaseEvent) {
        LEASE_HOOK.with(|cell| {
            let mut hook = cell.borrow_mut().take();
            if let Some(hook) = hook.as_mut() {
                hook(lease, event);
            }
            *cell.borrow_mut() = hook;
        });
    }

    fn saved_values(path: &std::path::Path) -> Vec<u32> {
        let mut reader = UnifiedLoggerRead::new(path).unwrap();
        let mut saved = Vec::new();
        while let Some(bytes) = reader
            .read_next_section_type(UnifiedLogType::CopperList)
            .unwrap()
        {
            let mut bytes = bytes.as_slice();
            while !bytes.is_empty() {
                let (value, used) = decode_from_slice::<u32, _>(bytes, standard()).unwrap();
                saved.push(value);
                bytes = &bytes[used..];
            }
        }
        saved
    }

    fn logger(path: &std::path::Path, policy: CapacityPolicy) -> crate::UnifiedLoggerWrite {
        match UnifiedLoggerBuilder::new()
            .file_base_name(path)
            .preallocated_size(16384)
            .capacity(16384, policy)
            .write(true)
            .create(true)
            .build()
            .unwrap()
        {
            UnifiedLogger::Write(logger) => logger,
            _ => unreachable!(),
        }
    }

    #[test]
    fn test_reclamation_cas_is_absent_from_append_only_records() {
        let dir = tempfile::tempdir().unwrap();
        for policy in [
            CapacityPolicy::Grow,
            CapacityPolicy::StopWhenFull,
            CapacityPolicy::OverwriteOldest,
        ] {
            let path = dir.path().join(format!("cas-{policy:?}.copper"));
            let mut logger = logger(&path, policy);
            let mut section = logger
                .add_section(UnifiedLogType::CopperList, 4096)
                .unwrap();
            let calls = Arc::new(AtomicUsize::new(0));
            let observed = calls.clone();
            LEASE_HOOK.with(|cell| {
                *cell.borrow_mut() = Some(Box::new(move |_, event| {
                    if matches!(event, LeaseEvent::BeforeCompareExchange) {
                        observed.fetch_add(1, Ordering::Relaxed);
                    }
                }))
            });
            for value in 0..100u32 {
                section.append(value).unwrap();
            }
            LEASE_HOOK.with(|cell| cell.borrow_mut().take());
            assert_eq!(
                calls.load(Ordering::Relaxed),
                if policy == CapacityPolicy::OverwriteOldest {
                    200
                } else {
                    0
                }
            );
            logger.try_flush_section(&mut section).unwrap();
            logger.close().unwrap();
            assert_eq!(saved_values(&path), (0..100).collect::<Vec<_>>());
        }
    }

    #[test]
    fn test_append_only_close_waits_for_the_owner_and_drop_seals_it() {
        let dir = tempfile::tempdir().unwrap();
        for policy in [CapacityPolicy::Grow, CapacityPolicy::StopWhenFull] {
            let path = dir.path().join(format!("close-{policy:?}.copper"));
            let mut logger = logger(&path, policy);
            let mut section = logger
                .add_section(UnifiedLogType::CopperList, 2048)
                .unwrap();
            section.append(41u32).unwrap();
            assert!(logger.close().is_err());
            section.append(42u32).unwrap();
            drop(section);
            logger.close().unwrap();
            assert_eq!(saved_values(&path), [41, 42]);
        }
    }

    #[test]
    fn test_metadata_owners_fit_independent_physical_blocks() {
        let header = SectionHeader {
            entry_type: UnifiedLogType::CopperList,
            allocated: u64::MAX,
            next_section: u64::MAX,
            used: u32::MAX,
            context: SectionContext {
                run_id: u64::MAX,
                instance_id: u32::MAX,
                mission_index: u32::MAX,
            },
            ..SectionHeader::default()
        };
        let mut descriptor = SectionDescriptor::new(&header, u64::MAX, u64::MAX);
        descriptor.next_generation = u64::MAX;
        let commit = SectionCommit {
            position: u64::MAX,
            generation: u64::MAX,
            used: u32::MAX,
            is_open: true,
        };
        let mut block = [0u8; 512];
        assert!(encode_into_slice(descriptor, &mut block, standard()).unwrap() < block.len());
        assert!(encode_into_slice(commit, &mut block, standard()).unwrap() < block.len());
    }

    #[test]
    fn test_grow_allocation_does_not_visit_retained_extents() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("allocation-visits.copper");
        let mut logger = logger(&path, CapacityPolicy::Grow);
        for value in 0..512u32 {
            let visits = logger.extent_visits.load(Ordering::Relaxed);
            let mut section = logger
                .add_section(UnifiedLogType::CopperList, 2048)
                .unwrap();
            assert_eq!(
                logger.extent_visits.load(Ordering::Relaxed),
                visits,
                "Grow allocation visited retained geometry after {value} sections"
            );
            section.append(value).unwrap();
            logger.try_flush_section(&mut section).unwrap();
        }
        drop(logger);
        assert_eq!(saved_values(&path), (0..512).collect::<Vec<_>>());
    }

    #[test]
    fn test_grow_after_wrap_uses_free_suffix_without_visiting_retained_extents() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("wrapped-allocation-visits.copper");
        {
            let mut logger = logger(&path, CapacityPolicy::OverwriteOldest);
            for value in 0..8u32 {
                let mut section = logger
                    .add_section(UnifiedLogType::CopperList, 2048)
                    .unwrap();
                section.append(value).unwrap();
                logger.try_flush_section(&mut section).unwrap();
            }
            assert_eq!(logger.header.tail_section, BLOCK, "the log must wrap");
        }
        let mut logger = match UnifiedLoggerBuilder::new()
            .file_base_name(&path)
            .preallocated_size(16384)
            .write(true)
            .create(true)
            .append(true)
            .build()
            .unwrap()
        {
            UnifiedLogger::Write(logger) => logger,
            _ => unreachable!(),
        };
        let mut section = logger
            .add_section(UnifiedLogType::CopperList, 2048)
            .unwrap();
        assert_eq!(logger.extent_visits.load(Ordering::Relaxed), 0);
        assert_eq!(
            logger.header.tail_section,
            BLOCK + 7 * 2048,
            "Grow must use the free suffix beyond the highest retained extent"
        );
        section.append(8u32).unwrap();
        logger.try_flush_section(&mut section).unwrap();
        drop(logger);
        assert_eq!(saved_values(&path), (1..9).collect::<Vec<_>>());
    }

    #[test]
    fn test_writer_finishing_during_slot_selection_keeps_its_generation() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("selection.copper");
        let mut logger = logger(&path, CapacityPolicy::OverwriteOldest);
        let mut first = logger
            .add_section(UnifiedLogType::CopperList, 2048)
            .unwrap();
        first.append(41u32).unwrap();
        let section = first.get_storage();
        let leases = section.leases.clone();
        let slot = section.slot;
        let idle = section.token(IDLE);
        leases[slot].store(section.token(BUSY), Ordering::Release);
        let loads = Arc::new(AtomicUsize::new(0));
        let observed = loads.clone();
        LEASE_HOOK.with(|cell| {
            *cell.borrow_mut() = Some(Box::new(move |lease, event| {
                if core::ptr::eq(lease, &leases[slot])
                    && let LeaseEvent::Loaded(state) = event
                    && state % 4 == BUSY
                    && observed.fetch_add(1, Ordering::Relaxed) == 1
                {
                    // Finish after the final slot-selection load has observed BUSY.
                    lease.store(idle, Ordering::Release);
                }
            }));
        });
        let mut next = logger
            .add_section(UnifiedLogType::CopperList, 2048)
            .unwrap();
        LEASE_HOOK.with(|cell| cell.borrow_mut().take());
        assert_eq!(loads.load(Ordering::Relaxed), 2);
        assert_ne!(
            next.get_storage().slot,
            slot,
            "an idle live lease cannot be reused"
        );
        first.append(42u32).unwrap();
        next.append(43u32).unwrap();
        logger.try_flush_section(&mut first).unwrap();
        logger.try_flush_section(&mut next).unwrap();
        drop(logger);
        assert_eq!(saved_values(&path), [41, 42, 43]);
    }

    #[test]
    fn test_sealing_between_stream_precheck_and_acquisition_rebinds() {
        struct Counted<'a>(&'a Cell<usize>);
        impl Encode for Counted<'_> {
            fn encode<E: bincode::enc::Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                self.0.set(self.0.get() + 1);
                42u32.encode(encoder)
            }
        }
        let dir = tempfile::tempdir().unwrap();
        for policy in [CapacityPolicy::OverwriteOldest] {
            for empty in [false, true] {
                let path = dir
                    .path()
                    .join(format!("acquisition-{policy:?}-{empty}.copper"));
                let logger = Arc::new(Mutex::new(logger(&path, policy)));
                let mut stream =
                    LogStream::new(UnifiedLogType::CopperList, logger.clone(), 2048).unwrap();
                if !empty {
                    stream.log(&41u32).unwrap();
                }
                let section = &stream.current_section.storage;
                let leases = section.leases.clone();
                let slot = section.slot;
                let idle = section.token(IDLE);
                let injected = Arc::new(AtomicBool::new(false));
                let observed = injected.clone();
                let mut pending = false;
                LEASE_HOOK.with(|cell| {
                    *cell.borrow_mut() = Some(Box::new(move |lease, event| {
                        if !core::ptr::eq(lease, &leases[slot]) {
                            return;
                        }
                        match event {
                            LeaseEvent::BeforeCompareExchange
                                if !observed.load(Ordering::Relaxed) =>
                            {
                                // Reclamation seals the idle section before acquisition.
                                lease.store(
                                    idle / GENERATION_STRIDE * GENERATION_STRIDE + SEALED,
                                    Ordering::Release,
                                );
                                pending = true;
                                observed.store(true, Ordering::Relaxed);
                            }
                            LeaseEvent::AfterCompareExchange if pending => {
                                // Publication completes before the stream handles the failed CAS.
                                lease.store(idle, Ordering::Release);
                                pending = false;
                            }
                            _ => {}
                        }
                    }));
                });
                let encodes = Cell::new(0);
                let result = stream.log(&Counted(&encodes));
                LEASE_HOOK.with(|cell| cell.borrow_mut().take());
                assert!(injected.load(Ordering::Relaxed));
                result.unwrap();
                assert_eq!(encodes.get(), 1, "rebinding must precede encoding");
                drop(stream);
                drop(logger);
                assert_eq!(
                    saved_values(&path),
                    if empty { vec![42] } else { vec![41, 42] }
                );
            }
        }
    }

    #[test]
    fn test_reclamation_before_acquisition_rebinds_a_stale_stream() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("revocation.copper");
        let logger = Arc::new(Mutex::new(logger(&path, CapacityPolicy::OverwriteOldest)));
        let mut stream = LogStream::new(UnifiedLogType::CopperList, logger.clone(), 2048).unwrap();
        stream.log(&41u32).unwrap();
        // Leave the next allocation to wrap and reclaim the idle first stream.
        for value in 0..6u32 {
            let mut logger = logger.lock().unwrap();
            let mut section = logger
                .add_section(UnifiedLogType::CopperList, 2048)
                .unwrap();
            section.append(value).unwrap();
            logger.try_flush_section(&mut section).unwrap();
        }
        let first = &stream.current_section.storage;
        let leases = first.leases.clone();
        let slot = first.slot;
        let parent = logger.clone();
        let mut injected = false;
        LEASE_HOOK.with(|cell| {
            *cell.borrow_mut() = Some(Box::new(move |lease, event| {
                if core::ptr::eq(lease, &leases[slot])
                    && matches!(event, LeaseEvent::BeforeCompareExchange)
                    && !injected
                {
                    injected = true;
                    let mut logger = parent.lock().unwrap();
                    let mut section = logger
                        .add_section(UnifiedLogType::CopperList, 2048)
                        .unwrap();
                    section.append(43u32).unwrap();
                    logger.try_flush_section(&mut section).unwrap();
                }
            }));
        });
        let result = stream.log(&42u32);
        LEASE_HOOK.with(|cell| cell.borrow_mut().take());
        result.unwrap();
        drop(stream);
        drop(logger);
        assert_eq!(saved_values(&path), [1, 2, 3, 4, 5, 43, 42]);
    }

    #[test]
    fn test_stream_encoding_error_keeps_its_section_and_committed_prefix() {
        struct Failing;
        impl Encode for Failing {
            fn encode<E: bincode::enc::Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                99u32.encode(encoder)?;
                Err(EncodeError::Other("injected encode failure"))
            }
        }
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("failure.copper");
        let logger = Arc::new(Mutex::new(logger(&path, CapacityPolicy::Grow)));
        let mut stream = LogStream::new(UnifiedLogType::CopperList, logger.clone(), 2048).unwrap();
        stream.log(&41u32).unwrap();
        let slot = stream.current_section.storage.slot;
        let generation = stream.current_section.storage.generation;
        assert!(stream.log(&Failing).is_err());
        assert_eq!(stream.current_section.storage.slot, slot);
        assert_eq!(stream.current_section.storage.generation, generation);
        stream.log(&42u32).unwrap();
        drop(stream);
        drop(logger);
        assert_eq!(saved_values(&path), [41, 42]);
    }
}
