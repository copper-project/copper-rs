use arrayvec::ArrayString;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29_traits::CuResult;
use hashbrown::HashMap;
use object_pool::{Pool, ReusableOwned};
use serde::de::{self, MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use smallvec::SmallVec;
use std::alloc::{Layout, alloc, dealloc};
use std::cell::Cell;
use std::cell::UnsafeCell;
use std::fmt::Debug;
use std::fs::OpenOptions;
use std::marker::PhantomData;
use std::mem::{align_of, size_of};
use std::ops::{Deref, DerefMut};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

use memmap2::{MmapMut, MmapOptions};
use tempfile::NamedTempFile;

type PoolID = ArrayString<64>;

/// Trait for a Pool to exposed to be monitored by the monitoring API.
pub trait PoolMonitor: Send + Sync {
    /// A unique and descriptive identifier for the pool.
    fn id(&self) -> PoolID;

    /// Number of buffer slots left in the pool.
    fn space_left(&self) -> usize;

    /// Total size of the pool in number of buffers.
    fn total_size(&self) -> usize;

    /// Size of one buffer
    fn buffer_size(&self) -> usize;
}

static POOL_REGISTRY: OnceLock<Mutex<HashMap<String, Arc<dyn PoolMonitor>>>> = OnceLock::new();
const MAX_POOLS: usize = 16;

fn lock_unpoison<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    match mutex.lock() {
        Ok(guard) => guard,
        Err(poison) => poison.into_inner(),
    }
}

// Register a pool to the global registry.
fn register_pool(pool: Arc<dyn PoolMonitor>) {
    POOL_REGISTRY
        .get_or_init(|| Mutex::new(HashMap::new()))
        .lock()
        .unwrap_or_else(|poison| poison.into_inner())
        .insert(pool.id().to_string(), pool);
}

type PoolStats = (PoolID, usize, usize, usize);

/// Get the list of pools and their statistics.
/// We use SmallVec here to avoid heap allocations while the stack is running.
pub fn pools_statistics() -> SmallVec<[PoolStats; MAX_POOLS]> {
    // Safely get the registry, returning empty stats if not initialized.
    let registry_lock = match POOL_REGISTRY.get() {
        Some(lock) => lock_unpoison(lock),
        None => return SmallVec::new(), // Return empty if registry is not initialized
    };
    let mut result = SmallVec::with_capacity(MAX_POOLS);
    for pool in registry_lock.values() {
        result.push((
            pool.id(),
            pool.space_left(),
            pool.total_size(),
            pool.buffer_size(),
        ));
    }
    result
}

/// Basic Type that can be used in a buffer in a CuPool.
pub trait ElementType: Default + Sized + Copy + Debug + Unpin + Send + Sync {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError>;
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError>;
}

/// Blanket implementation for all types that are Sized, Copy, Encode, Decode and Debug.
impl<T> ElementType for T
where
    T: Default + Sized + Copy + Debug + Unpin + Send + Sync,
    T: Encode,
    T: Decode<()>,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.encode(encoder)
    }

    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Self::decode(decoder)
    }
}

pub trait ArrayLike: Deref<Target = [Self::Element]> + DerefMut + Debug + Sync + Send {
    type Element: ElementType;
}

thread_local! {
    static SHARED_HANDLE_SERIALIZATION_ENABLED: Cell<bool> = const { Cell::new(false) };
}

pub struct SharedHandleSerializationGuard {
    previous: bool,
}

impl Drop for SharedHandleSerializationGuard {
    fn drop(&mut self) {
        SHARED_HANDLE_SERIALIZATION_ENABLED.with(|enabled| enabled.set(self.previous));
    }
}

pub fn enable_shared_handle_serialization() -> SharedHandleSerializationGuard {
    let previous = SHARED_HANDLE_SERIALIZATION_ENABLED.with(|enabled| {
        let previous = enabled.get();
        enabled.set(true);
        previous
    });
    SharedHandleSerializationGuard { previous }
}

fn shared_handle_serialization_enabled() -> bool {
    SHARED_HANDLE_SERIALIZATION_ENABLED.with(Cell::get)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CuSharedMemoryElementType {
    U8,
    U16,
    U32,
    U64,
    I8,
    I16,
    I32,
    I64,
    F32,
    F64,
}

impl CuSharedMemoryElementType {
    pub fn of<E: ElementType + 'static>() -> Option<Self> {
        let type_id = core::any::TypeId::of::<E>();
        if type_id == core::any::TypeId::of::<u8>() {
            Some(Self::U8)
        } else if type_id == core::any::TypeId::of::<u16>() {
            Some(Self::U16)
        } else if type_id == core::any::TypeId::of::<u32>() {
            Some(Self::U32)
        } else if type_id == core::any::TypeId::of::<u64>() {
            Some(Self::U64)
        } else if type_id == core::any::TypeId::of::<i8>() {
            Some(Self::I8)
        } else if type_id == core::any::TypeId::of::<i16>() {
            Some(Self::I16)
        } else if type_id == core::any::TypeId::of::<i32>() {
            Some(Self::I32)
        } else if type_id == core::any::TypeId::of::<i64>() {
            Some(Self::I64)
        } else if type_id == core::any::TypeId::of::<f32>() {
            Some(Self::F32)
        } else if type_id == core::any::TypeId::of::<f64>() {
            Some(Self::F64)
        } else {
            None
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CuSharedMemoryHandleDescriptor {
    #[serde(rename = "__cu_shm_handle__")]
    pub marker: bool,
    pub path: String,
    pub offset_bytes: usize,
    pub len_elements: usize,
    pub element_type: CuSharedMemoryElementType,
}

impl CuSharedMemoryHandleDescriptor {
    fn new(
        path: String,
        offset_bytes: usize,
        len_elements: usize,
        element_type: CuSharedMemoryElementType,
    ) -> Self {
        Self {
            marker: true,
            path,
            offset_bytes,
            len_elements,
            element_type,
        }
    }
}

struct CuSharedMemoryRegion {
    path: PathBuf,
    mmap: UnsafeCell<MmapMut>,
    _backing_file: Option<NamedTempFile>,
}

impl CuSharedMemoryRegion {
    fn create(byte_len: usize) -> CuResult<Arc<Self>> {
        let file = NamedTempFile::new()
            .map_err(|e| cu29_traits::CuError::new_with_cause("create shared memory file", e))?;
        file.as_file()
            .set_len(byte_len as u64)
            .map_err(|e| cu29_traits::CuError::new_with_cause("size shared memory file", e))?;
        let mmap = unsafe {
            MmapOptions::new()
                .len(byte_len)
                .map_mut(file.as_file())
                .map_err(|e| cu29_traits::CuError::new_with_cause("map shared memory file", e))?
        };
        let region = Arc::new(Self {
            path: file.path().to_path_buf(),
            mmap: UnsafeCell::new(mmap),
            _backing_file: Some(file),
        });
        cache_shared_region(region.clone());
        Ok(region)
    }

    fn open(path: &Path) -> CuResult<Arc<Self>> {
        if let Some(region) = cached_shared_region(path) {
            return Ok(region);
        }

        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .map_err(|e| cu29_traits::CuError::new_with_cause("open shared memory file", e))?;
        let len = file
            .metadata()
            .map_err(|e| cu29_traits::CuError::new_with_cause("stat shared memory file", e))?
            .len() as usize;
        let mmap = unsafe {
            MmapOptions::new()
                .len(len)
                .map_mut(&file)
                .map_err(|e| cu29_traits::CuError::new_with_cause("map shared memory file", e))?
        };
        let region = Arc::new(Self {
            path: path.to_path_buf(),
            mmap: UnsafeCell::new(mmap),
            _backing_file: None,
        });
        cache_shared_region(region.clone());
        Ok(region)
    }
}

impl Debug for CuSharedMemoryRegion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CuSharedMemoryRegion")
            .field("path", &self.path)
            .finish_non_exhaustive()
    }
}

// SAFETY:
// Access to the mapped bytes is mediated through Copper handles and pool slot
// leasing, so cross-thread aliasing follows the same external synchronization as
// other mutable payload buffers.
unsafe impl Send for CuSharedMemoryRegion {}
// SAFETY:
// See `Send` rationale above.
unsafe impl Sync for CuSharedMemoryRegion {}

fn shared_region_cache() -> &'static Mutex<HashMap<PathBuf, std::sync::Weak<CuSharedMemoryRegion>>>
{
    static CACHE: OnceLock<Mutex<HashMap<PathBuf, std::sync::Weak<CuSharedMemoryRegion>>>> =
        OnceLock::new();
    CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}

fn cache_shared_region(region: Arc<CuSharedMemoryRegion>) {
    lock_unpoison(shared_region_cache()).insert(region.path.clone(), Arc::downgrade(&region));
}

fn cached_shared_region(path: &Path) -> Option<Arc<CuSharedMemoryRegion>> {
    lock_unpoison(shared_region_cache())
        .get(path)
        .and_then(std::sync::Weak::upgrade)
}

fn shared_slot_stride<E: ElementType>(len_elements: usize) -> usize {
    let raw_bytes = len_elements
        .checked_mul(size_of::<E>())
        .expect("shared memory slot size overflow");
    let alignment = align_of::<E>().max(1);
    raw_bytes.div_ceil(alignment) * alignment
}

#[derive(Debug)]
pub struct CuSharedMemoryBuffer<E: ElementType> {
    region: Arc<CuSharedMemoryRegion>,
    offset_bytes: usize,
    len_elements: usize,
    _marker: PhantomData<E>,
}

impl<E: ElementType + 'static> CuSharedMemoryBuffer<E> {
    fn from_region(
        region: Arc<CuSharedMemoryRegion>,
        offset_bytes: usize,
        len_elements: usize,
    ) -> Self {
        Self {
            region,
            offset_bytes,
            len_elements,
            _marker: PhantomData,
        }
    }

    pub fn from_vec_detached(data: Vec<E>) -> CuResult<Self> {
        let len_elements = data.len();
        let slot_stride = shared_slot_stride::<E>(len_elements.max(1));
        let region = CuSharedMemoryRegion::create(slot_stride)?;
        let mut buffer = Self::from_region(region, 0, len_elements);
        if !data.is_empty() {
            buffer.copy_from_slice(&data);
        }
        Ok(buffer)
    }

    pub fn from_descriptor(descriptor: &CuSharedMemoryHandleDescriptor) -> CuResult<Self> {
        let expected = CuSharedMemoryElementType::of::<E>()
            .ok_or_else(|| cu29_traits::CuError::from("unsupported shared memory element type"))?;
        if descriptor.element_type != expected {
            return Err(cu29_traits::CuError::from(
                "shared memory descriptor element type mismatch",
            ));
        }
        let region = CuSharedMemoryRegion::open(Path::new(&descriptor.path))?;
        Ok(Self::from_region(
            region,
            descriptor.offset_bytes,
            descriptor.len_elements,
        ))
    }

    pub fn descriptor(&self) -> Option<CuSharedMemoryHandleDescriptor>
    where
        E: 'static,
    {
        CuSharedMemoryElementType::of::<E>().map(|element_type| {
            CuSharedMemoryHandleDescriptor::new(
                self.region.path.display().to_string(),
                self.offset_bytes,
                self.len_elements,
                element_type,
            )
        })
    }
}

impl<E: ElementType> Deref for CuSharedMemoryBuffer<E> {
    type Target = [E];

    fn deref(&self) -> &Self::Target {
        let ptr = unsafe { (*self.region.mmap.get()).as_ptr().add(self.offset_bytes) as *const E };
        unsafe { std::slice::from_raw_parts(ptr, self.len_elements) }
    }
}

impl<E: ElementType> DerefMut for CuSharedMemoryBuffer<E> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        let ptr = unsafe {
            (*self.region.mmap.get())
                .as_mut_ptr()
                .add(self.offset_bytes) as *mut E
        };
        unsafe { std::slice::from_raw_parts_mut(ptr, self.len_elements) }
    }
}

impl<E: ElementType> ArrayLike for CuSharedMemoryBuffer<E> {
    type Element = E;
}

impl<E: ElementType> Encode for CuSharedMemoryBuffer<E> {
    fn encode<Enc: Encoder>(&self, encoder: &mut Enc) -> Result<(), EncodeError> {
        let len = self.len_elements as u64;
        Encode::encode(&len, encoder)?;
        for value in self.deref() {
            value.encode(encoder)?;
        }
        Ok(())
    }
}

impl<E: ElementType + 'static> Decode<()> for CuSharedMemoryBuffer<E> {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let len = <u64 as Decode<()>>::decode(decoder)? as usize;
        let mut vec = Vec::with_capacity(len);
        for _ in 0..len {
            vec.push(E::decode(decoder)?);
        }
        Self::from_vec_detached(vec).map_err(|e| DecodeError::OtherString(e.to_string()))
    }
}

/// A handle to a pooled or detached object.
///
/// For onboard usages, large payloads should typically be pooled. The detached form exists for
/// offline/deserialization flows and for payloads that are intentionally heap-backed instead of
/// pool-backed.
pub enum CuHandleInner<T: Debug + Send + Sync> {
    Pooled(ReusableOwned<Box<T>>),
    Detached(Box<T>), // Should only be used in offline cases (e.g. deserialization)
}

impl<T> Debug for CuHandleInner<T>
where
    T: Debug + Send + Sync,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CuHandleInner::Pooled(r) => {
                write!(f, "Pooled: {:?}", r.deref().deref())
            }
            CuHandleInner::Detached(r) => write!(f, "Detached: {r:?}"),
        }
    }
}

impl<T> CuHandleInner<T>
where
    T: Debug + Send + Sync,
{
    fn inner_ref(&self) -> &T {
        match self {
            CuHandleInner::Pooled(pooled) => pooled.deref().as_ref(),
            CuHandleInner::Detached(detached) => detached.deref(),
        }
    }

    fn inner_mut(&mut self) -> &mut T {
        match self {
            CuHandleInner::Pooled(pooled) => pooled.deref_mut().as_mut(),
            CuHandleInner::Detached(detached) => detached.deref_mut(),
        }
    }
}

impl<T> AsRef<T> for CuHandleInner<T>
where
    T: Debug + Send + Sync,
{
    fn as_ref(&self) -> &T {
        self.inner_ref()
    }
}

impl<T> AsMut<T> for CuHandleInner<T>
where
    T: Debug + Send + Sync,
{
    fn as_mut(&mut self) -> &mut T {
        self.inner_mut()
    }
}

impl<T: ArrayLike> Deref for CuHandleInner<T> {
    type Target = [T::Element];

    fn deref(&self) -> &Self::Target {
        self.inner_ref().deref()
    }
}

impl<T: ArrayLike> DerefMut for CuHandleInner<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.inner_mut().deref_mut()
    }
}

/// A shareable handle to a pooled or detached object.
///
/// When `T: ArrayLike`, the handle also participates in Copper's buffer pool APIs.
#[derive(Debug)]
pub struct CuHandle<T: Debug + Send + Sync>(Arc<Mutex<CuHandleInner<T>>>);

impl<T: Debug + Send + Sync> Clone for CuHandle<T> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<T: Debug + Send + Sync> Deref for CuHandle<T> {
    type Target = Arc<Mutex<CuHandleInner<T>>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T: Debug + Send + Sync> CuHandle<T> {
    /// Create a new CuHandle not part of a Pool (not for onboard usages, use pools instead)
    pub fn new_detached(inner: T) -> Self {
        Self::new_detached_box(Box::new(inner))
    }

    /// Create a detached handle from an already heap-allocated object.
    pub fn new_detached_box(inner: Box<T>) -> Self {
        CuHandle(Arc::new(Mutex::new(CuHandleInner::Detached(inner))))
    }

    /// Safely access the inner value, applying a closure to it.
    pub fn with_inner<R>(&self, f: impl FnOnce(&CuHandleInner<T>) -> R) -> R {
        let lock = lock_unpoison(&self.0);
        f(&*lock)
    }

    /// Mutably access the inner value, applying a closure to it.
    pub fn with_inner_mut<R>(&self, f: impl FnOnce(&mut CuHandleInner<T>) -> R) -> R {
        let mut lock = lock_unpoison(&self.0);
        f(&mut *lock)
    }
}

impl<U> Serialize for CuHandle<Vec<U>>
where
    U: ElementType + Serialize + 'static,
{
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let inner = lock_unpoison(&self.0);
        inner.inner_ref().serialize(serializer)
    }
}

impl<'de, U> Deserialize<'de> for CuHandle<Vec<U>>
where
    U: ElementType + Deserialize<'de> + 'static,
{
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        Vec::<U>::deserialize(deserializer).map(CuHandle::new_detached)
    }
}

impl<U> Serialize for CuHandle<CuSharedMemoryBuffer<U>>
where
    U: ElementType + Serialize + 'static,
{
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let inner = lock_unpoison(&self.0);
        let buffer = inner.inner_ref();

        if shared_handle_serialization_enabled()
            && let Some(descriptor) = buffer.descriptor()
        {
            return descriptor.serialize(serializer);
        }

        buffer.deref().serialize(serializer)
    }
}

impl<'de, U> Deserialize<'de> for CuHandle<CuSharedMemoryBuffer<U>>
where
    U: ElementType + Deserialize<'de> + 'static,
{
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        enum Repr<U> {
            Descriptor(CuSharedMemoryHandleDescriptor),
            Data(Vec<U>),
        }

        impl<'de, U> Deserialize<'de> for Repr<U>
        where
            U: ElementType + Deserialize<'de>,
        {
            fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
                struct ReprVisitor<U>(PhantomData<U>);

                impl<'de, U> Visitor<'de> for ReprVisitor<U>
                where
                    U: ElementType + Deserialize<'de>,
                {
                    type Value = Repr<U>;

                    fn expecting(
                        &self,
                        formatter: &mut std::fmt::Formatter<'_>,
                    ) -> std::fmt::Result {
                        formatter
                            .write_str("a shared-memory handle descriptor or an element sequence")
                    }

                    fn visit_seq<A: SeqAccess<'de>>(self, seq: A) -> Result<Self::Value, A::Error> {
                        let data =
                            Vec::<U>::deserialize(de::value::SeqAccessDeserializer::new(seq))?;
                        Ok(Repr::Data(data))
                    }

                    fn visit_map<A: MapAccess<'de>>(self, map: A) -> Result<Self::Value, A::Error> {
                        let descriptor = CuSharedMemoryHandleDescriptor::deserialize(
                            de::value::MapAccessDeserializer::new(map),
                        )?;
                        Ok(Repr::Descriptor(descriptor))
                    }
                }

                deserializer.deserialize_any(ReprVisitor(PhantomData))
            }
        }

        match Repr::<U>::deserialize(deserializer)? {
            Repr::Descriptor(descriptor) => CuSharedMemoryBuffer::from_descriptor(&descriptor)
                .map(CuHandle::new_detached)
                .map_err(de::Error::custom),
            Repr::Data(data) => CuSharedMemoryBuffer::from_vec_detached(data)
                .map(CuHandle::new_detached)
                .map_err(de::Error::custom),
        }
    }
}

impl<T: ArrayLike + Encode> Encode for CuHandle<T>
where
    <T as ArrayLike>::Element: 'static,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let inner = lock_unpoison(&self.0);
        crate::monitoring::record_payload_handle_bytes(
            inner.inner_ref().len() * size_of::<T::Element>(),
        );
        inner.inner_ref().encode(encoder)
    }
}

impl<T: Debug + Send + Sync> Default for CuHandle<T> {
    fn default() -> Self {
        panic!("Cannot create a default CuHandle")
    }
}

impl<U: ElementType + Decode<()> + 'static> Decode<()> for CuHandle<Vec<U>> {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let vec: Vec<U> = Vec::decode(decoder)?;
        Ok(CuHandle::new_detached(vec))
    }
}

impl<U: ElementType + Decode<()> + 'static> Decode<()> for CuHandle<CuSharedMemoryBuffer<U>> {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let buffer = CuSharedMemoryBuffer::<U>::decode(decoder)?;
        Ok(CuHandle::new_detached(buffer))
    }
}

/// A CuPool is a pool of buffers that can be shared between different parts of the code.
/// Handles can be stored locally in the tasks and shared between them.
pub trait CuPool<T: ArrayLike>: PoolMonitor {
    /// Acquire a buffer from the pool.
    fn acquire(&self) -> Option<CuHandle<T>>;

    /// Copy data from a handle to a new handle from the pool.
    fn copy_from<O>(&self, from: &mut CuHandle<O>) -> CuHandle<T>
    where
        O: ArrayLike<Element = T::Element>;
}

/// A device memory pool can copy data from a device to a host memory pool on top.
pub trait DeviceCuPool<T: ArrayLike>: CuPool<T> {
    /// Takes a handle to a device buffer and copies it into a host buffer pool.
    /// It returns a new handle from the host pool with the data from the device handle given.
    fn copy_to_host_pool<O>(
        &self,
        from_device_handle: &CuHandle<T>,
        to_host_handle: &mut CuHandle<O>,
    ) -> CuResult<()>
    where
        O: ArrayLike<Element = T::Element>;
}

/// A pool of host memory buffers.
pub struct CuHostMemoryPool<T> {
    /// Underlying pool of host buffers.
    // Being an Arc is a requirement of try_pull_owned() so buffers can refer back to the pool.
    id: PoolID,
    pool: Arc<Pool<Box<T>>>,
    size: usize,
    buffer_size: usize,
}

impl<T: ArrayLike + 'static> CuHostMemoryPool<T> {
    pub fn new<F>(id: &str, size: usize, buffer_initializer: F) -> CuResult<Arc<Self>>
    where
        F: Fn() -> T,
    {
        let pool = Arc::new(Pool::new(size, move || Box::new(buffer_initializer())));
        let buffer_size = pool.try_pull().unwrap().len() * size_of::<T::Element>();

        let og = Self {
            id: PoolID::from(id).map_err(|_| "Failed to create PoolID")?,
            pool,
            size,
            buffer_size,
        };
        let og = Arc::new(og);
        register_pool(og.clone());
        Ok(og)
    }
}

impl<T: ArrayLike> PoolMonitor for CuHostMemoryPool<T> {
    fn id(&self) -> PoolID {
        self.id
    }

    fn space_left(&self) -> usize {
        self.pool.len()
    }

    fn total_size(&self) -> usize {
        self.size
    }

    fn buffer_size(&self) -> usize {
        self.buffer_size
    }
}

impl<T: ArrayLike> CuPool<T> for CuHostMemoryPool<T> {
    fn acquire(&self) -> Option<CuHandle<T>> {
        let owned_object = self.pool.try_pull_owned(); // Use the owned version

        owned_object.map(|reusable| CuHandle(Arc::new(Mutex::new(CuHandleInner::Pooled(reusable)))))
    }

    fn copy_from<O: ArrayLike<Element = T::Element>>(&self, from: &mut CuHandle<O>) -> CuHandle<T> {
        let to_handle = self.acquire().expect("No available buffers in the pool");
        {
            let from_lock = lock_unpoison(&from.0);
            let mut to_lock = lock_unpoison(&to_handle.0);
            to_lock.inner_mut().copy_from_slice(from_lock.inner_ref());
        }
        to_handle
    }
}

/// A pool of fixed-size shared-memory buffers that can be leased to a child
/// process without copying the underlying bytes.
pub struct CuSharedMemoryPool<E: ElementType> {
    id: PoolID,
    pool: Arc<Pool<Box<CuSharedMemoryBuffer<E>>>>,
    size: usize,
    buffer_size: usize,
}

impl<E: ElementType + 'static> CuSharedMemoryPool<E> {
    pub fn new(id: &str, size: usize, elements_per_buffer: usize) -> CuResult<Arc<Self>> {
        let slot_stride = shared_slot_stride::<E>(elements_per_buffer.max(1));
        let region = CuSharedMemoryRegion::create(
            slot_stride
                .checked_mul(size)
                .ok_or_else(|| cu29_traits::CuError::from("shared memory pool size overflow"))?,
        )?;
        let next_slot = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let initializer_region = region.clone();
        let initializer_next_slot = next_slot.clone();
        let pool = Arc::new(Pool::new(size, move || {
            let slot = initializer_next_slot.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            assert!(slot < size, "shared memory pool slot index overflow");
            Box::new(CuSharedMemoryBuffer::from_region(
                initializer_region.clone(),
                slot * slot_stride,
                elements_per_buffer,
            ))
        }));

        let pool = Arc::new(Self {
            id: PoolID::from(id).map_err(|_| "Failed to create PoolID")?,
            pool,
            size,
            buffer_size: elements_per_buffer * size_of::<E>(),
        });
        register_pool(pool.clone());
        Ok(pool)
    }
}

impl<E: ElementType> PoolMonitor for CuSharedMemoryPool<E> {
    fn id(&self) -> PoolID {
        self.id
    }

    fn space_left(&self) -> usize {
        self.pool.len()
    }

    fn total_size(&self) -> usize {
        self.size
    }

    fn buffer_size(&self) -> usize {
        self.buffer_size
    }
}

impl<E: ElementType> CuPool<CuSharedMemoryBuffer<E>> for CuSharedMemoryPool<E> {
    fn acquire(&self) -> Option<CuHandle<CuSharedMemoryBuffer<E>>> {
        self.pool
            .try_pull_owned()
            .map(|reusable| CuHandle(Arc::new(Mutex::new(CuHandleInner::Pooled(reusable)))))
    }

    fn copy_from<O>(&self, from: &mut CuHandle<O>) -> CuHandle<CuSharedMemoryBuffer<E>>
    where
        O: ArrayLike<Element = E>,
    {
        let to_handle = self.acquire().expect("No available buffers in the pool");
        {
            let from_lock = lock_unpoison(&from.0);
            let mut to_lock = lock_unpoison(&to_handle.0);
            to_lock.inner_mut().copy_from_slice(from_lock.inner_ref());
        }
        to_handle
    }
}

impl<E: ElementType + 'static> ArrayLike for Vec<E> {
    type Element = E;
}

#[cfg(all(feature = "cuda", not(target_os = "macos")))]
mod cuda {
    use super::*;
    use cu29_traits::CuError;
    use cudarc::driver::{
        CudaContext, CudaSlice, CudaStream, DeviceRepr, HostSlice, SyncOnDrop, ValidAsZeroBits,
    };
    use std::sync::Arc;

    #[derive(Debug)]
    pub struct CudaSliceWrapper<E>(CudaSlice<E>);

    impl<E> Deref for CudaSliceWrapper<E>
    where
        E: ElementType,
    {
        type Target = [E];

        fn deref(&self) -> &Self::Target {
            // Implement logic to return a slice
            panic!("You need to copy data to host memory pool before accessing it.");
        }
    }

    impl<E> DerefMut for CudaSliceWrapper<E>
    where
        E: ElementType,
    {
        fn deref_mut(&mut self) -> &mut Self::Target {
            panic!("You need to copy data to host memory pool before accessing it.");
        }
    }

    impl<E: ElementType> ArrayLike for CudaSliceWrapper<E> {
        type Element = E;
    }

    impl<E> CudaSliceWrapper<E> {
        pub fn as_cuda_slice(&self) -> &CudaSlice<E> {
            &self.0
        }

        pub fn as_cuda_slice_mut(&mut self) -> &mut CudaSlice<E> {
            &mut self.0
        }
    }

    // Create a wrapper type to bridge between ArrayLike and HostSlice
    pub struct HostSliceWrapper<'a, T: ArrayLike> {
        inner: &'a T,
    }

    impl<T: ArrayLike> HostSlice<T::Element> for HostSliceWrapper<'_, T> {
        fn len(&self) -> usize {
            self.inner.len()
        }

        // SAFETY: HostSlice requires the returned slice to remain valid for 'b.
        unsafe fn stream_synced_slice<'b>(
            &'b self,
            stream: &'b CudaStream,
        ) -> (&'b [T::Element], SyncOnDrop<'b>) {
            (self.inner.deref(), SyncOnDrop::sync_stream(stream))
        }

        // SAFETY: This wrapper cannot provide mutable access; callers must not rely on this.
        unsafe fn stream_synced_mut_slice<'b>(
            &'b mut self,
            _stream: &'b CudaStream,
        ) -> (&'b mut [T::Element], SyncOnDrop<'b>) {
            panic!("Cannot get mutable reference from immutable wrapper")
        }
    }

    // Mutable wrapper
    pub struct HostSliceMutWrapper<'a, T: ArrayLike> {
        inner: &'a mut T,
    }

    impl<T: ArrayLike> HostSlice<T::Element> for HostSliceMutWrapper<'_, T> {
        fn len(&self) -> usize {
            self.inner.len()
        }

        // SAFETY: HostSlice requires the returned slice to remain valid for 'b.
        unsafe fn stream_synced_slice<'b>(
            &'b self,
            stream: &'b CudaStream,
        ) -> (&'b [T::Element], SyncOnDrop<'b>) {
            (self.inner.deref(), SyncOnDrop::sync_stream(stream))
        }

        // SAFETY: HostSlice requires the returned slice to remain valid for 'b.
        unsafe fn stream_synced_mut_slice<'b>(
            &'b mut self,
            stream: &'b CudaStream,
        ) -> (&'b mut [T::Element], SyncOnDrop<'b>) {
            (self.inner.deref_mut(), SyncOnDrop::sync_stream(stream))
        }
    }

    // Add helper methods to the CuCudaPool implementation
    impl<E: ElementType + ValidAsZeroBits + DeviceRepr> CuCudaPool<E> {
        // Helper method to get a HostSliceWrapper from a CuHandleInner
        fn get_host_slice_wrapper<O: ArrayLike<Element = E>>(
            handle_inner: &CuHandleInner<O>,
        ) -> HostSliceWrapper<'_, O> {
            HostSliceWrapper {
                inner: handle_inner.inner_ref(),
            }
        }

        // Helper method to get a HostSliceMutWrapper from a CuHandleInner
        fn get_host_slice_mut_wrapper<O: ArrayLike<Element = E>>(
            handle_inner: &mut CuHandleInner<O>,
        ) -> HostSliceMutWrapper<'_, O> {
            HostSliceMutWrapper {
                inner: handle_inner.inner_mut(),
            }
        }
    }
    /// A pool of CUDA memory buffers.
    pub struct CuCudaPool<E>
    where
        E: ElementType + ValidAsZeroBits + DeviceRepr + Unpin,
    {
        id: PoolID,
        stream: Arc<CudaStream>,
        pool: Arc<Pool<Box<CudaSliceWrapper<E>>>>,
        nb_buffers: usize,
        nb_element_per_buffer: usize,
    }

    impl<E: ElementType + ValidAsZeroBits + DeviceRepr> CuCudaPool<E> {
        #[allow(dead_code)]
        pub fn new(
            id: &'static str,
            ctx: Arc<CudaContext>,
            nb_buffers: usize,
            nb_element_per_buffer: usize,
        ) -> CuResult<Self> {
            let stream = ctx.default_stream();
            let pool = (0..nb_buffers)
                .map(|_| {
                    stream
                        .alloc_zeros(nb_element_per_buffer)
                        .map(CudaSliceWrapper)
                        .map(Box::new)
                        .map_err(|_| "Failed to allocate device memory")
                })
                .collect::<Result<Vec<_>, _>>()?;

            Ok(Self {
                id: PoolID::from(id).map_err(|_| "Failed to create PoolID")?,
                stream,
                pool: Arc::new(Pool::from_vec(pool)),
                nb_buffers,
                nb_element_per_buffer,
            })
        }
    }

    impl<E> PoolMonitor for CuCudaPool<E>
    where
        E: DeviceRepr + ElementType + ValidAsZeroBits,
    {
        fn id(&self) -> PoolID {
            self.id
        }

        fn space_left(&self) -> usize {
            self.pool.len()
        }

        fn total_size(&self) -> usize {
            self.nb_buffers
        }

        fn buffer_size(&self) -> usize {
            self.nb_element_per_buffer * size_of::<E>()
        }
    }

    impl<E> CuPool<CudaSliceWrapper<E>> for CuCudaPool<E>
    where
        E: DeviceRepr + ElementType + ValidAsZeroBits,
    {
        fn acquire(&self) -> Option<CuHandle<CudaSliceWrapper<E>>> {
            self.pool
                .try_pull_owned()
                .map(|x| CuHandle(Arc::new(Mutex::new(CuHandleInner::Pooled(x)))))
        }

        fn copy_from<O>(&self, from_handle: &mut CuHandle<O>) -> CuHandle<CudaSliceWrapper<E>>
        where
            O: ArrayLike<Element = E>,
        {
            let to_handle = self.acquire().expect("No available buffers in the pool");

            {
                let from_lock = lock_unpoison(&from_handle.0);
                let mut to_lock = lock_unpoison(&to_handle.0);

                match &mut *to_lock {
                    CuHandleInner::Detached(to) => {
                        let wrapper = Self::get_host_slice_wrapper(&*from_lock);
                        self.stream
                            .memcpy_htod(&wrapper, to.deref_mut().as_cuda_slice_mut())
                            .expect("Failed to copy data to device");
                    }
                    CuHandleInner::Pooled(to) => {
                        let wrapper = Self::get_host_slice_wrapper(&*from_lock);
                        self.stream
                            .memcpy_htod(&wrapper, to.deref_mut().as_mut().as_cuda_slice_mut())
                            .expect("Failed to copy data to device");
                    }
                }
            } // locks are dropped here
            to_handle // now we can safely return to_handle
        }
    }

    impl<E> DeviceCuPool<CudaSliceWrapper<E>> for CuCudaPool<E>
    where
        E: ElementType + ValidAsZeroBits + DeviceRepr,
    {
        /// Copy from device to host
        fn copy_to_host_pool<O>(
            &self,
            device_handle: &CuHandle<CudaSliceWrapper<E>>,
            host_handle: &mut CuHandle<O>,
        ) -> Result<(), CuError>
        where
            O: ArrayLike<Element = E>,
        {
            let device_lock = device_handle.lock().map_err(|e| {
                CuError::from("Device handle mutex poisoned").add_cause(&e.to_string())
            })?;
            let mut host_lock = host_handle.lock().map_err(|e| {
                CuError::from("Host handle mutex poisoned").add_cause(&e.to_string())
            })?;
            let src = match &*device_lock {
                CuHandleInner::Pooled(source) => source.deref().as_ref().as_cuda_slice(),
                CuHandleInner::Detached(source) => source.deref().as_cuda_slice(),
            };
            let mut wrapper = Self::get_host_slice_mut_wrapper(&mut *host_lock);
            self.stream.memcpy_dtoh(src, &mut wrapper).map_err(|e| {
                CuError::from("Failed to copy data from device to host").add_cause(&e.to_string())
            })?;
            Ok(())
        }
    }
}

#[derive(Debug)]
/// A buffer that is aligned to a specific size with the Element of type E.
pub struct AlignedBuffer<E: ElementType> {
    ptr: *mut E,
    size: usize,
    layout: Layout,
}

impl<E: ElementType> AlignedBuffer<E> {
    pub fn new(num_elements: usize, alignment: usize) -> Self {
        assert!(
            num_elements > 0 && size_of::<E>() > 0,
            "AlignedBuffer requires a non-zero element count and non-zero-sized element type"
        );
        let alignment = alignment.max(align_of::<E>());
        let alloc_size = num_elements
            .checked_mul(size_of::<E>())
            .expect("AlignedBuffer allocation size overflow");
        let layout = Layout::from_size_align(alloc_size, alignment).unwrap();
        // SAFETY: layout describes a valid, non-zero allocation request.
        let ptr = unsafe { alloc(layout) as *mut E };
        if ptr.is_null() {
            panic!("Failed to allocate memory");
        }
        // SAFETY: ptr is valid for writes of `num_elements` elements.
        unsafe {
            for i in 0..num_elements {
                std::ptr::write(ptr.add(i), E::default());
            }
        }
        Self {
            ptr,
            size: num_elements,
            layout,
        }
    }
}

impl<E: ElementType> Deref for AlignedBuffer<E> {
    type Target = [E];

    fn deref(&self) -> &Self::Target {
        // SAFETY: `new` initializes all elements and keeps the pointer aligned.
        unsafe { std::slice::from_raw_parts(self.ptr, self.size) }
    }
}

impl<E: ElementType> DerefMut for AlignedBuffer<E> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: `new` initializes all elements and keeps the pointer aligned.
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.size) }
    }
}

impl<E: ElementType> Drop for AlignedBuffer<E> {
    fn drop(&mut self) {
        // SAFETY: `ptr` was allocated with `layout` in `new`.
        unsafe { dealloc(self.ptr as *mut u8, self.layout) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool() {
        use std::cell::RefCell;
        let objs = RefCell::new(vec![vec![1], vec![2], vec![3]]);
        let holding = objs.borrow().clone();
        let objs_as_slices = holding.iter().map(|x| x.as_slice()).collect::<Vec<_>>();
        let pool = CuHostMemoryPool::new("mytestcudapool", 3, || objs.borrow_mut().pop().unwrap())
            .unwrap();

        let obj1 = pool.acquire().unwrap();
        {
            let obj2 = pool.acquire().unwrap();
            assert!(objs_as_slices.contains(&obj1.lock().unwrap().deref().deref()));
            assert!(objs_as_slices.contains(&obj2.lock().unwrap().deref().deref()));
            assert_eq!(pool.space_left(), 1);
        }
        assert_eq!(pool.space_left(), 2);

        let obj3 = pool.acquire().unwrap();
        assert!(objs_as_slices.contains(&obj3.lock().unwrap().deref().deref()));

        assert_eq!(pool.space_left(), 1);

        let _obj4 = pool.acquire().unwrap();
        assert_eq!(pool.space_left(), 0);

        let obj5 = pool.acquire();
        assert!(obj5.is_none());
    }

    #[cfg(all(feature = "cuda", has_nvidia_gpu))]
    #[test]
    fn test_cuda_pool() {
        use crate::pool::cuda::CuCudaPool;
        use cudarc::driver::CudaContext;
        let ctx = CudaContext::new(0).unwrap();
        let pool = CuCudaPool::<f32>::new("mytestcudapool", ctx, 3, 1).unwrap();

        let _obj1 = pool.acquire().unwrap();

        {
            let _obj2 = pool.acquire().unwrap();
            assert_eq!(pool.space_left(), 1);
        }
        assert_eq!(pool.space_left(), 2);

        let _obj3 = pool.acquire().unwrap();

        assert_eq!(pool.space_left(), 1);

        let _obj4 = pool.acquire().unwrap();
        assert_eq!(pool.space_left(), 0);

        let obj5 = pool.acquire();
        assert!(obj5.is_none());
    }

    #[cfg(all(feature = "cuda", has_nvidia_gpu))]
    #[test]
    fn test_copy_roundtrip() {
        use crate::pool::cuda::CuCudaPool;
        use cudarc::driver::CudaContext;
        let ctx = CudaContext::new(0).unwrap();
        let host_pool = CuHostMemoryPool::new("mytesthostpool", 3, || vec![0.0; 1]).unwrap();
        let cuda_pool = CuCudaPool::<f32>::new("mytestcudapool", ctx, 3, 1).unwrap();

        let cuda_handle = {
            let mut initial_handle = host_pool.acquire().unwrap();
            {
                let mut inner_initial_handle = initial_handle.lock().unwrap();
                if let CuHandleInner::Pooled(ref mut pooled) = *inner_initial_handle {
                    pooled[0] = 42.0;
                } else {
                    panic!();
                }
            }

            // send that to the GPU
            cuda_pool.copy_from(&mut initial_handle)
        };

        // get it back to the host
        let mut final_handle = host_pool.acquire().unwrap();
        cuda_pool
            .copy_to_host_pool(&cuda_handle, &mut final_handle)
            .unwrap();

        let value = final_handle.lock().unwrap().deref().deref()[0];
        assert_eq!(value, 42.0);
    }
}
