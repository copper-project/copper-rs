use arrayvec::ArrayString;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29_traits::CuResult;
use object_pool::{Pool, ReusableOwned};
use smallvec::SmallVec;
use std::alloc::{alloc, dealloc, Layout};
use std::collections::HashMap;
use std::fmt::Debug;
use std::ops::{Deref, DerefMut};
use std::sync::{Arc, Mutex, OnceLock};

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

// Register a pool to the global registry.
fn register_pool(pool: Arc<dyn PoolMonitor>) {
    POOL_REGISTRY
        .get_or_init(|| Mutex::new(HashMap::new()))
        .lock()
        .unwrap()
        .insert(pool.id().to_string(), pool);
}

type PoolStats = (PoolID, usize, usize, usize);

/// Get the list of pools and their statistics.
/// We use SmallVec here to avoid heap allocations while the stack is running.
pub fn pools_statistics() -> SmallVec<[PoolStats; MAX_POOLS]> {
    // Safely get the registry, returning empty stats if not initialized.
    let registry_lock = match POOL_REGISTRY.get() {
        Some(lock) => lock.lock().unwrap(),
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

/// A Handle to a Buffer.
/// For onboard usages, the buffer should be Pooled (ie, coming from a preallocated pool).
/// The Detached version is for offline usages where we don't really need a pool to deserialize them.
pub enum CuHandleInner<T: Debug> {
    Pooled(ReusableOwned<T>),
    Detached(T), // Should only be used in offline cases (e.g. deserialization)
}

impl<T> Debug for CuHandleInner<T>
where
    T: Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CuHandleInner::Pooled(r) => {
                write!(f, "Pooled: {:?}", r.deref())
            }
            CuHandleInner::Detached(r) => write!(f, "Detached: {r:?}"),
        }
    }
}

impl<T: ArrayLike> Deref for CuHandleInner<T> {
    type Target = [T::Element];

    fn deref(&self) -> &Self::Target {
        match self {
            CuHandleInner::Pooled(pooled) => pooled,
            CuHandleInner::Detached(detached) => detached,
        }
    }
}

impl<T: ArrayLike> DerefMut for CuHandleInner<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match self {
            CuHandleInner::Pooled(pooled) => pooled.deref_mut(),
            CuHandleInner::Detached(detached) => detached,
        }
    }
}

/// A shareable handle to an Array coming from a pool (either host or device).
#[derive(Clone, Debug)]
pub struct CuHandle<T: ArrayLike>(Arc<Mutex<CuHandleInner<T>>>);

impl<T: ArrayLike> Deref for CuHandle<T> {
    type Target = Arc<Mutex<CuHandleInner<T>>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T: ArrayLike> CuHandle<T> {
    /// Create a new CuHandle not part of a Pool (not for onboard usages, use pools instead)
    pub fn new_detached(inner: T) -> Self {
        CuHandle(Arc::new(Mutex::new(CuHandleInner::Detached(inner))))
    }

    /// Safely access the inner value, applying a closure to it.
    pub fn with_inner<R>(&self, f: impl FnOnce(&CuHandleInner<T>) -> R) -> R {
        let lock = self.lock().unwrap();
        f(&*lock)
    }

    /// Mutably access the inner value, applying a closure to it.
    pub fn with_inner_mut<R>(&self, f: impl FnOnce(&mut CuHandleInner<T>) -> R) -> R {
        let mut lock = self.lock().unwrap();
        f(&mut *lock)
    }
}

impl<T: ArrayLike + Encode> Encode for CuHandle<T>
where
    <T as ArrayLike>::Element: 'static,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let inner = self.lock().unwrap();
        match inner.deref() {
            CuHandleInner::Pooled(pooled) => pooled.deref().encode(encoder),
            CuHandleInner::Detached(detached) => detached.encode(encoder),
        }
    }
}

impl<T: ArrayLike> Default for CuHandle<T> {
    fn default() -> Self {
        panic!("Cannot create a default CuHandle")
    }
}

impl<U: ElementType + Decode<()> + 'static> Decode<()> for CuHandle<Vec<U>> {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let vec: Vec<U> = Vec::decode(decoder)?;
        Ok(CuHandle(Arc::new(Mutex::new(CuHandleInner::Detached(vec)))))
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
    pool: Arc<Pool<T>>,
    size: usize,
    buffer_size: usize,
}

impl<T: ArrayLike + 'static> CuHostMemoryPool<T> {
    pub fn new<F>(id: &str, size: usize, buffer_initializer: F) -> CuResult<Arc<Self>>
    where
        F: Fn() -> T,
    {
        let pool = Arc::new(Pool::new(size, buffer_initializer));
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

        match from.lock().unwrap().deref() {
            CuHandleInner::Detached(source) => match to_handle.lock().unwrap().deref_mut() {
                CuHandleInner::Detached(destination) => {
                    destination.copy_from_slice(source);
                }
                CuHandleInner::Pooled(destination) => {
                    destination.copy_from_slice(source);
                }
            },
            CuHandleInner::Pooled(source) => match to_handle.lock().unwrap().deref_mut() {
                CuHandleInner::Detached(destination) => {
                    destination.copy_from_slice(source);
                }
                CuHandleInner::Pooled(destination) => {
                    destination.copy_from_slice(source);
                }
            },
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

        unsafe fn stream_synced_slice<'b>(
            &'b self,
            stream: &'b CudaStream,
        ) -> (&'b [T::Element], SyncOnDrop<'b>) {
            (self.inner.deref(), SyncOnDrop::sync_stream(stream))
        }

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

        unsafe fn stream_synced_slice<'b>(
            &'b self,
            stream: &'b CudaStream,
        ) -> (&'b [T::Element], SyncOnDrop<'b>) {
            (self.inner.deref(), SyncOnDrop::sync_stream(stream))
        }

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
            match handle_inner {
                CuHandleInner::Pooled(pooled) => HostSliceWrapper { inner: pooled },
                CuHandleInner::Detached(detached) => HostSliceWrapper { inner: detached },
            }
        }

        // Helper method to get a HostSliceMutWrapper from a CuHandleInner
        fn get_host_slice_mut_wrapper<O: ArrayLike<Element = E>>(
            handle_inner: &mut CuHandleInner<O>,
        ) -> HostSliceMutWrapper<'_, O> {
            match handle_inner {
                CuHandleInner::Pooled(pooled) => HostSliceMutWrapper { inner: pooled },
                CuHandleInner::Detached(detached) => HostSliceMutWrapper { inner: detached },
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
        pool: Arc<Pool<CudaSliceWrapper<E>>>,
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
                let from_lock = from_handle.lock().unwrap();
                let mut to_lock = to_handle.lock().unwrap();

                match &mut *to_lock {
                    CuHandleInner::Detached(CudaSliceWrapper(to)) => {
                        let wrapper = Self::get_host_slice_wrapper(&*from_lock);
                        self.stream
                            .memcpy_htod(&wrapper, to)
                            .expect("Failed to copy data to device");
                    }
                    CuHandleInner::Pooled(to) => {
                        let wrapper = Self::get_host_slice_wrapper(&*from_lock);
                        self.stream
                            .memcpy_htod(&wrapper, to.as_cuda_slice_mut())
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
            let device_lock = device_handle.lock().unwrap();
            let mut host_lock = host_handle.lock().unwrap();
            let src = match &*device_lock {
                CuHandleInner::Pooled(source) => source.as_cuda_slice(),
                CuHandleInner::Detached(source) => source.as_cuda_slice(),
            };
            let mut wrapper = Self::get_host_slice_mut_wrapper(&mut *host_lock);
            self.stream
                .memcpy_dtoh(src, &mut wrapper)
                .expect("Failed to copy data from device to host");
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
        let layout = Layout::from_size_align(num_elements * size_of::<E>(), alignment).unwrap();
        let ptr = unsafe { alloc(layout) as *mut E };
        if ptr.is_null() {
            panic!("Failed to allocate memory");
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
        unsafe { std::slice::from_raw_parts(self.ptr, self.size) }
    }
}

impl<E: ElementType> DerefMut for AlignedBuffer<E> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.size) }
    }
}

impl<E: ElementType> Drop for AlignedBuffer<E> {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe {
                dealloc(self.ptr as *mut u8, self.layout);
            }
        }
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
