use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use object_pool::{Pool, ReusableOwned};
use std::alloc::{alloc, dealloc, Layout};
use std::fmt::Debug;
use std::ops::{Deref, DerefMut};
use std::sync::{Arc, Mutex};

/// Basic Type that can be used in a buffer in a CuPool.
pub trait ElementType:
    Default + Sized + Copy + Encode + Decode + Debug + Unpin + Send + Sync
{
}

/// Blanket implementation for all types that are Sized, Copy, Encode, Decode and Debug.
impl<T> ElementType for T where
    T: Default + Sized + Copy + Encode + Decode + Debug + Unpin + Send + Sync
{
}

pub trait ArrayLike: Deref<Target = [Self::Element]> + DerefMut + Debug {
    type Element: ElementType;
}

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
            CuHandleInner::Detached(r) => write!(f, "Detached: {:?}", r),
        }
    }
}

impl<T> CuHandleInner<T>
where
    T: ArrayLike,
{
    pub fn slice(&self) -> &[T::Element] {
        match self {
            CuHandleInner::Pooled(pooled) => pooled,
            CuHandleInner::Detached(detached) => detached,
        }
    }
    pub fn slice_mut(&mut self) -> &mut [T::Element] {
        match self {
            CuHandleInner::Pooled(pooled) => pooled.deref_mut(),
            CuHandleInner::Detached(detached) => detached,
        }
    }

    pub fn len(&self) -> usize {
        match self {
            CuHandleInner::Pooled(pooled) => pooled.len(),
            CuHandleInner::Detached(detached) => detached.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        match self {
            CuHandleInner::Pooled(pooled) => pooled.is_empty(),
            CuHandleInner::Detached(detached) => detached.is_empty(),
        }
    }
}

/// A shareable handle to an Array coming from a pool (either host or device).
#[derive(Clone, Debug)]
pub struct CuHandle<T: ArrayLike>(Arc<Mutex<CuHandleInner<T>>>);

impl<T: ArrayLike> CuHandle<T> {
    /// Create a new CuHandle not part of a Pool (not for onboard usages, use pools instead)
    pub fn new_detached(inner: T) -> Self {
        CuHandle(Arc::new(Mutex::new(CuHandleInner::Detached(inner))))
    }

    pub fn inner_handle(&self) -> Arc<Mutex<CuHandleInner<T>>> {
        self.0.clone()
    }

    /// Safely access the inner value, applying a closure to it.
    pub fn with_inner<R>(&self, f: impl FnOnce(&CuHandleInner<T>) -> R) -> R {
        let lock = self.0.lock().unwrap();
        f(&*lock)
    }

    /// Mutably access the inner value, applying a closure to it.
    pub fn with_inner_mut<R>(&self, f: impl FnOnce(&mut CuHandleInner<T>) -> R) -> R {
        let mut lock = self.0.lock().unwrap();
        f(&mut *lock)
    }
}

impl<T: ArrayLike> Encode for CuHandle<T>
where
    <T as ArrayLike>::Element: 'static,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let inner = self.0.lock().unwrap();
        match inner.deref() {
            CuHandleInner::Pooled(pooled) => pooled.encode(encoder),
            CuHandleInner::Detached(detached) => detached.encode(encoder),
        }
    }
}

impl<T: ArrayLike> Default for CuHandle<T> {
    fn default() -> Self {
        panic!("Cannot create a default CuHandle")
    }
}

impl<U: ElementType + 'static> Decode for CuHandle<Vec<U>> {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let vec: Vec<U> = Vec::decode(decoder)?;
        Ok(CuHandle(Arc::new(Mutex::new(CuHandleInner::Detached(vec)))))
    }
}

pub trait CuPool<T: ArrayLike> {
    fn acquire(&self) -> Option<CuHandle<T>>;
    fn space_left(&self) -> usize;
    fn copy_from<O>(&self, from: &mut CuHandle<O>) -> CuHandle<T>
    where
        O: ArrayLike<Element = T::Element>;
}

pub trait DeviceCuPool<T: ArrayLike> {
    type O: ArrayLike<Element = T::Element>;

    /// Takes a handle to a device buffer and copies it into a host buffer pool.
    /// It returns a new handle from the host pool with the data from the device handle given.
    fn copy_into(
        &self,
        from_device_handle: &CuHandle<Self::O>,
        into_cu_host_memory_pool: &mut CuHostMemoryPool<T>,
    ) -> CuHandle<T>;
}

pub struct CuHostMemoryPool<T> {
    pool: Arc<Pool<T>>,
}

impl<T> CuHostMemoryPool<T> {
    pub fn new<F>(size: usize, f: F) -> Self
    where
        F: Fn() -> T,
    {
        Self {
            pool: Arc::new(Pool::new(size, f)),
        }
    }
}

impl<T: ArrayLike> CuPool<T> for CuHostMemoryPool<T> {
    fn acquire(&self) -> Option<CuHandle<T>> {
        let owned_object = self.pool.try_pull_owned(); // Use the owned version

        owned_object.map(|reusable| CuHandle(Arc::new(Mutex::new(CuHandleInner::Pooled(reusable)))))
    }

    fn space_left(&self) -> usize {
        self.pool.len()
    }

    fn copy_from<O: ArrayLike<Element = T::Element>>(&self, from: &mut CuHandle<O>) -> CuHandle<T> {
        let to_handle = self.acquire().expect("No available buffers in the pool");

        match from.0.lock().unwrap().deref() {
            CuHandleInner::Detached(source) => match to_handle.0.lock().unwrap().deref_mut() {
                CuHandleInner::Detached(destination) => {
                    destination.copy_from_slice(source);
                }
                CuHandleInner::Pooled(destination) => {
                    destination.copy_from_slice(source);
                }
            },
            CuHandleInner::Pooled(source) => match to_handle.0.lock().unwrap().deref_mut() {
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

impl<T> CuHostMemoryPool<T>
where
    T: ArrayLike,
{
    pub fn new_buffers<F>(size: usize, f: F) -> Self
    where
        F: Fn() -> T,
    {
        Self {
            pool: Arc::new(Pool::new(size, f)),
        }
    }
}

impl<E: ElementType + 'static> ArrayLike for Vec<E> {
    type Element = E;
}

#[cfg(all(feature = "cuda", not(target_os = "macos")))]
mod cuda {
    use super::*;
    use cudarc::driver::{CudaDevice, CudaSlice, DeviceRepr, DeviceSlice, ValidAsZeroBits};
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
            panic!("You need to copy data to host memory before accessing it.");
        }
    }

    impl<E> DerefMut for CudaSliceWrapper<E>
    where
        E: ElementType,
    {
        fn deref_mut(&mut self) -> &mut Self::Target {
            panic!("You need to copy data to host memory before accessing it.");
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

    pub struct CuCudaPool<E>
    where
        E: ElementType + ValidAsZeroBits + DeviceRepr + Unpin,
    {
        device: Arc<CudaDevice>,
        pool: Arc<Pool<CudaSliceWrapper<E>>>,
    }

    impl<E: ElementType + ValidAsZeroBits + DeviceRepr> CuCudaPool<E> {
        pub fn new(
            device: Arc<CudaDevice>,
            nb_buffers: usize,
            nb_element_per_buffer: usize,
        ) -> Self {
            Self {
                device: device.clone(),
                pool: Arc::new(Pool::new(nb_buffers, || {
                    CudaSliceWrapper(
                        device
                            .alloc_zeros(nb_element_per_buffer)
                            .expect("Failed to allocate device memory"),
                    )
                })),
            }
        }

        #[allow(dead_code)]
        pub fn new_with<F>(device: Arc<CudaDevice>, nb_buffers: usize, f: F) -> Self
        where
            F: Fn() -> CudaSliceWrapper<E>,
        {
            Self {
                device: device.clone(),
                pool: Arc::new(Pool::new(nb_buffers, f)),
            }
        }
    }

    impl<E: ElementType + ValidAsZeroBits + DeviceRepr> CuPool<CudaSliceWrapper<E>> for CuCudaPool<E> {
        fn acquire(&self) -> Option<CuHandle<CudaSliceWrapper<E>>> {
            self.pool
                .try_pull_owned()
                .map(|x| CuHandle(Arc::new(Mutex::new(CuHandleInner::Pooled(x)))))
        }

        fn space_left(&self) -> usize {
            self.pool.len()
        }

        /// Copy from host to device
        fn copy_from<O>(&self, from_handle: &mut CuHandle<O>) -> CuHandle<CudaSliceWrapper<E>>
        where
            O: ArrayLike<Element = E>,
        {
            let to_handle = self.acquire().expect("No available buffers in the pool");

            match from_handle.0.lock().unwrap().deref() {
                CuHandleInner::Detached(from) => match to_handle.0.lock().unwrap().deref_mut() {
                    CuHandleInner::Detached(CudaSliceWrapper(to)) => {
                        self.device
                            .htod_sync_copy_into(from, to)
                            .expect("Failed to copy data to device");
                    }
                    CuHandleInner::Pooled(to) => {
                        self.device
                            .htod_sync_copy_into(from, to.as_cuda_slice_mut())
                            .expect("Failed to copy data to device");
                    }
                },
                CuHandleInner::Pooled(from) => match to_handle.0.lock().unwrap().deref_mut() {
                    CuHandleInner::Detached(CudaSliceWrapper(to)) => {
                        self.device
                            .htod_sync_copy_into(from, to)
                            .expect("Failed to copy data to device");
                    }
                    CuHandleInner::Pooled(to) => {
                        self.device
                            .htod_sync_copy_into(from, to.as_cuda_slice_mut())
                            .expect("Failed to copy data to device");
                    }
                },
            }
            to_handle
        }
    }

    impl<E, T> DeviceCuPool<T> for CuCudaPool<E>
    where
        E: ElementType + ValidAsZeroBits + DeviceRepr,
        T: ArrayLike<Element = E>,
    {
        type O = CudaSliceWrapper<T::Element>;

        /// Copy from device to host
        fn copy_into(
            &self,
            device_handle: &CuHandle<Self::O>,
            cu_host_memory_pool: &mut CuHostMemoryPool<T>,
        ) -> CuHandle<T> {
            let destination_handle = cu_host_memory_pool
                .acquire()
                .expect("No available buffers in the pool");

            match device_handle.0.lock().unwrap().deref() {
                CuHandleInner::Pooled(source) => {
                    match destination_handle.0.lock().unwrap().deref_mut() {
                        CuHandleInner::Pooled(ref mut destination) => {
                            self.device
                                .dtoh_sync_copy_into(source.as_cuda_slice(), destination)
                                .expect("Failed to copy data to device");
                        }
                        CuHandleInner::Detached(ref mut destination) => {
                            self.device
                                .dtoh_sync_copy_into(source.as_cuda_slice(), destination)
                                .expect("Failed to copy data to device");
                        }
                    }
                }
                CuHandleInner::Detached(source) => {
                    match destination_handle.0.lock().unwrap().deref_mut() {
                        CuHandleInner::Pooled(ref mut destination) => {
                            self.device
                                .dtoh_sync_copy_into(source.as_cuda_slice(), destination)
                                .expect("Failed to copy data to device");
                        }
                        CuHandleInner::Detached(ref mut destination) => {
                            self.device
                                .dtoh_sync_copy_into(source.as_cuda_slice(), destination)
                                .expect("Failed to copy data to device");
                        }
                    }
                }
            }

            destination_handle
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
    #[cfg(feature = "cuda")]
    use crate::pool::cuda::CuCudaPool;
    use std::cell::RefCell;

    #[test]
    fn test_pool() {
        let objs = RefCell::new(vec![vec![1], vec![2], vec![3]]);
        let holding = objs.borrow().clone();
        let objs_as_slices = holding.iter().map(|x| x.as_slice()).collect::<Vec<_>>();
        let pool = CuHostMemoryPool::new(3, || objs.borrow_mut().pop().unwrap());

        let obj1 = pool.acquire().unwrap();
        {
            let obj2 = pool.acquire().unwrap();

            assert!(objs_as_slices.contains(&obj1.0.lock().unwrap().slice()));
            assert!(objs_as_slices.contains(&obj2.0.lock().unwrap().slice()));
            assert_eq!(pool.space_left(), 1);
        }
        assert_eq!(pool.space_left(), 2);

        let obj3 = pool.acquire().unwrap();
        assert!(objs_as_slices.contains(&obj3.0.lock().unwrap().slice()));

        assert_eq!(pool.space_left(), 1);

        let _obj4 = pool.acquire().unwrap();
        assert_eq!(pool.space_left(), 0);

        let obj5 = pool.acquire();
        assert!(obj5.is_none());
    }

    #[cfg(all(feature = "cuda", not(target_os = "macos")))]
    #[test]
    fn test_cuda_pool() {
        use cudarc::driver::CudaDevice;
        let device = CudaDevice::new(0).unwrap();
        let pool = CuCudaPool::<f32>::new(device, 3, 1);

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

    #[cfg(all(feature = "cuda", not(target_os = "macos")))]
    #[test]
    fn test_copy_roundtrip() {
        use cudarc::driver::CudaDevice;
        let device = CudaDevice::new(0).unwrap();
        let mut host_pool = CuHostMemoryPool::new(3, || vec![0.0; 1]);

        let cuda_pool = CuCudaPool::<f32>::new(device, 3, 1);

        let cuda_handle = {
            let mut initial_handle = host_pool.acquire().unwrap();
            {
                let mut inner_initial_handle = initial_handle.0.lock().unwrap();
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
        let final_handle = cuda_pool.copy_into(&cuda_handle, &mut host_pool);

        let value = final_handle.0.lock().unwrap().slice()[0];
        assert_eq!(value, 42.0);
    }
}
