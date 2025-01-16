use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use object_pool::{Pool, Reusable};
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

pub trait ArrayLike {
    type Element: ElementType;
    fn slice(&self) -> &[Self::Element];
    fn slice_mut(&mut self) -> &mut [Self::Element];
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// An Array coming from a pool.
pub struct PooledObjectArray<'a, T: ArrayLike>(Reusable<'a, T>);

#[derive(Clone, Debug)]
/// A shareable handle to an Array coming from a pool (either host or device).
pub enum CuHandle<'a, T: ArrayLike> {
    Pooled(Arc<Mutex<PooledObjectArray<'a, T>>>),
    Detached(Arc<Mutex<T>>), // Should only be used in offline cases (e.g. deserialization)
}

impl<T: ArrayLike> Encode for CuHandle<'_, T>
where
    <T as ArrayLike>::Element: 'static,
{
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        match self {
            CuHandle::Pooled(pooled) => {
                let arrayref = pooled.lock().unwrap();
                let slice = arrayref.slice();
                slice.encode(encoder)
            }
            CuHandle::Detached(detached) => {
                let arrayref = detached.lock().unwrap();
                let slice = arrayref.slice();
                slice.encode(encoder)
            }
        }
    }
}

impl<T: ArrayLike> Default for CuHandle<'_, T> {
    fn default() -> Self {
        panic!("Cannot create a default CuHandle")
    }
}

impl<U: ElementType + 'static> Decode for CuHandle<'static, Vec<U>> {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let vec: Vec<U> = Vec::decode(decoder)?;
        Ok(CuHandle::Detached(Arc::new(Mutex::new(vec))))
    }
}

impl<T: ArrayLike> Debug for PooledObjectArray<'_, T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_tuple("CuHandle of an array of size")
            .field(&self.0.len())
            .finish()
    }
}

impl<'a, T: ArrayLike> PooledObjectArray<'a, T> {
    fn new(inner: Reusable<'a, T>) -> Self {
        Self(inner)
    }

    fn inner(&self) -> &T {
        self.0.deref()
    }
}

impl<T: ArrayLike> Deref for PooledObjectArray<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<T: ArrayLike> DerefMut for PooledObjectArray<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

pub trait CuPool<T: ArrayLike> {
    fn acquire(&self) -> Option<CuHandle<T>>;
    fn space_left(&self) -> usize;
    fn copy_from<O: ArrayLike>(&self, from: &mut CuHandle<O>) -> CuHandle<T>
    where
        O: ArrayLike<Element = T::Element>;
}

pub trait DeviceCuPool<'a, T: ArrayLike> {
    type O: ArrayLike<Element = T::Element>;

    /// Takes a handle to a device buffer and copies it into a host buffer pool.
    /// It returns a new handle from the host pool with the data from the device handle given.
    fn copy_into(
        &self,
        from_device_handle: &CuHandle<Self::O>,
        into_cu_host_memory_pool: &'a mut CuHostMemoryPool<T>,
    ) -> CuHandle<'a, T>;
}

pub struct CuHostMemoryPool<T> {
    pool: Pool<T>,
}

impl<T> CuHostMemoryPool<T> {
    pub fn new<F>(size: usize, f: F) -> Self
    where
        F: Fn() -> T,
    {
        Self {
            pool: Pool::new(size, f),
        }
    }
}

impl<'a, T: ArrayLike> CuPool<T> for CuHostMemoryPool<T> {
    fn acquire(&self) -> Option<CuHandle<'_, T>> {
        let i = self.pool.try_pull();
        i.map(|x| CuHandle::Pooled(Arc::new(Mutex::new(PooledObjectArray::new(x)))))
    }

    fn space_left(&self) -> usize {
        self.pool.len()
    }

    fn copy_from<O: ArrayLike<Element = T::Element>>(&self, from: &mut CuHandle<O>) -> CuHandle<T> {
        let to_handle = self.acquire().expect("No available buffers in the pool");

        match from {
            CuHandle::Detached(detached) => {
                let locked_from_handle = detached.lock().unwrap();
                let from_slice = locked_from_handle.slice();
                match to_handle {
                    CuHandle::Detached(ref detached) => {
                        let mut locked_to_handle = detached.lock().unwrap();
                        locked_to_handle.slice_mut().copy_from_slice(from_slice);
                    }
                    CuHandle::Pooled(ref pooled) => {
                        let mut locked_to_handle = pooled.lock().unwrap();
                        locked_to_handle.slice_mut().copy_from_slice(from_slice);
                    }
                }
            }
            CuHandle::Pooled(pooled) => {
                let locked_from_handle = pooled.lock().unwrap();
                let from_slice = locked_from_handle.slice();
                match to_handle {
                    CuHandle::Detached(ref detached) => {
                        let mut locked_to_handle = detached.lock().unwrap();
                        locked_to_handle.slice_mut().copy_from_slice(from_slice);
                    }
                    CuHandle::Pooled(ref pooled) => {
                        let mut locked_to_handle = pooled.lock().unwrap();
                        locked_to_handle.slice_mut().copy_from_slice(from_slice);
                    }
                }
            }
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
            pool: Pool::new(size, f),
        }
    }
}

impl<E: ElementType + 'static> ArrayLike for Vec<E> {
    type Element = E;

    fn slice(&self) -> &[Self::Element] {
        self.as_slice()
    }

    fn slice_mut(&mut self) -> &mut [Self::Element] {
        self.as_mut_slice()
    }

    fn len(&self) -> usize {
        self.len()
    }
}

#[cfg(feature = "cuda")]
mod cuda {
    use super::*;
    use cudarc::driver::{CudaDevice, CudaSlice, DeviceRepr, DeviceSlice, ValidAsZeroBits};
    use std::sync::{Arc, MutexGuard};

    pub struct CuCudaPool<E>
    where
        E: ElementType + ValidAsZeroBits + DeviceRepr + Unpin,
    {
        device: Arc<CudaDevice>,
        pool: Pool<CudaSlice<E>>,
    }

    impl<E: ElementType + ValidAsZeroBits + DeviceRepr> CuCudaPool<E> {
        pub fn new(
            device: Arc<CudaDevice>,
            nb_buffers: usize,
            nb_element_per_buffer: usize,
        ) -> Self {
            Self {
                device: device.clone(),
                pool: Pool::new(nb_buffers, || {
                    device
                        .alloc_zeros(nb_element_per_buffer)
                        .expect("Failed to allocate device memory")
                }),
            }
        }

        #[allow(dead_code)]
        pub fn new_with<F>(device: Arc<CudaDevice>, nb_buffers: usize, f: F) -> Self
        where
            F: Fn() -> CudaSlice<E>,
        {
            Self {
                device: device.clone(),
                pool: Pool::new(nb_buffers, f),
            }
        }
    }

    impl<E: ElementType> ArrayLike for CudaSlice<E> {
        type Element = E;

        fn slice(&self) -> &[Self::Element] {
            panic!("You need to copy a handle to the host memory or a shared memory before accessing it")
        }

        fn slice_mut(&mut self) -> &mut [Self::Element] {
            panic!("You need to copy a handle to the host memory or a shared memory before accessing it")
        }

        fn len(&self) -> usize {
            DeviceSlice::<E>::len(self)
        }
    }

    impl<E: ElementType + ValidAsZeroBits + DeviceRepr> CuPool<CudaSlice<E>> for CuCudaPool<E> {
        fn acquire(&self) -> Option<CuHandle<CudaSlice<E>>> {
            self.pool
                .try_pull()
                .map(|x| CuHandle::Pooled(Arc::new(Mutex::new(PooledObjectArray::new(x)))))
        }

        fn space_left(&self) -> usize {
            self.pool.len()
        }

        /// Copy from host to device
        fn copy_from<O>(&self, from: &mut CuHandle<O>) -> CuHandle<CudaSlice<E>>
        where
            O: ArrayLike<Element = E>,
        {
            let r = match from {
                CuHandle::Pooled(pooled) => {
                    let locked_from_handle = pooled.lock().unwrap();
                    let device_buffer = self
                        .device
                        .htod_sync_copy(locked_from_handle.slice())
                        .expect("Failed to copy data to device");
                    Reusable::new(&self.pool, device_buffer) // this attachs it to the pool
                }
                CuHandle::Detached(detached) => {
                    // This kind of should never happen.
                    let locked_from_handle = detached.lock().unwrap();
                    let device_buffer = self
                        .device
                        .htod_sync_copy(locked_from_handle.slice())
                        .expect("Failed to copy data to device");
                    Reusable::new(&self.pool, device_buffer) // this attachs it to the pool
                }
            };

            self.pool
                .try_pull()
                .expect("No available buffers in the pool")
                .detach(); // drop one from the pool as we just added one
            CuHandle::Pooled(Arc::new(Mutex::new(PooledObjectArray::new(r))))
        }
    }

    impl<E, T> DeviceCuPool<'_, T> for CuCudaPool<E>
    where
        E: ElementType + ValidAsZeroBits + DeviceRepr,
        T: ArrayLike<Element = E>,
    {
        type O = CudaSlice<T::Element>;

        /// Copy from device to host
        fn copy_into<'a>(
            &self,
            device_handle: &CuHandle<Self::O>,
            cu_host_memory_pool: &'a mut CuHostMemoryPool<T>,
        ) -> CuHandle<'a, T> {
            let destination_handle = cu_host_memory_pool
                .acquire()
                .expect("No available buffers in the pool");

            match device_handle {
                CuHandle::Pooled(pooled) => {
                    let locked_device_handle = pooled.lock().unwrap();
                    match destination_handle {
                        CuHandle::Pooled(ref destination) => {
                            let mut locked_destination_handle = destination.lock().unwrap();
                            self.device
                                .dtoh_sync_copy_into(
                                    locked_device_handle.inner(),
                                    locked_destination_handle.slice_mut(),
                                )
                                .expect("Failed to copy data to device");
                        }
                        CuHandle::Detached(ref detached) => {
                            // Should not happen
                            let mut locked_destination_handle = detached.lock().unwrap();
                            self.device
                                .dtoh_sync_copy_into(
                                    locked_device_handle.inner(),
                                    locked_destination_handle.slice_mut(),
                                )
                                .expect("Failed to copy data to device");
                        }
                    }
                }
                CuHandle::Detached(detached) => {
                    let locked_device_handle: MutexGuard<CudaSlice<E>> = detached.lock().unwrap();
                    match destination_handle {
                        CuHandle::Pooled(ref destination) => {
                            let mut locked_destination_handle = destination.lock().unwrap();
                            self.device
                                .dtoh_sync_copy_into(
                                    &*locked_device_handle,
                                    locked_destination_handle.slice_mut(),
                                )
                                .expect("Failed to copy data to device");
                        }
                        CuHandle::Detached(ref detached) => {
                            // Should not happen
                            let mut locked_destination_handle = detached.lock().unwrap();
                            self.device
                                .dtoh_sync_copy_into(
                                    &*locked_device_handle,
                                    locked_destination_handle.slice_mut(),
                                )
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

// Implement ArrayLike for fixed-size arrays
impl<T, const N: usize> ArrayLike for [T; N]
where
    T: ElementType,
{
    type Element = T;

    fn slice(&self) -> &[Self::Element] {
        self
    }

    fn slice_mut(&mut self) -> &mut [Self::Element] {
        self
    }

    fn len(&self) -> usize {
        N
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
        let objs = RefCell::new(vec![[1; 1], [2; 1], [3; 1]]);
        let pool = CuHostMemoryPool::new(3, || objs.borrow_mut().pop().unwrap());

        let obj1 = pool.acquire().unwrap();
        let CuHandle::Pooled(obj1) = obj1 else {
            panic!()
        };

        {
            let obj2 = pool.acquire().unwrap();

            let CuHandle::Pooled(obj2) = obj2 else {
                panic!()
            };

            assert!([[1; 1], [2; 1], [3; 1]].contains(obj1.lock().unwrap().inner()));
            assert!([[1; 1], [2; 1], [3; 1]].contains(obj2.lock().unwrap().inner()));
            assert_eq!(pool.space_left(), 1);
        }
        assert_eq!(pool.space_left(), 2);

        let obj3 = pool.acquire().unwrap();
        let CuHandle::Pooled(obj3) = obj3 else {
            panic!()
        };
        assert!([[1; 1], [2; 1], [3; 1]].contains(obj3.lock().unwrap().inner()));

        assert_eq!(pool.space_left(), 1);

        let _obj4 = pool.acquire().unwrap();
        assert_eq!(pool.space_left(), 0);

        let obj5 = pool.acquire();
        assert!(obj5.is_none());
    }

    #[cfg(feature = "cuda")]
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

    #[cfg(feature = "cuda")]
    #[test]
    fn test_copy_roundtrip() {
        use cudarc::driver::CudaDevice;
        let device = CudaDevice::new(0).unwrap();
        let mut host_pool = CuHostMemoryPool::new(3, || vec![0.0; 1]);

        let cuda_pool = CuCudaPool::<f32>::new(device, 3, 1);

        let cuda_handle = {
            // Create a local handle
            let mut initial_handle = host_pool.acquire().unwrap();
            if let CuHandle::Pooled(ref inner_initial_handle) = initial_handle {
                inner_initial_handle.lock().unwrap()[0] = 42.0;

                // send that to the GPU
                cuda_pool.copy_from(&mut initial_handle)
            } else {
                panic!()
            }
        };

        // get it back to the host
        let final_handle = cuda_pool.copy_into(&cuda_handle, &mut host_pool);
        let CuHandle::Pooled(final_handle) = final_handle else {
            panic!()
        };

        assert_eq!(final_handle.lock().unwrap()[0], 42.0);
    }
}
