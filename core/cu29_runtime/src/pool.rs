use bincode::{Decode, Encode};
use object_pool::{Pool, Reusable};
use std::alloc::{alloc, dealloc, Layout};
use std::fmt::Debug;
use std::ops::{Deref, DerefMut};
use std::sync::Arc;

/// Basic Type that can be used in a buffer in a CuPool.
pub trait ElementType: Sized + Copy + Encode + Decode + Debug + Unpin {}

/// Blanket implementation for all types that are Sized, Copy, Encode, Decode and Debug.
impl<T> ElementType for T where T: Sized + Copy + Encode + Decode + Debug + Unpin {}

pub trait ArrayLike {
    type Element: ElementType;
    fn slice(&self) -> &[Self::Element];
    fn slice_mut(&mut self) -> &mut [Self::Element];
}

/// Handle to a buffer in a CuPool. The handle can be used as a type between tasks.
/// It holds a reference to the buffer in the pool that will be reclaimed when the handle is dropped.
pub struct CuHandle<'a, T: ArrayLike>(Reusable<'a, T>);

impl<'a, T: ArrayLike> CuHandle<'a, T> {
    fn new(inner: Reusable<'a, T>) -> Self {
        Self(inner)
    }

    fn inner(&self) -> &T {
        self.0.deref()
    }
}

impl<T: ArrayLike> Deref for CuHandle<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.0.deref()
    }
}

impl<T: ArrayLike> DerefMut for CuHandle<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.0.deref_mut()
    }
}

pub trait CuPool<'a, T: ArrayLike> {
    fn acquire(&'a self) -> Option<CuHandle<'a, T>>;
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

impl<'a, T: ArrayLike> CuPool<'a, T> for CuHostMemoryPool<T> {
    fn acquire(&'a self) -> Option<CuHandle<'a, T>> {
        self.pool.try_pull().map(CuHandle::new)
    }

    fn space_left(&self) -> usize {
        self.pool.len()
    }

    fn copy_from<O: ArrayLike<Element = T::Element>>(&self, from: &mut CuHandle<O>) -> CuHandle<T> {
        let mut handle = self.acquire().expect("No available buffers in the pool");

        let from_slice = from.slice();
        handle.slice_mut().copy_from_slice(from_slice);

        handle
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

impl<E: ElementType> ArrayLike for Vec<E> {
    type Element = E;

    fn slice(&self) -> &[Self::Element] {
        self.as_slice()
    }

    fn slice_mut(&mut self) -> &mut [Self::Element] {
        self.as_mut_slice()
    }
}

impl<E: ElementType, const N: usize> ArrayLike for [E; N] {
    type Element = E;

    fn slice(&self) -> &[Self::Element] {
        &self[..]
    }

    fn slice_mut(&mut self) -> &mut [Self::Element] {
        &mut self[..]
    }
}

#[cfg(feature = "cuda")]
mod cuda {
    use super::*;
    use cudarc::driver::{CudaDevice, CudaSlice, DeviceRepr, ValidAsZeroBits};

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
    }

    impl<E: ElementType + ValidAsZeroBits + DeviceRepr> CuPool<'_, CudaSlice<E>> for CuCudaPool<E> {
        fn acquire(&self) -> Option<CuHandle<CudaSlice<E>>> {
            self.pool.try_pull().map(CuHandle::new)
        }

        fn space_left(&self) -> usize {
            self.pool.len()
        }

        /// Copy from host to device
        fn copy_from<O>(&self, from: &mut CuHandle<O>) -> CuHandle<CudaSlice<E>>
        where
            O: ArrayLike<Element = E>,
        {
            let host_slice: &[E] = from.slice();

            // Copy data from host slice to a newly allocated device buffer
            let device_buffer = self
                .device
                .htod_sync_copy(host_slice)
                .expect("Failed to copy data to device");

            let r = Reusable::new(&self.pool, device_buffer); // this attachs it to the pool
            self.pool
                .try_pull()
                .expect("No available buffers in the pool")
                .detach(); // drop one from the pool as we just added one
            CuHandle::new(r)
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
            let mut destination_handle = cu_host_memory_pool
                .acquire()
                .expect("No available buffers in the pool");

            // Copy data from host slice to a newly allocated device buffer
            self.device
                .dtoh_sync_copy_into(device_handle.inner(), destination_handle.slice_mut())
                .expect("Failed to copy data to device");
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
        let objs = RefCell::new(vec![[1; 1], [2; 1], [3; 1]]);
        let pool = CuHostMemoryPool::new(3, || objs.borrow_mut().pop().unwrap());

        let obj1 = pool.acquire().unwrap();

        {
            let obj2 = pool.acquire().unwrap();

            assert!([[1; 1], [2; 1], [3; 1]].contains(&*obj1));
            assert!([[1; 1], [2; 1], [3; 1]].contains(&*obj2));
            assert_eq!(pool.space_left(), 1);
        }
        assert_eq!(pool.space_left(), 2);

        let obj3 = pool.acquire().unwrap();
        assert!([[1; 1], [2; 1], [3; 1]].contains(&*obj3));

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
        let mut initial_handle = host_pool.acquire().unwrap();
        initial_handle[0] = 42.0;

        let cuda_pool = CuCudaPool::<f32>::new(device, 3, 1);

        let cuda_handle = cuda_pool.copy_from(&mut initial_handle);
        drop(initial_handle);

        let final_handle = cuda_pool.copy_into(&cuda_handle, &mut host_pool);

        assert_eq!(final_handle[0], 42.0);
    }
}
