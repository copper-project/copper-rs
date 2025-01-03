use std::alloc::{alloc, dealloc, Layout};
use std::cell::RefCell;
use std::fmt::Debug;
use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering;

use crate::log::*;
use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{BorrowDecode, Decode, Encode};
use cu29_traits::CuResult;
use std::rc::{Rc, Weak};
use std::sync::Arc;

pub trait ElementType: Sized + Copy + Encode + Decode + Debug + ValidAsZeroBits {}
impl<T> ElementType for T where T: Sized + Copy + Encode + Decode + Debug {}

/// A trait that represents a buffer that can be used as a host buffer.
pub trait CuHostBuffer<E: ElementType>: AsRef<[E]> + AsMut<[E]> {}

/// Marker trait for a device (e.g. GPU, NPU, other NUMA node).
pub trait MemoryDevice {}

/// A trait that represents a buffer that can be used as a device buffer (e.g. CUDA buffer, NPU or other NUMA node).
pub trait CuDeviceBuffer<D: MemoryDevice, E: ElementType> {
    fn len(&self) -> usize;
    fn as_ptr(&self) -> *const E;
    fn as_mut_ptr(&mut self) -> *mut E;
    fn get_device(&self) -> Arc<D>;
}

/// A shared memory buffer between host and device is just a special case of a device buffer.
pub trait CuSharedBuffer<D: MemoryDevice, E: ElementType>:
    CuDeviceBuffer<D, E> + CuHostBuffer<E>
{
}

/// A pool of standard pre allocated host memory buffers.
pub trait CuHostPool<E: ElementType> {
    fn allocate(&self) -> Option<impl CuHostBuffer<E>>;
}

/// A pool of pre allocated memory buffers on device (as in GPU, NPU, other NUMA node ...).
pub trait CuDevicePool<D: MemoryDevice, E: ElementType> {
    /// Allocate a buffer from the device pool.
    fn allocate(&self) -> Option<impl CuDeviceBuffer<D, E>>;

    /// Copy the content of the source buffer to the destination buffer.
    /// dst can be allocated with self.allocate().
    fn copy_to_device(&self, src: &mut impl CuHostBuffer<E>, dst: &mut impl CuDeviceBuffer<D, E>);

    /// Copy the content of the source buffer to host destination buffer.
    fn copy_to_host(
        &self,
        src: &impl CuDeviceBuffer<D, E>,
        dst: &mut impl CuHostBuffer<E>,
    ) -> CuResult<()>;
}

/// A pool of pre allocated memory buffers that can be shared between host and device.
pub trait CuSharedPool<D: MemoryDevice, E: ElementType> {
    fn allocate(&self) -> Option<impl CuSharedBuffer<D, E>>;
}

/// A reusable pool of memory buffer locally accessible.
pub struct CuHostMemoryPool<E: ElementType> {
    buffers: RefCell<Vec<AlignedBuffer<E>>>,
    inflight_counters: Box<[AtomicUsize]>,
}

#[cfg(feature = "cuda")]
mod cuda {
    use super::*;
    use cu29_traits::CuError;
    use cudarc::driver::{CudaDevice, CudaSlice, CudaView};
    use std::sync::Arc;

    struct CuCudaDevice(CudaDevice);
    impl MemoryDevice for CuCudaDevice {}

    pub struct CuCudaPool<E: ElementType> {
        device: Arc<CudaDevice>,
        inner: CudaSlice<E>,
        counters: Box<[AtomicUsize]>,
        _phantom: std::marker::PhantomData<E>,
    }

    impl<E: ElementType> CuCudaPool<E> {
        pub fn new(
            device: Arc<CudaDevice>,
            buffer_element_count: usize,
            buffer_count: u32,
        ) -> CuResult<Self> {
            let inner = device
                .alloc_zeros(size_of::<E>() * buffer_element_count * buffer_count as usize)
                .map_err(|e| {
                    CuError::new_with_cause("Could not allocate the CUDA memory pool", e)
                })?;
            let counters = (0..buffer_count)
                .map(|_| AtomicUsize::new(0))
                .collect::<Vec<_>>()
                .into_boxed_slice();

            Ok(Self {
                device,
                inner,
                counters,
                _phantom: std::marker::PhantomData,
            })
        }
    }

    pub struct CuCudaBufferHandle<'a, E> {
        og_slice: &'a CudaSlice<E>,
        view: CudaView<'a, E>,
        index: usize,
    }

    impl<'a, E: ElementType> CuCudaBufferHandle<'a, E> {
        pub fn new(og_slice: &'a CudaSlice<E>, view: CudaView<'a, E>, index: usize) -> Self {
            Self {
                og_slice,
                view,
                index,
            }
        }
    }

    impl<E: ElementType> CuDevicePool<CuCudaDevice, E> for CuCudaPool<E> {
        fn allocate(&self) -> Option<impl CuDeviceBuffer<CuCudaDevice, E>> {
            for (index, counter) in self.counters.iter().enumerate() {
                let prev = counter.fetch_add(1, Ordering::SeqCst);
                if prev == 0 {
                    let left = size_of::<E>() * index;
                    let right = size_of::<E>() * (index + 1);
                    let subslice = self.inner.slice(left..right);

                    return Some(CuCudaBufferHandle::new(&self.inner, subslice, index));
                } else {
                    counter.fetch_sub(1, Ordering::SeqCst);
                }
            }
            None
        }

        fn copy_to_device(
            &self,
            src: &mut impl CuHostBuffer<E>,
            dst: &mut impl CuDeviceBuffer<CuCudaDevice, E>,
        ) {
            todo!()
        }

        fn copy_to_host(
            &self,
            src: &impl CuDeviceBuffer<CuCudaDevice, E>,
            dst: &mut impl CuHostBuffer<E>,
        ) -> CuResult<()> {
            todo!()
        }
    }

    impl<'a, D: MemoryDevice, E: ElementType> CuDeviceBuffer<D, E> for CuCudaBufferHandle<'a, E> {
        fn len(&self) -> usize {
            self.view.len()
        }

        fn as_ptr(&self) -> *const E {
            self.view.as_ptr()
        }

        fn as_mut_ptr(&mut self) -> *mut E {
            self.view.as_mut_ptr()
        }

        fn get_device(&self) -> Arc<D> {
            &self.og_slice.device().clone()
        }
    }
}

#[cfg(feature = "cuda")]
pub use cuda::CuCudaBufferHandle;

#[cfg(feature = "cuda")]
pub use cuda::CuCudaPool;

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
        debug!(
            "Allocated buffer at {} with size {}",
            ptr as usize, num_elements
        );
        if ptr.is_null() {
            panic!("Failed to allocate memory");
        }
        Self {
            ptr,
            size: num_elements,
            layout,
        }
    }

    pub fn as_slice(&self) -> &[E] {
        unsafe { std::slice::from_raw_parts(self.ptr, self.size) }
    }

    pub fn as_mut_slice(&mut self) -> &mut [E] {
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

pub struct CuBufferHandle<E: ElementType> {
    index: usize,
    pool: Weak<CuHostMemoryPool<E>>,
}

/// This is just a very inefficient Buffer impl to decode logs.
pub struct OrphanCuBufferHandle<E: ElementType>(Vec<E>);

impl<E: ElementType> Decode for OrphanCuBufferHandle<E> {
    fn decode<D: Decoder>(_decoder: &mut D) -> Result<Self, DecodeError> {
        let len = usize::decode(_decoder)?;
        let mut buffer = Vec::with_capacity(len);
        for _ in 0..len {
            buffer.push(E::decode(_decoder)?);
        }
        Ok(Self(buffer))
    }
}

impl AsRef<[u8]> for OrphanCuBufferHandle<u8> {
    fn as_ref(&self) -> &[u8] {
        self.0.as_slice()
    }
}

impl AsMut<[u8]> for OrphanCuBufferHandle<u8> {
    fn as_mut(&mut self) -> &mut [u8] {
        self.0.as_mut_slice()
    }
}

impl<T: ElementType> Encode for CuBufferHandle<T> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let miself = self.as_ref();
        miself.len().encode(encoder)?;
        for value in miself {
            value.encode(encoder)?;
        }
        Ok(())
    }
}

impl<T: ElementType> Decode for CuBufferHandle<T> {
    fn decode<D: Decoder>(_decoder: &mut D) -> Result<Self, DecodeError> {
        panic!("Cannot decode a CuBufferHandle directly, use OrphanCuBufferHandle instead");
    }
}

impl<T: ElementType> BorrowDecode<'_> for CuBufferHandle<T> {
    fn borrow_decode<D: Decoder>(_decoder: &mut D) -> Result<Self, DecodeError> {
        panic!("Cannot decode a CuBufferHandle directly, use OrphanCuBufferHandle instead");
    }
}

impl<E: ElementType> Default for CuBufferHandle<E> {
    fn default() -> Self {
        Self {
            index: 0,
            pool: Weak::new(), // An already dead ref
        }
    }
}

impl<E: ElementType> Clone for CuBufferHandle<E> {
    fn clone(&self) -> Self {
        if let Some(pool) = self.pool.upgrade() {
            pool.inflight_counters[self.index].fetch_add(1, Ordering::SeqCst);
        }

        Self {
            index: self.index,
            pool: self.pool.clone(),
        }
    }
}

impl<E: ElementType> CuBufferHandle<E> {
    fn new(index: usize, pool: &Rc<CuHostMemoryPool<E>>) -> Self {
        Self {
            index,
            pool: Rc::<CuHostMemoryPool<E>>::downgrade(pool),
        }
    }

    pub fn index(&self) -> usize {
        self.index
    }
}

impl<E: ElementType> AsRef<[E]> for CuBufferHandle<E> {
    fn as_ref(&self) -> &[E] {
        // as long as the pool is alive, the buffer is alive
        if let Some(pool) = self.pool.upgrade() {
            let buffers = pool.buffers.borrow();
            let buffer = buffers[self.index].as_slice();
            unsafe { std::slice::from_raw_parts(buffer.as_ptr(), buffer.len()) }
        } else {
            panic!("Pool is dead");
        }
    }
}

impl<E: ElementType> AsMut<[E]> for CuBufferHandle<E> {
    fn as_mut(&mut self) -> &mut [E] {
        if let Some(pool) = self.pool.upgrade() {
            let mut buffers = pool.buffers.borrow_mut();
            let buffer = buffers[self.index].as_mut_slice();
            unsafe { std::slice::from_raw_parts_mut(buffer.as_mut_ptr(), buffer.len()) }
        } else {
            panic!("Pool is dead");
        }
    }
}

impl<E: ElementType> Drop for CuBufferHandle<E> {
    fn drop(&mut self) {
        if let Some(pool) = self.pool.upgrade() {
            let remaining = pool.inflight_counters[self.index].fetch_sub(1, Ordering::SeqCst);
            debug!("Dropping buffer handle, remaining: {}", remaining);
        }
    }
}

impl<E: ElementType> Debug for CuBufferHandle<E> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let pool = self.pool.upgrade().unwrap();
        let buffers = pool.buffers.borrow();
        f.debug_struct("CuBufferHandle")
            .field("index", &self.index)
            .field("buffer", &buffers[self.index])
            .finish()
    }
}

impl<E: ElementType> CuHostMemoryPool<E> {
    pub fn new(buffer_size: usize, buffer_count: u32, alignment: usize) -> Self {
        let mut buffers: Vec<AlignedBuffer<E>> = Vec::with_capacity(buffer_count as usize);

        for _ in 0..buffer_count {
            buffers.push(AlignedBuffer::new(buffer_size, alignment));
        }

        let counters = (0..buffer_count)
            .map(|_| AtomicUsize::new(0))
            .collect::<Vec<_>>()
            .into_boxed_slice();
        Self {
            buffers: RefCell::new(buffers),
            inflight_counters: counters,
        }
    }

    pub fn allocate(self_rc: &Rc<Self>) -> Option<CuBufferHandle<E>> {
        for (index, counter) in self_rc.inflight_counters.iter().enumerate() {
            let prev = counter.fetch_add(1, Ordering::SeqCst);
            if prev == 0 {
                return Some(CuBufferHandle::new(index, self_rc));
            } else {
                counter.fetch_sub(1, Ordering::SeqCst);
            }
        }
        None
    }
    pub fn size(&self) -> usize {
        self.buffers.borrow().len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_full_size_pool() {
        let pool = Rc::new(CuHostMemoryPool::new(10, 10, 4096));
        let mut handles = Vec::new();
        for i in 0..10 {
            let mut handle = CuHostMemoryPool::allocate(&pool).unwrap();
            handle.as_mut()[0] = 10 - i;
            handles.push(handle);
        }
        assert!(CuHostMemoryPool::allocate(&pool).is_none());
        drop(handles);
    }

    #[test]
    fn test_pool_with_holes() {
        let pool = Rc::new(CuHostMemoryPool::new(10, 10, 4096));
        let mut handles = Vec::new();
        for i in 0..10 {
            let mut handle = CuHostMemoryPool::allocate(&pool).unwrap();
            handle.as_mut()[0] = 10 - i;
            if i % 2 == 0 {
                drop(handle);
            } else {
                handles.push(handle);
            }
        }
        for i in 0..5 {
            let mut handle = CuHostMemoryPool::allocate(&pool).unwrap();
            handle.as_mut()[0] = 10 - i;
            handles.push(handle);
        }
        assert!(CuHostMemoryPool::allocate(&pool).is_none());
        drop(handles);
    }

    #[test]
    fn test_alignment() {
        let pool = Rc::new(CuHostMemoryPool::new(10, 10, 4096));
        let handle = CuHostMemoryPool::allocate(&pool).unwrap();
        assert_eq!(handle.as_ref().as_ptr() as *const u8 as usize % 4096, 0);
    }

    #[test]
    fn test_cuda_buffers() {
        let device = cudarc::driver::CudaDevice::new(0).unwrap();
        let pool = CuCudaPool::<u8>::new(device.clone(), 10, 10).unwrap();
        let mut handle = pool.allocate().unwrap();
        handle.as_mut()[0] = 10;
        drop(handle);
        let mut handle = pool.allocate().unwrap();
        handle.as_mut()[0] = 20;
        drop(handle);
    }
}
