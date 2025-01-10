// use std::alloc::{alloc, dealloc, Layout};
// use std::cell::RefCell;
// use std::fmt::Debug;
// use std::sync::atomic::AtomicUsize;
// use std::sync::atomic::Ordering;
//
// use crate::log::*;
// use bincode::de::Decoder;
// use bincode::enc::Encoder;
// use bincode::error::{DecodeError, EncodeError};
// use bincode::{BorrowDecode, Decode, Encode};
// use cu29_traits::CuResult;
// use std::sync::Weak;
// use std::sync::{Arc, Mutex};
//
// #[cfg(not(feature = "cuda"))]
// pub trait ElementType: Sized + Copy + Encode + Decode + Debug {}
//
// #[cfg(not(feature = "cuda"))]
// impl<T> ElementType for T where T: Sized + Copy + Encode + Decode + Debug {}
//
// #[cfg(feature = "cuda")]
// pub use cuda::ElementType;
//
// /// A trait that represents a buffer that can be used as a host buffer.
// pub trait CuHostBuffer<E: ElementType>: AsRef<[E]> + AsMut<[E]> {}
//
// /// A trait that represents a buffer that can be used as a device buffer (e.g. CUDA buffer, NPU or other NUMA node).
// /// D is the device type, E is the element type, I is the inner buffer type and IM is the inner buffer mutable type.
// pub trait CuDeviceBuffer<D, E: ElementType, I> {
//     fn len(&self) -> usize;
//     fn get_device(&self) -> Arc<D>;
//     fn get_inner(&self) -> &I;
//     fn get_inner_mut(&mut self) -> &mut I;
// }
//
// /// A shared memory buffer between host and device is just a special case of a device buffer.
// pub trait CuSharedBuffer<D, E: ElementType, I>: CuDeviceBuffer<D, E, I> + CuHostBuffer<E> {}
//
// /// A pool of standard pre allocated host memory buffers.
// pub trait CuHostPool<E: ElementType> {
//     fn allocate(self_rc: &Arc<Mutex<Self>>) -> Option<impl CuHostBuffer<E>>;
// }
//
// /// A pool of pre allocated memory buffers on device (as in GPU, NPU, other NUMA node ...).
// pub trait CuDevicePool<D, E: ElementType, I> {
//     /// Allocate a buffer from the device pool.
//     fn allocate(self_rc: &Arc<Mutex<Self>>) -> Option<impl CuDeviceBuffer<D, E, I>>;
//
//     /// Copy the content of the source buffer to the destination buffer.
//     /// dst can be allocated with self.allocate().
//     fn copy_to_device(
//         &self,
//         src: &mut impl CuHostBuffer<E>,
//         dst: &mut impl CuDeviceBuffer<D, E, I>,
//     ) -> CuResult<()>;
//
//     /// Copy the content of the source buffer to host destination buffer.
//     fn copy_to_host(
//         &self,
//         src: &impl CuDeviceBuffer<D, E, I>,
//         dst: &mut impl CuHostBuffer<E>,
//     ) -> CuResult<()>;
// }
//
// /// A pool of pre allocated memory buffers that can be shared between host and device.
// pub trait CuSharedPool<D, E: ElementType, I, IM> {
//     fn allocate(&self) -> Option<impl CuSharedBuffer<D, E, I>>;
// }
//
// /// A reusable pool of memory buffer locally accessible.
// pub struct CuHostMemoryPool<E: ElementType> {
//     buffers: RefCell<Vec<AlignedBuffer<E>>>,
//     inflight_counters: Box<[AtomicUsize]>,
// }
//
// #[cfg(feature = "cuda")]
// mod cuda {
//     use super::*;
//     use cu29_traits::CuError;
//     use cudarc::driver::{CudaDevice, CudaSlice, DeviceSlice};
//     use cudarc::driver::{DeviceRepr, ValidAsZeroBits};
//     use std::sync::{Arc, Mutex};
//
//     pub trait ElementType:
//         Sized + Copy + Encode + Decode + Debug + ValidAsZeroBits + DeviceRepr
//     {
//     }
//     impl<T> ElementType for T where
//         T: Sized + Copy + Encode + Decode + Debug + ValidAsZeroBits + DeviceRepr
//     {
//     }
//
//     pub struct CuCudaPool<E: ElementType> {
//         device: Arc<CudaDevice>,
//         inner: Box<[CudaSlice<E>]>,
//         counters: Box<[usize]>,
//     }
//
//     impl<E: ElementType> CuCudaPool<E> {
//         pub fn new(
//             device: Arc<CudaDevice>,
//             buffer_element_count: usize,
//             buffer_count: u32,
//         ) -> CuResult<Self> {
//             let mut inner: Vec<CudaSlice<E>> = Vec::with_capacity(buffer_count as usize);
//
//             for _ in 0..buffer_count {
//                 let buffer = device
//                     .alloc_zeros(size_of::<E>() * buffer_element_count * buffer_count as usize)
//                     .map_err(|e| {
//                         CuError::new_with_cause("Could not allocate the CUDA memory pool", e)
//                     })?;
//                 inner.push(buffer);
//             }
//
//             let counters = (0..buffer_count)
//                 .map(|_| 0usize)
//                 .collect::<Vec<_>>()
//                 .into_boxed_slice();
//
//             Ok(Self {
//                 device,
//                 inner: inner.into_boxed_slice(),
//                 counters,
//             })
//         }
//     }
//
//     pub struct CuCudaBufferHandle<E: ElementType> {
//         pool: Arc<Mutex<CuCudaPool<E>>>,
//         index: usize,
//     }
//
//     impl<'a, E: ElementType> CuCudaBufferHandle<E> {
//         pub fn new(pool: Arc<Mutex<CuCudaPool<E>>>, index: usize) -> Self {
//             Self { pool, index }
//         }
//     }
//
//     impl<E: ElementType> CuDevicePool<CudaDevice, E, CudaSlice<E>> for CuCudaPool<E> {
//         fn allocate(
//             self_rc: &Arc<Mutex<Self>>,
//         ) -> Option<impl CuDeviceBuffer<CudaDevice, E, CudaSlice<E>>> {
//             let selfm = self_rc.lock().unwrap();
//             for (index, counter) in selfm.counters.iter().enumerate() {
//                 if *counter == 0usize {
//                     return Some(CuCudaBufferHandle::new(self_rc.clone(), index));
//                 }
//             }
//             None
//         }
//
//         fn copy_to_device(
//             &self,
//             src: &mut impl CuHostBuffer<E>,
//             dst: &mut impl CuDeviceBuffer<CudaDevice, E, CudaSlice<E>>,
//         ) -> CuResult<()> {
//             self.device
//                 .htod_sync_copy_into(src.as_ref(), dst.get_inner_mut())
//                 .map_err(|e| CuError::new_with_cause("Failed to copy to host", e))
//         }
//
//         fn copy_to_host(
//             &self,
//             src: &impl CuDeviceBuffer<CudaDevice, E, CudaSlice<E>>,
//             dst: &mut impl CuHostBuffer<E>,
//         ) -> CuResult<()> {
//             self.device
//                 .dtoh_sync_copy_into(&src.get_inner().slice(..), dst.as_mut())
//                 .map_err(|e| CuError::new_with_cause("Failed to copy to host", e))
//         }
//     }
//
//     impl<'a, E: ElementType> CuDeviceBuffer<CudaDevice, E, CudaSlice<E>> for CuCudaBufferHandle<E> {
//         fn len(&self) -> usize {
//             let pool = self.pool.lock().unwrap();
//             pool.inner[self.index].len()
//         }
//
//         fn get_device(&self) -> Arc<CudaDevice> {
//             let pool = self.pool.lock().unwrap();
//             pool.device.clone()
//         }
//
//         fn get_inner(&self) -> &CudaSlice<E> {
//             let pool = self.pool.lock().unwrap();
//             &pool.inner[self.index]
//         }
//
//         fn get_inner_mut(&mut self) -> &mut CudaSlice<E> {
//             let mut pool = self.pool.lock().unwrap();
//             &mut pool.inner[self.index]
//         }
//     }
// }
//
// #[cfg(feature = "cuda")]
// pub use cuda::CuCudaBufferHandle;
//
// #[cfg(feature = "cuda")]
// pub use cuda::CuCudaPool;
//
// #[derive(Debug)]
// /// A buffer that is aligned to a specific size with the Element of type E.
// pub struct AlignedBuffer<E: ElementType> {
//     ptr: *mut E,
//     size: usize,
//     layout: Layout,
// }
//
// impl<E: ElementType> AlignedBuffer<E> {
//     pub fn new(num_elements: usize, alignment: usize) -> Self {
//         let layout = Layout::from_size_align(num_elements * size_of::<E>(), alignment).unwrap();
//         let ptr = unsafe { alloc(layout) as *mut E };
//         debug!(
//             "Allocated buffer at {} with size {}",
//             ptr as usize, num_elements
//         );
//         if ptr.is_null() {
//             panic!("Failed to allocate memory");
//         }
//         Self {
//             ptr,
//             size: num_elements,
//             layout,
//         }
//     }
//
//     pub fn as_slice(&self) -> &[E] {
//         unsafe { std::slice::from_raw_parts(self.ptr, self.size) }
//     }
//
//     pub fn as_mut_slice(&mut self) -> &mut [E] {
//         unsafe { std::slice::from_raw_parts_mut(self.ptr, self.size) }
//     }
// }
//
// impl<E: ElementType> Drop for AlignedBuffer<E> {
//     fn drop(&mut self) {
//         if !self.ptr.is_null() {
//             unsafe {
//                 dealloc(self.ptr as *mut u8, self.layout);
//             }
//         }
//     }
// }
//
// pub struct CuBufferHandle<E: ElementType> {
//     index: usize,
//     pool: Weak<CuHostMemoryPool<E>>,
// }
//
// /// This is just a very inefficient Buffer impl to decode logs.
// pub struct OrphanCuBufferHandle<E: ElementType>(Vec<E>);
//
// impl<E: ElementType> Decode for OrphanCuBufferHandle<E> {
//     fn decode<D: Decoder>(_decoder: &mut D) -> Result<Self, DecodeError> {
//         let len = usize::decode(_decoder)?;
//         let mut buffer = Vec::with_capacity(len);
//         for _ in 0..len {
//             buffer.push(E::decode(_decoder)?);
//         }
//         Ok(Self(buffer))
//     }
// }
//
// impl AsRef<[u8]> for OrphanCuBufferHandle<u8> {
//     fn as_ref(&self) -> &[u8] {
//         self.0.as_slice()
//     }
// }
//
// impl AsMut<[u8]> for OrphanCuBufferHandle<u8> {
//     fn as_mut(&mut self) -> &mut [u8] {
//         self.0.as_mut_slice()
//     }
// }
//
// impl<T: ElementType> Encode for CuBufferHandle<T> {
//     fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
//         let miself = self.as_ref();
//         miself.len().encode(encoder)?;
//         for value in miself {
//             value.encode(encoder)?;
//         }
//         Ok(())
//     }
// }
//
// impl<T: ElementType> CuHostBuffer<T> for CuBufferHandle<T> {}
//
// impl<T: ElementType> Decode for CuBufferHandle<T> {
//     fn decode<D: Decoder>(_decoder: &mut D) -> Result<Self, DecodeError> {
//         panic!("Cannot decode a CuBufferHandle directly, use OrphanCuBufferHandle instead");
//     }
// }
//
// impl<T: ElementType> BorrowDecode<'_> for CuBufferHandle<T> {
//     fn borrow_decode<D: Decoder>(_decoder: &mut D) -> Result<Self, DecodeError> {
//         panic!("Cannot decode a CuBufferHandle directly, use OrphanCuBufferHandle instead");
//     }
// }
//
// impl<E: ElementType> Default for CuBufferHandle<E> {
//     fn default() -> Self {
//         Self {
//             index: 0,
//             pool: Weak::new(), // An already dead ref
//         }
//     }
// }
//
// impl<E: ElementType> Clone for CuBufferHandle<E> {
//     fn clone(&self) -> Self {
//         if let Some(pool) = self.pool.upgrade() {
//             pool.inflight_counters[self.index].fetch_add(1, Ordering::SeqCst);
//         }
//
//         Self {
//             index: self.index,
//             pool: self.pool.clone(),
//         }
//     }
// }
//
// impl<E: ElementType> CuBufferHandle<E> {
//     fn new(index: usize, pool: &Arc<CuHostMemoryPool<E>>) -> Self {
//         Self {
//             index,
//             pool: Arc::<CuHostMemoryPool<E>>::downgrade(pool),
//         }
//     }
//
//     pub fn index(&self) -> usize {
//         self.index
//     }
// }
//
// impl<E: ElementType> AsRef<[E]> for CuBufferHandle<E> {
//     fn as_ref(&self) -> &[E] {
//         // as long as the pool is alive, the buffer is alive
//         if let Some(pool) = self.pool.upgrade() {
//             let buffers = pool.buffers.borrow();
//             let buffer = buffers[self.index].as_slice();
//             unsafe { std::slice::from_raw_parts(buffer.as_ptr(), buffer.len()) }
//         } else {
//             panic!("Pool is dead");
//         }
//     }
// }
//
// impl<E: ElementType> AsMut<[E]> for CuBufferHandle<E> {
//     fn as_mut(&mut self) -> &mut [E] {
//         if let Some(pool) = self.pool.upgrade() {
//             let mut buffers = pool.buffers.borrow_mut();
//             let buffer = buffers[self.index].as_mut_slice();
//             unsafe { std::slice::from_raw_parts_mut(buffer.as_mut_ptr(), buffer.len()) }
//         } else {
//             panic!("Pool is dead");
//         }
//     }
// }
//
// impl<E: ElementType> Drop for CuBufferHandle<E> {
//     fn drop(&mut self) {
//         if let Some(pool) = self.pool.upgrade() {
//             let remaining = pool.inflight_counters[self.index].fetch_sub(1, Ordering::SeqCst);
//             debug!("Dropping buffer handle, remaining: {}", remaining);
//         }
//     }
// }
//
// impl<E: ElementType> Debug for CuBufferHandle<E> {
//     fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
//         let pool = self.pool.upgrade().unwrap();
//         let buffers = pool.buffers.borrow();
//         f.debug_struct("CuBufferHandle")
//             .field("index", &self.index)
//             .field("buffer", &buffers[self.index])
//             .finish()
//     }
// }
//
// impl<E: ElementType> CuHostMemoryPool<E> {
//     pub fn new(buffer_size: usize, buffer_count: u32, alignment: usize) -> Self {
//         let mut buffers: Vec<AlignedBuffer<E>> = Vec::with_capacity(buffer_count as usize);
//
//         for _ in 0..buffer_count {
//             buffers.push(AlignedBuffer::new(buffer_size, alignment));
//         }
//
//         let counters = (0..buffer_count)
//             .map(|_| AtomicUsize::new(0))
//             .collect::<Vec<_>>()
//             .into_boxed_slice();
//         Self {
//             buffers: RefCell::new(buffers),
//             inflight_counters: counters,
//         }
//     }
//
//     pub fn size(&self) -> usize {
//         self.buffers.borrow().len()
//     }
// }
//
// impl<E: ElementType> CuHostPool<E> for CuHostMemoryPool<E> {
//     fn allocate(self_rc: &Arc<Mutex<Self>>) -> Option<impl CuHostBuffer<E>> {
//         for (index, counter) in self_rc.inflight_counters.iter().enumerate() {
//             let prev = counter.fetch_add(1, Ordering::SeqCst);
//             if prev == 0 {
//                 return Some(CuBufferHandle::new(index, self_rc));
//             } else {
//                 counter.fetch_sub(1, Ordering::SeqCst);
//             }
//         }
//         None
//     }
// }
//
// #[cfg(test)]
// mod tests {
//     use super::*;
//
//     #[test]
//     fn test_full_size_pool() {
//         let pool = Arc::new(CuHostMemoryPool::new(10, 10, 4096));
//         let mut handles = Vec::new();
//         for i in 0..10 {
//             let mut handle = CuHostMemoryPool::allocate(&pool).unwrap();
//             handle.as_mut()[0] = 10 - i;
//             handles.push(handle);
//         }
//         assert!(CuHostMemoryPool::allocate(&pool).is_none());
//         drop(handles);
//     }
//
//     #[test]
//     fn test_pool_with_holes() {
//         let pool = Arc::new(CuHostMemoryPool::new(10, 10, 4096));
//         let mut handles = Vec::new();
//         for i in 0..10 {
//             let mut handle = CuHostMemoryPool::allocate(&pool).unwrap();
//             handle.as_mut()[0] = 10 - i;
//             if i % 2 == 0 {
//                 drop(handle);
//             } else {
//                 handles.push(handle);
//             }
//         }
//         for i in 0..5 {
//             let mut handle = CuHostMemoryPool::allocate(&pool).unwrap();
//             handle.as_mut()[0] = 10 - i;
//             handles.push(handle);
//         }
//         assert!(CuHostMemoryPool::allocate(&pool).is_none());
//         drop(handles);
//     }
//
//     #[test]
//     fn test_alignment() {
//         let pool = Arc::new(CuHostMemoryPool::new(10, 10, 4096));
//         let handle = CuHostMemoryPool::allocate(&pool).unwrap();
//         assert_eq!(handle.as_ref().as_ptr() as *const u8 as usize % 4096, 0);
//     }
//
//     #[test]
//     #[cfg(feature = "cuda")]
//     fn test_cuda_buffers_transfers() {
//         let host_pool = Arc::new(CuHostMemoryPool::new(10, 10, 4096));
//         let mut host_buffer = CuHostMemoryPool::allocate(&host_pool).unwrap();
//         host_buffer.as_mut()[0] = 42u8;
//
//         let device = cudarc::driver::CudaDevice::new(0).unwrap();
//         let device_pool = Arc::new(CuCudaPool::<u8>::new(device.clone(), 10, 10).unwrap());
//
//         let mut device_buffer = CuCudaPool::allocate(&device_pool).unwrap();
//         device_pool
//             .copy_to_device(&mut host_buffer, &mut device_buffer)
//             .unwrap();
//     }
// }
//
