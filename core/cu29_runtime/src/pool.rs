use cu29_log::CuLogEntry;
use cu29_log::ANONYMOUS;
use cu29_log_runtime::log_debug_mode;
use cu29_value::to_value;
use std::alloc::{alloc, dealloc, Layout};
use std::cell::RefCell;
use std::fmt::Debug;
use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering;

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29_log_derive::debug;
use std::rc::{Rc, Weak};

pub struct CuHostMemoryPool {
    buffers: RefCell<Vec<AlignedBuffer>>,
    inflight_counters: Box<[AtomicUsize]>,
}

#[derive(Debug)]
pub struct AlignedBuffer {
    ptr: *mut u8,
    size: usize,
    layout: Layout,
}

impl AlignedBuffer {
    pub fn new(size: usize, alignment: usize) -> Self {
        let layout = Layout::from_size_align(size, alignment).unwrap();
        let ptr = unsafe { alloc(layout) };
        debug!("Allocated buffer at {} with size {}", ptr as usize, size);
        if ptr.is_null() {
            panic!("Failed to allocate memory");
        }
        Self { ptr, size, layout }
    }

    pub fn as_slice(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self.ptr, self.size) }
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.size) }
    }
}

impl Drop for AlignedBuffer {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe {
                dealloc(self.ptr, self.layout);
            }
        }
    }
}

pub struct CuBufferHandle {
    index: usize,
    pool: Weak<CuHostMemoryPool>,
}

impl Encode for CuBufferHandle {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.as_slice().encode(encoder)
    }
}

impl Decode for CuBufferHandle {
    fn decode<D: Decoder>(_decoder: &mut D) -> Result<Self, DecodeError> {
        // TODO: maybe implement a owned version of this
        Ok(Self {
            index: 0,
            pool: Weak::new(), // An already dead ref
        })
    }
}
impl Default for CuBufferHandle {
    fn default() -> Self {
        Self {
            index: 0,
            pool: Weak::new(), // An already dead ref
        }
    }
}

impl Clone for CuBufferHandle {
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

impl CuBufferHandle {
    fn new(index: usize, pool: &Rc<CuHostMemoryPool>) -> Self {
        Self {
            index,
            pool: Rc::<CuHostMemoryPool>::downgrade(&pool),
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        // as long as the pool is alive, the buffer is alive
        if let Some(pool) = self.pool.upgrade() {
            let buffers = pool.buffers.borrow();
            let buffer = buffers[self.index].as_slice();
            unsafe { std::slice::from_raw_parts(buffer.as_ptr(), buffer.len()) }
        } else {
            panic!("Pool is dead");
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [u8] {
        if let Some(pool) = self.pool.upgrade() {
            let mut buffers = pool.buffers.borrow_mut();
            let buffer = buffers[self.index].as_mut_slice();
            unsafe { std::slice::from_raw_parts_mut(buffer.as_mut_ptr(), buffer.len()) }
        } else {
            panic!("Pool is dead");
        }
    }

    pub fn index(&self) -> usize {
        self.index
    }
}

impl Drop for CuBufferHandle {
    fn drop(&mut self) {
        if let Some(pool) = self.pool.upgrade() {
            let remaining = pool.inflight_counters[self.index].fetch_sub(1, Ordering::SeqCst);
            debug!("Dropping buffer handle, remaining: {}", remaining);
        }
    }
}

impl Debug for CuBufferHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let pool = self.pool.upgrade().unwrap();
        let buffers = pool.buffers.borrow();
        f.debug_struct("CuBufferHandle")
            .field("index", &self.index)
            .field("buffer", &buffers[self.index])
            .finish()
    }
}

impl CuHostMemoryPool {
    pub fn new(buffer_size: usize, buffer_count: u32, alignment: usize) -> Self {
        let mut buffers: Vec<AlignedBuffer> = Vec::with_capacity(buffer_count as usize);

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

    pub fn allocate(self_rc: &Rc<Self>) -> Option<CuBufferHandle> {
        for (index, counter) in self_rc.inflight_counters.iter().enumerate() {
            let prev = counter.fetch_add(1, Ordering::SeqCst);
            if prev == 0 {
                return Some(CuBufferHandle::new(index, &self_rc));
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

    #[repr(align(4096))]
    #[derive(Default, Clone)]
    struct TestStruct {
        value: u32,
    }

    #[test]
    fn test_full_size_pool() {
        let mut pool = CuHostMemoryPool::new(10, 10, 4096);
        let mut handles = Vec::new();
        for i in 0..10 {
            let mut handle = pool.allocate().unwrap();
            handle.deref_mut()[0] = 10 - i;
            handles.push(handle);
        }
        assert!(pool.allocate().is_none());
        drop(handles);
    }

    #[test]
    fn test_pool_with_holes() {
        let mut pool = CuHostMemoryPool::new(10, 10, 4096);
        let mut handles = Vec::new();
        for i in 0..10 {
            let mut handle = pool.allocate().unwrap();
            handle.deref_mut()[0] = 10 - i;
            if i % 2 == 0 {
                drop(handle);
            } else {
                handles.push(handle);
            }
        }
        for i in 0..5 {
            let mut handle = pool.allocate().unwrap();
            handle.deref_mut()[0] = 10 - i;
            handles.push(handle);
        }
        assert!(pool.allocate().is_none());
        drop(handles);
    }

    #[test]
    fn test_alignment() {
        let mut pool = CuHostMemoryPool::new(10, 10, 4096);
        let handle = pool.allocate().unwrap();
        assert_eq!(handle.as_ptr() as usize % 4096, 0);
    }
}
