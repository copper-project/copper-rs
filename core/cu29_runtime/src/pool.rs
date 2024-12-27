use std::alloc::{alloc, dealloc, Layout};
use std::cell::RefCell;
use std::fmt::Debug;
use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering;

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use std::ops::{Deref, DerefMut};
use std::rc::{Rc, Weak};

pub struct CuMemoryPool<const ES: usize> {
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
        println!("Allocated buffer at {ptr:?} with size {size}");
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

pub struct CuBufferHandle<const ES: usize> {
    index: usize,
    pool: Weak<CuMemoryPool<ES>>,
}

impl<const ES: usize> Encode for CuBufferHandle<ES> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.buffer().encode(encoder)
    }
}

impl<const ES: usize> Decode for CuBufferHandle<ES> {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        // TODO: maybe implement a owned version of this
        Ok(Self {
            index: 0,
            pool: Weak::new(), // An already dead ref
        })
    }
}
impl<const ES: usize> Default for CuBufferHandle<ES> {
    fn default() -> Self {
        println!("Default handle");
        Self {
            index: 0,
            pool: Weak::new(), // An already dead ref
        }
    }
}

impl<const ES: usize> Clone for CuBufferHandle<ES> {
    fn clone(&self) -> Self {
        println!("Cloning handle {}", self.index);
        if let Some(pool) = self.pool.upgrade() {
            pool.inflight_counters[self.index].fetch_add(1, Ordering::SeqCst);
        }

        Self {
            index: self.index,
            pool: self.pool.clone(),
        }
    }
}

impl<const ES: usize> CuBufferHandle<ES> {
    fn new(index: usize, pool: &Rc<CuMemoryPool<ES>>) -> Self {
        println!("Creating handle {}", index);
        Self {
            index,
            pool: Rc::<CuMemoryPool<ES>>::downgrade(&pool),
        }
    }

    fn buffer(&self) -> &[u8] {
        // as long as the pool is alive, the buffer is alive
        if let Some(pool) = self.pool.upgrade() {
            let buffers = pool.buffers.borrow();
            let buffer = buffers[self.index].as_slice();
            unsafe { std::slice::from_raw_parts(buffer.as_ptr(), buffer.len()) }
        } else {
            panic!("Pool is dead");
        }
    }

    fn buffer_mut(&mut self) -> &mut [u8] {
        if let Some(pool) = self.pool.upgrade() {
            let mut buffers = pool.buffers.borrow_mut();
            let buffer = buffers[self.index].as_mut_slice();
            unsafe { std::slice::from_raw_parts_mut(buffer.as_mut_ptr(), buffer.len()) }
        } else {
            panic!("Pool is dead");
        }
    }
}

impl<const ES: usize> Deref for CuBufferHandle<ES> {
    type Target = [u8];
    fn deref(&self) -> &Self::Target {
        self.buffer()
    }
}

impl<const ES: usize> DerefMut for CuBufferHandle<ES> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.buffer_mut()
    }
}

impl<const ES: usize> Drop for CuBufferHandle<ES> {
    fn drop(&mut self) {
        if let Some(pool) = self.pool.upgrade() {
            let remaining = pool.inflight_counters[self.index].fetch_sub(1, Ordering::SeqCst);
            println!("Remaining: {remaining}");
        }
    }
}
impl<const ES: usize> Debug for CuBufferHandle<ES> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let pool = self.pool.upgrade().unwrap();
        let buffers = pool.buffers.borrow();
        f.debug_struct("CuBufferHandle")
            .field("index", &self.index)
            .field("buffer", &buffers[self.index])
            .finish()
    }
}

impl<const ES: usize> CuMemoryPool<ES> {
    pub fn new(count: u32, alignment: usize) -> Self {
        let mut buffers: Vec<AlignedBuffer> = Vec::with_capacity(count as usize);

        for _ in 0..count {
            buffers.push(AlignedBuffer::new(ES, alignment));
        }

        let counters = (0..count)
            .map(|_| AtomicUsize::new(0))
            .collect::<Vec<_>>()
            .into_boxed_slice();
        Self {
            buffers: RefCell::new(buffers),
            inflight_counters: counters,
        }
    }

    pub fn allocate(self_rc: &Rc<Self>) -> Option<CuBufferHandle<ES>> {
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
        let mut pool = CuMemoryPool::<10>::new(10, 4096);
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
        let mut pool = CuMemoryPool::<10>::new(10, 4096);
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
        let mut pool = CuMemoryPool::<10>::new(10, 4096);
        let handle = pool.allocate().unwrap();
        assert_eq!(handle.as_ptr() as usize % 4096, 0);
    }
}
