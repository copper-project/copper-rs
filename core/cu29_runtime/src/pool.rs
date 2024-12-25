use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering;

use std::ops::{Deref, DerefMut};

pub struct FixedSizePool<T> {
    buffer: Vec<T>,
    counters: Box<[AtomicUsize]>,
}

pub struct Handle<T> {
    index: usize,
    pool: *mut FixedSizePool<T>,
}

impl<T> Handle<T> {
    fn new(index: usize, pool: *mut FixedSizePool<T>) -> Self {
        Self { index, pool }
    }

    fn buffer(&self) -> &T {
        unsafe { &(*self.pool).buffer[self.index] }
    }

    fn buffer_mut(&mut self) -> &mut T {
        unsafe { &mut (*self.pool).buffer[self.index] }
    }
}

impl<T> Deref for Handle<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.buffer()
    }
}

impl<T> DerefMut for Handle<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.buffer_mut()
    }
}

impl<T> Drop for Handle<T> {
    fn drop(&mut self) {
        unsafe {
            (*self.pool).counters[self.index].fetch_sub(1, Ordering::SeqCst);
        }
    }
}

impl<T: Default + Clone> FixedSizePool<T> {
    pub fn new(size: usize) -> Self {
        let buffer = vec![T::default(); size];
        let counters = (0..size)
            .map(|_| AtomicUsize::new(0))
            .collect::<Vec<_>>()
            .into_boxed_slice();
        Self { buffer, counters }
    }

    pub fn allocate(&mut self) -> Option<Handle<T>> {
        for (index, counter) in self.counters.iter().enumerate() {
            let prev = counter.fetch_add(1, Ordering::SeqCst);
            if prev == 0 {
                return Some(Handle::new(index, self as *mut _));
            } else {
                counter.fetch_sub(1, Ordering::SeqCst);
            }
        }
        None
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
        let mut pool = FixedSizePool::<TestStruct>::new(10);
        let mut handles = Vec::new();
        for i in 0..10 {
            let mut handle = pool.allocate().unwrap();
            handle.value = 10 - i;
            handles.push(handle);
        }
        assert!(pool.allocate().is_none());
        drop(handles);
    }

    #[test]
    fn test_pool_with_holes() {
        let mut pool = FixedSizePool::<TestStruct>::new(10);
        let mut handles = Vec::new();
        for i in 0..10 {
            let mut handle = pool.allocate().unwrap();
            handle.value = 10 - i;
            if i % 2 == 0 {
                drop(handle);
            } else {
                handles.push(handle);
            }
        }
        for i in 0..5 {
            let mut handle = pool.allocate().unwrap();
            handle.value = 10 - i;
            handles.push(handle);
        }
        assert!(pool.allocate().is_none());
        drop(handles);
    }

    #[test]
    fn test_alignments() {
        let mut pool = FixedSizePool::<TestStruct>::new(10);
        let handle1 = pool.allocate().unwrap();
        let handle2 = pool.allocate().unwrap();
        let ptr1 = &handle1.value as *const u32 as usize;
        assert_eq!(ptr1 % page_size::get(), 0);
        let ptr2 = &handle2.value as *const u32 as usize;
        assert_eq!(ptr2 % page_size::get(), 0);
    }
}
