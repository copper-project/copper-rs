use std::ops::Deref;
use std::sync::{Arc, Mutex};

// Pooled object
pub struct Pooled<T>(Arc<PoolObject<T>>);

impl<T> Pooled<T> {
    fn new(inner: Arc<PoolObject<T>>) -> Self {
        Pooled(inner)
    }

    pub fn value(&self) -> &T {
        &self.0.value
    }
}

impl<T> Deref for Pooled<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.0.value
    }
}

impl<T> Clone for Pooled<T> {
    fn clone(&self) -> Self {
        Pooled(Arc::clone(&self.0))
    }
}

struct FreeList(Arc<Mutex<Vec<u32>>>);

impl FreeList {
    fn new(capacity: usize) -> Self {
        FreeList(Arc::new(Mutex::new(
            (0..capacity).map(|i| i as u32).collect(),
        )))
    }

    fn pop(&self) -> Option<usize> {
        let mut free_slots = self.0.lock().unwrap();
        free_slots.pop().map(|index| index as usize)
    }

    fn push(&self, index: usize) {
        let mut free_slots = self.0.lock().unwrap();
        free_slots.push(index as u32);
    }

    fn contains(&self, index: usize) -> bool {
        let free_slots = self.0.lock().unwrap();
        free_slots.contains(&(index as u32))
    }
}

impl Clone for FreeList {
    fn clone(&self) -> Self {
        FreeList(Arc::clone(&self.0))
    }
}

struct Pool<T> {
    pool: Box<[Pooled<T>]>,
    free_slots: FreeList,
}

/// An object stored in the pool
struct PoolObject<T> {
    index: usize,                // Index in the pool
    value: T,                    // The actual value
    original_freelist: FreeList, // the freelist this pooled object is from
}

impl<T> Pool<T> {
    fn new<F>(capacity: usize, factory: F) -> Arc<Self>
    where
        F: Fn(usize) -> T, // Factory function to create objects
    {
        let mut v = Vec::with_capacity(capacity);
        for i in 0..capacity {
            v.push(Pooled(Arc::new(PoolObject {
                index: i,
                value: factory(i),
                original_freelist: FreeList::new(capacity),
            })));
        }
        let pool = v.into_boxed_slice();
        let free_slots = FreeList::new(capacity);
        Arc::new(Pool { pool, free_slots })
    }

    fn acquire(&self) -> Option<Pooled<T>> {
        self.free_slots.pop().map(|index| self.pool[index].clone())
    }

    fn is_free(&self, index: usize) -> bool {
        self.free_slots.contains(index)
    }
}

impl<T> Drop for PoolObject<T> {
    fn drop(&mut self) {
        self.original_freelist.push(self.index); // Free itself up
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool() {
        let pool = Pool::new(3, |i| format!("Object {}", i + 1));

        let obj1 = pool.acquire().unwrap();
        let obj2 = pool.acquire().unwrap();

        assert_eq!(obj1.value(), "Object 1");
        assert_eq!(obj2.value(), "Object 2");

        drop(obj1);

        let obj3 = pool.acquire().unwrap();
        assert_eq!(obj3.value(), "Object 1");

        assert!(!pool.is_free(0));
        assert!(pool.is_free(1));
        assert!(!pool.is_free(2));
    }
}
