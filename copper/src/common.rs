extern crate alloc;

use std::iter::{Chain, Rev};
use std::mem::replace;
use std::slice::{Iter as SliceIter, IterMut as SliceIterMut};

const MAX_TASKS: usize = 255;
const MAX_CONCURRENT_TASKS: usize = 8;

#[derive(Debug)]
struct CopperLiskMask {
    mask: [u128; MAX_TASKS / 128 + 1],
}

#[derive(Debug)]
enum CopperListState {
    Empty,
    PrecessingTasks(CopperLiskMask),
    BeingSerialized,
}

/// A circular buffer-like queue.
#[derive(Debug)]
pub struct CopperListsManager<T: Sized + PartialEq, const N: usize> {
    // copper_list_masks: CopperListMask,
    data: Box<[T; N]>,
    length: usize,
    insertion_index: usize,
}

/// An iterator over `CircularQueue<T>`.
pub type Iter<'a, T> = Chain<Rev<SliceIter<'a, T>>, Rev<SliceIter<'a, T>>>;

/// A mutable iterator over `CircularQueue<T>`.
pub type IterMut<'a, T> = Chain<Rev<SliceIterMut<'a, T>>, Rev<SliceIterMut<'a, T>>>;

/// An ascending iterator over `CircularQueue<T>`.
pub type AscIter<'a, T> = Chain<SliceIter<'a, T>, SliceIter<'a, T>>;

/// An mutable ascending iterator over `CircularQueue<T>`.
pub type AscIterMut<'a, T> = Chain<SliceIterMut<'a, T>, SliceIterMut<'a, T>>;

/// A value popped from `CircularQueue<T>` as the result of a push operation.
pub type Popped<T> = Option<T>;

impl<T: Sized + PartialEq, const N: usize> PartialEq for CopperListsManager<T, N> {
    fn eq(&self, other: &Self) -> bool {
        if self.len() != other.len() {
            return false;
        }
        other.iter().zip(self.iter()).all(|(a, b)| a == b)
    }
}

impl<T: Sized + PartialEq, const N: usize> CopperListsManager<T, N> {
    pub fn new() -> Self {
        let data = unsafe {
            let layout = std::alloc::Layout::new::<[T; N]>();
            let ptr = std::alloc::alloc_zeroed(layout) as *mut [T; N];
            Box::from_raw(ptr)
        };
        CopperListsManager {
            data,
            length: 0,
            insertion_index: 0,
        }
    }

    /// Returns the current number of elements in the queue.
    ///
    #[inline]
    pub fn len(&self) -> usize {
        self.length
    }

    /// Returns `true` if the queue contains no elements.
    ///
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.length == 0
    }

    /// Returns `true` if the queue is full.
    ///
    #[inline]
    pub fn is_full(&self) -> bool {
        self.capacity() == self.len()
    }

    /// Returns the capacity of the queue.
    ///
    #[inline]
    pub fn capacity(&self) -> usize {
        N
    }

    /// Clears the queue.
    ///
    #[inline]
    pub fn clear(&mut self) {
        self.insertion_index = 0;
        self.length = 0;
    }

    /// Pushes a new element into the queue.
    ///
    /// Once the capacity is reached, pushing new items will overwrite old ones.
    ///
    /// In case an old value is overwritten, it will be returned.
    ///
    #[inline]
    pub fn push(&mut self, x: T) -> Popped<T> {
        let mut old = None;

        if self.capacity() == 0 {
            return old;
        }

        if !self.is_full() {
            self.data[self.insertion_index] = x;
            self.length += 1;
        } else {
            old = Some(replace(&mut self.data[self.insertion_index], x));
        }

        self.insertion_index = (self.insertion_index + 1) % self.capacity();

        old
    }

    #[inline]
    pub fn pop(&mut self) -> Popped<&T> {
        if self.capacity() == 0 || self.length == 0 {
            return None;
        }
        if self.insertion_index == 0 {
            self.insertion_index = self.capacity() - 1;
        } else {
            self.insertion_index -= 1;
        }
        self.length -= 1;
        Some(&self.data[self.insertion_index])
    }

    /// Returns an iterator over the queue's contents.
    ///
    /// The iterator goes from the most recently pushed items to the oldest ones.
    ///
    #[inline]
    pub fn iter(&self) -> Iter<T> {
        let (a, b) = self.data[0..self.length].split_at(self.insertion_index);
        a.iter().rev().chain(b.iter().rev())
    }

    /// Returns a mutable iterator over the queue's contents.
    ///
    /// The iterator goes from the most recently pushed items to the oldest ones.
    ///
    #[inline]
    pub fn iter_mut(&mut self) -> IterMut<T> {
        let (a, b) = self.data.split_at_mut(self.insertion_index);
        a.iter_mut().rev().chain(b.iter_mut().rev())
    }

    /// Returns an ascending iterator over the queue's contents.
    ///
    /// The iterator goes from the least recently pushed items to the newest ones.
    ///
    #[inline]
    pub fn asc_iter(&self) -> AscIter<T> {
        let (a, b) = self.data.split_at(self.insertion_index);
        b.iter().chain(a.iter())
    }

    /// Returns a mutable ascending iterator over the queue's contents.
    ///
    /// The iterator goes from the least recently pushed items to the newest ones.
    ///
    #[inline]
    pub fn asc_iter_mut(&mut self) -> AscIterMut<T> {
        let (a, b) = self.data.split_at_mut(self.insertion_index);
        b.iter_mut().chain(a.iter_mut())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_capacity() {
        let mut q = CopperListsManager::<i32, 0>::new();
        assert_eq!(q.len(), 0);
        assert_eq!(q.capacity(), 0);
        assert!(q.is_empty());

        q.push(3);
        q.push(4);
        q.push(5);

        assert_eq!(q.len(), 0);
        assert_eq!(q.capacity(), 0);
        assert!(q.is_empty());

        assert_eq!(q.iter().count(), 0);
        assert_eq!(q.asc_iter().count(), 0);

        q.clear();
    }

    #[test]
    fn empty_queue() {
        let q = CopperListsManager::<i32, 5>::new();

        assert!(q.is_empty());
        assert_eq!(q.iter().next(), None);
    }

    #[test]
    fn partially_full_queue() {
        let mut q = CopperListsManager::<_, 5>::new();
        q.push(1);
        q.push(2);
        q.push(3);

        assert!(!q.is_empty());
        assert_eq!(q.len(), 3);

        let res: Vec<_> = q.iter().map(|&x| x).collect();
        assert_eq!(res, [3, 2, 1]);
    }

    #[test]
    fn full_queue() {
        let mut q = CopperListsManager::<_, 5>::new();
        q.push(1);
        q.push(2);
        q.push(3);
        q.push(4);
        q.push(5);

        assert_eq!(q.len(), 5);

        let res: Vec<_> = q.iter().map(|&x| x).collect();
        assert_eq!(res, [5, 4, 3, 2, 1]);
    }

    #[test]
    fn over_full_queue() {
        let mut q = CopperListsManager::<_, 5>::new();
        q.push(1);
        q.push(2);
        q.push(3);
        q.push(4);
        q.push(5);
        q.push(6);
        q.push(7);

        assert_eq!(q.len(), 5);

        let res: Vec<_> = q.iter().map(|&x| x).collect();
        assert_eq!(res, [7, 6, 5, 4, 3]);
    }

    #[test]
    fn clear() {
        let mut q = CopperListsManager::<_, 5>::new();
        q.push(1);
        q.push(2);
        q.push(3);
        q.push(4);
        q.push(5);
        q.push(6);
        q.push(7);

        q.clear();

        assert_eq!(q.len(), 0);
        assert_eq!(q.iter().next(), None);

        q.push(1);
        q.push(2);
        q.push(3);

        assert_eq!(q.len(), 3);

        let res: Vec<_> = q.iter().map(|&x| x).collect();
        assert_eq!(res, [3, 2, 1]);
    }

    #[test]
    fn mutable_iterator() {
        let mut q = CopperListsManager::<_, 5>::new();
        q.push(1);
        q.push(2);
        q.push(3);
        q.push(4);
        q.push(5);
        q.push(6);
        q.push(7);

        for x in q.iter_mut() {
            *x *= 2;
        }

        let res: Vec<_> = q.iter().map(|&x| x).collect();
        assert_eq!(res, [14, 12, 10, 8, 6]);
    }

    #[test]
    fn zero_sized() {
        let mut q = CopperListsManager::<_, 5>::new();
        assert_eq!(q.capacity(), 5);

        q.push(());
        q.push(());
        q.push(());

        assert_eq!(q.len(), 3);

        let mut iter = q.iter();
        assert_eq!(iter.next(), Some(&()));
        assert_eq!(iter.next(), Some(&()));
        assert_eq!(iter.next(), Some(&()));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn empty_queue_eq() {
        let q1 = CopperListsManager::<i32, 5>::new();
        let q2 = CopperListsManager::<i32, 5>::new();
        assert_eq!(q1, q2);

        // I am not sure here
        // let q3 = CircularQueue::<i32, U6>::new();
        // assert_eq!(q1, q3); // Capacity doesn't matter as long as the same elements are yielded.
    }

    #[test]
    fn partially_full_queue_eq() {
        let mut q1 = CopperListsManager::<i32, 5>::new();
        q1.push(1);
        q1.push(2);
        q1.push(3);

        let mut q2 = CopperListsManager::<i32, 5>::new();
        q2.push(1);
        q2.push(2);
        assert_ne!(q1, q2);

        q2.push(3);
        assert_eq!(q1, q2);

        q2.push(4);
        assert_ne!(q1, q2);
    }

    #[test]
    fn full_queue_eq() {
        let mut q1 = CopperListsManager::<i32, 5>::new();
        q1.push(1);
        q1.push(2);
        q1.push(3);
        q1.push(4);
        q1.push(5);

        let mut q2 = CopperListsManager::<i32, 5>::new();
        q2.push(1);
        q2.push(2);
        q2.push(3);
        q2.push(4);
        q2.push(5);

        assert_eq!(q1, q2);
    }

    #[test]
    fn over_full_queue_eq() {
        let mut q1 = CopperListsManager::<i32, 5>::new();
        q1.push(1);
        q1.push(2);
        q1.push(3);
        q1.push(4);
        q1.push(5);
        q1.push(6);
        q1.push(7);

        let mut q2 = CopperListsManager::<i32, 5>::new();
        q2.push(1);
        q2.push(2);
        q2.push(3);
        q2.push(4);
        q2.push(5);
        q2.push(6);
        assert_ne!(q1, q2);

        q2.push(7);
        assert_eq!(q1, q2);

        q2.push(8);
        assert_ne!(q1, q2);

        q2.push(3);
        q2.push(4);
        q2.push(5);
        q2.push(6);
        q2.push(7);
        assert_eq!(q1, q2);
    }

    #[test]
    fn clear_eq() {
        let mut q1 = CopperListsManager::<i32, 5>::new();
        q1.push(1);
        q1.push(2);
        q1.push(3);
        q1.push(4);
        q1.push(5);
        q1.push(6);
        q1.push(7);
        q1.clear();

        let mut q2 = CopperListsManager::<i32, 5>::new();
        assert_eq!(q1, q2);

        q2.push(1);
        q2.clear();
        assert_eq!(q1, q2);
    }

    #[test]
    fn zero_sized_eq() {
        let mut q1 = CopperListsManager::<_, 3>::new();
        q1.push(());
        q1.push(());
        q1.push(());
        q1.push(());

        let mut q2 = CopperListsManager::<_, 3>::new();
        q2.push(());
        q2.push(());
        assert_ne!(q1, q2);

        q2.push(());
        assert_eq!(q1, q2);

        q2.push(());
        assert_eq!(q1, q2);

        q2.push(());
        assert_eq!(q1, q2);
    }

    #[test]
    fn be_sure_we_wont_stackoverflow_at_init() {
        let _ = CopperListsManager::<[u8; 10_000_000], 3>::new();
    }
}
