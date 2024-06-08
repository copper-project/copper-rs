extern crate alloc;

use std::iter::{Chain, Rev};
use std::slice::{Iter as SliceIter, IterMut as SliceIterMut};

const MAX_TASKS: usize = 512;

#[derive(Debug)]
struct CopperLiskMask {
    mask: [u128; MAX_TASKS / 128 + 1],
}

#[derive(Debug)]
enum CopperListState {
    Free,
    ProcessingTasks(CopperLiskMask),
    BeingSerialized,
}

/// This structure maintains the entire memory needed by Copper for one process for the inter task communication.
#[derive(Debug)]
pub struct CuListsManager<T: Sized + PartialEq, const N: usize> {
    copper_list_states: [CopperListState; N],
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

impl<T: Sized + PartialEq, const N: usize> PartialEq for CuListsManager<T, N> {
    fn eq(&self, other: &Self) -> bool {
        if self.len() != other.len() {
            return false;
        }
        other.iter().zip(self.iter()).all(|(a, b)| a == b)
    }
}

impl<T: Sized + PartialEq, const N: usize> CuListsManager<T, N> {
    pub fn new() -> Self {
        let data = unsafe {
            let layout = std::alloc::Layout::new::<[T; N]>();
            let ptr = std::alloc::alloc_zeroed(layout) as *mut [T; N];
            Box::from_raw(ptr)
        };
        const INITIAL_SLSTATE: CopperListState = CopperListState::Free;
        CuListsManager {
            copper_list_states: [INITIAL_SLSTATE; N],
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
        N == self.len()
    }

    /// Clears the queue.
    ///
    #[inline]
    pub fn clear(&mut self) {
        self.insertion_index = 0;
        self.length = 0;
    }

    #[inline]
    pub fn create(&mut self) -> Option<&mut T> {
        if self.is_full() {
            return None;
        }
        let result = &mut self.data[self.insertion_index];
        self.insertion_index = (self.insertion_index + 1) % N;
        self.length += 1;

        Some(result)
    }

    #[inline]
    pub fn pop(&mut self) -> Option<&mut T> {
        if self.length == 0 {
            return None;
        }
        if self.insertion_index == 0 {
            self.insertion_index = N - 1;
        } else {
            self.insertion_index -= 1;
        }
        self.length -= 1;
        Some(&mut self.data[self.insertion_index])
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
    fn empty_queue() {
        let q = CuListsManager::<i32, 5>::new();

        assert!(q.is_empty());
        assert_eq!(q.iter().next(), None);
    }

    #[test]
    fn partially_full_queue() {
        let mut q = CuListsManager::<i32, 5>::new();
        *q.create().unwrap() = 1;
        *q.create().unwrap() = 2;
        *q.create().unwrap() = 3;

        assert!(!q.is_empty());
        assert_eq!(q.len(), 3);

        let res: Vec<_> = q.iter().map(|&x| x).collect();
        assert_eq!(res, [3, 2, 1]);
    }

    #[test]
    fn full_queue() {
        let mut q = CuListsManager::<_, 5>::new();
        *q.create().unwrap() = 1;
        *q.create().unwrap() = 2;
        *q.create().unwrap() = 3;
        *q.create().unwrap() = 4;
        *q.create().unwrap() = 5;
        assert_eq!(q.len(), 5);

        let res: Vec<_> = q.iter().map(|&x| x).collect();
        assert_eq!(res, [5, 4, 3, 2, 1]);
    }

    #[test]
    fn over_full_queue() {
        let mut q = CuListsManager::<_, 5>::new();
        *q.create().unwrap() = 1;
        *q.create().unwrap() = 2;
        *q.create().unwrap() = 3;
        *q.create().unwrap() = 4;
        *q.create().unwrap() = 5;
        assert_eq!(q.create(), None);
        assert_eq!(q.len(), 5);

        let res: Vec<_> = q.iter().map(|&x| x).collect();
        assert_eq!(res, [5, 4, 3, 2, 1]);
    }

    #[test]
    fn clear() {
        let mut q = CuListsManager::<_, 5>::new();
        *q.create().unwrap() = 1;
        *q.create().unwrap() = 2;
        *q.create().unwrap() = 3;
        *q.create().unwrap() = 4;
        *q.create().unwrap() = 5;
        assert_eq!(q.create(), None);
        assert_eq!(q.len(), 5);

        q.clear();

        assert_eq!(q.len(), 0);
        assert_eq!(q.iter().next(), None);

        *q.create().unwrap() = 1;
        *q.create().unwrap() = 2;
        *q.create().unwrap() = 3;

        assert_eq!(q.len(), 3);

        let res: Vec<_> = q.iter().map(|&x| x).collect();
        assert_eq!(res, [3, 2, 1]);
    }

    #[test]
    fn mutable_iterator() {
        let mut q = CuListsManager::<_, 5>::new();
        *q.create().unwrap() = 1;
        *q.create().unwrap() = 2;
        *q.create().unwrap() = 3;
        *q.create().unwrap() = 4;
        *q.create().unwrap() = 5;

        for x in q.iter_mut() {
            *x *= 2;
        }

        let res: Vec<_> = q.iter().map(|&x| x).collect();
        assert_eq!(res, [10, 8, 6, 4, 2]);
    }

    #[test]
    fn zero_sized() {
        let mut q = CuListsManager::<_, 5>::new();
        *q.create().unwrap() = ();
        *q.create().unwrap() = ();
        *q.create().unwrap() = ();

        assert_eq!(q.len(), 3);

        let mut iter = q.iter();
        assert_eq!(iter.next(), Some(&()));
        assert_eq!(iter.next(), Some(&()));
        assert_eq!(iter.next(), Some(&()));
        assert_eq!(iter.next(), None);
    }

    #[test]
    fn empty_queue_eq() {
        let q1 = CuListsManager::<i32, 5>::new();
        let q2 = CuListsManager::<i32, 5>::new();
        assert_eq!(q1, q2);
    }

    #[test]
    fn partially_full_queue_eq() {
        let mut q1 = CuListsManager::<i32, 5>::new();
        *q1.create().unwrap() = 1;
        *q1.create().unwrap() = 2;
        *q1.create().unwrap() = 3;

        let mut q2 = CuListsManager::<i32, 5>::new();
        *q2.create().unwrap() = 1;
        *q2.create().unwrap() = 2;
        assert_ne!(q1, q2);

        *q2.create().unwrap() = 3;
        assert_eq!(q1, q2);

        *q2.create().unwrap() = 4;
        assert_ne!(q1, q2);
    }

    #[test]
    fn full_queue_eq() {
        let mut q1 = CuListsManager::<i32, 5>::new();
        *q1.create().unwrap() = 1;
        *q1.create().unwrap() = 2;
        *q1.create().unwrap() = 3;
        *q1.create().unwrap() = 4;
        *q1.create().unwrap() = 5;

        let mut q2 = CuListsManager::<i32, 5>::new();
        *q2.create().unwrap() = 1;
        *q2.create().unwrap() = 2;
        *q2.create().unwrap() = 3;
        *q2.create().unwrap() = 4;
        *q2.create().unwrap() = 5;

        assert_eq!(q1, q2);
    }

    #[test]
    fn clear_eq() {
        let mut q1 = CuListsManager::<i32, 5>::new();
        *q1.create().unwrap() = 1;
        *q1.create().unwrap() = 2;
        *q1.create().unwrap() = 3;
        *q1.create().unwrap() = 4;
        *q1.create().unwrap() = 5;
        q1.clear();

        let mut q2 = CuListsManager::<i32, 5>::new();
        assert_eq!(q1, q2);

        *q2.create().unwrap() = 1;
        q2.clear();
        assert_eq!(q1, q2);
    }

    #[test]
    fn zero_sized_eq() {
        let mut q1 = CuListsManager::<_, 3>::new();
        *q1.create().unwrap() = ();
        *q1.create().unwrap() = ();
        *q1.create().unwrap() = ();

        let mut q2 = CuListsManager::<_, 3>::new();
        *q2.create().unwrap() = ();
        *q2.create().unwrap() = ();
        assert_ne!(q1, q2);

        *q2.create().unwrap() = ();
        assert_eq!(q1, q2);

        q2.create();
        assert_eq!(q1, q2);

        q2.create();
        assert_eq!(q1, q2);
    }

    #[test]
    fn be_sure_we_wont_stackoverflow_at_init() {
        let _ = CuListsManager::<[u8; 10_000_000], 3>::new();
    }
}
