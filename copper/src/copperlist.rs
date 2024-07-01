extern crate alloc;

use bincode_derive::{Decode, Encode};
use std::fmt;

use std::iter::{Chain, Rev};
use std::slice::{Iter as SliceIter, IterMut as SliceIterMut};

use copper_traits::CopperListPayload;

const MAX_TASKS: usize = 512;

#[derive(Debug, Encode, Decode)]
pub struct CopperLiskMask {
    #[allow(dead_code)]
    mask: [u128; MAX_TASKS / 128 + 1],
}

#[derive(Debug, Encode, Decode)]
pub enum CopperListState {
    Free,
    Initialized,
    ProcessingTasks(CopperLiskMask),
    BeingSerialized,
}

#[derive(Debug, Encode, Decode)]
pub struct CopperList<P: CopperListPayload> {
    state: CopperListState,
    pub payload: P, // This is generated from the runtime.
}

impl<P: CopperListPayload> CopperList<P> {
    // This is not the usual way to create a CopperList, this is just for testing.
    fn new(payload: P) -> Self {
        CopperList {
            state: CopperListState::Initialized,
            payload,
        }
    }
}

/// This structure maintains the entire memory needed by Copper for one loop for the inter tasks communication within a process.
/// T is typically a Tuple of various types of messages that are exchanged between tasks.
pub struct CuListsManager<P: CopperListPayload, DC: Fn(&CopperList<P>), const N: usize> {
    #[allow(dead_code)]
    data: Box<[CopperList<P>; N]>,
    length: usize,
    insertion_index: usize,
    on_drop: DC, // callback when the top of the queue is dropped.
}

impl<P: CopperListPayload + fmt::Debug, DC: Fn(&CopperList<P>), const N: usize> fmt::Debug
    for CuListsManager<P, DC, N>
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CuListsManager")
            .field("data", &self.data)
            .field("length", &self.length)
            .field("insertion_index", &self.insertion_index)
            // Do not include on_drop field
            .finish()
    }
}

/// An iterator over `CircularQueue<T>`.
pub type Iter<'a, T> = Chain<Rev<SliceIter<'a, T>>, Rev<SliceIter<'a, T>>>;

/// A mutable iterator over `CircularQueue<T>`.
pub type IterMut<'a, T> = Chain<Rev<SliceIterMut<'a, T>>, Rev<SliceIterMut<'a, T>>>;

/// An ascending iterator over `CircularQueue<T>`.
pub type AscIter<'a, T> = Chain<SliceIter<'a, T>, SliceIter<'a, T>>;

/// An mutable ascending iterator over `CircularQueue<T>`.
pub type AscIterMut<'a, T> = Chain<SliceIterMut<'a, T>, SliceIterMut<'a, T>>;

impl<P: CopperListPayload, DC: Fn(&CopperList<P>), const N: usize> CuListsManager<P, DC, N> {
    pub fn new(on_drop: DC) -> Self {
        let data = unsafe {
            let layout = std::alloc::Layout::new::<[CopperList<P>; N]>();
            let ptr = std::alloc::alloc_zeroed(layout) as *mut [CopperList<P>; N];
            Box::from_raw(ptr)
        };
        const INITIAL_SLSTATE: CopperListState = CopperListState::Free;
        CuListsManager {
            data,
            length: 0,
            insertion_index: 0,
            on_drop,
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
    pub fn create(&mut self) -> Option<&mut CopperList<P>> {
        if self.is_full() {
            return None;
        }
        let result = &mut self.data[self.insertion_index];
        self.insertion_index = (self.insertion_index + 1) % N;
        self.length += 1;

        Some(result)
    }

    #[inline]
    pub fn peek(&self) -> Option<&CopperList<P>> {
        if self.length == 0 {
            return None;
        }
        let index = if self.insertion_index == 0 {
            N - 1
        } else {
            self.insertion_index - 1
        };
        Some(&self.data[index])
    }

    #[inline]
    pub fn peek_mut(&mut self) -> Option<&mut CopperList<P>> {
        if self.length == 0 {
            return None;
        }
        let index = if self.insertion_index == 0 {
            N - 1
        } else {
            self.insertion_index - 1
        };
        Some(&mut self.data[index])
    }

    #[inline]
    fn drop_last(&mut self) {
        if self.length == 0 {
            return;
        }
        (self.on_drop)(&self.data[self.insertion_index]);
        if self.insertion_index == 0 {
            self.insertion_index = N - 1;
        } else {
            self.insertion_index -= 1;
        }
        self.length -= 1;
    }

    #[inline]
    pub fn pop(&mut self) -> Option<&mut CopperList<P>> {
        if self.length == 0 {
            return None;
        }
        (self.on_drop)(&self.data[self.insertion_index]);
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
    pub fn iter(&self) -> Iter<CopperList<P>> {
        let (a, b) = self.data[0..self.length].split_at(self.insertion_index);
        a.iter().rev().chain(b.iter().rev())
    }

    /// Returns a mutable iterator over the queue's contents.
    ///
    /// The iterator goes from the most recently pushed items to the oldest ones.
    ///
    #[inline]
    pub fn iter_mut(&mut self) -> IterMut<CopperList<P>> {
        let (a, b) = self.data.split_at_mut(self.insertion_index);
        a.iter_mut().rev().chain(b.iter_mut().rev())
    }

    /// Returns an ascending iterator over the queue's contents.
    ///
    /// The iterator goes from the least recently pushed items to the newest ones.
    ///
    #[inline]
    pub fn asc_iter(&self) -> AscIter<CopperList<P>> {
        let (a, b) = self.data.split_at(self.insertion_index);
        b.iter().chain(a.iter())
    }

    /// Returns a mutable ascending iterator over the queue's contents.
    ///
    /// The iterator goes from the least recently pushed items to the newest ones.
    ///
    #[inline]
    pub fn asc_iter_mut(&mut self) -> AscIterMut<CopperList<P>> {
        let (a, b) = self.data.split_at_mut(self.insertion_index);
        b.iter_mut().chain(a.iter_mut())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curuntime::no_action;
    use std::sync::{Arc, Mutex};

    type NoAction32 = fn(&CopperList<i32>);
    pub fn no_action_32(_: &CopperList<i32>) {}

    #[test]
    fn empty_queue() {
        let q = CuListsManager::<i32, NoAction32, 5>::new(|_| {});

        assert!(q.is_empty());
        assert!(q.iter().next().is_none());
    }

    #[test]
    fn partially_full_queue() {
        let mut q = CuListsManager::<i32, NoAction32, 5>::new(|_| {});
        q.create().unwrap().payload = 1;
        q.create().unwrap().payload = 2;
        q.create().unwrap().payload = 3;

        assert!(!q.is_empty());
        assert_eq!(q.len(), 3);

        let res: Vec<i32> = q.iter().map(|x| x.payload).collect();
        assert_eq!(res, [3, 2, 1]);
    }

    #[test]
    fn full_queue() {
        let mut q = CuListsManager::<i32, NoAction32, 5>::new(|_| {});
        q.create().unwrap().payload = 1;
        q.create().unwrap().payload = 2;
        q.create().unwrap().payload = 3;
        q.create().unwrap().payload = 4;
        q.create().unwrap().payload = 5;
        assert_eq!(q.len(), 5);

        let res: Vec<_> = q.iter().map(|x| x.payload).collect();
        assert_eq!(res, [5, 4, 3, 2, 1]);
    }

    #[test]
    fn over_full_queue() {
        let mut q = CuListsManager::<i32, NoAction32, 5>::new(|_| {});
        q.create().unwrap().payload = 1;
        q.create().unwrap().payload = 2;
        q.create().unwrap().payload = 3;
        q.create().unwrap().payload = 4;
        q.create().unwrap().payload = 5;
        assert!(q.create().is_none());
        assert_eq!(q.len(), 5);

        let res: Vec<_> = q.iter().map(|x| x.payload).collect();
        assert_eq!(res, [5, 4, 3, 2, 1]);
    }

    #[test]
    fn clear() {
        let mut q = CuListsManager::<i32, NoAction32, 5>::new(|_| {});
        q.create().unwrap().payload = 1;
        q.create().unwrap().payload = 2;
        q.create().unwrap().payload = 3;
        q.create().unwrap().payload = 4;
        q.create().unwrap().payload = 5;
        assert!(q.create().is_none());
        assert_eq!(q.len(), 5);

        q.clear();

        assert_eq!(q.len(), 0);
        assert!(q.iter().next().is_none());

        q.create().unwrap().payload = 1;
        q.create().unwrap().payload = 2;
        q.create().unwrap().payload = 3;

        assert_eq!(q.len(), 3);

        let res: Vec<_> = q.iter().map(|x| x.payload).collect();
        assert_eq!(res, [3, 2, 1]);
    }

    #[test]
    fn mutable_iterator() {
        let mut q = CuListsManager::<i32, NoAction32, 5>::new(|_| {});
        q.create().unwrap().payload = 1;
        q.create().unwrap().payload = 2;
        q.create().unwrap().payload = 3;
        q.create().unwrap().payload = 4;
        q.create().unwrap().payload = 5;

        for x in q.iter_mut() {
            x.payload *= 2;
        }

        let res: Vec<_> = q.iter().map(|x| x.payload).collect();
        assert_eq!(res, [10, 8, 6, 4, 2]);
    }

    #[test]
    fn zero_sized() {
        let mut q = CuListsManager::<(), fn(&CopperList<()>), 5>::new(|_| {});
        *q.create().unwrap() = CopperList::new(());
        *q.create().unwrap() = CopperList::new(());
        *q.create().unwrap() = CopperList::new(());

        assert_eq!(q.len(), 3);

        let mut iter = q.iter();
        assert_eq!(iter.next().unwrap().payload, ());
        assert_eq!(iter.next().unwrap().payload, ());
        assert_eq!(iter.next().unwrap().payload, ());
        assert!(iter.next().is_none());
    }

    #[test]
    fn be_sure_we_wont_stackoverflow_at_init() {
        let _ =
            CuListsManager::<[u8; 10_000_000], fn(&CopperList<[u8; 10_000_000]>), 3>::new(|_| {});
    }

    #[test]
    fn test_drop_last() {
        let mut q = CuListsManager::<i32, NoAction32, 5>::new(|_| {});
        q.create().unwrap().payload = 1;
        q.create().unwrap().payload = 2;
        q.create().unwrap().payload = 3;
        q.create().unwrap().payload = 4;
        q.create().unwrap().payload = 5;
        assert_eq!(q.len(), 5);

        q.drop_last();
        assert_eq!(q.len(), 4);

        let res: Vec<_> = q.iter().map(|x| x.payload).collect();
        assert_eq!(res, [4, 3, 2, 1]);
    }

    #[test]
    fn test_pop() {
        let mut q = CuListsManager::<i32, NoAction32, 5>::new(|_| {});
        q.create().unwrap().payload = 1;
        q.create().unwrap().payload = 2;
        q.create().unwrap().payload = 3;
        q.create().unwrap().payload = 4;
        q.create().unwrap().payload = 5;
        assert_eq!(q.len(), 5);

        let mut last = q.pop().unwrap();
        assert_eq!(last.payload, 5);
        assert_eq!(q.len(), 4);

        let res: Vec<_> = q.iter().map(|x| x.payload).collect();
        assert_eq!(res, [4, 3, 2, 1]);
    }

    #[test]
    fn test_peek() {
        let mut q = CuListsManager::<i32, NoAction32, 5>::new(|_| {});
        q.create().unwrap().payload = 1;
        q.create().unwrap().payload = 2;
        q.create().unwrap().payload = 3;
        q.create().unwrap().payload = 4;
        q.create().unwrap().payload = 5;
        assert_eq!(q.len(), 5);

        let last = q.peek().unwrap();
        assert_eq!(last.payload, 5);
        assert_eq!(q.len(), 5);

        let res: Vec<_> = q.iter().map(|x| x.payload).collect();
        assert_eq!(res, [5, 4, 3, 2, 1]);
    }

    #[test]
    fn test_callback() {
        let called = Arc::new(Mutex::new(false));
        let called_clone = Arc::clone(&called);
        let mut q = CuListsManager::<i32, _, 5>::new(move |_: &CopperList<i32>| {
            *called_clone.lock().unwrap() = true
        });
        q.create().unwrap().payload = 1;
        q.create().unwrap().payload = 2;
        q.create().unwrap().payload = 3;
        q.create().unwrap().payload = 4;
        q.create().unwrap().payload = 5;
        assert_eq!(q.len(), 5);

        q.pop();
        assert!(*called.lock().unwrap());
    }
}
