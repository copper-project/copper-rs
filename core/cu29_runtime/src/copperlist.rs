//! CopperList is the main data structure used by Copper to communicate between tasks.
//! It is a queue that can be used to store preallocated messages between tasks in memory order.
#[cfg(not(feature = "std"))]
extern crate alloc;

use alloc::alloc::{alloc_zeroed, handle_alloc_error};
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::alloc::Layout;

use bincode::{Decode, Encode};
use core::fmt;

use core::fmt::Display;
use core::iter::{Chain, Rev};
use core::slice::{Iter as SliceIter, IterMut as SliceIterMut};
use cu29_traits::{CopperListTuple, ErasedCuStampedData, ErasedCuStampedDataSet};
use serde_derive::{Deserialize, Serialize};

const MAX_TASKS: usize = 512;

/// Not implemented yet.
/// This mask will be used to for example filter out necessary regions of a copper list between remote systems.
#[derive(Debug, Encode, Decode, PartialEq, Clone, Copy)]
pub struct CopperLiskMask {
    #[allow(dead_code)]
    mask: [u128; MAX_TASKS / 128 + 1],
}

/// Those are the possible states along the lifetime of a CopperList.
#[derive(Debug, Encode, Decode, Serialize, Deserialize, PartialEq, Copy, Clone)]
pub enum CopperListState {
    Free,
    Initialized,
    Processing,
    DoneProcessing,
    QueuedForSerialization,
    BeingSerialized,
}

impl Display for CopperListState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CopperListState::Free => write!(f, "Free"),
            CopperListState::Initialized => write!(f, "Initialized"),
            CopperListState::Processing => write!(f, "Processing"),
            CopperListState::DoneProcessing => write!(f, "DoneProcessing"),
            CopperListState::QueuedForSerialization => write!(f, "QueuedForSerialization"),
            CopperListState::BeingSerialized => write!(f, "BeingSerialized"),
        }
    }
}

#[derive(Debug, Encode, Decode, Serialize, Deserialize)]
pub struct CopperList<P: CopperListTuple> {
    pub id: u64,
    state: CopperListState,
    pub msgs: P, // This is generated from the runtime.
}

impl<P: CopperListTuple> Default for CopperList<P> {
    fn default() -> Self {
        CopperList {
            id: 0,
            state: CopperListState::Free,
            msgs: P::default(),
        }
    }
}

impl<P: CopperListTuple> CopperList<P> {
    // This is not the usual way to create a CopperList, this is just for testing.
    pub fn new(id: u64, msgs: P) -> Self {
        CopperList {
            id,
            state: CopperListState::Initialized,
            msgs,
        }
    }

    pub fn change_state(&mut self, new_state: CopperListState) {
        self.state = new_state; // TODO: probably wise here to enforce a state machine.
    }

    pub fn get_state(&self) -> CopperListState {
        self.state
    }

    /// Restores the lifecycle state expected at allocation time and reruns
    /// zero-memory fixups for payload containers that cannot remain valid after
    /// raw zeroing. This does not imply a full `P::default()` payload reset.
    #[doc(hidden)]
    pub fn reset_for_runtime_use(&mut self, id: u64)
    where
        P: CuListZeroedInit,
    {
        self.id = id;
        self.state = CopperListState::Initialized;
        self.msgs.init_zeroed();
    }
}

impl<P: CopperListTuple> ErasedCuStampedDataSet for CopperList<P> {
    fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
        self.msgs.cumsgs()
    }
}

/// This structure maintains the entire memory needed by Copper for one loop for the inter tasks communication within a process.
/// P or Payload is typically a Tuple of various types of messages that are exchanged between tasks.
/// N is the maximum number of in flight Copper List the runtime can support.
pub struct CuListsManager<P: CopperListTuple, const N: usize> {
    data: Box<[CopperList<P>; N]>,
    length: usize,
    insertion_index: usize,
    current_cl_id: u64,
}

impl<P: CopperListTuple + fmt::Debug, const N: usize> fmt::Debug for CuListsManager<P, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("CuListsManager")
            .field("data", &self.data)
            .field("length", &self.length)
            .field("insertion_index", &self.insertion_index)
            // Do not include on_drop field
            .finish()
    }
}

pub type Iter<'a, T> = Chain<Rev<SliceIter<'a, T>>, Rev<SliceIter<'a, T>>>;
pub type IterMut<'a, T> = Chain<Rev<SliceIterMut<'a, T>>, Rev<SliceIterMut<'a, T>>>;
pub type AscIter<'a, T> = Chain<SliceIter<'a, T>, SliceIter<'a, T>>;
pub type AscIterMut<'a, T> = Chain<SliceIterMut<'a, T>, SliceIterMut<'a, T>>;

/// Initializes fields that cannot be zeroed after allocating a zeroed
/// [`CopperList`].
pub trait CuListZeroedInit: CopperListTuple {
    /// Fixes up a zero-initialized copper list so that all internal fields are
    /// in a valid state.
    fn init_zeroed(&mut self);
}

impl<P: CopperListTuple + CuListZeroedInit, const N: usize> Default for CuListsManager<P, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P: CopperListTuple, const N: usize> CuListsManager<P, N> {
    pub fn new() -> Self
    where
        P: CuListZeroedInit,
    {
        // SAFETY: We allocate zeroed memory and immediately initialize required fields.
        let data = unsafe {
            let layout = Layout::new::<[CopperList<P>; N]>();
            let ptr = alloc_zeroed(layout) as *mut [CopperList<P>; N];
            if ptr.is_null() {
                handle_alloc_error(layout);
            }
            Box::from_raw(ptr)
        };
        let mut manager = CuListsManager {
            data,
            length: 0,
            insertion_index: 0,
            current_cl_id: 0,
        };

        for cl in manager.data.iter_mut() {
            cl.msgs.init_zeroed();
        }

        manager
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
    pub fn create(&mut self) -> Option<&mut CopperList<P>>
    where
        P: CuListZeroedInit,
    {
        if self.is_full() {
            return None;
        }
        let next_id = self.current_cl_id;
        let result = &mut self.data[self.insertion_index];
        self.insertion_index = (self.insertion_index + 1) % N;
        self.length += 1;

        // We assign a unique id to each CopperList to be able to track them across their lifetime.
        result.reset_for_runtime_use(next_id);
        self.current_cl_id += 1;

        Some(result)
    }

    /// Returns the next copper-list id that will be assigned by [`create`](Self::create).
    #[inline]
    pub fn next_cl_id(&self) -> u64 {
        self.current_cl_id
    }

    /// Returns the most recently assigned copper-list id.
    ///
    /// Before the first call to [`create`](Self::create), this returns `0`.
    #[inline]
    pub fn last_cl_id(&self) -> u64 {
        self.current_cl_id.saturating_sub(1)
    }

    /// Peeks at the last element in the queue.
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
    #[allow(dead_code)]
    fn drop_last(&mut self) {
        if self.length == 0 {
            return;
        }
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
    pub fn iter(&self) -> Iter<'_, CopperList<P>> {
        let (a, b) = self.data[0..self.length].split_at(self.insertion_index);
        a.iter().rev().chain(b.iter().rev())
    }

    /// Returns a mutable iterator over the queue's contents.
    ///
    /// The iterator goes from the most recently pushed items to the oldest ones.
    ///
    #[inline]
    pub fn iter_mut(&mut self) -> IterMut<'_, CopperList<P>> {
        let (a, b) = self.data[0..self.length].split_at_mut(self.insertion_index);
        a.iter_mut().rev().chain(b.iter_mut().rev())
    }

    /// Returns an ascending iterator over the queue's contents.
    ///
    /// The iterator goes from the least recently pushed items to the newest ones.
    ///
    #[inline]
    pub fn asc_iter(&self) -> AscIter<'_, CopperList<P>> {
        let (a, b) = self.data[0..self.length].split_at(self.insertion_index);
        b.iter().chain(a.iter())
    }

    /// Returns a mutable ascending iterator over the queue's contents.
    ///
    /// The iterator goes from the least recently pushed items to the newest ones.
    ///
    #[inline]
    pub fn asc_iter_mut(&mut self) -> AscIterMut<'_, CopperList<P>> {
        let (a, b) = self.data[0..self.length].split_at_mut(self.insertion_index);
        b.iter_mut().chain(a.iter_mut())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29_traits::{ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks};
    use serde::{Deserialize, Serialize, Serializer};

    #[derive(Debug, Encode, Decode, PartialEq, Clone, Copy, Serialize, Deserialize, Default)]
    struct CuStampedDataSet(i32);

    impl ErasedCuStampedDataSet for CuStampedDataSet {
        fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
            Vec::new()
        }
    }

    impl MatchingTasks for CuStampedDataSet {
        fn get_all_task_ids() -> &'static [&'static str] {
            &[]
        }
    }

    impl CuListZeroedInit for CuStampedDataSet {
        fn init_zeroed(&mut self) {}
    }

    #[test]
    fn empty_queue() {
        let q = CuListsManager::<CuStampedDataSet, 5>::new();

        assert!(q.is_empty());
        assert!(q.iter().next().is_none());
        assert!(q.asc_iter().next().is_none());
        assert!(q.peek().is_none());
    }

    #[test]
    fn partially_full_queue() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;

        assert!(!q.is_empty());
        assert_eq!(q.len(), 3);

        let res: Vec<i32> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [3, 2, 1]);
    }

    #[test]
    fn full_queue() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;
        q.create().unwrap().msgs.0 = 4;
        q.create().unwrap().msgs.0 = 5;
        assert_eq!(q.len(), 5);

        let res: Vec<_> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [5, 4, 3, 2, 1]);
    }

    #[test]
    fn over_full_queue() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;
        q.create().unwrap().msgs.0 = 4;
        q.create().unwrap().msgs.0 = 5;
        assert!(q.create().is_none());
        assert_eq!(q.len(), 5);

        let res: Vec<_> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [5, 4, 3, 2, 1]);
    }

    #[test]
    fn clear() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;
        q.create().unwrap().msgs.0 = 4;
        q.create().unwrap().msgs.0 = 5;
        assert!(q.create().is_none());
        assert_eq!(q.len(), 5);

        q.clear();

        assert_eq!(q.len(), 0);
        assert!(q.iter().next().is_none());

        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;

        assert_eq!(q.len(), 3);

        let res: Vec<_> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [3, 2, 1]);
    }

    #[test]
    fn create_fresh_slot_starts_initialized_with_zeroed_payload() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();

        let cl = q.create().unwrap();
        assert_eq!(cl.id, 0);
        assert_eq!(cl.get_state(), CopperListState::Initialized);
        assert_eq!(cl.msgs.0, 0);
        assert_eq!(q.next_cl_id(), 1);
        assert_eq!(q.last_cl_id(), 0);
    }

    #[test]
    fn create_reused_slot_reinitializes_state_but_preserves_payload_storage() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();

        {
            let cl = q.create().unwrap();
            cl.msgs.0 = 41;
            cl.change_state(CopperListState::Processing);
        }

        let popped = q.pop().unwrap();
        assert_eq!(popped.id, 0);
        assert_eq!(popped.get_state(), CopperListState::Processing);
        assert_eq!(popped.msgs.0, 41);

        let reused = q.create().unwrap();
        assert_eq!(reused.id, 1);
        assert_eq!(reused.get_state(), CopperListState::Initialized);
        assert_eq!(reused.msgs.0, 41);
        assert_eq!(q.next_cl_id(), 2);
        assert_eq!(q.last_cl_id(), 1);
    }

    #[test]
    fn mutable_iterator() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;
        q.create().unwrap().msgs.0 = 4;
        q.create().unwrap().msgs.0 = 5;

        for x in q.iter_mut() {
            x.msgs.0 *= 2;
        }

        let res: Vec<_> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [10, 8, 6, 4, 2]);
    }

    #[test]
    fn mutable_iterator_non_wrapped_only_visits_active_slots() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;

        let mut visited = Vec::new();
        for cl in q.iter_mut() {
            visited.push(cl.id);
            cl.msgs.0 *= 10;
        }

        assert_eq!(visited, vec![2, 1, 0]);
        let res: Vec<_> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [30, 20, 10]);
    }

    #[test]
    fn test_drop_last() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;
        q.create().unwrap().msgs.0 = 4;
        q.create().unwrap().msgs.0 = 5;
        assert_eq!(q.len(), 5);

        q.drop_last();
        assert_eq!(q.len(), 4);

        let res: Vec<_> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [4, 3, 2, 1]);
    }

    #[test]
    fn drop_last_on_empty_queue_is_a_noop() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();

        q.drop_last();

        assert!(q.is_empty());
        assert!(q.peek().is_none());
        assert!(q.pop().is_none());
    }

    #[test]
    fn drop_last_on_non_wrapped_queue_removes_most_recent_slot() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;

        q.drop_last();

        assert_eq!(q.len(), 2);
        assert_eq!(q.peek().unwrap().msgs.0, 2);
        let res: Vec<_> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [2, 1]);
    }

    #[test]
    fn test_pop() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;
        q.create().unwrap().msgs.0 = 4;
        q.create().unwrap().msgs.0 = 5;
        assert_eq!(q.len(), 5);

        let last = q.pop().unwrap();
        assert_eq!(last.msgs.0, 5);
        assert_eq!(q.len(), 4);

        let res: Vec<_> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [4, 3, 2, 1]);
    }

    #[test]
    fn pop_on_empty_queue_returns_none() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();

        assert!(q.pop().is_none());
        assert!(q.is_empty());
    }

    #[test]
    fn test_peek() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 1;
        q.create().unwrap().msgs.0 = 2;
        q.create().unwrap().msgs.0 = 3;
        q.create().unwrap().msgs.0 = 4;
        q.create().unwrap().msgs.0 = 5;
        assert_eq!(q.len(), 5);

        let last = q.peek().unwrap();
        assert_eq!(last.msgs.0, 5);
        assert_eq!(q.len(), 5);

        let res: Vec<_> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [5, 4, 3, 2, 1]);
    }

    #[test]
    fn peek_on_empty_queue_returns_none() {
        let q = CuListsManager::<CuStampedDataSet, 5>::new();

        assert!(q.peek().is_none());
    }

    #[test]
    fn next_and_last_cl_id_track_assigned_ids() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();

        // Before first allocation, next id is 0 and last id saturates to 0.
        assert_eq!(q.next_cl_id(), 0);
        assert_eq!(q.last_cl_id(), 0);

        let cl0 = q.create().unwrap();
        assert_eq!(cl0.id, 0);
        assert_eq!(q.next_cl_id(), 1);
        assert_eq!(q.last_cl_id(), 0);

        let cl1 = q.create().unwrap();
        assert_eq!(cl1.id, 1);
        assert_eq!(q.next_cl_id(), 2);
        assert_eq!(q.last_cl_id(), 1);

        let _ = q.pop().unwrap();
        let cl2 = q.create().unwrap();
        assert_eq!(cl2.id, 2);
        assert_eq!(q.next_cl_id(), 3);
        assert_eq!(q.last_cl_id(), 2);
    }

    #[test]
    fn asc_iter_non_wrapped_returns_oldest_to_newest_without_free_slots() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 10;
        q.create().unwrap().msgs.0 = 20;
        q.create().unwrap().msgs.0 = 30;

        let res: Vec<_> = q.asc_iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [10, 20, 30]);
    }

    #[test]
    fn asc_iter_mut_non_wrapped_only_visits_active_slots_in_oldest_first_order() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        q.create().unwrap().msgs.0 = 10;
        q.create().unwrap().msgs.0 = 20;
        q.create().unwrap().msgs.0 = 30;

        let mut visited = Vec::new();
        for (offset, cl) in q.asc_iter_mut().enumerate() {
            visited.push(cl.id);
            cl.msgs.0 += offset as i32;
        }

        assert_eq!(visited, vec![0, 1, 2]);
        let res: Vec<_> = q.asc_iter().map(|x| x.msgs.0).collect();
        assert_eq!(res, [10, 21, 32]);
    }

    #[test]
    fn asc_iter_wrapped_layout_tracks_reused_slots_in_ascending_order() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        for value in 1..=5 {
            q.create().unwrap().msgs.0 = value;
        }
        assert_eq!(q.pop().unwrap().msgs.0, 5);
        assert_eq!(q.pop().unwrap().msgs.0, 4);
        q.create().unwrap().msgs.0 = 6;
        q.create().unwrap().msgs.0 = 7;

        let desc: Vec<_> = q.iter().map(|x| x.msgs.0).collect();
        assert_eq!(desc, [7, 6, 3, 2, 1]);

        let asc: Vec<_> = q.asc_iter().map(|x| x.msgs.0).collect();
        assert_eq!(asc, [1, 2, 3, 6, 7]);
    }

    #[test]
    fn asc_iter_mut_wrapped_layout_updates_oldest_to_newest_order() {
        let mut q = CuListsManager::<CuStampedDataSet, 5>::new();
        for value in 1..=5 {
            q.create().unwrap().msgs.0 = value;
        }
        let _ = q.pop().unwrap();
        let _ = q.pop().unwrap();
        q.create().unwrap().msgs.0 = 6;
        q.create().unwrap().msgs.0 = 7;

        let mut visited = Vec::new();
        for (offset, cl) in q.asc_iter_mut().enumerate() {
            visited.push(cl.id);
            cl.msgs.0 += offset as i32;
        }

        assert_eq!(visited, vec![0, 1, 2, 5, 6]);
        let asc: Vec<_> = q.asc_iter().map(|x| x.msgs.0).collect();
        assert_eq!(asc, [1, 3, 5, 9, 11]);
    }

    #[derive(Decode, Encode, Debug, PartialEq, Clone, Copy)]
    struct TestStruct {
        content: [u8; 10_000_000],
    }

    impl Default for TestStruct {
        fn default() -> Self {
            TestStruct {
                content: [0; 10_000_000],
            }
        }
    }

    impl ErasedCuStampedDataSet for TestStruct {
        fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
            Vec::new()
        }
    }

    impl Serialize for TestStruct {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: Serializer,
        {
            serializer.serialize_i8(0)
        }
    }

    impl MatchingTasks for TestStruct {
        fn get_all_task_ids() -> &'static [&'static str] {
            &[]
        }
    }

    impl CuListZeroedInit for TestStruct {
        fn init_zeroed(&mut self) {}
    }

    #[test]
    fn be_sure_we_wont_stackoverflow_at_init() {
        let _ = CuListsManager::<TestStruct, 3>::new();
    }
}
