//! Preallocated CopperList storage whose move-only leases cross runtime queues.

use crate::copperlist::{CopperList, CuListZeroedInit};
use crate::curuntime::KeyFrame;
use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
use core::ptr::NonNull;
use cu29_traits::CopperListTuple;
use std::sync::{Arc, OnceLock};

struct Storage<P: CopperListTuple> {
    lists: Box<[UnsafeCell<CopperList<P>>]>,
    keyframes: OnceLock<Box<[UnsafeCell<KeyFrame>]>>,
}

// Every element has exactly one lease. Generated execution may lend disjoint
// fields to lanes, then joins them before accessing or transferring the lease.
unsafe impl<P: CopperListTuple + Send> Send for Storage<P> {}
unsafe impl<P: CopperListTuple + Send> Sync for Storage<P> {}

/// Exclusive ownership of one fixed arena slot.
#[doc(hidden)]
pub struct CuSlotLease<P: CopperListTuple> {
    storage: Arc<Storage<P>>,
    ptr: NonNull<CopperList<P>>,
    generation: u64,
}

// The unique lease may move between the dispatcher, execution lanes, and the
// output worker. Its pointer always remains inside `storage`.
unsafe impl<P: CopperListTuple + Send> Send for CuSlotLease<P> {}

// Shared access exposes only shared list/keyframe references. Obtaining the
// raw execution pointer requires a mutable borrow of the unique lease.
unsafe impl<P: CopperListTuple + Send + Sync> Sync for CuSlotLease<P> {}

impl<P: CopperListTuple> CuSlotLease<P> {
    #[inline]
    pub fn index(&self) -> usize {
        // SAFETY: both pointers belong to the same contiguous allocation.
        unsafe {
            self.ptr
                .as_ptr()
                .offset_from(self.storage.lists.as_ptr().cast::<CopperList<P>>())
                as usize
        }
    }

    #[inline]
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// Reset an exclusively owned slot before publishing it to execution.
    pub fn reset(&mut self, id: u64)
    where
        P: CuListZeroedInit,
    {
        self.generation = self
            .generation
            .checked_add(1)
            .expect("CopperList slot generation exhausted");
        self.reset_for_runtime_use(id);
    }

    #[inline]
    pub fn as_ptr(&mut self) -> *mut CopperList<P> {
        self.ptr.as_ptr()
    }

    #[inline]
    pub fn keyframe(&self) -> Option<&KeyFrame> {
        let keyframes = self.storage.keyframes.get()?;
        // SAFETY: the unique slot lease controls this keyframe's lifecycle.
        Some(unsafe { &*keyframes[self.index()].get() })
    }

    #[inline]
    pub fn keyframe_mut(&mut self) -> Option<&mut KeyFrame> {
        let keyframes = self.storage.keyframes.get()?;
        // SAFETY: mutation requires the unique slot lease.
        Some(unsafe { &mut *keyframes[self.index()].get() })
    }

    /// Initialize every slot's keyframe regions together on the cold path.
    pub fn prepare_keyframes(&mut self, capacities: &[usize]) {
        self.storage.keyframes.get_or_init(|| {
            (0..self.storage.lists.len())
                .map(|_| UnsafeCell::new(KeyFrame::distributed(capacities)))
                .collect()
        });
    }

    #[inline]
    pub fn keyframe_capture(&mut self) -> (*mut u8, usize) {
        let Some(keyframe) = self.keyframe_mut() else {
            return (core::ptr::null_mut(), 0);
        };
        (
            keyframe.serialized_tasks.as_mut_ptr(),
            keyframe.serialized_tasks.len(),
        )
    }
}

impl<P: CopperListTuple> Deref for CuSlotLease<P> {
    type Target = CopperList<P>;

    fn deref(&self) -> &Self::Target {
        // SAFETY: the unique lease owns this element. Generated code joins all
        // raw field projections before borrowing the lease again.
        unsafe { &*self.ptr.as_ptr() }
    }
}

impl<P: CopperListTuple> DerefMut for CuSlotLease<P> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: mutable access requires the unique lease.
        unsafe { &mut *self.ptr.as_ptr() }
    }
}

impl<P: CopperListTuple> AsRef<CopperList<P>> for CuSlotLease<P> {
    fn as_ref(&self) -> &CopperList<P> {
        self
    }
}

impl<P: CopperListTuple> AsMut<CopperList<P>> for CuSlotLease<P> {
    fn as_mut(&mut self) -> &mut CopperList<P> {
        self
    }
}

/// Allocate one contiguous CopperList arena and its fixed ownership leases.
pub fn allocate_slots<P: CopperListTuple + CuListZeroedInit>(count: usize) -> Vec<CuSlotLease<P>> {
    let mut lists = Vec::<UnsafeCell<CopperList<P>>>::with_capacity(count);
    for index in 0..count {
        // SAFETY: capacity is fixed and every slot is initialized before it is
        // included in the vector length.
        unsafe {
            CopperList::<P>::init_in_place(lists.as_mut_ptr().add(index).cast());
            lists.set_len(index + 1);
        }
    }
    let storage = Arc::new(Storage {
        lists: lists.into_boxed_slice(),
        keyframes: OnceLock::new(),
    });
    (0..count)
        .map(|index| CuSlotLease {
            storage: Arc::clone(&storage),
            ptr: NonNull::new(storage.lists[index].get()).expect("allocated CopperList slot"),
            generation: 0,
        })
        .collect()
}
