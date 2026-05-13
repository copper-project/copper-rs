use crate::bincode::{Decode, Encode};
use crate::copperlist::{CopperList, CopperListState, CuListZeroedInit, CuListsManager};
use crate::curuntime::SyncCopperListsManager;
use crate::{
    CuResult, ErasedCuStampedData, ErasedCuStampedDataSet, MatchingTasks, WriteStream, safety_case,
    safety_check, safety_check_eq,
};
use serde::{Deserialize, Serialize};
use std::boxed::Box;
use std::sync::{Arc, Mutex};
use std::vec::Vec;

#[derive(Debug, Default, Encode, Decode, Serialize, Deserialize)]
struct IntMsgs(i32);

impl ErasedCuStampedDataSet for IntMsgs {
    fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
        Vec::new()
    }
}

impl MatchingTasks for IntMsgs {
    fn get_all_task_ids() -> &'static [&'static str] {
        &[]
    }
}

impl CuListZeroedInit for IntMsgs {
    fn init_zeroed(&mut self) {}
}

#[derive(Debug)]
struct RecordingSyncWriter {
    ids: Arc<Mutex<Vec<u64>>>,
}

impl RecordingSyncWriter {
    fn new(ids: Arc<Mutex<Vec<u64>>>) -> Self {
        Self { ids }
    }
}

impl WriteStream<CopperList<IntMsgs>> for RecordingSyncWriter {
    fn log(&mut self, culist: &CopperList<IntMsgs>) -> CuResult<()> {
        self.ids.lock().unwrap().push(culist.id);
        Ok(())
    }
}

fn desc_values<const N: usize>(q: &CuListsManager<IntMsgs, N>) -> Vec<i32> {
    q.iter().map(|cl| cl.msgs.0).collect()
}

fn asc_values<const N: usize>(q: &CuListsManager<IntMsgs, N>) -> Vec<i32> {
    q.asc_iter().map(|cl| cl.msgs.0).collect()
}

#[cfg_attr(test, test)]
#[safety_case("CLM-TEST-001")]
fn copperlist_capacity_is_bounded() {
    let mut q = CuListsManager::<IntMsgs, 2>::new();
    let _ = q.create().unwrap();
    let _ = q.create().unwrap();

    safety_check!("CLM-TEST-001-C1", "CLM-REQ-001", q.create().is_none(),);

    let mut sync = SyncCopperListsManager::<IntMsgs, 1>::new(None).unwrap();
    let _ = sync.create().unwrap();
    let exhausted = sync.create().unwrap_err();

    safety_check!(
        "CLM-TEST-001-C2",
        "CLM-REQ-001",
        exhausted
            .to_string()
            .contains("Ran out of space for copper lists"),
    );
}

#[cfg_attr(test, test)]
#[safety_case("CLM-TEST-002")]
fn copperlist_ids_are_monotonic_across_reuse() {
    let mut q = CuListsManager::<IntMsgs, 3>::new();
    let before = (q.next_cl_id(), q.last_cl_id());

    let cl0 = q.create().unwrap().id;
    let cl1 = q.create().unwrap().id;
    let _ = q.pop().unwrap();
    let cl2 = q.create().unwrap().id;
    let after = (cl0, cl1, cl2, q.next_cl_id(), q.last_cl_id());

    safety_check_eq!(
        "CLM-TEST-002-C1",
        "CLM-REQ-002",
        (before, after),
        ((0, 0), (0, 1, 2, 3, 2)),
    );
}

#[cfg_attr(test, test)]
#[safety_case("CLM-TEST-003")]
fn copperlist_accessors_preserve_active_slot_ordering() {
    let mut empty = CuListsManager::<IntMsgs, 3>::new();
    let empty_peek = empty.peek().is_none();
    let empty_pop = empty.pop().is_none();
    let empty_asc = empty.asc_iter().next().is_none();

    safety_check!(
        "CLM-TEST-003-C1",
        "CLM-REQ-003",
        empty_peek && empty_pop && empty_asc,
    );

    let mut desc = CuListsManager::<IntMsgs, 5>::new();
    desc.create().unwrap().msgs.0 = 1;
    desc.create().unwrap().msgs.0 = 2;
    desc.create().unwrap().msgs.0 = 3;

    let mut visited_desc = Vec::new();
    for cl in desc.iter_mut() {
        visited_desc.push(cl.id);
        cl.msgs.0 *= 10;
    }

    safety_check_eq!(
        "CLM-TEST-003-C2",
        "CLM-REQ-003",
        (visited_desc, desc_values(&desc)),
        (vec![2, 1, 0], vec![30, 20, 10]),
    );

    let mut asc_nonwrapped = CuListsManager::<IntMsgs, 5>::new();
    asc_nonwrapped.create().unwrap().msgs.0 = 10;
    asc_nonwrapped.create().unwrap().msgs.0 = 20;
    asc_nonwrapped.create().unwrap().msgs.0 = 30;

    let mut visited_asc = Vec::new();
    for (offset, cl) in asc_nonwrapped.asc_iter_mut().enumerate() {
        visited_asc.push(cl.id);
        cl.msgs.0 += offset as i32;
    }

    safety_check_eq!(
        "CLM-TEST-003-C3",
        "CLM-REQ-003",
        (visited_asc, asc_values(&asc_nonwrapped)),
        (vec![0, 1, 2], vec![10, 21, 32]),
    );

    let mut wrapped = CuListsManager::<IntMsgs, 5>::new();
    for value in 1..=5 {
        wrapped.create().unwrap().msgs.0 = value;
    }
    let _ = wrapped.pop().unwrap();
    let _ = wrapped.pop().unwrap();
    wrapped.create().unwrap().msgs.0 = 6;
    wrapped.create().unwrap().msgs.0 = 7;

    safety_check_eq!(
        "CLM-TEST-003-C4",
        "CLM-REQ-003",
        (desc_values(&wrapped), asc_values(&wrapped)),
        (vec![7, 6, 3, 2, 1], vec![1, 2, 3, 6, 7]),
    );
}

#[cfg_attr(test, test)]
#[safety_case("CLM-TEST-004")]
fn copperlist_allocation_resets_runtime_state() {
    let mut fresh_manager = CuListsManager::<IntMsgs, 1>::new();
    let fresh = {
        let cl = fresh_manager.create().unwrap();
        (cl.id, cl.get_state(), cl.msgs.0)
    };
    let fresh_ids = (fresh_manager.next_cl_id(), fresh_manager.last_cl_id());

    safety_check_eq!(
        "CLM-TEST-004-C1",
        "CLM-REQ-004",
        (fresh, fresh_ids),
        ((0, CopperListState::Initialized, 0), (1, 0)),
    );

    let mut reused_manager = CuListsManager::<IntMsgs, 1>::new();
    {
        let cl = reused_manager.create().unwrap();
        cl.msgs.0 = 41;
        cl.change_state(CopperListState::Processing);
    }
    let popped = {
        let cl = reused_manager.pop().unwrap();
        (cl.id, cl.get_state(), cl.msgs.0)
    };
    assert_eq!(popped, (0, CopperListState::Processing, 41));

    let reused = {
        let cl = reused_manager.create().unwrap();
        (cl.id, cl.get_state(), cl.msgs.0)
    };
    let reused_ids = (reused_manager.next_cl_id(), reused_manager.last_cl_id());

    safety_check_eq!(
        "CLM-TEST-004-C2",
        "CLM-REQ-004",
        (reused, reused_ids),
        ((1, CopperListState::Initialized, 41), (2, 1)),
    );

    let mut sync = SyncCopperListsManager::<IntMsgs, 1>::new(None).unwrap();
    {
        let cl = sync.create().unwrap();
        cl.msgs.0 = 41;
        cl.change_state(CopperListState::Processing);
    }
    sync.end_of_processing(0).unwrap();
    let sync_reused = {
        let cl = sync.create().unwrap();
        (cl.id, cl.get_state(), cl.msgs.0)
    };

    safety_check_eq!(
        "CLM-TEST-004-C3",
        "CLM-REQ-004",
        sync_reused,
        (1, CopperListState::Initialized, 41),
    );
}

#[cfg_attr(test, test)]
#[safety_case("CLM-TEST-005")]
fn sync_runtime_reclaims_only_the_completed_top_suffix() {
    let ids = Arc::new(Mutex::new(Vec::new()));
    let mut sync = SyncCopperListsManager::<IntMsgs, 2>::new(Some(Box::new(
        RecordingSyncWriter::new(ids.clone()),
    )))
    .unwrap();

    {
        let cl = sync.create().unwrap();
        cl.msgs.0 = 10;
        cl.change_state(CopperListState::Processing);
    }
    {
        let cl = sync.create().unwrap();
        cl.msgs.0 = 20;
        cl.change_state(CopperListState::Processing);
    }

    sync.end_of_processing(0).unwrap();
    let pre_ids = ids.lock().unwrap().clone();
    let pre_available = sync.available_copper_lists().unwrap();

    sync.end_of_processing(1).unwrap();
    let post_ids = ids.lock().unwrap().clone();
    let post_available = sync.available_copper_lists().unwrap();

    safety_check_eq!(
        "CLM-TEST-005-C1",
        "CLM-REQ-005",
        ((pre_ids, pre_available), (post_ids, post_available)),
        ((Vec::<u64>::new(), 0usize), (vec![1, 0], 2usize)),
    );
}

#[cfg(feature = "safety-ids")]
pub fn link_safety_ids() {
    let _ = copperlist_capacity_is_bounded as fn();
    let _ = copperlist_ids_are_monotonic_across_reuse as fn();
    let _ = copperlist_accessors_preserve_active_slot_ordering as fn();
    let _ = copperlist_allocation_resets_runtime_state as fn();
    let _ = sync_runtime_reclaims_only_the_completed_top_suffix as fn();
}
