//! Ownership handoff for manually serviced serial output.
use crate::copperlist::{CopperList, CuListZeroedInit, CuListsManager};
use crate::curuntime::KeyFrame;
use alloc::{collections::VecDeque, vec::Vec};
use cu29_traits::{CopperListTuple, CuResult};

/// Borrowed only while firmware services a completed record.
#[doc(hidden)]
pub enum ManualRecord<'a, P: CopperListTuple> {
    Capture(&'a CopperList<P>),
    KeyFrame(&'a KeyFrame),
}

pub(crate) struct ManualOutput<P: CopperListTuple> {
    pub active: CuListsManager<P, 1>,
    free: Vec<CuListsManager<P, 1>>,
    frames: Vec<KeyFrame>,
    pending: VecDeque<(CuListsManager<P, 1>, Option<KeyFrame>)>,
    pub drops: u64,
    pub frame_drops: u64,
}

impl<P: CuListZeroedInit> ManualOutput<P> {
    pub fn new(count: u16, frame_count: u16) -> CuResult<Self> {
        if count == 0 || frame_count == 0 {
            return Err("manual telemetry requires output slots".into());
        }
        let frames = (0..frame_count)
            .map(|_| KeyFrame::from_serialized(0, Default::default(), Vec::new()))
            .collect();
        Ok(Self {
            active: CuListsManager::new(),
            free: (0..count).map(|_| CuListsManager::new()).collect(),
            pending: VecDeque::with_capacity(usize::from(count)),
            frames,
            drops: 0,
            frame_drops: 0,
        })
    }
}

impl<P: CopperListTuple> ManualOutput<P> {
    pub fn prepare_frames(&mut self, capacity: usize) -> CuResult<()> {
        for frame in self
            .frames
            .iter_mut()
            .chain(self.pending.iter_mut().filter_map(|(_, f)| f.as_mut()))
        {
            crate::curuntime::reserve_keyframe_capacity(frame, capacity)?;
        }
        Ok(())
    }
    pub fn complete(&mut self, frame: Option<&mut KeyFrame>) {
        let Some(mut next) = self.free.pop() else {
            self.active.pop();
            self.frame_drops = self.frame_drops.saturating_add(u64::from(frame.is_some()));
            self.drops = self.drops.saturating_add(1);
            return;
        };
        next.set_next_replay_id(self.active.next_cl_id());
        core::mem::swap(&mut self.active, &mut next);
        let frame = frame.and_then(|current| {
            let Some(mut spare) = self.frames.pop() else {
                self.frame_drops = self.frame_drops.saturating_add(1);
                return None;
            };
            core::mem::swap(&mut spare, current);
            Some(spare)
        });
        self.pending.push_back((next, frame));
    }

    pub fn service(
        &mut self,
        send: impl FnOnce(ManualRecord<'_, P>) -> CuResult<()>,
    ) -> CuResult<()> {
        let Some((_, frame)) = self.pending.front_mut() else {
            return Ok(());
        };
        if let Some(frame) = frame.take() {
            let result = send(ManualRecord::KeyFrame(&frame));
            self.frames.push(frame);
            return result;
        }
        let (mut slot, _) = self.pending.pop_front().expect("pending capture");
        let result = send(ManualRecord::Capture(
            slot.pop().expect("completed capture"),
        ));
        self.free.push(slot);
        result
    }
}
