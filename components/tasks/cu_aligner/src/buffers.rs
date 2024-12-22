use circular_buffer::CircularBuffer;
use cu29::clock::{CuTime, Tov};
use cu29::cutask::{CuMsg, CuMsgPayload};
use cu29::{CuError, CuResult};

/// An augmented circular buffer that allows for time-based operations.
pub struct TimeboundCircularBuffer<const S: usize, P>
where
    P: CuMsgPayload,
{
    pub inner: CircularBuffer<S, CuMsg<P>>,
}

#[allow(dead_code)]
fn extract_tov_time_left(tov: &Tov) -> Option<CuTime> {
    match tov {
        Tov::Time(time) => Some(*time),
        Tov::Range(range) => Some(range.start), // Use the start of the range for alignment
        Tov::None => None,
    }
}

fn extract_tov_time_right(tov: &Tov) -> Option<CuTime> {
    match tov {
        Tov::Time(time) => Some(*time),
        Tov::Range(range) => Some(range.end), // Use the end of the range for alignment
        Tov::None => None,
    }
}

impl<const S: usize, P> Default for TimeboundCircularBuffer<S, P>
where
    P: CuMsgPayload,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const S: usize, P> TimeboundCircularBuffer<S, P>
where
    P: CuMsgPayload,
{
    pub fn new() -> Self {
        TimeboundCircularBuffer {
            // It is assumed to be sorted by time with non overlapping ranges if they are Tov::Range
            inner: CircularBuffer::<S, CuMsg<P>>::new(),
        }
    }

    /// Gets a slice of messages that fall within the given time range.
    /// In case of a Tov::Range, the message is included if its start and end time fall within the range.
    pub fn iter_window(
        &self,
        start_time: CuTime,
        end_time: CuTime,
    ) -> impl Iterator<Item = &CuMsg<P>> {
        self.inner.iter().filter(move |msg| match msg.metadata.tov {
            Tov::Time(time) => time >= start_time && time <= end_time,
            Tov::Range(range) => range.start >= start_time && range.end <= end_time,
            _ => false,
        })
    }

    /// Remove all the messages that are older than the given time horizon.
    pub fn purge(&mut self, time_horizon: CuTime) {
        // Find the index of the first element that should be retained
        let drain_end = self
            .inner
            .iter()
            .position(|msg| match msg.metadata.tov {
                Tov::Time(time) => time >= time_horizon,
                Tov::Range(range) => range.end >= time_horizon,
                _ => false,
            })
            .unwrap_or(self.inner.len()); // If none match, drain the entire buffer

        // Drain all elements before the `drain_end` index
        self.inner.drain(..drain_end);
    }

    /// Get the most recent time of the messages in the buffer.
    pub fn most_recent_time(&self) -> CuResult<Option<CuTime>> {
        self.inner
            .iter()
            .map(|msg| extract_tov_time_right(&msg.metadata.tov))
            .try_fold(None, |acc, time| {
                let time = time.ok_or_else(|| {
                    CuError::from("Trying to align temporal data with no time information")
                })?;
                Ok(Some(
                    acc.map_or(time, |current_max: CuTime| current_max.max(time)),
                ))
            })
    }

    /// Push a message into the buffer.
    pub fn push(&mut self, msg: CuMsg<P>) {
        self.inner.push_back(msg);
    }
}

#[macro_export]
macro_rules! alignment_buffers {
    ($struct_name:ident, $($name:ident: TimeboundCircularBuffer<$size:expr, CuMsg<$payload:ty>>),*) => {
        struct $struct_name {
            target_alignment_window: cu29::clock::CuDuration, // size of the most recent data window to align
            stale_data_horizon: cu29::clock::CuDuration,  // time horizon for purging stale data
            $(pub $name: $crate::buffers::TimeboundCircularBuffer<$size, $payload>),*
        }

        impl $struct_name {
            pub fn new(target_alignment_window: cu29::clock::CuDuration, stale_data_horizon: cu29::clock::CuDuration) -> Self {
                Self {
                    target_alignment_window,
                    stale_data_horizon,
                    $($name: $crate::buffers::TimeboundCircularBuffer::<$size, $payload>::new()),*
                }
            }

            /// Call this to be sure we discard the old/ non relevant data
            #[allow(dead_code)]
            pub fn purge(&mut self, now: cu29::clock::CuTime) {
                let horizon_time = now - self.stale_data_horizon;
                // purge all the stale data from the TimeboundCircularBuffers first
                $(self.$name.purge(horizon_time);)*
            }

            /// Get the most recent set of aligned data from all the buffers matching the constraints set at construction.
            #[allow(dead_code)]
            pub fn get_latest_aligned_data(
                &mut self,
            ) -> Option<($(impl Iterator<Item = &cu29::cutask::CuMsg<$payload>>),*)> {
                // Now find the min of the max of the last time for all buffers
                // meaning the most recent time at which all buffers have data
                let most_recent_time = [
                    $(self.$name.most_recent_time().unwrap_or(None)),*
                ]
                .iter()
                .filter_map(|&time| time)
                .min();

                // If there is no data in any of the buffers, return early
                most_recent_time?;

                let most_recent_time = most_recent_time.unwrap();

                let time_to_get_complete_window = most_recent_time - self.target_alignment_window;
                Some(($(self.$name.iter_window(time_to_get_complete_window, most_recent_time)),*))
            }
        }
    };
}

pub use alignment_buffers;

#[cfg(test)]
mod tests {
    use cu29::clock::Tov;
    use cu29::cutask::CuMsg;
    use std::time::Duration;

    #[test]
    fn simple_init_test() {
        alignment_buffers!(AlignmentBuffers, buffer1: TimeboundCircularBuffer<10, CuMsg<u32>>, buffer2: TimeboundCircularBuffer<12, CuMsg<u64>>);

        let buffers =
            AlignmentBuffers::new(Duration::from_secs(1).into(), Duration::from_secs(2).into());
        assert_eq!(buffers.buffer1.inner.capacity(), 10);
        assert_eq!(buffers.buffer2.inner.capacity(), 12);
    }

    #[test]
    fn purge_test() {
        alignment_buffers!(AlignmentBuffers, buffer1: TimeboundCircularBuffer<10, CuMsg<u32>>, buffer2: TimeboundCircularBuffer<12, CuMsg<u32>>);

        let mut buffers =
            AlignmentBuffers::new(Duration::from_secs(1).into(), Duration::from_secs(2).into());

        let mut msg1 = CuMsg::new(Some(1));
        msg1.metadata.tov = Tov::Time(Duration::from_secs(1).into());
        buffers.buffer1.inner.push_back(msg1.clone());
        buffers.buffer2.inner.push_back(msg1);
        // within the horizon
        buffers.purge(Duration::from_secs(2).into());
        assert_eq!(buffers.buffer1.inner.len(), 1);
        assert_eq!(buffers.buffer2.inner.len(), 1);
        // outside the horizon
        buffers.purge(Duration::from_secs(5).into());
        assert_eq!(buffers.buffer1.inner.len(), 0);
        assert_eq!(buffers.buffer2.inner.len(), 0);
    }

    #[test]
    fn empty_buffers_test() {
        alignment_buffers!(
            AlignmentBuffers,
            buffer1: TimeboundCircularBuffer<10, CuMsg<u32>>,
            buffer2: TimeboundCircularBuffer<12, CuMsg<u32>>
        );

        let mut buffers = AlignmentBuffers::new(
            Duration::from_secs(2).into(), // 2-second alignment window
            Duration::from_secs(5).into(), // 5-second stale data horizon
        );

        // Advance time to 10 seconds
        assert!(buffers.get_latest_aligned_data().is_none());
    }

    #[test]
    fn horizon_and_window_alignment_test() {
        alignment_buffers!(
            AlignmentBuffers,
            buffer1: TimeboundCircularBuffer<10, CuMsg<u32>>,
            buffer2: TimeboundCircularBuffer<12, CuMsg<u32>>
        );

        let mut buffers = AlignmentBuffers::new(
            Duration::from_secs(2).into(), // 2-second alignment window
            Duration::from_secs(5).into(), // 5-second stale data horizon
        );

        // Insert messages with timestamps
        let mut msg1 = CuMsg::new(Some(1));
        msg1.metadata.tov = Tov::Time(Duration::from_secs(1).into());
        buffers.buffer1.inner.push_back(msg1.clone());
        buffers.buffer2.inner.push_back(msg1);

        let mut msg2 = CuMsg::new(Some(3));
        msg2.metadata.tov = Tov::Time(Duration::from_secs(3).into());
        buffers.buffer2.inner.push_back(msg2);

        let mut msg3 = CuMsg::new(Some(4));
        msg3.metadata.tov = Tov::Time(Duration::from_secs(4).into());
        buffers.buffer1.inner.push_back(msg3.clone());
        buffers.buffer2.inner.push_back(msg3);

        // Advance time to 7 seconds; horizon is 7 - 5 = everything 2+ should stay
        let now = Duration::from_secs(7).into();
        // Emulate a normal workflow here.
        buffers.purge(now);
        if let Some((iter1, iter2)) = buffers.get_latest_aligned_data() {
            let collected1: Vec<_> = iter1.collect();
            let collected2: Vec<_> = iter2.collect();

            // Verify only messages within the alignment window [5, 7] are returned
            assert_eq!(collected1.len(), 1);
            assert_eq!(collected2.len(), 2);

            assert_eq!(collected1[0].payload(), Some(&4));
            assert_eq!(collected2[0].payload(), Some(&3));
            assert_eq!(collected2[1].payload(), Some(&4));
        } else {
            panic!("Expected aligned data, but got None");
        }

        // Ensure older messages outside the horizon [>2 seconds] are purged
        assert_eq!(buffers.buffer1.inner.len(), 1);
        assert_eq!(buffers.buffer2.inner.len(), 2);
    }
}
