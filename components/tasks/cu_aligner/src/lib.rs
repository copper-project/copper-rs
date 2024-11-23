use circular_buffer::CircularBuffer;
use cu29::clock::{CuTime, Tov};
use cu29::cutask::CuMsg;
use cu29::cutask::CuMsgPayload;
use cu29::{CuError, CuResult};

pub struct TimeboundCircularBuffer<const S: usize, P>
where
    P: CuMsgPayload,
{
    pub inner: CircularBuffer<S, CuMsg<P>>,
}

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
}

macro_rules! alignment_buffers {
    ($struct_name:ident, $($name:ident: TimeboundCircularBuffer<$size:expr, CuMsg<$payload:ty>>),*) => {
        struct $struct_name {
            target_alignment_window: CuDuration, // size of the most recent data window to align
            stale_data_horizon: CuDuration,  // time horizon for purging stale data
            $(pub $name: TimeboundCircularBuffer<$size, $payload>),*
        }

        impl $struct_name {
            pub fn new(target_alignment_window: CuDuration, stale_data_horizon: CuDuration) -> Self {
                Self {
                    target_alignment_window,
                    stale_data_horizon,
                    $($name: TimeboundCircularBuffer::<$size, $payload>::new()),*
                }
            }
            pub fn update(
                &mut self,
                now: CuTime,
            ) -> Vec<($(Option<&CuMsg<$payload>>,)*)> {
                let mut aligned_data = Vec::new();

                let horizon_time = now - self.stale_data_horizon;

                // purge all the stale data from the TimeboundCircularBuffers first
                $(self.$name.purge(horizon_time);)*

                // Now find the min of the max of the last time for all buffers
                // meaning the most recent time at which all buffers have data
                let most_recent_time = [
                    $(self.$name.most_recent_time().unwrap_or(None)),*
                ]
                .iter()
                .filter_map(|&time| time)
                .min()
                .unwrap_or(now);

                aligned_data
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::clock::CuDuration;
    use std::time::Duration;

    #[test]
    fn simple_init_test() {
        alignment_buffers!(AlignmentBuffers, buffer1: TimeboundCircularBuffer<10, CuMsg<u32>>, buffer2: TimeboundCircularBuffer<12, CuMsg<u64>>);

        let buffers =
            AlignmentBuffers::new(Duration::from_secs(1).into(), Duration::from_secs(2).into());
        assert_eq!(buffers.buffer1.inner.capacity(), 10);
        assert_eq!(buffers.buffer2.inner.capacity(), 20);
    }

    #[test]
    fn update_test() {
        alignment_buffers!(AlignmentBuffers, buffer1: TimeboundCircularBuffer<10, CuMsg<u32>>, buffer2: TimeboundCircularBuffer<12, CuMsg<u32>>);

        let mut buffers =
            AlignmentBuffers::new(Duration::from_secs(1).into(), Duration::from_secs(2).into());

        let mut msg1 = CuMsg::new(Some(1));
        msg1.metadata.tov = Tov::Time(Duration::from_secs(1).into());
        buffers.buffer1.inner.push_back(msg1.clone());
        buffers.buffer2.inner.push_back(msg1);
        let aligned_data = buffers.update(Duration::from_secs(2).into());
        assert_eq!(aligned_data.len(), 0);
    }
}
