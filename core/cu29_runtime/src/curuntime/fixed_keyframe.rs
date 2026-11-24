//! Inline capture storage for generated static-config no_std applications.
use super::{
    CapturePayload, CuError, CuResult, CuTime, EncodeError, Freezable, KEYFRAME_FRAME_HEADER_LEN,
    KEYFRAME_PAYLOAD_HEADER, KeyFrame, RobotClock, WriteStream, frozen_payload_size,
};
use crate::log::*;
use alloc::boxed::Box;
pub use heapless::Vec as InlineVec;

impl<const N: usize> CapturePayload for InlineVec<u8, N> {
    fn append_bytes(&mut self, bytes: &[u8]) -> Result<(), EncodeError> {
        self.extend_from_slice(bytes)
            .map_err(|_| EncodeError::UnexpectedEnd)
    }

    fn truncate_to(&mut self, len: usize) {
        self.truncate(len);
    }
}

// The macro visits exactly the same components, in the same order, as capture.
pub struct KeyFrameSizer(usize);

impl KeyFrameSizer {
    pub fn include_capture_capacity(&mut self, item: &impl Freezable) -> Result<(), EncodeError> {
        let payload = frozen_payload_size(item)?;
        self.0 = self
            .0
            .checked_add(KEYFRAME_FRAME_HEADER_LEN)
            .and_then(|size| size.checked_add(payload))
            .ok_or(EncodeError::Other("keyframe size overflow"))?;
        Ok(())
    }
}

pub struct FixedKeyFrameCapture<const N: usize> {
    frame: KeyFrame<InlineVec<u8, N>>,
    sink: Option<Box<dyn WriteStream<KeyFrame<InlineVec<u8, N>>>>>,
    interval: u32,
    capturing: bool,
    pub dropped: u64,
}

impl<const N: usize> FixedKeyFrameCapture<N> {
    pub fn new(
        sink: impl WriteStream<KeyFrame<InlineVec<u8, N>>> + 'static,
        interval: u32,
    ) -> CuResult<Self> {
        if interval == 0 {
            return Err(CuError::from("Keyframe interval cannot be zero"));
        }
        Ok(Self {
            frame: KeyFrame {
                culistid: 0,
                timestamp: CuTime::default(),
                serialized_tasks: InlineVec::new(),
            },
            sink: Some(Box::new(sink)),
            interval,
            capturing: false,
            dropped: 0,
        })
    }

    pub fn check_initial_capacity(
        &self,
        measure: impl FnOnce(&mut KeyFrameSizer) -> Result<(), EncodeError>,
    ) {
        let mut size = KeyFrameSizer(KEYFRAME_PAYLOAD_HEADER.len());
        if measure(&mut size).is_err() {
            error!(
                "Could not measure initial keyframe; capture will be retried at its scheduled interval"
            );
        } else if size.0 > N {
            let required = size.0;
            error!(
                "Initial keyframe needs {} bytes; logging.keyframe_capacity is {}. In the logging block set keyframe_capacity to at least {} or set enable_keyframe_logging to false.",
                required, N, required
            );
        }
    }

    pub fn reset(&mut self, id: u64, clock: &RobotClock) {
        self.capturing = id.is_multiple_of(u64::from(self.interval));
        if self.capturing {
            self.frame.culistid = id;
            self.frame.timestamp = clock.now();
            self.frame.serialized_tasks.clear();
            if self
                .frame
                .serialized_tasks
                .extend_from_slice(KEYFRAME_PAYLOAD_HEADER)
                .is_err()
            {
                self.reject(EncodeError::UnexpectedEnd);
            }
        }
    }

    fn reject(&mut self, error: EncodeError) {
        self.capturing = false;
        self.dropped = self.dropped.saturating_add(1);
        match error {
            EncodeError::UnexpectedEnd => error!(
                "Skipping keyframe {}: snapshot exceeds logging.keyframe_capacity ({} bytes)",
                self.frame.culistid, N
            ),
            _ => error!(
                "Skipping keyframe {}: component serialization failed",
                self.frame.culistid
            ),
        }
    }

    pub fn freeze_task(&mut self, _: u64, item: &impl Freezable) -> CuResult<()> {
        if self.capturing
            && let Err(error) = self.frame.add_frozen_task(item)
        {
            self.reject(error);
        }
        Ok(())
    }

    pub fn freeze_any(&mut self, id: u64, item: &impl Freezable) -> CuResult<()> {
        self.freeze_task(id, item)
    }

    pub fn finish(&mut self) -> u64 {
        if core::mem::take(&mut self.capturing)
            && let Some(sink) = &mut self.sink
        {
            if sink.log(&self.frame).is_ok() {
                return sink.last_log_bytes().unwrap_or(0) as u64;
            }
            self.dropped = self.dropped.saturating_add(1);
            error!(
                "Failed to write keyframe {}; continuing without it",
                self.frame.culistid
            );
        }
        0
    }

    pub fn close_output(&mut self) -> CuResult<()> {
        self.sink.take().map_or(Ok(()), |mut sink| sink.flush())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curuntime::{KeyFrame, KeyFramePayloadReader};
    use alloc::sync::Arc;
    use bincode::enc::Encoder;
    use std::sync::Mutex;

    struct Bytes<'a>(&'a [u8]);

    impl Freezable for Bytes<'_> {
        fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
            bincode::enc::write::Writer::write(encoder.writer(), self.0)
        }
    }

    #[derive(Debug, Default, Clone)]
    struct Recorder(Arc<Mutex<Vec<Vec<u8>>>>);

    impl<const N: usize> WriteStream<KeyFrame<InlineVec<u8, N>>> for Recorder {
        fn log(&mut self, frame: &KeyFrame<InlineVec<u8, N>>) -> CuResult<()> {
            self.0
                .lock()
                .unwrap()
                .push(bincode::encode_to_vec(frame, bincode::config::standard()).unwrap());
            Ok(())
        }
    }

    #[test]
    fn oversized_frame_is_skipped_and_the_next_exact_fit_replays() {
        let (clock, _) = RobotClock::mock();
        let sink = Recorder::default();
        let mut capture = FixedKeyFrameCapture::<14>::new(sink.clone(), 2).unwrap();
        capture.check_initial_capacity(|size| {
            size.include_capture_capacity(&Bytes(&[1]))?;
            size.include_capture_capacity(&Bytes(&[2]))?;
            assert_eq!(size.0, 15);
            Ok(())
        });
        for (id, second) in [(0, &[2][..]), (1, &[]), (2, &[])] {
            capture.reset(id, &clock);
            capture.freeze_task(id, &Bytes(&[1])).unwrap();
            capture.freeze_any(id, &Bytes(second)).unwrap();
            capture.finish();
        }
        assert_eq!(capture.dropped, 1);
        let records = sink.0.lock().unwrap();
        assert_eq!(records.len(), 1);
        let (frame, consumed): (KeyFrame, _) =
            bincode::decode_from_slice(&records[0], bincode::config::standard()).unwrap();
        assert_eq!(consumed, records[0].len());
        assert_eq!(frame.culistid, 2);
        let mut reader = KeyFramePayloadReader::new(&frame).unwrap();
        assert_eq!(reader.next_frame().unwrap(), &[1]);
        assert!(reader.next_frame().unwrap().is_empty());
        reader.finish().unwrap();
        assert_eq!(
            bincode::encode_to_vec(&frame, bincode::config::standard()).unwrap(),
            records[0]
        );
    }

    #[test]
    fn insufficient_space_for_header_is_nonfatal() {
        let (clock, _) = RobotClock::mock();
        let sink = Recorder::default();
        let mut capture = FixedKeyFrameCapture::<0>::new(sink.clone(), 1).unwrap();
        capture.reset(0, &clock);
        capture.freeze_task(0, &Bytes(&[])).unwrap();
        capture.finish();
        assert_eq!(capture.dropped, 1);
        assert!(sink.0.lock().unwrap().is_empty());
    }
}
