//! Inline capture storage for generated static-config no_std applications.
use super::{
    BincodeAdapter, CuError, CuResult, CuTime, Encode, EncodeError, EncoderImpl, Freezable,
    KEYFRAME_FRAME_HEADER_LEN, KEYFRAME_PAYLOAD_HEADER, RegionWriter, RobotClock, WriteStream,
    frozen_payload_size,
};
use crate::log::*;
use alloc::boxed::Box;
use bincode::enc::Encoder;

/// The array stays in the application. Only the occupied prefix goes on the wire.
pub struct FixedKeyFrame<const N: usize> {
    id: u64,
    timestamp: CuTime,
    bytes: [u8; N],
    used: usize,
}

impl<const N: usize> Encode for FixedKeyFrame<N> {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.id.encode(encoder)?;
        self.timestamp.encode(encoder)?;
        self.bytes[..self.used].encode(encoder)
    }
}

impl<const N: usize> FixedKeyFrame<N> {
    fn append(&mut self, item: &impl Freezable) -> Result<(), EncodeError> {
        let (header, payload) = self.bytes[self.used..]
            .split_first_chunk_mut::<KEYFRAME_FRAME_HEADER_LEN>()
            .ok_or(EncodeError::UnexpectedEnd)?;
        let writer = RegionWriter {
            bytes: payload,
            written: 0,
        };
        let mut encoder = EncoderImpl::new(writer, bincode::config::standard());
        BincodeAdapter(item).encode(&mut encoder)?;
        let written = encoder.into_writer().written;
        *header = u32::try_from(written)
            .map_err(|_| EncodeError::Other("keyframe component exceeds u32 size"))?
            .to_le_bytes();
        self.used += KEYFRAME_FRAME_HEADER_LEN + written;
        Ok(())
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
    frame: FixedKeyFrame<N>,
    sink: Option<Box<dyn WriteStream<FixedKeyFrame<N>>>>,
    interval: u32,
    capturing: bool,
    pub dropped: u64,
}

impl<const N: usize> FixedKeyFrameCapture<N> {
    pub fn new(
        sink: impl WriteStream<FixedKeyFrame<N>> + 'static,
        interval: u32,
    ) -> CuResult<Self> {
        if interval == 0 {
            return Err(CuError::from("Keyframe interval cannot be zero"));
        }
        Ok(Self {
            frame: FixedKeyFrame {
                id: 0,
                timestamp: CuTime::default(),
                bytes: [0; N],
                used: 0,
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
            self.frame.id = id;
            self.frame.timestamp = clock.now();
            if let Some(header) = self.frame.bytes.get_mut(..KEYFRAME_PAYLOAD_HEADER.len()) {
                header.copy_from_slice(KEYFRAME_PAYLOAD_HEADER);
                self.frame.used = header.len();
            } else {
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
                self.frame.id, N
            ),
            _ => error!(
                "Skipping keyframe {}: component serialization failed",
                self.frame.id
            ),
        }
    }

    pub fn freeze_task(&mut self, _: u64, item: &impl Freezable) -> CuResult<()> {
        if self.capturing
            && let Err(error) = self.frame.append(item)
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
                self.frame.id
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
    use std::sync::Mutex;

    struct Bytes<'a>(&'a [u8]);

    impl Freezable for Bytes<'_> {
        fn freeze<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
            bincode::enc::write::Writer::write(encoder.writer(), self.0)
        }
    }

    #[derive(Debug, Default, Clone)]
    struct Recorder(Arc<Mutex<Vec<Vec<u8>>>>);

    impl<const N: usize> WriteStream<FixedKeyFrame<N>> for Recorder {
        fn log(&mut self, frame: &FixedKeyFrame<N>) -> CuResult<()> {
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
