use cu29::prelude::*;
use gstreamer::prelude::*;

use bincode::de::{BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{BorrowDecode, Decode, Encode};
use circular_buffer::FixedCircularBuffer;
use gstreamer::buffer::{BufferMap, Readable};
use gstreamer::{Buffer, BufferRef, Caps, FlowSuccess, Pipeline, parse};
use gstreamer_app::{AppSink, AppSinkCallbacks};
use serde::{Deserialize, Serialize};
use std::fmt::Debug;
use std::ops::Deref;
use std::str::FromStr;
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Reflect)]
#[reflect(opaque, from_reflect = false, no_field_bounds)]
pub enum CuGstBuffer {
    Live(Buffer),
    Replay(Arc<[u8]>),
}

pub enum CuGstBufferRead<'a> {
    Live(BufferMap<'a, Readable>),
    Replay(&'a [u8]),
}

impl<'a> CuGstBufferRead<'a> {
    pub fn as_slice(&self) -> &[u8] {
        match self {
            Self::Live(buffer) => buffer.as_slice(),
            Self::Replay(bytes) => bytes,
        }
    }
}

impl AsRef<[u8]> for CuGstBufferRead<'_> {
    fn as_ref(&self) -> &[u8] {
        self.as_slice()
    }
}

impl Deref for CuGstBufferRead<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl Default for CuGstBuffer {
    fn default() -> Self {
        Self::Replay(Vec::<u8>::new().into())
    }
}

impl From<Buffer> for CuGstBuffer {
    fn from(buffer: Buffer) -> Self {
        Self::Live(buffer)
    }
}

impl CuGstBuffer {
    pub fn map_readable(&self) -> CuResult<CuGstBufferRead<'_>> {
        match self {
            Self::Live(buffer) => buffer
                .as_ref()
                .map_readable()
                .map(CuGstBufferRead::Live)
                .map_err(|e| CuError::new_with_cause("Could not map the gstreamer buffer", e)),
            Self::Replay(bytes) => Ok(CuGstBufferRead::Replay(bytes)),
        }
    }

    pub fn as_live(&self) -> Option<&Buffer> {
        match self {
            Self::Live(buffer) => Some(buffer),
            Self::Replay(_) => None,
        }
    }
}

impl Serialize for CuGstBuffer {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.map_readable()
            .map_err(serde::ser::Error::custom)?
            .as_slice()
            .serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for CuGstBuffer {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let data = Vec::<u8>::deserialize(deserializer)?;
        Ok(CuGstBuffer::Replay(data.into()))
    }
}

impl<Context> Decode<Context> for CuGstBuffer {
    fn decode<D: Decoder<Context = Context>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let vec: Vec<u8> = Vec::decode(decoder)?;
        Ok(CuGstBuffer::Replay(vec.into()))
    }
}

impl<'de, Context> BorrowDecode<'de, Context> for CuGstBuffer {
    fn borrow_decode<D: BorrowDecoder<'de, Context = Context>>(
        decoder: &mut D,
    ) -> Result<Self, DecodeError> {
        CuGstBuffer::decode(decoder)
    }
}

impl Encode for CuGstBuffer {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.map_readable()
            .map_err(|e| EncodeError::OtherString(e.to_string()))?
            .as_slice()
            .encode(encoder)
    }
}

pub type CuDefaultGStreamer = CuGStreamer<8>;

/// Frame delivery policy for [`CuGStreamer`]'s internal ring buffer.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Reflect)]
pub enum FrameMode {
    /// Deliver buffered frames in FIFO order (default).
    #[default]
    Fifo,
    /// Deliver the most recent buffered frame and discard older queued frames.
    Newest,
}

impl FrameMode {
    pub fn from_config(config: &ComponentConfig) -> CuResult<Self> {
        match config.get::<String>("frame_mode")? {
            None => Ok(Self::default()),
            Some(mode) => match mode.as_str() {
                "fifo" => Ok(Self::Fifo),
                "newest" => Ok(Self::Newest),
                other => Err(CuError::from(format!(
                    "Invalid frame_mode '{other}': expected \"fifo\" or \"newest\"."
                ))),
            },
        }
    }
}

struct FrameRing<const N: usize> {
    buffer: FixedCircularBuffer<CuGstBuffer, N>,
    mode: FrameMode,
    dropped_frames: u64,
}

impl<const N: usize> FrameRing<N> {
    fn new(mode: FrameMode) -> Self {
        Self {
            buffer: FixedCircularBuffer::new(),
            mode,
            dropped_frames: 0,
        }
    }

    fn push(&mut self, frame: CuGstBuffer) {
        if self.buffer.push_back(frame).is_some() {
            self.dropped_frames = self.dropped_frames.saturating_add(1);
        }
    }

    fn pop(&mut self) -> Option<CuGstBuffer> {
        match self.mode {
            FrameMode::Fifo => self.buffer.pop_front(),
            FrameMode::Newest => {
                let newest = self.buffer.pop_back()?;
                let stale = self.buffer.len() as u64;
                if stale > 0 {
                    self.dropped_frames = self.dropped_frames.saturating_add(stale);
                    self.buffer.clear();
                }
                Some(newest)
            }
        }
    }

    fn dropped_frames(&self) -> u64 {
        self.dropped_frames
    }

    fn reset(&mut self) {
        self.buffer.clear();
        self.dropped_frames = 0;
    }

    fn clear_buffer(&mut self) {
        self.buffer.clear();
    }
}

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct CuGStreamer<const N: usize> {
    #[reflect(ignore)]
    pipeline: Pipeline,
    #[reflect(ignore)]
    frame_ring: Arc<Mutex<FrameRing<N>>>,
    #[reflect(ignore)]
    _appsink: AppSink,
    dropped_frames: u64,
}

impl<const N: usize> CuGStreamer<N> {
    /// Returns the number of frames dropped by the frame ring since the last `start()`.
    pub fn dropped_frames(&self) -> u64 {
        self.frame_ring.lock().unwrap().dropped_frames()
    }
}

impl<const N: usize> Freezable for CuGStreamer<N> {}

impl<const N: usize> CuSrcTask for CuGStreamer<N> {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(CuGstBuffer);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        if !gstreamer::INITIALIZED.load(std::sync::atomic::Ordering::SeqCst) {
            gstreamer::init()
                .map_err(|e| CuError::new_with_cause("Failed to initialize gstreamer.", e))?;
        } else {
            debug!("Gstreamer already initialized.");
        }

        let config = config.ok_or_else(|| CuError::from("No config provided."))?;
        let frame_mode = FrameMode::from_config(config)?;

        let pipeline = if let Some(pipeline_str) = config.get::<String>("pipeline")? {
            // Pipeline properties can contain credentials.
            debug!("Creating GStreamer pipeline.");
            let pipeline = parse::launch(pipeline_str.as_str())
                .map_err(|e| CuError::new_with_cause("Failed to parse pipeline.", e))?;
            Ok(pipeline)
        } else {
            Err(CuError::from("No pipeline provided."))
        }?;
        let caps_str = if let Some(caps_str) = config.get::<String>("caps")? {
            debug!("Creating with caps: {}", &caps_str);
            Ok(caps_str)
        } else {
            Err(CuError::from(
                "No Caps (ie format for example \"video/x-raw, format=NV12, width=1920, height=1080\") provided for the appsink element.",
            ))
        }?;

        let pipeline = pipeline
            .dynamic_cast::<Pipeline>()
            .map_err(|_| CuError::from("Failed to cast pipeline to gstreamer::Pipeline."))?;

        let appsink = pipeline.by_name("copper").ok_or::<CuError>("Failed to get find the \"appsink\" element in the pipeline string, be sure you have an appsink name=copper to feed this task.".into())?;
        let appsink = appsink
            .dynamic_cast::<AppSink>()
            .map_err(|_| CuError::from("Failed to cast appsink to gstreamer::AppSink."))?;
        let caps = Caps::from_str(caps_str.as_str())
            .map_err(|e| CuError::new_with_cause("Failed to create caps for appsink.", e))?;

        appsink.set_caps(Some(&caps));

        let frame_ring = Arc::new(Mutex::new(FrameRing::new(frame_mode)));

        // Configure `appsink` to handle incoming buffers
        appsink.set_callbacks(
            AppSinkCallbacks::builder()
                .new_sample({
                    let frame_ring = frame_ring.clone();
                    move |appsink| {
                        let sample = appsink
                            .pull_sample()
                            .map_err(|_| gstreamer::FlowError::Eos)?;
                        let buffer: &BufferRef =
                            sample.buffer().ok_or(gstreamer::FlowError::Error)?;
                        frame_ring
                            .lock()
                            .unwrap()
                            .push(CuGstBuffer::Live(buffer.to_owned()));
                        Ok(FlowSuccess::Ok)
                    }
                })
                .build(),
        );

        let s = CuGStreamer {
            pipeline,
            frame_ring,
            _appsink: appsink,
            dropped_frames: 0,
        };
        Ok(s)
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        debug!(ctx, "Gstreamer: Starting pipeline.");
        self.frame_ring.lock().unwrap().reset();
        self.dropped_frames = 0;
        self.pipeline
            .set_state(gstreamer::State::Playing)
            .map_err(|e| CuError::new_with_cause("Failed to start the gstreamer pipeline.", e))?;
        debug!(ctx, "Gstreamer: Starting pipeline OK.");
        Ok(())
    }

    fn process(&mut self, ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        let (buffer, dropped) = {
            let mut frame_ring = self.frame_ring.lock().unwrap();
            let buffer = frame_ring.pop();
            let dropped = frame_ring.dropped_frames();
            (buffer, dropped)
        };
        let prev_dropped = self.dropped_frames;
        self.dropped_frames = dropped;

        if let Some(buffer) = buffer {
            // TODO: do precise timing metadata from gstreamer
            new_msg.tov = ctx.now().into();
            new_msg.set_payload(buffer);
        } else {
            debug!(ctx, "Gstreamer: Empty circular buffer, sending no payload.");
            new_msg.clear_payload();
        }

        if dropped > 0 {
            new_msg
                .metadata
                .set_status(format_args!("dropped: {dropped}"));
            if dropped > prev_dropped {
                debug!(
                    ctx,
                    "Gstreamer: dropped {} frames (+{} since last tick).",
                    dropped,
                    dropped - prev_dropped
                );
            }
        }
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.pipeline
            .set_state(gstreamer::State::Null)
            .map_err(|e| CuError::new_with_cause("Failed to stop the gstreamer pipeline.", e))?;
        let mut frame_ring = self.frame_ring.lock().unwrap();
        frame_ring.clear_buffer();
        self.dropped_frames = frame_ring.dropped_frames();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(id: u8) -> CuGstBuffer {
        CuGstBuffer::Replay(vec![id].into())
    }

    fn frame_id(buffer: CuGstBuffer) -> u8 {
        buffer.map_readable().unwrap().as_slice()[0]
    }

    #[test]
    fn fifo_keeps_up_without_drops() {
        let mut ring = FrameRing::<4>::new(FrameMode::Fifo);
        ring.push(frame(1));
        ring.push(frame(2));
        ring.push(frame(3));

        assert_eq!(ring.dropped_frames(), 0);
        assert_eq!(frame_id(ring.pop().unwrap()), 1);
        assert_eq!(frame_id(ring.pop().unwrap()), 2);
        assert_eq!(frame_id(ring.pop().unwrap()), 3);
        assert!(ring.pop().is_none());
        assert_eq!(ring.dropped_frames(), 0);
    }

    #[test]
    fn fifo_counts_overwritten_frames() {
        let mut ring = FrameRing::<4>::new(FrameMode::Fifo);
        for id in 0..6u8 {
            ring.push(frame(id));
        }

        assert_eq!(ring.dropped_frames(), 2);
        assert_eq!(frame_id(ring.pop().unwrap()), 2);
        assert_eq!(frame_id(ring.pop().unwrap()), 3);
        assert_eq!(frame_id(ring.pop().unwrap()), 4);
        assert_eq!(frame_id(ring.pop().unwrap()), 5);
        assert!(ring.pop().is_none());
        assert_eq!(ring.dropped_frames(), 2);
    }

    #[test]
    fn newest_delivers_latest_and_counts_overwritten_and_stale_frames() {
        let mut ring = FrameRing::<4>::new(FrameMode::Newest);
        for id in 0..6u8 {
            ring.push(frame(id));
        }

        // 2 frames overwritten on push (0, 1); popping back delivers 5 and clears 3 stale (2, 3, 4).
        assert_eq!(ring.dropped_frames(), 2);
        assert_eq!(frame_id(ring.pop().unwrap()), 5);
        assert_eq!(ring.dropped_frames(), 5);
        assert!(ring.pop().is_none());
        assert_eq!(ring.dropped_frames(), 5);
    }

    #[test]
    fn newest_single_frame_has_no_drops() {
        let mut ring = FrameRing::<4>::new(FrameMode::Newest);
        ring.push(frame(42));

        assert_eq!(frame_id(ring.pop().unwrap()), 42);
        assert_eq!(ring.dropped_frames(), 0);
        assert!(ring.pop().is_none());
    }

    #[test]
    fn capacity_one_counts_every_overwrite_in_both_modes() {
        for mode in [FrameMode::Fifo, FrameMode::Newest] {
            let mut ring = FrameRing::<1>::new(mode);
            for id in 0..4u8 {
                ring.push(frame(id));
            }
            assert_eq!(ring.dropped_frames(), 3);
            assert_eq!(frame_id(ring.pop().unwrap()), 3);
            assert!(ring.pop().is_none());
            assert_eq!(ring.dropped_frames(), 3);
        }
    }

    #[test]
    fn empty_pop_does_not_modify_drop_count() {
        for mode in [FrameMode::Fifo, FrameMode::Newest] {
            let mut ring = FrameRing::<2>::new(mode);
            assert!(ring.pop().is_none());
            assert_eq!(ring.dropped_frames(), 0);

            ring.push(frame(1));
            ring.push(frame(2));
            ring.push(frame(3));
            let _ = ring.pop();
            let _ = ring.pop();
            let drops = ring.dropped_frames();
            assert!(ring.pop().is_none());
            assert_eq!(ring.dropped_frames(), drops);
        }
    }

    #[test]
    fn reset_and_clear_buffer_lifecycle_semantics() {
        let mut ring = FrameRing::<2>::new(FrameMode::Fifo);
        ring.push(frame(1));
        ring.push(frame(2));
        ring.push(frame(3));
        assert_eq!(ring.dropped_frames(), 1);

        // stop(): clear queued frames while preserving dropped_frames
        ring.clear_buffer();
        assert!(ring.pop().is_none());
        assert_eq!(ring.dropped_frames(), 1);

        // start(): clear queued frames and reset dropped_frames
        ring.push(frame(4));
        ring.reset();
        assert!(ring.pop().is_none());
        assert_eq!(ring.dropped_frames(), 0);
    }

    #[test]
    fn frame_mode_from_config_parsing() {
        let empty = ComponentConfig::new();
        assert_eq!(FrameMode::from_config(&empty).unwrap(), FrameMode::Fifo);

        let mut fifo_cfg = ComponentConfig::new();
        fifo_cfg.set("frame_mode", "fifo".to_string());
        assert_eq!(FrameMode::from_config(&fifo_cfg).unwrap(), FrameMode::Fifo);

        let mut newest_cfg = ComponentConfig::new();
        newest_cfg.set("frame_mode", "newest".to_string());
        assert_eq!(
            FrameMode::from_config(&newest_cfg).unwrap(),
            FrameMode::Newest
        );

        let mut invalid_cfg = ComponentConfig::new();
        invalid_cfg.set("frame_mode", "drop_all".to_string());
        let err = FrameMode::from_config(&invalid_cfg).unwrap_err();
        let msg = err.to_string();
        assert!(
            msg.contains("drop_all") && msg.contains("fifo") && msg.contains("newest"),
            "unexpected error message: {msg}"
        );
    }
}
