use cu29::prelude::*;
use gstreamer::prelude::*;

use bincode::de::{BorrowDecoder, Decoder};
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{BorrowDecode, Decode, Encode};
use circular_buffer::CircularBuffer;
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

#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct CuGStreamer<const N: usize> {
    #[reflect(ignore)]
    pipeline: Pipeline,
    #[reflect(ignore)]
    circular_buffer: Arc<Mutex<CircularBuffer<N, CuGstBuffer>>>,
    #[reflect(ignore)]
    _appsink: AppSink,
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

        let pipeline = if let Some(pipeline_str) = config.get::<String>("pipeline")? {
            debug!("Creating with pipeline: {}", &pipeline_str);
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

        let circular_buffer = Arc::new(Mutex::new(CircularBuffer::new()));

        // Configure `appsink` to handle incoming buffers
        appsink.set_callbacks(
            AppSinkCallbacks::builder()
                .new_sample({
                    let circular_buffer = circular_buffer.clone();
                    move |appsink| {
                        let sample = appsink
                            .pull_sample()
                            .map_err(|_| gstreamer::FlowError::Eos)?;
                        let buffer: &BufferRef =
                            sample.buffer().ok_or(gstreamer::FlowError::Error)?;
                        circular_buffer
                            .lock()
                            .unwrap()
                            .push_back(CuGstBuffer::Live(buffer.to_owned()));
                        Ok(FlowSuccess::Ok)
                    }
                })
                .build(),
        );

        let s = CuGStreamer {
            pipeline,
            circular_buffer,
            _appsink: appsink,
        };
        Ok(s)
    }

    fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
        debug!(ctx, "Gstreamer: Starting pipeline.");
        self.circular_buffer.lock().unwrap().clear();
        self.pipeline
            .set_state(gstreamer::State::Playing)
            .map_err(|e| CuError::new_with_cause("Failed to start the gstreamer pipeline.", e))?;
        debug!(ctx, "Gstreamer: Starting pipeline OK.");
        Ok(())
    }

    fn process(&mut self, ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
        let mut circular_buffer = self.circular_buffer.lock().unwrap();
        if let Some(buffer) = circular_buffer.pop_front() {
            // TODO: do precise timing metadata from gstreamer
            new_msg.tov = ctx.now().into();
            new_msg.set_payload(buffer);
        } else {
            debug!(ctx, "Gstreamer: Empty circular buffer, sending no payload.");
            new_msg.clear_payload();
        }
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        self.pipeline
            .set_state(gstreamer::State::Null)
            .map_err(|e| CuError::new_with_cause("Failed to stop the gstreamer pipeline.", e))?;
        self.circular_buffer.lock().unwrap().clear();
        Ok(())
    }
}

// No test here, see the integration tests.
