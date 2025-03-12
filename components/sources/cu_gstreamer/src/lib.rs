use cu29::prelude::*;
use gstreamer::prelude::*;

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use circular_buffer::CircularBuffer;
use gstreamer::{parse, Buffer, BufferRef, Caps, FlowSuccess, Pipeline};
use gstreamer_app::{AppSink, AppSinkCallbacks};
use std::fmt::Debug;
use std::ops::{Deref, DerefMut};
use std::str::FromStr;
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Default)]
pub struct CuGstBuffer(pub Buffer);

impl Deref for CuGstBuffer {
    type Target = Buffer;

    fn deref(&self) -> &Self::Target {
        let Self(r) = self;
        r
    }
}

impl DerefMut for CuGstBuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        let Self(r) = self;
        r
    }
}

impl Decode<()> for CuGstBuffer {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let vec: Vec<u8> = Vec::decode(decoder)?;
        let buffer = Buffer::from_slice(vec);
        Ok(CuGstBuffer(buffer))
    }
}

impl Encode for CuGstBuffer {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let Self(r) = self;
        r.as_ref()
            .map_readable()
            .map_err(|_| EncodeError::Other("Could not map readable"))?
            .encode(encoder)
    }
}

pub type CuDefaultGStreamer = CuGStreamer<8>;

pub struct CuGStreamer<const N: usize> {
    pipeline: Pipeline,
    circular_buffer: Arc<Mutex<CircularBuffer<N, CuGstBuffer>>>,
    _appsink: AppSink,
}

impl<const N: usize> Freezable for CuGStreamer<N> {}

impl<'cl, const N: usize> CuSrcTask<'cl> for CuGStreamer<N> {
    type Output = output_msg!('cl, CuGstBuffer);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
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

        let pipeline = if let Some(pipeline_str) = config.get::<String>("pipeline") {
            debug!("Creating with pipeline: {}", &pipeline_str);
            let pipeline = parse::launch(pipeline_str.as_str())
                .map_err(|e| CuError::new_with_cause("Failed to parse pipeline.", e))?;
            Ok(pipeline)
        } else {
            Err(CuError::from("No pipeline provided."))
        }?;
        let caps_str = if let Some(caps_str) = config.get::<String>("caps") {
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
                            .push_back(CuGstBuffer(buffer.to_owned()));
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

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        debug!("Gstreamer: Starting pipeline.");
        self.circular_buffer.lock().unwrap().clear();
        self.pipeline
            .set_state(gstreamer::State::Playing)
            .map_err(|e| CuError::new_with_cause("Failed to start the gstreamer pipeline.", e))?;
        debug!("Gstreamer: Starting pipeline OK.");
        Ok(())
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let mut circular_buffer = self.circular_buffer.lock().unwrap();
        if let Some(buffer) = circular_buffer.pop_front() {
            // TODO: do precise timing metadata from gstreamer
            new_msg.metadata.tov = clock.now().into();
            new_msg.set_payload(buffer);
        } else {
            debug!("Gstreamer: Empty circular buffer, sending no payload.");
            new_msg.clear_payload();
        }
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.pipeline
            .set_state(gstreamer::State::Null)
            .map_err(|e| CuError::new_with_cause("Failed to stop the gstreamer pipeline.", e))?;
        self.circular_buffer.lock().unwrap().clear();
        Ok(())
    }
}

// No test here, see the integration tests.
