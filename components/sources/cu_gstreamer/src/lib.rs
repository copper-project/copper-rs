use cu29::prelude::*;
use gstreamer::prelude::*;
use std::error::Error;

use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use circular_buffer::CircularBuffer;
use gstreamer::{parse, Buffer, BufferRef, Caps, FlowSuccess, Pipeline};
use gstreamer_app::{AppSink, AppSinkCallbacks};
use std::fmt::Debug;
use std::str::FromStr;
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, Default)]
pub struct GstBufferWrapper(Buffer);
impl Decode for GstBufferWrapper {
    fn decode<D: Decoder>(decoder: &mut D) -> Result<Self, DecodeError> {
        let vec: Vec<u8> = Vec::decode(decoder)?;
        let buffer = Buffer::from_slice(vec);
        Ok(GstBufferWrapper(buffer))
    }
}

impl Encode for GstBufferWrapper {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        self.0
            .as_ref()
            .map_readable()
            .map_err(|_| EncodeError::Other {
                0: "Could not map readable",
            })?
            .encode(encoder)
    }
}

pub type CuDefaultGStreamer = CuGStreamer<8>;

pub struct CuGStreamer<const N: usize> {
    pipeline: Pipeline,
    circular_buffer: Arc<Mutex<CircularBuffer<N, GstBufferWrapper>>>,
}

impl<const N: usize> Freezable for CuGStreamer<N> {}

impl<'cl, const N: usize> CuSrcTask<'cl> for CuGStreamer<N> {
    type Output = output_msg!('cl, GstBufferWrapper);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        if !gstreamer::INITIALIZED.load(std::sync::atomic::Ordering::SeqCst) {
            gstreamer::init().unwrap();
        }

        let config = config.ok_or_else(|| CuError::from("No config provided."))?;

        let pipeline = if let Some(pipeline_str) = config.get::<String>("pipeline") {
            let pipeline = parse::launch(pipeline_str.as_str())
                .map_err(|e| CuError::new_with_cause("Failed to parse pipeline.", e))?;
            Ok(pipeline)
        } else {
            Err(CuError::from("No pipeline provided."))
        }?;
        let caps_str = if let Some(caps_str) = config.get::<String>("caps") {
            Ok(caps_str)
        } else {
            Err(CuError::from(
                "No Caps (ie format for example \"video/x-raw, format=NV12, width=1920, height=1080\") provided for the appsink element.",
            ))
        }?;

        let pipeline = pipeline.dynamic_cast::<Pipeline>().unwrap();

        let appsink = pipeline.by_name("copper").unwrap();
        let appsink = appsink.dynamic_cast::<AppSink>().unwrap();
        let caps = Caps::from_str(caps_str.as_str())
            .map_err(|e| CuError::new_with_cause("Failed to create caps for appsink.", e))?;

        appsink.set_caps(Some(&caps));

        let circular_buffer = Arc::new(Mutex::new(CircularBuffer::new()));

        // Configure `appsink` to handle incoming buffers
        appsink.set_callbacks(
            AppSinkCallbacks::builder()
                .new_sample({
                    let mut circular_buffer = circular_buffer.clone();
                    move |appsink| {
                        println!("Callback!");
                        let sample = appsink
                            .pull_sample()
                            .map_err(|_| gstreamer::FlowError::Eos)?;
                        let buffer: &BufferRef =
                            sample.buffer().ok_or(gstreamer::FlowError::Error)?;
                        circular_buffer
                            .lock()
                            .unwrap()
                            .push_back(GstBufferWrapper(buffer.to_owned()));

                        Ok(FlowSuccess::Ok)
                    }
                })
                .build(),
        );

        let s = CuGStreamer {
            pipeline,
            circular_buffer,
        };
        Ok(s)
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.circular_buffer.lock().unwrap().clear();
        self.pipeline
            .set_state(gstreamer::State::Playing)
            .map_err(|e| CuError::new_with_cause("Failed to start the gstreamer pipeline.", e))?;
        Ok(())
    }
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.pipeline
            .set_state(gstreamer::State::Null)
            .map_err(|e| CuError::new_with_cause("Failed to stop the gstreamer pipeline.", e))?;
        self.circular_buffer.lock().unwrap().clear();
        Ok(())
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let mut circular_buffer = self.circular_buffer.lock().unwrap();
        if let Some(buffer) = circular_buffer.pop_front() {
            // TODO: timing metadata
            new_msg.set_payload(buffer);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cu29::prelude::*;
    use gstreamer::{parse, Buffer, BufferRef, Caps, FlowSuccess, Pipeline};
    use gstreamer_app::{AppSink, AppSinkCallbacks};
    use rerun::{ChannelDatatype, ColorModel, Image, RecordingStreamBuilder};
    use std::thread::sleep;
    use std::time::Duration;

    #[test]
    fn test_end_to_end() {
        let mut task = CuDefaultGStreamer::new(None).unwrap();
        let clock = RobotClock::new();
        let mut msg = CuMsg::new(Some(GstBufferWrapper(Buffer::new())));
        task.process(&clock, &mut msg).unwrap();
    }

    #[test]
    fn old() {
        let rec = RecordingStreamBuilder::new("Camera B&W Viz")
            .spawn()
            .unwrap();

        gstreamer::init().unwrap();

        let pipeline = parse::launch(
            "v4l2src device=/dev/video2 ! video/x-raw, format=NV12, width=1920, height=1080 ! appsink name=sink",
        ).unwrap();
        println!("launched");
        let pipeline = pipeline.dynamic_cast::<Pipeline>().unwrap();

        let appsink = pipeline.by_name("sink").unwrap();
        let appsink = appsink.dynamic_cast::<AppSink>().unwrap();

        appsink.set_caps(Some(
            &Caps::builder("video/x-raw")
                .field("format", &"NV12")
                .build(),
        ));

        // Configure `appsink` to handle incoming buffers
        appsink.set_callbacks(
            AppSinkCallbacks::builder()
                .new_sample(move |appsink| {
                    println!("Callback!");
                    let sample = appsink
                        .pull_sample()
                        .map_err(|_| gstreamer::FlowError::Eos)?;
                    let buffer: &BufferRef = sample.buffer().ok_or(gstreamer::FlowError::Error)?;

                    // Get the buffer's memory (zero-copy access)
                    let data = buffer
                        .map_readable()
                        .map_err(|_| gstreamer::FlowError::Error)?;
                    println!("Received buffer: {} bytes", data.len());
                    let width = 1920;
                    let height = 1080;
                    let y_plane_size = width * height;
                    let grey_image = &data[0..y_plane_size];

                    // Rerun stuff
                    let image = Image::from_color_model_and_bytes(
                        grey_image.to_vec(),
                        [width as u32, height as u32],
                        ColorModel::L,
                        ChannelDatatype::U8,
                    );
                    {
                        rec.log("camera/image", &image).map_err(|err| {
                            eprintln!("Error logging image to rerun: {:?}", err);
                            gstreamer::FlowError::Error
                        })?;
                    }
                    // end rerun

                    Ok(FlowSuccess::Ok)
                })
                .build(),
        );

        // Start streaming
        pipeline.set_state(gstreamer::State::Playing).unwrap();

        println!("Streaming... Press Ctrl+C to stop.");
        loop {
            sleep(Duration::from_millis(100));
        }
    }
}
