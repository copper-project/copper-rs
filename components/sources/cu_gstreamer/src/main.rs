use gstreamer::prelude::*;
use gstreamer::{parse, BufferRef, Caps, FlowSuccess, Pipeline};
use gstreamer_app::{AppSink, AppSinkCallbacks};
use rerun::{ChannelDatatype, ColorModel, Image, RecordingStreamBuilder};
use std::error::Error;
use std::thread::sleep;
use std::time::Duration;

fn main() -> Result<(), Box<dyn Error>> {
    let rec = RecordingStreamBuilder::new("Camera B&W Viz")
        .spawn()
        .unwrap();

    gstreamer::init()?;

    let pipeline = parse::launch(
        "v4l2src device=/dev/video2 ! video/x-raw, format=NV12, width=1920, height=1080 ! appsink name=sink",
    )?;
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
    pipeline.set_state(gstreamer::State::Playing)?;

    println!("Streaming... Press Ctrl+C to stop.");
    loop {
        sleep(Duration::from_millis(100));
    }
}
