use std::time::Duration;
use v4l::video::Capture;
mod cuv4l;

use crate::cuv4l::CuV4LStream;
use cu29::prelude::*;
use v4l::buffer::Type;
use v4l::io::traits::{CaptureStream, Stream};
use v4l::prelude::*;
use v4l::{Format, FourCC};

const IMG_WIDTH: usize = 2560;
const IMG_HEIGHT: usize = 1440;
const IMG_STRIDE: usize = 2560;
const IMG_BUFFER_SIZE: usize = IMG_STRIDE * IMG_HEIGHT * 3;

struct V4l {
    stream: CuV4LStream<IMG_BUFFER_SIZE>, // move that as a generic parameter
}

impl Freezable for V4l {}

impl<'cl> CuSrcTask<'cl> for V4l {
    type Output = output_msg!('cl, CuBufferHandle<IMG_BUFFER_SIZE>);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        // FIXME: make this configurable
        let mut dev =
            Device::new(0).map_err(|e| CuError::new_with_cause("Failed to open camera", e))?;

        // FIXME: make this configurable
        // List all formats supported by the device
        let formats = dev
            .enum_formats()
            .map_err(|e| CuError::new_with_cause("Failed to enum formats", e))?;

        // Find the first BGR3 format
        let bgr3_fourcc: FourCC = FourCC::new(b"BGR3");
        if let Some(format) = formats.iter().find(|f| f.fourcc == bgr3_fourcc) {
            println!("Found BGR3 format: {:?}", format);

            // Enumerate resolutions for the BGR3 format
            let resolutions = dev
                .enum_framesizes(format.fourcc)
                .map_err(|e| CuError::new_with_cause("Failed to enum frame sizes", e))?;
            println!("Resolutions: {:?}", resolutions);

            // Set the format with the chosen resolution
            let fmt = Format::new(IMG_WIDTH as u32, IMG_HEIGHT as u32, bgr3_fourcc);
            let actual_format = dev
                .set_format(&fmt)
                .map_err(|e| CuError::new_with_cause("Failed to set format", e))?;

            println!("Format successfully set: {:?}", actual_format);
        } else {
            println!("BGR3 format not found.");
        }

        let mut stream = CuV4LStream::with_buffers(&mut dev, Type::VideoCapture, 4)
            .map_err(|e| CuError::new_with_cause("could get formats", e))?;
        stream.set_timeout(Duration::from_secs(1)); // FIXME: make this configurable

        Ok(Self { stream })
    }

    fn start(&mut self, _robot_clock: &RobotClock) -> CuResult<()> {
        self.stream
            .start()
            .map_err(|e| CuError::new_with_cause("could not start stream", e))
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let tuple = self.stream.next();
        if tuple.is_err() {
            return Ok(());
        }
        if let Ok((handle, meta)) = tuple {
            if meta.bytesused != 0 {
                // timedout
                new_msg.set_payload(handle.clone());
            }
            return Ok(());
        }
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.stream
            .stop()
            .map_err(|e| CuError::new_with_cause("could not stop stream", e))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rerun::components::ImageBuffer;
    use rerun::datatypes::{Blob, ImageFormat};
    use rerun::external::re_types::ArrowBuffer;
    use rerun::{ChannelDatatype, Image};
    use rerun::{ColorModel, RecordingStreamBuilder};

    #[test]
    fn emulate_copper_backend() {
        let rec = RecordingStreamBuilder::new("Camera Viz")
            .spawn()
            .map_err(|e| CuError::new_with_cause("Failed to spawn rerun stream", e))
            .unwrap();

        let mut v4l = V4l::new(None).unwrap();
        let clock = RobotClock::new();
        v4l.start(&clock).unwrap();
        let mut msg = CuMsg::new(None);
        let image_size_in_bytes = IMG_STRIDE * IMG_HEIGHT * 3;
        let cm = ColorModel::BGR;
        // Define the image format
        let format = rerun::components::ImageFormat(ImageFormat {
            width: IMG_WIDTH as u32,
            height: IMG_HEIGHT as u32,
            pixel_format: None,
            color_model: Some(ColorModel::BGR),
            channel_datatype: Some(ChannelDatatype::U8),
        });
        for _ in 0..1000 {
            let _output = v4l.process(&clock, &mut msg);
            if let Some(frame) = msg.payload() {
                println!("Frame: {:?}", frame);
                let slice = frame.as_slice();
                let mut flipped = Vec::with_capacity(slice.len());
                for y in (0..IMG_HEIGHT).rev() {
                    let start = y * IMG_STRIDE * 3;
                    let end = start + IMG_STRIDE * 3;
                    flipped.extend_from_slice(&slice[start..end]);
                }
                let arrow_buffer = ArrowBuffer::from(flipped);
                let blob = Blob::from(arrow_buffer);
                let rerun_img = ImageBuffer::from(blob);
                let image = Image::new(rerun_img, format.clone());

                rec.log("images", &image).unwrap();
            } else {
                println!("----> No frame");
            }
        }

        v4l.stop(&clock).unwrap();
    }
}
