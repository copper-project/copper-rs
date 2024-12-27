use std::io::Write;
use v4l::video::Capture;
mod cuv4l;

use crate::cuv4l::CuV4LStream;
use cu29::prelude::*;
use v4l::buffer::Type;
use v4l::io::traits::{CaptureStream, Stream};
use v4l::prelude::*;
use v4l::{Format, FourCC};

const IMG_WIDTH: usize = 3840;
const IMG_HEIGHT: usize = 720;
const IMG_SIZE: usize = IMG_WIDTH * IMG_HEIGHT * 3;

struct V4l {
    dev: Device,
    stream: CuV4LStream<IMG_SIZE>, // move that as a generic parameter
}

impl Freezable for V4l {}

impl<'cl> CuSrcTask<'cl> for V4l {
    type Output = output_msg!('cl, CuBufferHandle<IMG_SIZE>);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let mut dev =
            Device::new(0).map_err(|e| CuError::new_with_cause("Failed to open camera", e))?;

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

            if let Some(frame_size) = resolutions.first() {
                match frame_size.size {
                    v4l::framesize::FrameSizeEnum::Discrete(v4l::framesize::Discrete {
                        width,
                        height,
                    }) => {
                        println!("Using resolution: {}x{}", width, height);

                        // Set the format with the chosen resolution
                        let fmt = Format::new(width, height, bgr3_fourcc);
                        let actual_format = dev
                            .set_format(&fmt)
                            .map_err(|e| CuError::new_with_cause("Failed to set format", e))?;

                        println!("Format successfully set: {:?}", actual_format);
                    }
                    _ => {
                        println!("Non-discrete frame sizes are not supported in this example.");
                    }
                }
            } else {
                println!("No resolutions found for BGR3 format.");
            }
        } else {
            println!("BGR3 format not found.");
        }

        let stream = CuV4LStream::with_buffers(&mut dev, Type::VideoCapture, 4)
            .map_err(|e| CuError::new_with_cause("could get formats", e))?;

        Ok(Self { dev, stream })
    }

    fn start(&mut self, _robot_clock: &RobotClock) -> CuResult<()> {
        self.stream
            .start()
            .map_err(|e| CuError::new_with_cause("could not start stream", e))
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        let tuple = self.stream.next();
        if tuple.is_err() {
            return Ok(());
        }
        if let Ok((handle, meta)) = tuple {
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
    #[test]
    fn emulate_copper_backend() {
        let mut v4l = V4l::new(None).unwrap();
        let clock = RobotClock::new();
        v4l.start(&clock).unwrap();
        let mut msg = CuMsg::new(None);
        let output = v4l.process(&clock, &mut msg);

        v4l.stop(&clock).unwrap();
    }
}
