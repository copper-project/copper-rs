#[cfg(target_os = "linux")]
mod v4lstream;

// This allows this module to be used on simulation on Windows and MacOS
#[cfg(not(target_os = "linux"))]
mod empty_impl {
    use cu29::prelude::*;
    use cu_sensor_payloads::CuImage;

    pub struct V4l {}

    impl Freezable for V4l {}

    impl<'cl> CuSrcTask<'cl> for V4l {
        type Output = output_msg!('cl, CuImage<Vec<u8>>);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self {})
        }

        fn process(&mut self, _clock: &RobotClock, _new_msg: Self::Output) -> CuResult<()> {
            Ok(())
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub use empty_impl::V4l;

#[cfg(target_os = "linux")]
pub use linux_impl::V4l;

#[cfg(target_os = "linux")]
mod linux_impl {
    use std::time::Duration;
    use v4l::video::Capture;

    use crate::v4lstream::CuV4LStream;
    use cu29::prelude::*;
    use cu_sensor_payloads::{CuImage, CuImageBufferFormat};

    use nix::time::{clock_gettime, ClockId};

    pub use v4l::buffer::Type;
    pub use v4l::framesize::FrameSizeEnum;
    pub use v4l::io::traits::{CaptureStream, Stream};
    pub use v4l::prelude::*;
    pub use v4l::video::capture::Parameters;
    pub use v4l::{Format, FourCC, Timestamp};

    // A Copper source task that reads frames from a V4L device.
    pub struct V4l {
        stream: CuV4LStream,
        settled_format: CuImageBufferFormat,
        v4l_clock_time_offset_ns: i64,
    }

    impl Freezable for V4l {}

    fn cutime_from_v4ltime(offset_ns: i64, v4l_time: Timestamp) -> CuTime {
        let duration: Duration = v4l_time.into();
        ((duration.as_nanos() as i64 + offset_ns) as u64).into()
    }

    impl<'cl> CuSrcTask<'cl> for V4l {
        type Output = output_msg!('cl, CuImage<Vec<u8>>);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            // reasonable defaults
            let mut v4l_device = 0usize;
            let mut req_width: Option<u32> = None;
            let mut req_height: Option<u32> = None;
            let mut req_fps: Option<u32> = None;
            let mut req_fourcc: Option<String> = None;
            let mut req_buffers: u32 = 4;
            let mut req_timeout: Duration = Duration::from_millis(500); // 500ms tolerance to get a frame

            if let Some(config) = _config {
                if let Some(device) = config.get::<u32>("device") {
                    v4l_device = device as usize;
                }
                if let Some(width) = config.get::<u32>("width") {
                    req_width = Some(width);
                }
                if let Some(height) = config.get::<u32>("height") {
                    req_height = Some(height);
                }
                if let Some(fps) = config.get::<u32>("fps") {
                    req_fps = Some(fps);
                }
                if let Some(fourcc) = config.get::<String>("fourcc") {
                    req_fourcc = Some(fourcc);
                }
                if let Some(buffers) = config.get::<u32>("buffers") {
                    req_buffers = buffers;
                }
                if let Some(timeout) = config.get::<u32>("timeout_ms") {
                    req_timeout = Duration::from_millis(timeout as u64);
                }
            }
            let dev = Device::new(v4l_device)
                .map_err(|e| CuError::new_with_cause("Failed to open camera", e))?;

            // List all formats supported by the device
            let formats = dev
                .enum_formats()
                .map_err(|e| CuError::new_with_cause("Failed to enum formats", e))?;

            if formats.is_empty() {
                return Err("The V4l device did not provide any video format.".into());
            }

            // Either use the 4CC or just pick one for the user
            let fourcc: FourCC = if let Some(fourcc) = req_fourcc {
                if fourcc.len() != 4 {
                    return Err("Invalid fourcc provided".into());
                }
                FourCC::new(fourcc.as_bytes()[0..4].try_into().unwrap())
            } else {
                debug!("No fourcc provided, just use the first one we can find.");
                formats.first().unwrap().fourcc
            };
            debug!("V4L: Using fourcc: {}", fourcc.to_string());
            let actual_fmt = if let Some(format) = formats.iter().find(|f| f.fourcc == fourcc) {
                // Enumerate resolutions for the BGR3 format
                let resolutions = dev
                    .enum_framesizes(format.fourcc)
                    .map_err(|e| CuError::new_with_cause("Failed to enum frame sizes", e))?;
                let (width, height) =
                    if let (Some(req_width), Some(req_height)) = (req_width, req_height) {
                        let mut frame_size: (u32, u32) = (0, 0);
                        for frame in resolutions.iter() {
                            let FrameSizeEnum::Discrete(size) = &frame.size else {
                                todo!()
                            };
                            if size.width == req_width && size.height == req_height {
                                frame_size = (size.width, size.height);
                                break;
                            }
                        }
                        frame_size
                    } else {
                        // just pick the first available
                        let fs = resolutions.first().unwrap();
                        let FrameSizeEnum::Discrete(size) = &fs.size else {
                            todo!()
                        };
                        (size.width, size.height)
                    };

                // Set the format with the chosen resolution
                let req_fmt = Format::new(width, height, fourcc);
                let actual_fmt = dev
                    .set_format(&req_fmt)
                    .map_err(|e| CuError::new_with_cause("Failed to set format", e))?;

                if let Some(fps) = req_fps {
                    debug!("V4L: Set fps to {}", fps);
                    let new_params = Parameters::with_fps(fps);
                    dev.set_params(&new_params)
                        .map_err(|e| CuError::new_with_cause("Failed to set params", e))?;
                }
                debug!(
                    "V4L: Negotiated resolution: {}x{}",
                    actual_fmt.width, actual_fmt.height
                );
                actual_fmt
            } else {
                return Err(format!(
                    "The V4l device {v4l_device} does not provide a format with the FourCC {fourcc}."
                )
                .into());
            };
            debug!(
                "V4L: Init stream: device {} with {} buffers of size {} bytes",
                v4l_device, req_buffers, actual_fmt.size
            );

            let mut stream = CuV4LStream::with_buffers(
                &dev,
                Type::VideoCapture,
                req_buffers,
                CuHostMemoryPool::new(
                    format!("V4L Host Pool {v4l_device}").as_str(),
                    req_buffers as usize + 1,
                    || vec![0; actual_fmt.size as usize],
                )
                .map_err(|e| {
                    CuError::new_with_cause(
                        "Could not create host memory pool backing the V4lStream",
                        e,
                    )
                })?,
            )
            .map_err(|e| CuError::new_with_cause("Could not create the V4lStream", e))?;
            let req_timeout_ms = req_timeout.as_millis() as u64;
            debug!("V4L: Set timeout to {} ms", req_timeout_ms);
            stream.set_timeout(req_timeout);

            let cuformat = CuImageBufferFormat {
                width: actual_fmt.width,
                height: actual_fmt.height,
                stride: actual_fmt.stride,
                pixel_format: actual_fmt.fourcc.repr,
            };

            Ok(Self {
                stream,
                settled_format: cuformat,
                v4l_clock_time_offset_ns: 0, // will be set at start
            })
        }

        fn start(&mut self, robot_clock: &RobotClock) -> CuResult<()> {
            let rb_ns = robot_clock.now().as_nanos();
            clock_gettime(ClockId::CLOCK_MONOTONIC)
                .map(|ts| {
                    self.v4l_clock_time_offset_ns =
                        ts.tv_sec() * 1_000_000_000 + ts.tv_nsec() - rb_ns as i64
                })
                .map_err(|e| CuError::new_with_cause("Failed to get the current time", e))?;

            self.stream
                .start()
                .map_err(|e| CuError::new_with_cause("could not start stream", e))
        }

        fn process(&mut self, _clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
            let (handle, meta) = self
                .stream
                .next()
                .map_err(|e| CuError::new_with_cause("could not get next frame from stream", e))?;
            if meta.bytesused != 0 {
                let cutime = cutime_from_v4ltime(self.v4l_clock_time_offset_ns, meta.timestamp);
                let image = CuImage::new(self.settled_format, handle.clone());
                new_msg.set_payload(image);
                new_msg.metadata.tov = Tov::Time(cutime);
            } else {
                debug!("Empty frame received");
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
        use rerun::RecordingStreamBuilder;
        use rerun::{Image, PixelFormat};
        use std::thread;

        use simplelog::{ColorChoice, Config, LevelFilter, TermLogger, TerminalMode};

        const IMG_WIDTH: usize = 3840;
        const IMG_HEIGHT: usize = 2160;

        #[derive(Debug)]
        struct NullLog {}
        impl WriteStream<CuLogEntry> for NullLog {
            fn log(&mut self, _obj: &CuLogEntry) -> CuResult<()> {
                Ok(())
            }
            fn flush(&mut self) -> CuResult<()> {
                Ok(())
            }
        }

        #[test]
        #[ignore]
        fn emulate_copper_backend() {
            let clock = RobotClock::new();

            let term_logger = TermLogger::new(
                LevelFilter::Debug,
                Config::default(),
                TerminalMode::Mixed,
                ColorChoice::Auto,
            );
            let _logger = LoggerRuntime::init(clock.clone(), NullLog {}, Some(*term_logger));

            let rec = RecordingStreamBuilder::new("Camera Viz")
                .spawn()
                .map_err(|e| CuError::new_with_cause("Failed to spawn rerun stream", e))
                .unwrap();

            let mut config = ComponentConfig::new();
            config.set("device", 0);
            config.set("width", IMG_WIDTH as u32);
            config.set("height", IMG_HEIGHT as u32);
            config.set("fps", 30);
            config.set("fourcc", "NV12".to_string());
            config.set("buffers", 4);
            config.set("timeout_ms", 500);

            let mut v4l = V4l::new(Some(&config)).unwrap();
            v4l.start(&clock).unwrap();

            let mut msg = CuMsg::new(None);
            // Define the image format
            let format = rerun::components::ImageFormat(ImageFormat {
                width: IMG_WIDTH as u32,
                height: IMG_HEIGHT as u32,
                pixel_format: Some(PixelFormat::NV12),
                color_model: None,      // Some(ColorModel::BGR),
                channel_datatype: None, // Some(ChannelDatatype::U8),
            });
            for _ in 0..1000 {
                let _output = v4l.process(&clock, &mut msg);
                if let Some(frame) = msg.payload() {
                    let slice: &[u8] = &frame.buffer_handle.lock().unwrap();
                    let blob = Blob::from(slice);
                    let rerun_img = ImageBuffer::from(blob);
                    let image = Image::new(rerun_img, format);

                    rec.log("images", &image).unwrap();
                } else {
                    debug!("----> No frame");
                    thread::sleep(Duration::from_millis(300)); // don't burn through empty buffers at the beginning, what for the device to actually start
                }
            }

            v4l.stop(&clock).unwrap();
        }
    }
}
