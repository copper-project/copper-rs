#[cfg(feature = "gst")]
mod tests {
    use cu29::prelude::*;
    use cu29_helpers::basic_copper_setup;
    use cu_gstreamer::CuGstBuffer;
    use rerun::{ChannelDatatype, ColorModel, Image, RecordingStream, RecordingStreamBuilder};
    use std::thread::sleep;
    use std::time::Duration;

    struct GStreamerTester {
        rec: RecordingStream,
    }

    impl Freezable for GStreamerTester {}

    impl<'cl> CuSinkTask<'cl> for GStreamerTester {
        type Input = input_msg!('cl, CuGstBuffer);

        fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let rec = RecordingStreamBuilder::new("Camera B&W Viz")
                .spawn()
                .unwrap();
            Ok(Self { rec })
        }

        fn process(&mut self, _clock: &RobotClock, msg: Self::Input) -> CuResult<()> {
            if msg.payload().is_none() {
                debug!("Skipped");
                return Ok(());
            }
            // Get the buffer's memory (zero-copy access)
            let data = msg.payload().unwrap().map_readable().unwrap();
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
            self.rec.log("camera/image", &image).unwrap();
            Ok(())
        }
    }

    #[copper_runtime(config = "tests/copperconfig.ron")]
    struct GStreamerTestApp {}

    #[test]
    #[ignore]
    fn end_2_end() {
        let tmp_dir = tempfile::TempDir::new().expect("could not create a tmp dir");
        let logger_path = tmp_dir.path().join("caterpillar.copper");
        let copper_ctx =
            basic_copper_setup(&logger_path, None, true, None).expect("Failed to setup logger.");
        debug!("Logger created at {}.", logger_path);
        debug!("Creating application... ");
        let mut application = GStreamerTestAppBuilder::new()
            .with_context(&copper_ctx)
            .build()
            .expect("Failed to create runtime.");

        debug!("Running...");
        application
            .start_all_tasks()
            .expect("Failed to start tasks.");
        for _ in 0..1000 {
            application
                .run_one_iteration()
                .expect("Failed to run application.");
            sleep(Duration::from_millis(100)); // avoid zapping through 1000 buffers before the first image arrived
        }
        application
            .stop_all_tasks()
            .expect("Failed to start tasks.");

        debug!("End of program.");
    }
}
