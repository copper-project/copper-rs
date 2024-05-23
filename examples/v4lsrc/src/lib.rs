use linux_video::types::*;
use linux_video::{Device, Stream};
use serde::{Deserialize, Serialize};

use copper::config::NodeInstanceConfig;
use copper::cutask::{CuMsg, CuSrcTask, CuTaskLifecycle};
use copper::serde::arrays;
use copper::CuResult;

#[derive(Serialize, Deserialize)]
pub struct ImageMsg {
    #[serde(with = "arrays")]
    pub buffer: [[u8; 1920]; 1200],
}

impl Default for ImageMsg {
    fn default() -> Self {
        ImageMsg {
            buffer: [[0; 1920]; 1200],
        }
    }
}

impl PartialEq for ImageMsg {
    fn eq(&self, other: &Self) -> bool {
        self.buffer
            .iter()
            .flatten()
            .eq(other.buffer.iter().flatten())
    }
}

impl ImageMsg {
    fn copy_from(&mut self, buff_src: &[u8]) {
        let mut x = 0usize;
        let mut y = 0usize;

        for el in buff_src {
            self.buffer[y][x] = *el;
            x += 1;
            if x == 1920 {
                x = 0;
                y += 1;
                if y == 1200 {
                    break;
                }
            }
        }
    }
}

pub struct Video4LinuxSource {
    device: Device,
    stream: Option<Stream<In, Mmap>>,
}
impl CuTaskLifecycle for Video4LinuxSource {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config
            .ok_or("Video4LinuxSource needs a config, None was passed as NodeInstanceConfig")?;

        let dev: String = (*config
            .get("dev")
            .expect("v4lsrc expects a dev config value pointing to the video device"))
        .clone()
        .into();
        let device = Device::open(dev).unwrap();
        let mut fmt = device
            .format(BufferType::VideoCapture)
            .expect("Failed to get formats");
        println!("Found supported format {}", fmt);
        device.set_format(&mut fmt).expect("Failed to set format");
        Ok(Video4LinuxSource {
            device,
            stream: None,
        })
    }

    fn start(&mut self) -> CuResult<()> {
        self.stream = Some(
            self.device
                .stream::<In, Mmap>(ContentType::Video, 4)
                .expect("Failed to start streaming from the v4l device."),
        );
        Ok(())
    }

    fn stop(&mut self) -> CuResult<()> {
        self.stream = None; // This will trigger the Drop implementation on Stream
        Ok(())
    }
}

impl CuSrcTask for Video4LinuxSource {
    type Payload = ImageMsg;

    fn process(&mut self, empty_msg: &mut CuMsg<Self::Payload>) -> CuResult<()> {
        let stream = self.stream.as_ref().unwrap();
        if let Ok(buffer) = stream.next() {
            let buffer = buffer.lock();
            empty_msg.payload.copy_from(buffer.as_ref());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::mem;

    use super::*;

    // ignore this test because it requires a camera to be connected to the computer
    // it is still runnable if specifically called with `cargo test emulate_runtime`
    #[ignore]
    #[test]
    fn emulate_runtime() -> CuResult<()> {
        println!("Build config");
        let mut config = NodeInstanceConfig::default();
        config.insert("dev".to_string(), "/dev/video0".to_string().into());
        println!("Build task");
        let mut task = Video4LinuxSource::new(Some(&config))?;
        println!("Build img");
        // emulates the inplace behavior of copper's runtime.
        let size_of_image_msg = mem::size_of::<CuMsg<ImageMsg>>();
        let mut memory: Vec<u8> = vec![0; size_of_image_msg];
        let ptr = memory.as_mut_ptr() as *mut CuMsg<ImageMsg>;
        let msg = unsafe { &mut *ptr };

        println!("Start");
        task.start()?;
        println!("Process");
        task.process(msg)?;
        println!("First byte: {}", msg.payload.buffer[0][0]);
        println!("Stop");
        task.stop()?;
        Ok(())
    }
}
