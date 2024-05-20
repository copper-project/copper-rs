use linux_video::{Device, Stream};
use linux_video::types::*;
use serde::{Deserialize, Serialize};

use copper::config::NodeConfig;
use copper::cutask::{CuResult, CuSrcTask};
use copper::serde::arrays;

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

impl CuSrcTask for Video4LinuxSource {
    type Msg = ImageMsg;

    fn new(config: NodeConfig) -> CuResult<Self>
    where
        Self: Sized,
    {
        let dev: String = (*config.get("dev").unwrap()).clone().into();
        let device = Device::open(dev).unwrap();
        Ok(Video4LinuxSource {
            device,
            stream: None,
        })
    }

    fn start(&mut self) -> CuResult<()> {
        self.stream = Some(
            self.device
                .stream::<In, Mmap>(ContentType::Video, 4)
                .unwrap(),
        );
        Ok(())
    }

    fn process(&mut self, empty_msg: &mut Self::Msg) -> CuResult<()> {
        let stream = self.stream.as_ref().unwrap();
        if let Ok(buffer) = stream.next() {
            let buffer = buffer.lock();
            empty_msg.copy_from(buffer.as_ref());
        }
        Ok(())
    }
    fn stop(&mut self) -> CuResult<()> {
        self.stream = None; // This will trigger the Drop implementation on Stream
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
        let config = NodeConfig::default();
        println!("Build task");
        let mut task = Video4LinuxSource::new(config)?;
        println!("Build img");
        // emulates the inplace behavior of copper's runtime.
        let size_of_image_msg = mem::size_of::<ImageMsg>();
        let mut memory: Vec<u8> = vec![0; size_of_image_msg];
        let ptr = memory.as_mut_ptr() as *mut ImageMsg;
        let img = unsafe { &mut *ptr };

        println!("Start");
        task.start()?;
        println!("Process");
        task.process(img)?;
        println!("First byte: {}", img.buffer[0][0]);
        println!("Stop");
        task.stop()?;
        Ok(())
    }
}
