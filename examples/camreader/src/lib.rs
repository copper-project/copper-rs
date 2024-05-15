use std::thread;

use serde::{Deserialize, Serialize};

use copper::config::NodeConfig;
use copper::cutask::{CuMsg, CuMsgLifecycle, CuSrcTask};
use copper::serde::arrays;

#[derive(Serialize, Deserialize)]
struct ImageBuffer {
    #[serde(with = "arrays")]
    buffer: [[u8; 1920]; 1200],
}

impl Default for ImageBuffer {
    fn default() -> Self {
        ImageBuffer {
            buffer: [[0; 1920]; 1200],
        }
    }
}

// Concrete struct for CamReader.
struct CamReader<L: CuMsgLifecycle<Msg = ImageBuffer>> {
    buff_management: L,
}

// Implement CuSrcTask for CamReader with a specific type (ImageBuffer).
impl<L: CuMsgLifecycle<Msg = ImageBuffer>> CuSrcTask<ImageBuffer, L> for CamReader<L> {
    fn new(_config: NodeConfig, msgif: L) -> CamReader<L> {
        CamReader {
            buff_management: msgif,
        }
    }
}
impl<L: CuMsgLifecycle<Msg = ImageBuffer>> CamReader<L> {
    pub fn start(&self) {
        let handle = thread::spawn({
            let buff_management = self.buff_management.clone();
            move || {
                let mut i = 0;
                loop {
                    let image = buff_management.create();
                    image.buffer[0][0] = i;
                    i += 1;
                    buff_management.send(image);
                }
            }
        });
    }
}

// impl<'a> CuSrcTask<'a, ImageBuffer> for CamReader<'a> {
//     fn new(_config: NodeConfig, msgif: dyn CuMsgLifecycle<'a, ImageBuffer>) -> CamReader<'a> {
//         CamReader {
//             buff_management: Box::new(msgif),
//         }
//     }
// }

// impl<FB, FP> CamReader<'_, FB, FP> {
//     pub fn run(&self) {
//         let get_buffer = self.get_buffer.unwrap();
//         let push_buffer = self.push_buffer.unwrap();
//
//         // let get_buffer = Arc::new(Mutex::new(get_buffer));
//         // let push_buffer = Arc::new(Mutex::new(push_buffer));
//
//         // thread::spawn(move || {
//         //     let mut i = 0;
//         //     loop {
//         //         {
//         //             let get_buffer = get_buffer.lock().unwrap();
//         //             let push_buffer = push_buffer.lock().unwrap();
//         //             let buffer = get_buffer();
//         //             buffer.buffer[0][0] = i;
//         //             i += 1;
//         //             push_buffer(buffer);
//         //         }
//         //         thread::sleep(Duration::from_secs(1));
//         //     }
//         // });
//     }
// }
#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {}
}
