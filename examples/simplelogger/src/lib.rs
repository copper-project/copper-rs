use std::fs::File;
use std::io::Write;

use serde::{Deserialize, Serialize};

use copper::config::NodeConfig;
use copper::cutask::{CuResult, CuSinkTask};
use v4lsrc::ImageMsg;

struct SimpleLogger {
    path: String,
    log_file: Option<File>,
}

impl CuSinkTask for SimpleLogger {
    type Input = ImageMsg;

    fn new(config: NodeConfig) -> CuResult<Self>
    where
        Self: Sized,
    {
        let path = (*config.get("path").unwrap()).clone().into();
        Ok(SimpleLogger {
            path,
            log_file: None,
        })
    }

    fn start(&mut self) -> CuResult<()> {
        let log_file = File::create(&self.path).unwrap();
        self.log_file = Some(log_file);
        Ok(())
    }

    fn stop(&mut self) -> CuResult<()> {
        Ok(())
    }

    fn process(&mut self, input: &Self::Input) -> CuResult<()> {
        let log_file = self.log_file.as_mut().unwrap();
        for line in input.buffer.iter() {
            log_file.write_all(line).unwrap();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ignore this test because it requires a camera to be connected to the computer
    // it is still runnable if specifically called with `cargo test emulate_runtime`
    #[ignore]
    #[test]
    fn emulate_runtime() -> CuResult<()> {
        Ok(())
    }
}
