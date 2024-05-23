use std::fs::File;
use std::io::Write;

use copper::config::NodeInstanceConfig;
use copper::cutask::{CuMsg, CuSinkTask, CuTaskLifecycle};
use copper::CuResult;
use v4lsrc::ImageMsg;

pub struct SimpleLogger {
    path: String,
    log_file: Option<File>,
}

impl CuTaskLifecycle for SimpleLogger {
    fn new(config: Option<&NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let config = config
            .ok_or_else(|| "SimpleLogger needs a config, None was passed as NodeInstanceConfig")?;
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
        self.log_file = None;
        Ok(())
    }
}

impl CuSinkTask for SimpleLogger {
    type Input = ImageMsg;

    fn process(&mut self, input: &CuMsg<Self::Input>) -> CuResult<()> {
        let log_file = self.log_file.as_mut().unwrap();
        for line in input.payload.buffer.iter() {
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
