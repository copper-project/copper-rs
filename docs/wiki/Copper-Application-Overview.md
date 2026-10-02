# Copper Application Overview

This page shows a minimal Copper application: a task graph definition in RON and the Rust code that implements a simple task and runtime.

## Task graph (RON)

```ron
(
    tasks: [
        (
            id: "src",
            type: "FlippingSource",
        ),
        (
            id: "gpio",
            type: "cu_rp_gpio::RPGpio",
            config: {
                "pin": 4,
            },
        ),
    ],
    cnx: [
        (src: "src", dst: "gpio", msg: "cu_rp_gpio::RPGpioMsg"),
    ],
)
```

## Runtime and task implementation (Rust)

```rust
use std::path::PathBuf;

use cu29::prelude::*;

#[copper_runtime(config = "copperconfig.ron")]
struct MyApplication {}

pub struct FlippingSource {
    state: bool,
}

impl CuSrcTask for FlippingSource {
    type Output<'m> = output_msg!(RPGpioPayload);

    fn new(config: Option<&copper::config::ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { state: true })
    }

    fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.state = !self.state;
        output.set_payload(RPGpioPayload {
            on: self.state,
            creation: Some(clock.now()).into(),
            actuation: Some(clock.now()).into(),
        });
        Ok(())
    }
}

fn main() {
    let logger_path = "/tmp/mylogfile.copper";

    debug!("Creating application... ");
    let mut application = MyApplication::builder()
        .with_log_path(logger_path, Some(10 * 1024 * 1024))
        .expect("Failed to setup logger.")
        .build()
        .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", application.clock().now());
    application.run().expect("Failed to run application.");
    debug!("End of program: {}.", application.clock().now());
}
```

For a deeper explanation of task lifecycle hooks and scheduling, see [Task Lifecycle](Task-Lifecycle).
