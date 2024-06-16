# Copper Project

A user friendly robotics framework to create fast and reliable robots.

Easy: Copper combines a high level configuration and a natural Rust first API.

Fast: Copper leverages the 0-cost abstraction features of Rust with a data oriented approach (ie. hardware friendly, no allocation on head during the execution etc...) to achieve sub microsecond latency on commodity hardware.

Reliable: Copper leverages Rust's ownership, type system, and concurrency model to reduce bugs and ensure thread safety.

[![copper](https://github.com/gbin/copper-project/actions/workflows/general.yml/badge.svg)](https://github.com/gbin/copper-project/actions/workflows/general.yml)
![GitHub last commit](https://img.shields.io/github/last-commit/gbin/copper-project)
![](https://img.shields.io/badge/Rust-1.79+-orange.svg)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

Copper is a data oriented runtime that is made of those key components:

* A task graph described in RON configuring the overall topology of the system (ie. which task talk to which) and sets types for the nodes and messages betweens the tasks.
* A runtime generator that will decide on an execution plan based on the metadata from the graph. Based on this execution plan, the internal datastructure for the communication will be preallocated called a "Copper List" that will maximize sequential memory accesses during execution
* A 0 copy data logging facility that will record all the messages between all the tasks
* A super fast structured logging for regular textual logs. All the logging strings are interned and indexes at compile time to avoid any string construction at runtime.

## Hello World for Copper

Enough! Show me how it is easy to build something!


```toml
[dependencies]
copper-derive = "*"       # the macro support that will generate your robot runtime at compile time
copper = "*"              # the supporting runtime
copper-log = "*"          # the super fast structured logging facility
copper-log-derive = "*"   # the macro to index the strcuture logging statements at compile time
copper-log-runtime = "*"  # the supporting runtime for the structured logging
copper-datalogger = "*"   # the datalogger (to log messages)
cu-rp-gpio = "*"          # a copper task we reuse from another crate
```

```RON
(
    tasks: [
        (
            id: "src",                   // this is a friendly name
            type: "FlippingSource",      // This is a Rust struct name for this task
        ),
        (
            id: "gpio",                // another task, another name
            type: "cu_rp_gpio::RPGpio",  // This is the Rust struct name
            config: {                    // You can attach config elements to yout task
                "pin": 4,
            },
        ),
    ],
     cnx: [
        // Here we simply connect the tasks telling to the framework what type of messages we want to use. 
        (src: "src",  dst: "gpio",   msg: "cu_rp_gpio::RPGpioMsg"),
    ],    
```

Then, on your main.rs:

```rust,no_run

#[copper_runtime(config = "copperconfig.ron")]  // this is the ron config we just created.
struct MyApplication {}

// Here we define our own Copper Task
// It will be a source flipping a boolean
pub struct FlippingSource {
    state: bool,
}

impl CuTaskLifecycle for FlippingSource {
    fn new(_config: Option<&copper::config::NodeInstanceConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { state: true })
    }
}

impl CuSrcTask for FlippingSource {
    type Output = RPGpioMsg;

    fn process(&mut self, clock: &RobotClock, output: &mut CuMsg<Self::Output>) -> CuResult<()> {
        self.state = !self.state;   // Flip our internal state and send the message in our output.
        output.payload = RPGpioMsg {
            on: self.state,
            creation: clock.now().into(),
        };
        Ok(())
    }
}


// And that's it we can create the runtime and run it!
fn main() {
    // This is where we want to log the data at runtime.
    let path: PathBuf = PathBuf::from("/tmp/caterpillar.copper");
    let DataLogger::Write(logger) = DataLoggerBuilder::new()
        .write(true)
        .create(true)
        .file_path(&path)
        .preallocated_size(100000)
        .build()
        .expect("Failed to create logger")
    else {
        panic!("Failed to create logger")
    };
    let data_logger = Arc::new(Mutex::new(logger));

    // This hooks up the structure logging to the main data logger
    let stream = stream_write(data_logger.clone(), DataLogType::StructuredLogLine, 1024);

    // We define our main robot clock
    let clock = RobotClock::default();
    let mut application = MyApplication::new(clock.clone()).expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", mytime = clock.now());  // this will be indexed as "mytime" in the main datalogger
    application.run().expect("Failed to run application.");
}


```
