<img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/doc/static/cu29.png?raw=true" alt="logo"/>

#

# Copper

[![copper](https://github.com/gbin/copper-project/actions/workflows/general.yml/badge.svg)](https://github.com/gbin/copper-project/actions/workflows/general.yml)
![GitHub last commit](https://img.shields.io/github/last-commit/copper-project/copper-rs)
![](https://img.shields.io/badge/Rust-1.80+-orange.svg)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Gitter](https://img.shields.io/gitter/room/copper-project/copper-rs)](https://app.gitter.im/#/room/#copper:gitter.im)

Copper is a user-friendly runtime engine for creating fast and reliable robots. Copper is to robots what a
game engine is to games.

* **Easy**: Copper offers a high-level configuration system and a natural Rust-first API.

* **Fast**: Copper uses Rust's zero-cost abstractions and a data-oriented approach to achieve sub-microsecond latency on
  commodity hardware, avoiding heap allocation during execution.

* **Reliable**: Copper leverages Rust's ownership, type system, and concurrency model to minimize bugs and ensure thread
  safety.

* **Product Oriented**: Copper aims to avoid late-stage infra integration issues by generating a very predictable
  runtime.

> [!NOTE]
> Copper is still in **early development / alpha stage**, and the APIs are subject to change. We are looking for
> contributors to help us build the best robotics framework possible. If you are interested, please join us
> on [Gitter](https://gitter.im/copper-project/copper-rs) or open an issue.

Copper has been tested on: Linux (x86_64, armv7, aarch64 & riskv64) and MacOS (arm64).
Testers would be welcomed on Windows and other platforms.

### Technical Overview

Copper is a data-oriented runtime with these key components:

* **Task Graph**:
  <img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/doc/graph.png?raw=true" alt="graph"/>
  Described in [RON](https://github.com/ron-rs/ron), this configures the system's topology, specifying which tasks
  communicate and setting types for nodes and messages.

* **Runtime Generator**: This component decides on an execution plan based on the graph's metadata. It preallocates a "
  Copper List" to maximize sequential memory access during execution.

* **Zero-Copy Data Logging**: Records all messages between tasks without copying data, ensuring efficient logging.

* **Fast Structured Logging**: Interns and indexes logging strings at compile time, avoiding runtime string construction
  and ensuring high-speed textual logging.

### You don't have a real robot yet? Try it in a virtual environment!

[![Copper in virtual action](https://img.youtube.com/vi/kC6sGRZUxLE/maxresdefault.jpg)](https://youtu.be/kC6sGRZUxLE)

Here is robot developed with Copper in action driving its digital twin in a simulation environment with [Bevy](https://crates.io/crates/bevy) (Game Engine
in Rust) and [Avian3d](https://crates.io/crates/avian3d) (Physics Engine in Rust)

You have a mac or a linux linux machine just run ...

```bash
$ cargo install cu-rp-balancebot
$ balancebot-sim 
```
... to try it locally.

The source code for this demo is available in the [examples/cu_rp_balancebot](examples/cu_rp_balancebot) directory.

### What features are already implemented?

1. **Basic task lifecycle interface**: Should be relatively stable for you to start contributing new algorithms,
   sensors, and actuators.
2. **Runtime generation**: Works but is very simple; this is just a BFS type of execution.
3. **Log reader & structured log reader**: Can export data, currently in Rust debug format.
4. **Components**: Those are also good examples if you want to write your own!

| **Category** | **Type**        |                                                                                                                                                                           | **Description**                                                       | **Crate Name** |
|--------------|-----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------|----------------|
| Sensors      | Lidar           | <img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/components/sources/cu_vlp16/doc/vlp16.jpg?raw=true" alt="vlp16"/>             | [Velodyne/Ouster VLP16](components/sources/cu_vlp16)                  | cu-vlp16       |
|              | IMU             | <img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/components/sources/cu_wt901/doc/wt901.jpg?raw=true" alt="wt901"/>             | [WitMotion WT901](components/sources/cu_wt901)                        | cu-wt901       |
|              | ADC/Position    | <img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/components/sources/cu_ads7883/doc/ads7883-scale.jpg?raw=true" alt="ads7883"/> | [ADS 7883 3MPSPS SPI ADC](components/sources/cu_ads7883)              | cu-ads7883     |
|              | Encoder         | <img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/components/sources/cu_rp_encoder/doc/encoder.jpg?raw=true" alt="ads7883"/>    | [Generic Directional Wheel encoder](components/sources/cu_rp_encoder) | cu-rp-encoder  |
| Actuators    | GPIO            | <img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/components/sinks/cu_rp_gpio/doc/rp.jpg?raw=true" alt="gpio"/>                 | [Raspberry Pi](components/sinks/cu_rp_gpio)                           | cu-rp-gpio     |
|              | Servo           | <img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/components/sinks/cu_lewansoul/doc/lewansoul.jpg?raw=true" alt="lewansoul"/>   | [Lewansoul Servo Bus (LX-16A, etc.)](components/sinks/cu_lewansoul)   | cu-lewansoul   |
|              | DC Motor Driver | <img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/components/sinks/cu_rp_sn754410/doc/sn754410.jpeg?raw=true" alt="sn754410"/>  | [Half-H Driver for CD Motors](components/sinks/cu_rp_sn754410)        | cu-rp-sn754410 |
| Monitors     | TUI Monitor     | <img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/components/monitors/cu_consolemon/doc/tasks.png?raw=true" alt="monitor"/>     | [Console based monitor](components/monitors/cu_consolemon)            | cu-consolemon  |
| Algorithms   | PID Controller  |                                                                                                                                                                           | [PID Controller](components/tasks/cu_pid)                             | cu-pid         |

### What features are missing? What do we plan to implement next?

Here are some of the features we plan to implement next (in ~order of priority), if you are interested in contributing
on any of those, please let us know!:

To Reach Beta:

- [x] **Merging**: Add a feature to merge messages from multiple sources as an input to a task.
- [x] **Monitoring**: We need a parallel system that can listen to monitoring messages and act accordingly.
- [x] **Simulation**: Standardized set of interfaces to control Copper based codebases in simulation.
- [ ] **Batching/Aligning**: add a feature to batch messages for high frequency sources to reduce the number of Copper
  Lists.
- [ ] **Deterministic log replay**: The ability to inject logs and replay them deterministically.

To Reach RC1:

- [ ] **Parallel Copper Lists**: Today Copper is monothreaded; this should enable concurrent Copper Lists to be executed
  at the same time with no contention.
- [ ] **ROS/DDS interfacing**: Build a pair of sink and source to connect to existing ROS systems, helping users migrate
  their infra bit by bit.
- [ ] **Extensible scheduling**: Enables a way to give hints to copper to schedule the workload
- [ ] **Modular Configuration**: As robots built with Copper gain complexity, users will need to build "variations" of
  their robots without duplicating their entire RON file.
- [ ] **Distributed Copper**: Currently, we can only create one process. We need proper RPC filtering copper lists per
  subsystem.
  So we are only at the beginning, but so much cool stuff is coming up!

### Kickstarting a copper project for the impatients

You can generate a project from a template present in the repo.
It will ask you the name you want to pick interactively.

```bash
cargo install cargo-generate
git clone https://github.com/copper-project/copper-rs
cd copper-rs/templates
cargo cunew [path_where_you_want_your_project_created]
    🤷   Project Name:
```

Check out [copper-templates](templates/README.md) for more info.

### How does a Copper application look like?

Here is a simple example of a task graph in RON:

```RON
(
    tasks: [
        (
            id: "src",                   // this is a friendly name
            type: "FlippingSource",      // This is a Rust struct name for this task see main below
        ),
        (
            id: "gpio",                  // another task, another name
            type: "cu_rp_gpio::RPGpio",  // This is the Rust struct name from another crate
            config: {                    // You can attach config elements to your task
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

```rust,ignore

// Your application will be a struct that will hold the runtime, loggers etc.
// This proc macro is where all the runtime generation happens. You can see the code generated by the macro at
// compile time.
#[copper_runtime(config = "copperconfig.ron")]  // this is the ron config we just created.
struct MyApplication {}

// Here we define our own Copper Task
// It will be a source flipping a boolean
pub struct FlippingSource {
    state: bool,
}

// You need to provide at least "new". But you have other hooks in to the Lifecycle you can leverage 
// to maximize your opportunity to not use resources outside of the critical execution path: for example start, stop, 
// pre_process, post_process etc...
impl CuTaskLifecycle for FlippingSource {
    fn new(_config: Option<&copper::config::ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { state: true })
    }
}

// We implement the CuSrcTask trait for our task as it is a source / driver (with no internal input from Copper itself).
impl<'cl> CuSrcTask<'cl> for FlippingSource {
    type Output = output_msg!('cl, RPGpioPayload);

    // Process is called by the runtime at each cycle. It will give:
    // 1. the reference to a monotonic clock
    // 2. a mutable reference to the output message (so no need to allocate of copy anything)
    // 3. a CuResult to handle errors
    fn process(&mut self, clock: &RobotClock, output: Self::Output) -> CuResult<()> {
        self.state = !self.state;   // Flip our internal state and send the message in our output.
        output.payload = RPGpioPayload {
            on: self.state,
            creation: Some(clock.now()).into(),
            actuation: Some(clock.now()).into(),
        };
        Ok(())
    }
}


fn main() {

    // Copper uses a special log format called "unified logger" that is optimized for writing. It stores the messages between tasks 
    // but also the structured logs and telemetry.
    // A log reader can be generated at the same time as the application to convert this format for post processing.
  
    let logger_path = "/tmp/mylogfile.copper";
    
    // This basic setup is a shortcut to get you running. If needed you can check out the content of it and customize it. 
    let copper_ctx =
        basic_copper_setup(&PathBuf::from(logger_path), true).expect("Failed to setup logger.");
        
    // This is the struct logging implementation tailored for Copper.
    // It will store the string away from the application in an index format at compile time.
    // and will store the parameter as an actual field.
    // You can even name those: debug!("This string will not be constructed at runtime at all: my_parameter: {} <- but this will be logged as 1 byte.", my_parameter = 42);  
    debug!("Logger created at {}.", logger_path); 
    
    // A high precision monotonic clock is provided. It can be mocked for testing. 
    // Cloning the clock is cheap and gives you the exact same clock.
    let clock = copper_ctx.clock;  
    
    debug!("Creating application... ");
    let mut application =
        MyApplication::new(clock.clone(), copper_ctx.unified_logger.clone())
            .expect("Failed to create runtime.");
    debug!("Running... starting clock: {}.", clock.now());  // The clock will be displayed with units etc. 
    application.run().expect("Failed to run application.");
    debug!("End of program: {}.", clock.now());
}

```

But this is a very minimal example for a task, please see [lifecycle](doc/lifecycle.md) for a more complete explanation
of a task lifecycle.

## Deployment of the application

Check out the [deployment](doc/deploy.md) page for more information.

## How is it better or different from ROS?

### Performance

In the example directory, we have 2 equivalent applications. One written in C++ for ROS and a port in Rust with Copper.

```bash
examples/cu_caterpillar
examples/ros_caterpillar
```

You can them out either just logging on a desktop or with GPIOs on a RPi and you should see a couple order of magnitude
difference in performance.

Copper has been design for performance first. Unlike a game engine we use a data oriented approach to minimize latency
and maximize throughput.

### Safety

As Copper is written in Rust, it is memory safe and thread safe by design. It is also designed to be easy to use and to
avoid common pitfalls.

As we progress on this project we plan on implementing more and more early warning to help you avoid "the death by a
thousand cuts" that can happen in a complex system.
