<h1 style="color: #b87333;"><img src="https://github.com/copper-project/copper-rs/blob/master/doc/static/cu29.png?raw=true" width="60" /> &nbsp;&nbsp;&nbsp;&nbsp; Copper Runtime & SDK</h1>

[![copper](https://github.com/gbin/copper-project/actions/workflows/general.yml/badge.svg)](https://github.com/gbin/copper-project/actions/workflows/general.yml)
![GitHub last commit](https://img.shields.io/github/last-commit/copper-project/copper-rs)
![](https://img.shields.io/badge/Rust-1.80+-orange.svg)
[![dependency status](https://deps.rs/repo/github/copper-project/copper-rs/status.svg)](https://deps.rs/repo/github/copper-project/copper-rs)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Discord](https://img.shields.io/discord/1305916875741597826?logo=discord)](https://discord.gg/VkCG7Sb9Kw)

<blockquote>
🤖&nbsp&nbsp&nbsp&nbsp
  <em style="font-size: 1.2em;">
    Copper is to robots what a game engine is to games - build, run, and replay your entire robot deterministically.
  </em>
</blockquote>

### Why Copper

<p><strong style="color: #b87333;">🦀 Rust-first</strong> – ergonomic & safe  
<p><strong style="color: #b87333;">⚡ Sub-microsecond latency</strong> – zero-alloc, data-oriented runtime  
<p><strong style="color: #b87333;">⏱️ Deterministic replay</strong> – every run, bit-for-bit identical  
<p><strong style="color: #b87333;">🧠 Interoperable with ROS2</strong> – bridges via Zenoh opening the path for a progressive migration.  
<p><strong style="color: #b87333;">🪶 Runs anywhere</strong> – from Linux servers to bare-metal RP2350  
<p><strong style="color: #b87333;">📦 Built to ship</strong> – one stack from simulation to production

### Example Applications

| Flying | Driving | Swimming | Spacefaring |
|:--:|:--:|:--:|:--:|
| <img width="200" alt="copper-drone" src="https://github.com/user-attachments/assets/31096307-fe1b-4315-b876-0f7237d69fa4" /> | <img width="200" alt="copper-driving" src="https://github.com/user-attachments/assets/1f051359-1ff4-45c6-838b-44442dc06ac4" /> | <img width="200" alt="copper-swimming" src="https://github.com/user-attachments/assets/5a0d0279-da98-4d4d-b5e1-e0cd8890a368" /> | <img width="200" alt="copper-space" src="https://github.com/user-attachments/assets/c535413e-014f-4846-ab06-a49e1151e42e" /> |

### Technical Overview

Copper is a deterministic and data-oriented Robot SDK with these key components:

* **Task Graph**:
  <img align="right" width="100" src="https://github.com/copper-project/copper-rs/blob/master/doc/graph.png?raw=true" alt="graph"/>
  Configures the system's topology, specifies inter-task communication paths, and sets types for nodes and messages. The
  Task Graph is generated in [RON](https://github.com/ron-rs/ron),

* **Runtime Generator**: Creates an execution plan based on the Task Graph's metadata. It preallocates a
  "Copper List" to maximize sequential memory access during execution.

* **Zero-Copy Data Logging**: Records all messages between tasks without copying data, ensuring efficient logging and
  perfect determinism.

* **Fast Structured Logging**: Interns and indexes logging strings at compile time, avoiding runtime string construction
  and ensuring high-speed textual logging.

### You don't have a real robot yet? Try it in our minimalistic sim environment!

[![Copper in virtual action](https://img.youtube.com/vi/kC6sGRZUxLE/maxresdefault.jpg)](https://youtu.be/kC6sGRZUxLE)

Here is a Copper-based robot in action in a Bevy simulation environment!  
The realistic sim is created using [Bevy](https://crates.io/crates/bevy) (A Rust Game Engine)
and [Avian3d](https://crates.io/crates/avian3d) (Physics Engine in Rust).

On your mac or linux machine (x86-64 or Arm) just run ...

```bash
$ cargo install cu-rp-balancebot
$ balancebot-sim 
```

... to try it locally.

The source code for this demo is available in the [examples/cu_rp_balancebot](examples/cu_rp_balancebot) directory.

### Supported Platforms

You can deploy and run Copper on those platforms:

<img src="https://upload.wikimedia.org/wikipedia/commons/3/35/Tux.svg" width="50"/>&nbsp;&nbsp;<img src="https://upload.wikimedia.org/wikipedia/commons/3/31/Apple_logo_white.svg" width="50"/>&nbsp;&nbsp;<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/6/6a/Windows_logo_-_2021_%28White%29.svg/768px-Windows_logo_-_2021_%28White%29.svg.png?20230326181935" width="50"/>&nbsp;&nbsp;<img src="https://upload.wikimedia.org/wikipedia/commons/d/d7/Android_robot.svg" width="50"/>

### ... but we also support baremetal!

On the embedded side, we do have a reference platform you can create from readily available components or you can get from us already made, feel free to inquire: info@copper-robotics.com

<img src="https://github.com/copper-project/copper-rs/wiki/imgs/baremetal-w-camera.jpg" width="200"/>

More info about baremetal development with Copper [here](https://github.com/copper-project/copper-rs/wiki/Baremetal-Development)


### Features provided with the SDK

1. **Task interface and Lifecycle**: Traits you can use to implement new algorithms, sensors, and actuators.
2. **Runtime generation**: Generates a scheduling at compile time for your robot.
3. **Log reader**: You can export the logs generated by Copper to any format supported by Rust Serde.
4. **Structured log reader**: Debug logs are indexed and string interned at compile time for maximum efficiency.
5. **Components**: We have a growing number of drivers, algorithms and standard interfaces. If you have implemented a
   new component, ping us and we will add it to the list below!
6. **log replay / resim**: You can deterministically replay/resim a log. You will get the exact same result
   as a real log on the robot or from the sim.
7. **Simulation**: We have a simple simulation environment to test your robot. Test your robot before the hardware is
   built and try out your robot the first time without risking a real crash.

With a growing list of readily available [Components](https://github.com/copper-project/copper-rs/wiki/Available-Components).


## Get Started

### Kickstarting a Copper project for the impatients

You can generate a project from one of Copper's templates.
The generator will ask you the name for your new project interactively:

```bash
cargo install cargo-generate
git clone https://github.com/copper-project/copper-rs
cd copper-rs/templates
cargo cunew [path_where_you_want_your_project_created]
    🤷   Project Name:
```

Check out [copper-templates](templates/README.md) for more info.

### Task automation with `just`

We use [`just`](https://github.com/casey/just) for repeatable tasks. `just` loads the nearest `justfile` in the directory tree, so `cd` into the project you want before running commands.

- Root helpers (CI/lint): `just -l` in the repo root.
- Example-specific helpers: `cd examples/cu_caterpillar && just -l`, `cd examples/ros_caterpillar && just -l`, `cd examples/ros_zenoh_caterpillar && just -l`, `cd examples/cu_rp_balancebot && just -l`, `cd examples/cu_elrs_bdshot_demo && just -l`, `cd examples/cu_standalone_structlog && just -l`.
- Component helpers: `cd components/sources/cu_ads7883 && just -l`, `cd components/sinks/cu_rp_sn754410 && just -l`, `cd components/payloads/cu_ros_payloads && just -l`.
- Support utilities: `cd support && just -l` (general deploy/sdcard), `cd support/docker && just -l` (build/run dev containers).

If you do not have `just` installed, follow the instructions in the [`just` README](https://github.com/casey/just#installation) for your platform.

### Overview of a Copper Application

Here is a simple example of a Task Graph in RON:

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

// Import the prelude to get all the macros and traits you need.
use cu29::prelude::*;

// Your application will be a struct that will hold the runtime, loggers etc.
// This proc macro is where all the runtime generation happens. If you are curious about what code is generated by this macro
// you can activate the feature macro_debug and it will display it at compile time.
#[copper_runtime(config = "copperconfig.ron")]  // this is the ron config we just created.
struct MyApplication {}

// Here we define our own Copper Task
// It will be a source flipping a boolean
pub struct FlippingSource {
    state: bool,
}

// We implement the CuSrcTask trait for our task as it is a source / driver (with no internal input from Copper itself).
impl CuSrcTask for FlippingSource {
    type Output<'m> = output_msg!(RPGpioPayload);

    // You need to provide at least "new" out of the lifecycle methods.
    // But you have other hooks in to the Lifecycle you can leverage to maximize your opportunity 
    // to not use resources outside of the critical execution path: for example start, stop, 
    // pre_process, post_process etc...
    fn new(config: Option<&copper::config::ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        // the config is passed from the RON config file as a Map.
        Ok(Self { state: true })
    }
    
    // Process is called by the runtime at each cycle. It will give:
    // 1. the reference to a monotonic clock
    // 2. a mutable reference to the output message (so no need to allocate of copy anything)
    // 3. a CuResult to handle errors
    fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        self.state = !self.state;   // Flip our internal state and send the message in our output.
        output.set_payload(RPGpioPayload {
            on: self.state,
            creation: Some(clock.now()).into(),
            actuation: Some(clock.now()).into(),
        });
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

But this is a very minimal example for a task; please see [lifecycle](https://github.com/copper-project/copper-rs/wiki/Task-Lifecycle) for a more complete explanation
of a task lifecycle.

### Modular Configuration

Copper supports modular configuration through file includes and parameter substitution, allowing you to:

1. Split large configurations into manageable, reusable chunks
2. Create configuration variations without duplicating the entire RON file
3. Parameterize configurations for different deployment environments

#### Including Configuration Files

You can include other RON configuration files using the `includes` section:

```ron
(
    tasks: [
        // Your main configuration tasks...
    ],
    cnx: [
        // Your main configuration connections...
    ],
    includes: [
        (
            path: "path/to/included_config.ron",
            params: {}, // Optional parameter substitutions
        ),
    ],
)
```

#### Parameter Substitution

You can parameterize your included configurations using template variables:

```ron
// included_config.ron
(
    tasks: [
        (
            id: "task_{{instance_id}}", // Will be replaced with the provided instance_id
            type: "tasks::Task{{instance_id}}",
            config: {
                "param_value": {{param_value}}, // Will be replaced with the provided param_value
            },
        ),
    ],
    cnx: [],
)

// main_config.ron
(
    tasks: [],
    cnx: [],
    includes: [
        (
            path: "included_config.ron",
            params: {
                "instance_id": "42", // Replaces {{instance_id}} with "42"
                "param_value": 100,  // Replaces {{param_value}} with 100
            },
        ),
    ],
)
```

For more details on modular configuration, see the [Modular Configuration documentation](https://github.com/copper-project/copper-rs/wiki/Modular-Configuration).

### Deployment of the application

Check out the [deployment](https://github.com/copper-project/copper-rs/wiki/Build-and-Deploy-a-Copper-Application) page for more information.

## FAQ

### How is Copper better or different from the ROS (Robot Operating System)?

As explained above, Copper is a "user-friendly runtime engine" written in Rust which manages task execution, data flow,
logging, and more.

In contrast, the [ROS](https://github.com/ros) is an open-source set of software libraries and tools primarily written
for C++ and Python.


### 🚀 Is it *that* more performant?

In the example directory, we have 2 equivalent applications. One written in C++ for ROS and a port in Rust with Copper.

```bash
examples/cu_caterpillar
examples/ros_caterpillar
```

You can try them out by either just logging onto a desktop, or with GPIOs on a RPi.
You should see a couple order of magnitude difference in performance.

Copper has been designed for performance first. Not unlike a game engine,
we use a data-oriented approach to minimize latency
and maximize throughput.

### What about the safety considerations?

Copper is fully deterministic, it means that you can have a provable match between your stack when it replays the data captured by the robot and your stack actual behavior running on your robot.

Also Copper is written in Rust, it is memory safe and thread safe by design. It is also designed to be easy to use and
avoid common pitfalls.

## Project

> [!NOTE]
> We are looking for contributors to help us build the best robotics framework possible. If you are interested, please
> join us on [Discord](https://discord.gg/VkCG7Sb9Kw) or open an issue.

### Release Notes

You can find the full release notes [here](https://github.com/copper-project/copper-rs/wiki/Copper-Release-Log).

### Roadmap

Here are some of the features we just release and some we plan to implement next, if you are interested in contributing
on any of those, please let us know!:

- [x] **Buffer Pools**: Implement a large buffer (Host or Device/GPU) pools for 0 copy large inter-task transfers.
- [x] **Missions**: Implement a way to have multiple DAGS in the RON configuration file and have a centralized way to
  switch from one to another. This is useful for running the stack for a specific mission: for example Data acquisition
  missiom, Full autonomy mission, Charging, etc.
- [x] **Modular Configuration**: As robots built with Copper gain complexity, users will need to build "variations" of
  their robots without duplicating their entire RON file.
- [x] **ROS2/DDS interfacing**: Build a pair of sink and source to connect to existing [ROS2](https://github.com/ros2)
  systems, helping users migrate their stack bit by bit.
- [x] **Microcontroller support**: Modify all necessary Copper code packages to remove dependencies on the
  standard library (std) to support "no_std" (#![no_std])
  to support running the code on bare-metal on microcontrollers. This will allow a seamless environment for high level
  calls on a standard kernel (ML inference etc...)
  and low level tasks on MCUs (control, HW interfacing...).
- [ ] **Log Compression & Selection**: Implement a pluggable compression system for logs and its resim counterpart.
  For example to encode video from images. Selection is about NOT logging something if it is not needed.
- [ ] **Parallel Copper Lists**: allow Copper lists to be executed in a staggered and parallel way to improve
  throughput. Status: We built a background tasks that helps with this.
- [ ] **Extensible scheduling**: Enable a way to give hints to Copper to better schedule workloads at compile time.
- [ ] **Swarm support**: Implement Zenoh to allow a swarm of robots powered by Copper to cooperate. Status: Sink done,
  needs a Source.
- [ ] **MCAP support**: Allow the interfacing of Foxglove and [ROS](https://github.com/ros) ecosystems at the logging
  level.
