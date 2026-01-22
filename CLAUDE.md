# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Copper is a deterministic robotics runtime and SDK written in Rust. It's designed to build, run, and replay robot applications bit-for-bit identically, from simulation to production, with sub-microsecond latency. The framework supports Linux, embedded Linux (RPi, Xavier), and bare-metal microcontrollers (ARM Cortex-M, RISC-V).

## Essential Build Commands

### Standard Development (Linux/macOS/Windows)

```bash
# Run full CI-aligned checks (formatting, linting, clippy, build, tests)
just std-ci

# Run only linting (formatting + typos)
just lint

# Format all code (Rust, TOML, RON files)
just fmt

# Run tests with all features
cargo nextest run --workspace --all-targets --features mock,image,kornia,gst,faer,nalgebra,glam,debug_pane,bincode,log-level-debug

# Run release mode CI
just std-ci release

# Run CUDA-enabled CI
just std-ci cuda-release

# Build specific workspace member
cargo build -p cu29-runtime

# Run a specific example
cargo run -p cu-caterpillar --bin cu-caterpillar
```

### Embedded/Bare-Metal Development

```bash
# Run no_std CI checks (embedded targets)
just nostd-ci

# Build without std
cargo build --no-default-features

# Build specific embedded example
cd examples/cu_rp2350_skeleton
cargo build-arm
```

### Testing

```bash
# Run all tests (using cargo-nextest)
cargo nextest run --workspace --all-targets

# Run specific test
cargo nextest run -p cu29-runtime --test test_name

# Run doc tests (debug mode only)
cargo test --doc --workspace
```

### Template and Project Generation

```bash
# Generate new project from template (tested in CI during debug mode)
cd templates
cargo generate -p cu_project --name my_project

# Generate full workspace template
cargo generate -p cu_full --name my_workspace
```

### Documentation

```bash
# Build and open documentation locally
just docs

# Build API docs only
cargo +nightly doc --no-deps
```

### Utility Commands

```bash
# Visualize task graph from copperconfig.ron
just dag

# Run with RTSan (realtime sanitizer) to detect non-RT violations
just rtsan-smoke cu-caterpillar cu-caterpillar

# Format SD card for Copper logging (destructive!)
just mkpartition /dev/sdX

# Extract logs from SD card
just extract-log /dev/sdX1 logs/embedded_0.copper
```

## Required Development Tools

- **Rust**: 1.80+ (stable toolchain)
- **just**: Command runner (install: `cargo install just`)
- **cargo-nextest**: Fast test runner (install: `cargo install cargo-nextest`)
- **typos-cli**: Spell checker (install: `cargo install typos-cli`)
- **taplo-cli**: TOML formatter (install: `cargo install --locked taplo-cli`)
- **ronfmt**: RON formatter (install: `cargo install ronfmt`)
- **cargo-generate**: Project template tool (install: `cargo install cargo-generate`)
- **cargo-machete**: Unused dependency detection (install: `cargo install cargo-machete`)

For platform-specific dependencies, see `support/docker/Dockerfile.ubuntu`.

## Architecture Overview

### Core Runtime Model

Copper uses a **deterministic, zero-copy message passing architecture**:

- **CopperList**: Pre-allocated circular buffer holding all messages for one execution cycle. Each CopperList has a fixed pool size (default 2 for double-buffering) and transitions through states: Free → Initialized → Processing → DoneProcessing → BeingSerialized.

- **CuRuntime**: The execution engine that manages tasks, bridges, resources, and the message passing system. Tasks execute in dependency order (computed from the task graph), reading from and writing to CopperLists.

- **Task Graph**: Defined in RON (Rust Object Notation) configuration files, the task graph specifies tasks, connections between them, resources, and runtime settings. The `#[copper_runtime(config = "path.ron")]` proc macro parses this at compile-time and generates all runtime code.

### Component Types

**CuSrcTask** (Source): Generates data (sensors, simulators)
- Has `Output` type
- Implements `process(&mut self, clock, output) -> CuResult<()>`

**CuTask** (Transform): Processes data (algorithms, filters)
- Has `Input` and `Output` types
- Implements `process(&mut self, clock, input, output) -> CuResult<()>`

**CuSinkTask** (Sink): Consumes data (actuators, loggers)
- Has `Input` type
- Implements `process(&mut self, clock, input) -> CuResult<()>`

**CuBridge** (Bidirectional): Multiplexes multiple channels over a single backend (CRSF, MSP, Zenoh, ROS2)
- Supports typed channel sets for Tx/Rx
- Used for protocol adapters and middleware integration

### Message System

All messages are wrapped in `CuMsg<T>` (alias for `CuStampedData<T, M>`):
```rust
pub struct CuStampedData<T, M> {
    payload: Option<T>,  // Actual data
    tov: Tov,           // Time of Validity (point or range)
    metadata: M,        // Processing metadata (status, timing)
}
```

Payloads must implement: `Default + Debug + Clone + Encode + Decode + Serialize`

### Resource Management

Resources (hardware, shared state) are managed centrally:
- Defined in RON config under `resources: []`
- Accessed by tasks via `Resources<'r>` lifetime parameter
- Wrapped in `Owned<T>` or `Borrowed<'r, T>` for ownership semantics
- Use `ResourceKey<T>` as typed handles

### Logging and Replay

Copper's unified logging system records three streams:
1. **StructuredLogLine**: Debug/info/warning/error messages
2. **CopperList**: All inter-task messages for each cycle
3. **FrozenTasks**: Task state snapshots (keyframes)

Logs are stored in binary format (`.copper` files) and are **deterministically replayable** bit-for-bit. Tasks implement the `Freezable` trait to serialize/deserialize state. Export to MCAP format for visualization in Foxglove.

### Configuration System

RON files define the complete application:
```ron
(
    tasks: [
        (id: "sensor", type: "cu_bmi088::Bmi088", config: {...}),
        (id: "filter", type: "cu_ahrs::MadgwickFilter", config: {...}),
    ],
    cnx: [
        (src: "sensor", dst: "filter", msg: "ImuData"),
    ],
    resources: [...],
    logging: (keyframe_interval: 1000, slab_size_mib: 1024),
    monitor: (type: "cu_consolemon::CuConsoleMon"),
)
```

Supports modular configuration with includes and multiple missions per binary.

## Codebase Structure

### Core Crates (`core/`)
- **cu29**: Main prelude and public API
- **cu29_runtime**: Task execution engine, CopperList management
- **cu29_derive**: Proc macros for `#[copper_runtime]` code generation
- **cu29_clock**: Time abstractions (robot clock, monotonic time)
- **cu29_log**: Structured logging macros and types
- **cu29_log_runtime**: Log stream management
- **cu29_unifiedlog**: Binary log format implementation
- **cu29_traits**: Core trait definitions (CuMsgPayload, Freezable, etc.)
- **cu29_export**: Python bindings and MCAP export
- **cu29_helpers**: Std-only utilities
- **cu29_value**: Dynamic value types for configuration

### Component Organization (`components/`)
- **sources/**: Sensor drivers (IMU, cameras, LiDAR, encoders)
- **sinks/**: Actuator drivers (GPIO, motors, servos)
- **tasks/**: Processing algorithms (PID, AHRS, alignment, rate limiting)
- **bridges/**: Protocol adapters (Zenoh, Iceoryx2, MSP, CRSF, BDSHOT)
- **payloads/**: Common message types (`cu_sensor_payloads`, `cu_ros_payloads`, `cu_spatial_payloads`)
- **res/**: Platform-specific resource bundles (e.g., `cu_micoairh743`)
- **libs/**: Reusable libraries (transforms, loggers, embedded registry)
- **monitors/**: Runtime monitoring (console, log monitor)
- **testing/**: Test utilities (UDP injection)

### Examples (`examples/`)
Demonstrates various use cases from minimal to complex:
- **cu_caterpillar**: Simple producer-consumer benchmark
- **cu_rp_balancebot**: Complete balance bot with Bevy simulation
- **cu_flight_controller**: Flight controller application
- **cu_min_baremetal**: Minimal bare-metal example
- **cu_iceoryx2_bridge_demo**: Iceoryx2 bridge ping/pong demo
- **cu_zenoh**: Zenoh middleware integration

### Support (`support/`)
- **ci/**: CI scripts (embedded crate management, wiki generation)
- **cargo_cubuild/**: Custom Cargo subcommand for Copper builds
- **docker/**: Development environment containers
- **nix/**: Nix flake for reproducible environments

### Templates (`templates/`)
- **cu_project**: Minimal single-application template
- **cu_full**: Full workspace template with apps and components

## Key Development Patterns

### Adding a New Task

1. Define payload types in a `payloads/` crate or locally
2. Implement the appropriate task trait (`CuSrcTask`, `CuTask`, or `CuSinkTask`)
3. Add configuration struct (deserializable from RON)
4. Implement `Freezable` trait if task has state to replay
5. Add task to RON configuration file
6. Connect task in `cnx: []` section

### Working with Proc Macros

The `#[copper_runtime(config = "path.ron")]` macro generates:
- CopperList type with all message slots
- Task execution plan (topologically sorted)
- Builder struct with resource wiring
- `CuApplication` or `CuSimApplication` implementations

Expand macro output for debugging: `cargo expand -p your-package`

### Feature Flags

- **std** (default): Standard library support
- **defmt**: Embedded logging via defmt
- **textlogs**: Text-based logging for no_std
- **rtsan**: Realtime sanitizer integration
- **cuda**: CUDA support for GPU tasks
- **mock**: Mock implementations for testing
- **image/kornia/gst/faer/nalgebra/glam**: Optional processing backends
- **python**: Python bindings for log analysis

### Testing Best Practices

- Use `cargo nextest` for faster test execution
- Run `just std-ci` before submitting PRs to catch CI failures locally
- Add tests for new payload types
- Test task logic with mock inputs
- Verify deterministic replay for stateful tasks

### Platform-Specific Development

**For bare-metal targets**:
- Set `default-features = false` in dependencies
- Add feature flag for std: `default = ["std"]`
- Use `.cargo/config.toml` to specify target triple
- Implement `SectionStorage` trait for custom log storage
- Use `embedded-alloc` or LLFF heap for allocation

**For embedded Linux**:
- Full std support but limited features (no GUI, no Python)
- File-based logging instead of memory-mapped
- Test on actual hardware (RPi, Xavier) before deployment

## Code Quality Standards

- **Formatting**: Run `just fmt` before committing (auto-formats Rust, TOML, RON)
- **Linting**: `just lint` runs formatting checks and typo detection
- **Clippy**: All warnings are denied in CI (`--deny warnings`)
- **Unused Dependencies**: Run `cargo machete --with-metadata` and add false positives to `[package.metadata.cargo-machete]` ignored list
- **Commit Messages**: Use Conventional Commits format (e.g., `feat:`, `fix:`, `chore:`)

## Common Pitfalls

- **Don't allocate in hot paths**: Copper is zero-allocation by design. Use pre-allocated buffers.
- **Task order matters**: Tasks execute in dependency order. Check the generated execution plan if tasks aren't running as expected.
- **Resource lifetimes**: Resources must outlive task execution. Use `'r` lifetime parameters correctly.
- **Feature conflicts**: Only one of `restore-state-*` features can be enabled at a time.
- **RON syntax**: RON is similar to Rust but not identical. Use `ronfmt` to validate.
- **Frozen state**: Implement `Freezable` properly or replay will fail.

## Debugging Tips

- **View task graph**: `just dag` renders the task graph from `copperconfig.ron`
- **Expand proc macros**: `cargo expand -p package-name` shows generated code
- **Check execution order**: Look for `execute_tasks` in expanded output
- **Inspect logs**: Use logreader binaries in examples to analyze `.copper` files
- **Export to MCAP**: Use `cu29-export` to convert logs for Foxglove visualization
- **RTSan violations**: Run `just rtsan-smoke` to detect non-realtime operations
- **Memory layout**: Use `#[repr(C)]` for cross-language payloads

## CI Workflow

CI runs on Linux, macOS, and Windows with multiple modes:
- **Debug mode**: All features, doc tests, template generation tests
- **Release mode**: Optimized build, no doc tests
- **CUDA release**: With CUDA features enabled
- **No-std CI**: Bare-metal targets, embedded crates

Match CI locally: `just std-ci` or `just nostd-ci`

## Additional Resources

- Full documentation: https://copper-project.github.io/copper-rs/
- Discord community: https://discord.gg/VkCG7Sb9Kw
- GitHub: https://github.com/copper-project/copper-rs
- Contributing guide: See CONTRIBUTING.md for detailed workflow
