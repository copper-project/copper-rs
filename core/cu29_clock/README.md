## High Performance Monotonic Clock for robotics

This crate provides a monotonic clock and a mockable clock for testing and replay.
It has a wide range of support for robots running on OSes and bare metal.

It is no-std compatible with some limitations, see below.

Low level CPU counter implementations are provided for:

- x86-64
- arm64
- armv7
- risc-v

We also provide a TOV (Time Of Validity) enum for tagging sensor data.

It can fall back to the posix monotonic clock for other platforms (with std).

It has been created originally for the Copper runtime but can be perfectly used independently.
See the main crate cu29 for more information about the overall Copper project.

## Examples

### Basic Clock Usage

```rust
use cu29_clock::{RobotClock, CuTime, CuDuration};

// Create a new robot clock
let clock = RobotClock::new();

// Get the current time (returns CuTime, which is nanoseconds since clock start)
let now = clock.now();
println!("Current time: {}", now); // Displays with appropriate units (ns, µs, ms, s, etc.)

// Get a less precise but faster time reading
let recent = clock.recent();

// Clock is cloneable and all clones share the same time reference
let clock_clone = clock.clone();
assert_eq!(clock.now(), clock_clone.now()); // more or less :)

// Create a clock with a specific reference time
let ref_time_ns = 1_000_000_000; // 1 second
let clock_with_ref = RobotClock::from_ref_time(ref_time_ns);
```

### Working with Time Durations

```rust
use cu29_clock::{CuDuration, CuTime};
use std::time::Duration;

// Create durations
let duration1 = CuDuration::from(Duration::from_millis(100));
let duration2 = CuDuration::from(500_000_000u64); // 500ms in nanoseconds

// Arithmetic operations
let sum = duration1 + duration2;  // 600ms
let diff = duration2 - duration1; // 400ms
let scaled = duration1 * 2u32;    // 200ms
let divided = duration2 / 5u32;   // 100ms

// Display formatting automatically chooses appropriate units
println!("Duration: {}", sum);        // "600.000 ms"
println!("Large duration: {}", CuDuration::from(Duration::from_secs(3600))); // "1.000 h"

// Min/max operations
let min_duration = duration1.min(duration2);
let max_duration = duration1.max(duration2);

// Convert to standard Duration
let std_duration: Duration = duration1.into();
```

### Mock Clock for Testing and Simulation

```rust
use cu29_clock::{RobotClock, RobotClockMock, CuDuration};
use std::time::Duration;

// Create a mock clock that you can control
let (clock, mock_control) = RobotClock::mock();

// The clock starts at 0
assert_eq!(clock.now(), CuDuration(0));

// Advance the clock by specific amounts
mock_control.increment(Duration::from_millis(100));
assert_eq!(clock.now(), CuDuration(100_000_000)); // 100ms in nanoseconds

// Jump to absolute time values
mock_control.set_value(5_000_000_000); // 5 seconds
assert_eq!(clock.now(), CuDuration(5_000_000_000));

// You can also decrement (breaks monotonicity, use with caution)
mock_control.decrement(Duration::from_secs(1));
assert_eq!(clock.now(), CuDuration(4_000_000_000)); // 4 seconds

// Get current mock time directly from the mock controller
let current_mock_time = mock_control.now();
let current_raw_value = mock_control.value();

// All clones of the clock are synchronized with the mock
let clock_clone = clock.clone();
assert_eq!(clock.now(), clock_clone.now());
```

### Time of Validity (Tov)

Tov represents when sensor data was actually valid, which is crucial for robotics applications:

```rust
use cu29_clock::{RobotClock, Tov, CuTimeRange, CuTime, CuDuration};
use std::time::Duration;

let clock = RobotClock::new();
let current_time = clock.now();

// Single timestamp - most common case
let tov_single = Tov::Time(current_time);

// Time range - for sensors that collect data over time (e.g., lidar scans)
let tov_range = Tov::Range(CuTimeRange {
start: current_time,
end: current_time + CuDuration::from(Duration::from_millis(10)),
});

// No valid time - for error conditions or initialization
let tov_none = Tov::None;

// Converting from common types
let tov_from_option: Tov = Some(current_time).into();
let tov_from_duration: Tov = current_time.into();

// Pattern matching on Tov in algorithms
fn process_sensor_data(tov: Tov, last_time: &mut CuTime) -> Result<CuDuration, &'static str> {
    match tov {
        Tov::Time(timestamp) => {
            let dt = timestamp - *last_time;
            *last_time = timestamp;
            Ok(dt)
        }
        Tov::Range(range) => {
            // Use start of range for timing calculations
            let dt = range.start - *last_time;
            *last_time = range.start;
            Ok(dt)
        }
        Tov::None => Err("No valid timestamp available"),
    }
}

// Display Tov values with automatic formatting
println!("Single time: {}", tov_single);     // "1.500 s"
println!("Time range: {}", tov_range);       // "[1.500 s – 1.510 s]"
println!("No time: {}", tov_none);           // "None"

// Create time ranges from collections of timestamps
let timestamps = [
CuTime::from(100u64),
CuTime::from(150u64),
CuTime::from(120u64)
];
let range_from_slice = CuTimeRange::from( & timestamps[..]);
// range_from_slice.start = 100, range_from_slice.end = 150
```

### Optional Time Values

```rust
use cu29_clock::{OptionCuTime, CuTime};

// Efficient optional time representation (avoids 128-bit Option overhead)
let some_time = OptionCuTime::from(CuTime::from(1000u64));
let no_time = OptionCuTime::none();

// Check if time is present
if ! some_time.is_none() {
let time_value = some_time.unwrap();
println! ("Time: {}", time_value);
}

// Convert to/from standard Option
let std_option: Option<CuTime> = some_time.into();
let back_to_option_time: OptionCuTime = std_option.into();

// Display formatting
println!("Some time: {}", some_time);  // "1.000 µs"
println!("No time: {}", no_time);      // "None"
```

### Platform-Specific High-Performance Timing

The clock automatically uses the best available timing mechanism for your platform:

- **x86-64**: RDTSC instruction for sub-nanosecond precision
- **ARM64**: ARM generic timer counters
- **ARM32**: 64-bit ARM performance counters
- **RISC-V**: Cycle counter
- **Other platforms**: Falls back to system monotonic clock (std only)

```rust
use cu29_clock::{RobotClock, platform};

// The clock automatically calibrates frequency at first use
let clock = RobotClock::new();

// For no-std environments, frequency calibration is simplified
// but still provides high precision timing suitable for robotics
```
