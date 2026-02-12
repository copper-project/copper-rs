# cu-ratelimit

## Overview

### This is a Generic Rate limiter for Copper

If you have a source that is going too fast, you can use this simple task to skip over messages.

## Interface

### Task and Input

```ron
  (
            id: "rl1",
            type: "cu_ratelimit::CuRateLimit<MyPayload>", // Set your type here
            config: {
                "rate": 10.0,  // in Hz
            },
        ),
 [...]
```

### Output

It will match the Input type, set it as a type specialization like ie: replace MyPayload to your actual type in the config.

## Configuration

- `rate`: the maximum rate in Hz

## Usage

Add the task type to your app registry and connect it in the mission graph.

## Compatibility

Supports `std`/`no_std` according to crate features and dependencies.

## Links

- Crate path: `components/tasks/cu_ratelimit`
- docs.rs: <https://docs.rs/cu-ratelimit>
