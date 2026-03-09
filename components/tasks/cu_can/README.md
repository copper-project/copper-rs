# cu_can

CAN bus source, sink, and filter tasks for the Copper runtime.

Provides `CuSrcTask`, `CuSinkTask`, and `CuTask` implementations for interfacing with CAN bus networks via Linux SocketCAN or mock mode for testing.

## Tasks

| Task | Trait | Description |
|------|-------|-------------|
| `CanSource` | `CuSrcTask` | Reads CAN frames from SocketCAN (or produces mock frames) |
| `CanSink` | `CuSinkTask` | Writes CAN frames to SocketCAN (or counts in mock mode) |
| `CanFilter` | `CuTask` | Passes frames matching an ID/mask filter |

## Configuration (RON)

```ron
(
    id: "can_src",
    type: "cu_can::CanSource",
    config: { "interface": "vcan0" },
),
(
    id: "can_filter",
    type: "cu_can::CanFilter",
    config: { "accept_id": 0x100, "accept_mask": 0x7FF },
),
(
    id: "can_sink",
    type: "cu_can::CanSink",
    config: { "interface": "vcan0" },
),
```

## Features

- `mock` — Enables mock mode (no SocketCAN hardware required). CanSource generates synthetic frames; CanSink counts without transmitting.

## Dependencies

```toml
[dependencies]
cu-can = { path = "...", features = ["mock"] }
```
