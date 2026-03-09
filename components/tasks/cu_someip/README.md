# cu_someip

AUTOSAR SOME/IP protocol tasks for the Copper runtime.

Provides UDP-based SOME/IP source, sink, router, and service discovery monitor. Supports mock mode for testing without real network sockets.

## Tasks

| Task | Trait | Description |
|------|-------|-------------|
| `SomeIpSource` | `CuSrcTask` | Receives SOME/IP messages from UDP socket (or mock) |
| `SomeIpSink` | `CuSinkTask` | Sends SOME/IP messages via UDP socket (or mock) |
| `SomeIpRouter` | `CuTask` | Routes messages by service ID, generates error responses |
| `SomeIpSdMonitor` | `CuTask` | Parses SOME/IP-SD announcements, tracks service availability |

## Configuration (RON)

```ron
(
    id: "someip_src",
    type: "cu_someip::SomeIpSource",
    config: { "bind_addr": "0.0.0.0", "port": 30490 },
),
(
    id: "someip_router",
    type: "cu_someip::SomeIpRouter",
),
(
    id: "someip_sink",
    type: "cu_someip::SomeIpSink",
    config: { "dest_addr": "127.0.0.1", "dest_port": 30491 },
),
```

## Features

- `mock` — Mock mode (no UDP sockets). Source produces empty messages; sink counts transmissions.

## Protocol Support

- SOME/IP wire format (8-byte header + payload)
- Message types: REQUEST, REQUEST_NO_RETURN, NOTIFICATION, RESPONSE, ERROR
- SOME/IP-SD service discovery (OfferService, StopOffer parsing)
- Service routing with up to 32 registered services
