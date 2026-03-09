# cu_uds

ISO 14229 Unified Diagnostic Services (UDS) server and client tasks for the Copper runtime.

## Tasks

| Task | Trait | Description |
|------|-------|-------------|
| `UdsServer` | `CuTask` | Full UDS diagnostic server with session, security, and DID support |
| `UdsClient` | `CuTask` | UDS tester/client with request queue and timeout management |

### UDS Server Features

- **Diagnostic Session Control** (0x10) — Default, Programming, Extended sessions
- **ECU Reset** (0x11) — Hard/soft/key-off reset
- **Security Access** (0x27) — Seed/key authentication (pluggable)
- **Tester Present** (0x3E) — Keep-alive with suppress-positive-response support
- **Read Data By Identifier** (0x22) — DID read with built-in VIN (0xF190)
- **Write Data By Identifier** (0x2E) — DID write (extended session required)
- **Routine Control** (0x31) — Start/stop/request results

### UDS Client Features

- Request queue (up to 16 pending requests)
- P2/P2* timeout management
- Automatic response matching

## Configuration (RON)

```ron
(
    id: "uds_server",
    type: "cu_uds::UdsServer",
    config: {
        "source_addr": 0x7E8,
        "target_addr": 0x7E0,
    },
),
```

## I/O Types

Both server and client use `IsotpPdu` as input and output, designed to connect directly to the ISO-TP codec layer.
