# cu-iceoryx2-bridge

## Overview

A Copper bridge that maps Copper channels to Iceoryx2 publish/subscribe services.

## Interface

Defines typed `Tx`/`Rx` channels and maps them to external transport routes.

## Configuration

- `node_name`: optional Iceoryx2 node name (string).
- `max_payload_bytes`: optional default max payload size for channels (usize, defaults to 65536).

Each channel uses `route` as the Iceoryx2 service name. Channels can override `max_payload_bytes`
in their `config` block.

Example:

```ron
(
  bridges: [
    (
      id: "iceoryx2",
      type: "bridges::DemoIceoryx2Bridge",
      config: { "node_name": "cu_demo" },
      channels: [
        Tx(id: "ping", route: "demo/ping"),
        Rx(id: "pong", route: "demo/pong"),
      ],
    ),
  ],
)
```

## Usage

Declare bridge types in app code, then reference them under `bridges` in `copperconfig.ron`.

## Compatibility

Depends on enabled transport features and target platform support for the selected backend.

## Links

- Crate path: `components/bridges/cu_iceoryx2_bridge`
- docs.rs: <https://docs.rs/cu-iceoryx2-bridge>
