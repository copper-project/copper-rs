# cu-iceoryx2-bridge

A Copper bridge that maps Copper channels to Iceoryx2 publish/subscribe services.

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
