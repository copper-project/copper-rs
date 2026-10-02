 ## What bridges are (and why)

  - A bridge lets Copper talk to an external transport (serial, CAN, DDS, middleware, ESC bus…) through multiple logical channels that share the same connection.
  - Each channel has a typed payload and a direction: Tx (Copper → outside) or Rx (outside → Copper). Types are enforced at compile time, so you don’t accidentally mix payloads.
  - Missions can override transport routes/topics/paths per channel without changing code (e.g., switch topics per robot or per mission).

## Defining channels and the bridge

 - Declare Tx/Rx channels with the tx_channels! / rx_channels! macros. You give each channel:
     - an ID (used in the mission file to connect),
     - a payload type,
     - an optional default route/topic/path (used if the mission doesn’t override it).
 - Tx channels skip `send` by default when the upstream `CuMsg` has no payload.
     - Prefix a `tx_channels!` entry with `[publish_empty]` when metadata-only sends are part of that channel's contract.
 - Implement CuBridge:
     - Specify which channel sets are Tx/Rx.
     - new receives bridge-level config plus per-channel configs (including any mission route overrides) so you can set baud, QoS, topic names, etc.
     - receive fills a CuMsg for the selected Rx channel; send consumes a CuMsg for a Tx channel.
     - Optional hooks: start (before first I/O), preprocess/postprocess per iteration, stop at shutdown. Use them to open sockets/ports, prefill buffers, flush, or close.

## Wiring in a mission

 - In the mission config, add a bridges entry with an ID, the Rust type, optional config, and the list of channels (Rx/Tx). You can override any channel’s default route here.
 - Connect graph edges by using bridge_id/channel_id in cnx. Direction is enforced: Rx can only be a source endpoint; Tx can only be a destination endpoint.

 ## Scheduling: when bridge methods run

 - Lifecycle:
     - start runs with task starts; stop runs with task stops.
     - preprocess/postprocess bracket each iteration like tasks do.
 - Per-iteration order:
     - Rx channels behave like sources: receive is called before any tasks that consume that channel’s output.
     - Tx channels behave like sinks: send is called after the upstream task(s) that feed that channel have run.
         - If the Tx input `CuMsg` is empty, the runtime skips `send` unless that channel was declared with `[publish_empty]`.
     - If a bridge has both Rx and Tx used in the same dataflow, the runtime linearizes them: Rx → tasks → Tx on that bridge, preserving dependencies.
 - Unconnected channels:
     - If an Rx channel is never connected, its receive is never called.
     - If a Tx channel is never connected, its send is never called.
     - Idle channels still see start/stop (the bridge instance exists), so keep those cheap.

## Routes and overrides

 - Default routes set in macros are just suggestions. Missions can override per channel. At runtime, use the provided BridgeChannelConfig to pick “effective” routes; it already merges
    defaults and overrides.
 - Config blocks per channel are also passed in, so you can tweak QoS, IDs, baud rates, etc., without recompiling.

## When to use a bridge vs. plain sources/sinks

 - Use a bridge when multiple channels share the same transport or need shared state (one serial port, one CAN bus, one Zenoh session, multiple ESCs on one bus).
 - Use plain CuSrcTask/CuSinkTask when each endpoint is independent hardware or logic and doesn’t need shared transport management.

## Practical patterns

 - Single transport, many outputs: e.g., one ESC bus bridge with four Tx channels, each an ESC command; four Rx channels for telemetry. Tasks feed/consume those channels; ports/QoS live in bridge config.
 - Telemetry uplink/downlink: one middleware bridge with Rx for inbound commands and Tx for outbound telemetry; routes are swapped per mission (sim vs. robot).
