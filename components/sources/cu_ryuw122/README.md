# cu-ryuw122

Anchor-side Copper source for the REYAX `RYUW122` UWB modem.

This crate is a serially-driven ranging source. It turns the modem's
`+ANCHOR_RCV=...` UART responses into normalized Copper
`cu_sensor_payloads::RangeObservation` messages.

Use it when you want:

- a safety-oriented distance feed to a set of known tags
- a generic UWB range source for downstream estimation tasks
- a clean Copper boundary between modem I/O and localization logic

This source does not solve 2D or 3D position itself. It only emits
single-peer range observations. Snapshotting, safety decisions, and
multilateration belong in downstream Copper tasks.

## Behavior

The source runs one outstanding anchor poll at a time:

- send `AT+ANCHOR_SEND` for the current peer
- watch later runtime cycles for the matching `+ANCHOR_RCV`
- emit a `RangeObservation` when a response arrives
- immediately move on to the next peer
- if a peer stays silent, advance on timeout and continue polling

This keeps the driver non-blocking and suitable for regular Copper source
execution.

## Current assumptions

- the modem is already configured in `ANCHOR` mode
- network id, address, and CPIN are already set appropriately
- the serial resource timeout is kept low enough for runtime use

## Output

The source emits:

- `cu_sensor_payloads::RangeObservation`

Each observation contains:

- `peer_id`: fixed-capacity peer identifier
- `distance`: meters
- `rssi_dbm`: optional RSSI when the modem is configured to include it

## Configuration

The task binds a single `serial` resource and expects these config keys:

- `peer_ids`: list of modem peer addresses to poll
- `poll_payload`: ASCII payload sent with each `AT+ANCHOR_SEND`
- `response_timeout_ms`: timeout before the source advances to the next peer
- `read_buffer_bytes`: serial read scratch size
- `max_pending_observations`: queue depth for parsed observations

`peer_ids` and `poll_payload` must be ASCII. RYUW122 modem addresses are 8
bytes max, and the poll payload is 12 bytes max.

## Example

On Linux, bind the source through `cu_linux_resources::LinuxResources`:

```ron
(
    resources: [
        (
            id: "linux",
            provider: "cu_linux_resources::LinuxResources",
            config: {
                "serial3_dev": "/dev/ttyUSB0",
                "serial3_baudrate": 115200,
                "serial3_timeout_ms": 20,
            },
        ),
    ],
    tasks: [
        (
            id: "uwb_ranges",
            type: "cu_ryuw122::Ryuw122AnchorSource",
            resources: {
                "serial": "linux.serial3",
            },
            config: {
                "peer_ids": ["TAG00001", "TAG00002"],
                "poll_payload": "PING",
                "response_timeout_ms": 250,
                "read_buffer_bytes": 512,
                "max_pending_observations": 32,
            },
        ),
    ],
)
```

## Downstream usage

Typical next steps are:

- threshold the output for safety logic
- accumulate the most recent ranges by peer id
- feed those snapshots into a separate multilateration task
