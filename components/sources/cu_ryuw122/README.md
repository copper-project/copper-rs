# cu-ryuw122

Initiator-side Copper range source for the REYAX `RYUW122` UWB modem.

This source runs on the modem that uses the REYAX vendor `ANCHOR` role. That is the side that
can issue `AT+ANCHOR_SEND` and receive `+ANCHOR_RCV=...` lines with distance over UART.

In robotics localization terms, this modem is typically mounted on the moving robot. The fixed
anchors in the space usually run the vendor `TAG` role and respond to the robot's initiator
requests.

Use it when you want:

- a robot-side distance feed to fixed anchors
- a generic UWB range source for downstream estimation tasks
- a clean Copper boundary between modem I/O and localization logic

This source does not solve 2D or 3D position itself. It only emits single-anchor range
observations. Snapshotting, safety decisions, and multilateration belong in downstream Copper
tasks.

## Behavior

The source runs one outstanding initiator request at a time:

- send `AT+ANCHOR_SEND` for the current anchor
- watch later runtime cycles for the matching `+ANCHOR_RCV`
- emit a `PeerRangeObservation` when a response arrives
- immediately move on to the next anchor
- if an anchor stays silent, advance on timeout and continue polling

This keeps the driver non-blocking and suitable for regular Copper source execution.

## Current assumptions

- the modem that runs this source is already configured in REYAX vendor `ANCHOR` mode
- network id, address, and CPIN are already set appropriately
- the serial resource timeout is kept low enough for runtime use

## Output

The source emits:

- `cu_sensor_payloads::PeerRangeObservation`

Each observation contains:

- `peer_id`: fixed-capacity identifier for the responding anchor
- `distance`: meters
- `rssi_dbm`: optional RSSI when the modem is configured to include it

## Configuration

The task binds a single `serial` resource and expects these config keys:

- `anchor_ids`: list of fixed anchor addresses to poll
- `poll_payload`: ASCII payload sent with each `AT+ANCHOR_SEND`
- `response_timeout_ms`: timeout before the source advances to the next anchor
- `read_buffer_bytes`: serial read scratch size
- `max_pending_observations`: queue depth for parsed observations

`anchor_ids` and `poll_payload` must be ASCII. RYUW122 modem addresses are 8 bytes max, and the
poll payload is 12 bytes max.

## Example

On Linux, bind the source through `cu_linux_resources::LinuxResources`:

```ron
(
    resources: [
        (
            id: "linux",
            provider: "cu_linux_resources::LinuxResources",
            config: {
                "serial3_dev": "/dev/ttyACM0",
                "serial3_baudrate": 115200,
                "serial3_timeout_ms": 20,
            },
        ),
    ],
    tasks: [
        (
            id: "uwb_ranges",
            type: "cu_ryuw122::Ryuw122InitiatorSource",
            resources: {
                "serial": "linux.serial3",
            },
            config: {
                "anchor_ids": ["ANCH0001", "ANCH0002"],
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
- accumulate the most recent ranges by anchor id
- feed those snapshots into a separate multilateration task
