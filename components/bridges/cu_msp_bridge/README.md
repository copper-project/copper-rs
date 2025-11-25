# cu-msp-bridge

Bridge that combines the MSP source/sink tasks into one serial transport.  It exposes a single TX channel (`requests`) that accepts batches of [`cu_msp_lib::structs::MspRequest`] messages and a single RX channel (`responses`) that yields [`MspResponse`] batches decoded from the line.

## Configuration

The bridge reads the following optional keys from its component configuration:

| Key          | Type   | Default        | Description                                                |
|--------------|--------|----------------|------------------------------------------------------------|
| `device`     | string | `/dev/ttyUSB0` | Path to the serial device (can be a PTY).                  |
| `baudrate`   | u32    | `115200`       | Serial baud rate.                                          |
| `timeout_ms` | u64    | `50`           | Read timeout passed to the serial driver in milliseconds.  |

Each Copper route that connects to `requests` or `responses` can override the default channel route if needed via the bridge channel configuration.

## Usage

1. Register `cu_msp_bridge::CuMspBridge` as a bridge in your Copper config and declare the `requests`/`responses` channels.
2. Connect your task nodes to those channels using the `cu_msp_bridge::MspRequestBatch` and `cu_msp_bridge::MspResponseBatch` payloads.
3. Provide the serial configuration (e.g. `device`, `baudrate`) through the bridge config block or override them programmatically using `CuConfig`.

An end-to-end test lives in `examples/cu_msp_bridge_loopback`, which spins up a pseudo-terminal flight controller emulator, wires it to the bridge, sends an MSP `MspRc` request, and validates the echoed response to prove the bridge performs a complete MSP round-trip.
