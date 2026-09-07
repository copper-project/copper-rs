# HC-12 radio resources

`cu-hc12` configures an HC-12 in FU3 transparent mode at startup and exports an
owned `radio.serial` resource implementing `cu_serial::SerialIo`. Channel selection
lives in RON. The same resource can feed a raw byte bridge or a serial LogStream
adapter; each radio has one owner.

## Wiring and startup

Supply a nonblocking UART, an `embedded_hal::digital::OutputPin` connected to SET,
and a startup-only `embedded_hal::delay::DelayNs` resource. The UART must already
match the module's saved baud rate and format (factory default: 9600, 8N1).
Both radio peers must use matching channel, FU3 mode and baud rate.

Startup waits 200 ms before entering command mode, holds SET low for 40 ms,
checks `AT`, queries/selects FU3, and queries/sets the channel with `AT+Cxxx`.
Each command has a bounded 500 ms timeout. Existing matching settings are left
alone to avoid unnecessary persistent writes. SET is released and an 80 ms
settling delay is applied on both success and command failure. SET remains owned
by the radio during operation. The exported serial resource carries transparent
mode data after startup completes.

Channels 1–127 follow the command range in the supplied V2.6 manual; the manual
only specifies normal radio performance through channel 100. The radio is
half duplex and can lose bytes: applications must arrange turn-taking and
implement the delivery checks required by their protocol.

## Resource composition

For the Unix USB-to-TTL wiring example, enable `cu-linux-resources`'s
`serial-rts` Cargo feature and connect the adapter's active-low RTS# output to SET.
Use compatible 3.3 V logic levels and a common ground.
The resource disables flow control and initializes RTS# high; `set_low` asserts
RTS to enter command mode, and `set_high` releases it. Adapters with inverted
RTS polarity require a different pin adapter.

```rust,ignore
use cu_linux_resources::{LinuxNonblockingSerialPort, LinuxSerialRtsPin};
type Radio = cu_hc12::Hc12<LinuxNonblockingSerialPort, LinuxSerialRtsPin>;
type RadioResources = cu_hc12::Hc12Resources<
    LinuxNonblockingSerialPort, LinuxSerialRtsPin, cu_hc12::host::StartupDelay,
>;
type RadioBridge = cu_serial_bridge::SerialBridge<Radio>;
```

```ron
resources: [
    (id: "board", provider: "cu_linux_resources::LinuxResources", config: {
        "serial0_dev": "/dev/ttyUSB0", "serial0_baudrate": 9600,
        "serial0_nonblocking": true,
        "serial0_rts": true,
    }),
    (id: "timing", provider: "cu_hc12::host::DelayResources"),
    (id: "radio", provider: "RadioResources", resources: {
        "serial": "board.serial0", "set": "board.serial0_rts", "delay": "timing.delay",
    }, config: { "channel": 21 }),
],
```

For a separate Raspberry Pi GPIO, use `LinuxOutputPin`, configure a `gpioN`
output initially high, and bind `set` to that slot instead of `serial0_rts`.
Other hardware can supply any compatible SET-pin resource. For embedded hardware, export
`cu_serial::ReadySerial<YourUart>` when the HAL implements embedded-io readiness,
or implement `SerialIo` using the HAL's nonblocking/DMA facilities. Bind the
board's delay and SET-pin resources. The driver and adapters support `no_std`.

Bind a `RadioBridge` to `radio.serial`; its channels are `bytes_rx` and `bytes_tx`,
with `cu_serial_bridge::ByteChunk` payloads. See the complete
[echo configuration](examples/echo.ron) and [application](examples/echo.rs).
From this directory, `just dag` writes `output.svg` showing resource users and
provider-to-consumer arrows. `just echo` runs the hardware example after you edit
its device and channel; run the echo application on only one end of a pair.
Logs go into this component's `logs/` directory.

With two radios attached to this host, run `just pair` to configure both and
verify 1-, 31-, 128-, and 256-byte binary transfers in each direction. The test
uses `/dev/ttyACM0` and `/dev/ttyACM1` by default; override them with
`just pair /dev/ttyUSB0 /dev/ttyUSB1`. Baud rate and channel come from
`examples/echo.ron`. Each transfer has a five-second deadline, and the test exits
on a timeout, mismatch, or unexpected trailing bytes. Run it while both radios
are otherwise idle, with RTS# connected to SET on each adapter.

## LogStream

For LogStream telemetry, bind a framing provider to the radio:

```rust,ignore
type RadioLogResources = cu29_logstream_serial::SerialLogStreamTxResources<Radio>;
type RadioLogTx = cu29_logstream_serial::SerialLogStreamTx<Radio>;
```

```ron
(id: "telemetry", provider: "RadioLogResources",
 resources: { "serial": "radio.serial" }),
```

Use `transport: (type: "RadioLogTx", resource: "telemetry.tx")` in an existing
`log_streaming.destinations` entry. Enable the application's `cu29/logstream`
feature. The default serial adapter supports packets up to 256 bytes, so use
`link.mtu_bytes: 256` or less. Start conservatively at `bitrate_bps: 3000` with
`burst_packets: 1` for 9600-baud UARTs, allowing for serial framing and 8N1 overhead;
measure the actual radio link before increasing the rate. This is a low-bandwidth
telemetry link, so select logged messages and bound record sizes accordingly.

At the receiver, use `SerialLogStreamRxResources<Radio>` and consume
`telemetry.rx` as `SerialLogStreamRx<Radio>`. Its packets feed the existing
LogStream receiver/session router. Assign one radio to the TX provider and the
peer radio to the RX provider for one-way telemetry. See
[serial framing](../cu29_logstream_serial/README.md).

From the repository root, `just hc12-check` checks drivers, framing, resource
composition, DAG rendering and `no_std` compatibility using software tests.
