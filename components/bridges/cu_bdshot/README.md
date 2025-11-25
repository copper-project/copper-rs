# cu-bdshot

Copper bridge for bidirectional DSHOT (BDShot) ESCs. It exposes up to four `EscCommand` transmit channels and matching `EscTelemetry` receive channels. The default `cu_bdshot::RpBdshotBridge` drives the RP2350 reference board PIO/DMA stack; custom boards can implement `BdshotBoard`/`BdshotBoardProvider`.

## Channels
- `esc{0-3}_tx` (`EscCommand`): throttle 0–2047, optional telemetry request bit.
- `esc{0-3}_rx` (`EscTelemetry`): latest `DShotTelemetry` sample per ESC.

Only the channels declared in the Copper config are driven; others stay idle.

## Board setup
- Build an `Rp2350Board` from your PIO0 state machines and pin map (defaults: GPIO6–GPIO9, 15.3 MHz PIO clock) and register it once before Copper boots:
  ```rust
  let board = Rp2350Board::new(resources, system_clock_hz, Rp2350BoardConfig::default())?;
  cu_bdshot::register_rp2350_board(board)?;
  ```
- On startup the bridge sends repeated disarm frames and waits for telemetry; it errors out if ESCs never answer.

## Configuration
There are no component-level config keys yet. Channel activation is driven solely by the Copper bridge channel list.

## Usage
Declare the bridge and wire tasks to it:
```ron
(
  bridges: [
    (
      id: "bdshot",
      type: "cu_bdshot::RpBdshotBridge",
      channels: [
        Tx (id: "esc0_tx"), Tx (id: "esc1_tx"), Tx (id: "esc2_tx"), Tx (id: "esc3_tx"),
        Rx (id: "esc0_rx"), Rx (id: "esc1_rx"), Rx (id: "esc2_rx"), Rx (id: "esc3_rx"),
      ],
    ),
  ],
  cnx: [
    (src: "thr", dst: "bdshot/esc0_tx", msg: "cu_bdshot::EscCommand"),
    (src: "bdshot/esc0_rx", dst: "tele0", msg: "cu_bdshot::EscTelemetry"),
  ],
)
```

See `examples/cu_elrs_bdshot_demo` for a full mission wiring CRSF RC input into BDShot ESCs on the RP2350 reference board.
