# cu-bdshot

Copper bridge for bidirectional DSHOT (BDShot) ESCs. It exposes up to four `EscCommand` transmit channels and matching `EscTelemetry` receive channels. The default `cu_bdshot::Stm32BdshotBridge` targets an STM32 backend (placeholder today); enable the `rp2350` feature to use the RP2350 reference board PIO/DMA stack. Custom boards can implement `BdshotBoard`/`BdshotBoardProvider`.

## Channels
- `esc{0-3}_tx` (`EscCommand`): throttle 0–2047, optional telemetry request bit.
- `esc{0-3}_rx` (`EscTelemetry`): latest `DShotTelemetry` sample per ESC.

Only the channels declared in the Copper config are driven; others stay idle.

## Board setup
- RP2350: Build an `Rp2350Board` from your PIO0 state machines and pin map (defaults: GPIO6–GPIO9, 15.3 MHz PIO clock) and register it once before Copper boots:
  ```rust
  let board = Rp2350Board::new(resources, system_clock_hz, Rp2350BoardConfig::default())?;
  cu_bdshot::register_rp2350_board(board)?;
  ```
- STM32: `Stm32BdshotBridge` expects you to register a backend that can emit DSHOT frames and capture the 21-bit telemetry response. Build a backend for your STM32H7 timer/DMA setup (see `H7Dshot600Backend` + `encode_dshot600_dma` helpers) and register it at startup with `cu_bdshot::register_stm32_bdshot_backend(...)`.
- On startup the RP2350 bridge sends repeated disarm frames and waits for telemetry; it errors out if ESCs never answer.

## Configuration
There are no component-level config keys yet. Channel activation is driven solely by the Copper bridge channel list.

## Usage
Declare the bridge and wire tasks to it:
```ron
(
  bridges: [
    (
      id: "bdshot",
      // default feature uses STM32 placeholder, switch to RpBdshotBridge with `--features rp2350`
      type: "cu_bdshot::Stm32BdshotBridge",
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
