# cu-micoairh743

## Overview

Resource bundle for the MicoAir H743 flight controller board.

### What it provides

- UART6 (`Uart6Port`), UART2 (`Uart2Port`), and UART4 (`Uart4Port`) with overrun logging
- `SerialPortError` for embedded-io bridge error types
- `GreenLed`
- SDMMC logger (`Logger` + `LogStorage`, Copper partition)
- BMI088 SPI bus + chip selects + delay (`Bmi088Spi`, `Bmi088AccCs`, `Bmi088GyrCs`, `Bmi088Delay`)
- STM32H7 BDshot board registration

## Interface

Provides named resource handles consumed by runtime components.

## Configuration

### UART aliases in config

The CRSF and MSP bridges in `copperconfig.ron` use the provided aliases:

```ron
(
    id: "crsf",
    type: "cu_crsf::CrsfBridge<cu_micoairh743::Uart6Port, cu_micoairh743::SerialPortError>",
),
(
    id: "msp",
    type: "cu_msp_bridge::CuMspBridge<cu_micoairh743::Uart2Port, cu_micoairh743::SerialPortError>",
),
```

UART2 and UART4 default to 115200 baud (override with `uart2_baud` / `uart4_baud` in config).

## Usage

Add the crate to your firmware dependencies and point the Copper config to the bundle:

```ron
resources: [
    (
        id: "fc",
        provider: "cu_micoairh743::MicoAirH743",
    ),
],
```

For resource IDs, use `cu_micoairh743::MicoAirH743Id` in your application code.

## Compatibility

Depends on target board/OS support and resource provider implementation.

## Links

- Crate path: `components/res/cu_micoairh743`
- docs.rs: <https://docs.rs/cu-micoairh743>
