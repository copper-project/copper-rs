# cu-micoairh743

Resource bundle for the MicoAir H743 flight controller board.

## What it provides

- UART6 (`Uart6Port`) and UART2 (`Uart2Port`) with overrun logging
- `SerialPortError` for embedded-io bridge error types
- `GreenLed`
- SDMMC logger (`Logger` + `LogStorage`, Copper partition)
- BMI088 SPI bus + chip selects + delay (`Bmi088Spi`, `Bmi088AccCs`, `Bmi088GyrCs`, `Bmi088Delay`)
- STM32H7 BDshot board registration

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

## UART aliases in config

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
