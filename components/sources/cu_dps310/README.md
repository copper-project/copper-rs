# cu-dps310

Copper source component for the Infineon DPS310 barometer.

## Output

`cu_dps310::Dps310Source` emits `cu_sensor_payloads::BarometerPayload`
(pressure in Pa, temperature in C) at 30 Hz.

## How To Use

1. Add the crate dependency:

```toml
cu-dps310 = { path = "../../components/sources/cu_dps310", default-features = false, features = ["defmt"] }
```

2. Define the task type alias in your app:

```rust
pub type Dps310Source = cu_dps310::Dps310Source<MyDps310Bus>;
```

`MyDps310Bus` is your board/resource type that implements `cu_dps310::Dps310Bus`.

3. Add the source task in `copperconfig.ron`:

```ron
(
    id: "dps310",
    type: "tasks::Dps310Source",
    logging: (enabled: true),
    resources: {"i2c": "fc.dps310"},
)
```

4. Connect it to a consumer:

```ron
(
    src: "dps310",
    dst: "baro_logger",
    msg: "cu_sensor_payloads::BarometerPayload",
)
```

The source has no component-specific config fields; wiring the resource and
connection is enough.
