# cu-ist8310

Copper source component for the iSentek IST8310 magnetometer.

## Output

`cu_ist8310::Ist8310Source` emits `cu_sensor_payloads::MagnetometerPayload`
(magnetic flux density in microtesla) at 100 Hz.

## How To Use

1. Add the crate dependency:

```toml
cu-ist8310 = { path = "../../components/sources/cu_ist8310", default-features = false, features = ["defmt"] }
```

2. Define the task type alias in your app:

```rust
pub type Ist8310Source = cu_ist8310::Ist8310Source<MyIst8310Bus>;
```

`MyIst8310Bus` is your board/resource type that implements `cu_ist8310::Ist8310Bus`.

3. Add the source task in `copperconfig.ron`:

```ron
(
    id: "ist8310",
    type: "tasks::Ist8310Source",
    logging: (enabled: true),
    resources: {"i2c": "fc.i2c2_ist8310"},
)
```

4. Connect it to a consumer:

```ron
(
    src: "ist8310",
    dst: "mag_logger",
    msg: "cu_sensor_payloads::MagnetometerPayload",
)
```

The source has no component-specific config fields; wiring the resource and
connection is enough.
