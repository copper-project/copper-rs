# cu29-units

Copper-native SI quantity wrappers with a `uom`-shaped API, plus serde/bincode support and optional reflection support.

## Feature Flags

- `default` = `["std"]`
- `std`: enables `uom/std`
- `reflect`: enables `bevy_reflect` derive/traits for wrapped unit types
- `textlogs`: compatibility no-op for downstream feature forwarding

## Namespace via `cu29`

When using the `cu29` facade crate with its `units` feature (enabled by default), these types are available at:

```rust
use cu29::units::si::f32::Length;
use cu29::units::si::length::meter;
```

You can also depend on `cu29-units` directly:

```rust
use cu29_units::si::f32::Length;
use cu29_units::si::length::meter;
```

## Const construction

The existing wrapper types provide const construction for a deliberately limited set of
robotics motion units:

```rust
use cu29_units::si::angle::degree;
use cu29_units::si::f32::{Angle, Length};
use cu29_units::si::length::meter;

const SENSOR_OFFSET: Length = Length::new_const::<meter>(0.12);
const SENSOR_YAW: Angle = Angle::new_const::<degree>(90.0);
```

The supported quantities and units are:

- `Length`: meter, centimeter, millimeter
- `Angle`: radian, degree
- `Time`: second, millisecond, microsecond, nanosecond
- `Velocity`: meter per second, kilometer per hour
- `AngularVelocity`: radian per second, degree per second
- `Acceleration`: meter per second squared, standard gravity

Other quantities intentionally have no `new_const` method, and other units do not satisfy the
const constructor's bound. The runtime `new::<U>` API remains available for every `uom` unit.
