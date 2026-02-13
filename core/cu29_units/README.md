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
