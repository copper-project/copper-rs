# cu-rerun

Optional Rerun representations for Copper payloads and projected task states.
Implement Rerun's native `AsComponents`, then attach the adapter as reflection metadata:

```rust
#[derive(Reflect)]
#[cfg_attr(feature = "rerun", reflect(@cu_rerun::representation::<Self, cu_rerun::views::Spatial3DView>()))]
pub struct ArmState {
    // Your existing fields.
}

#[cfg(feature = "rerun")]
impl cu_rerun::rerun::AsComponents for ArmState {
    fn as_serialized_batches(&self) -> Vec<cu_rerun::rerun::SerializedComponentBatch> {
        use cu_rerun::rerun::{AsComponents, LineStrips3D};
        // Compute joint positions from your matrices here.
        LineStrips3D::new([[[0., 0., 0.], [1., 0., 0.], [1., 0., 1.]]])
            .as_serialized_batches()
    }
}
```

```toml
[features]
rerun = ["dep:cu-rerun", "cu29/reflect"]

[dependencies]
cu-rerun = { version = "~1.4.0-dev", optional = true }
```

`Spatial3DView` is Rerun's own view type. The adapter accepts any Rerun `View`;
Copper carries its native `ViewClassIdentifier` through reflection metadata and the
debug protocol to Timetraveler.

The same implementation works with a local `recording.log("arm", &state)`.
For task internals, put the attribute on the type exposed through the task's
`with_debug_state` / `debug_state_type_path` hooks. Existing type registration
makes the representation discoverable; there is no separate registration list.

Enable `cu29/rerun` plus the component's representation feature in the replay/debug
server (`cu-ahrs/rerun` for the AHRS example). `cu29/rerun` includes `remote-debug`
and enables the representation RPCs, descriptors, and response fields. Local
`RecordingStream::log` calls only need the component feature.

The debugger
calls `AsComponents` on its typed payload or task-state value, then sends a small
RRD stream over CBOR with the requested entity path and Copper timelines. Timetraveler
imports those components into its own recording. Users do not rebuild Timetraveler
or supply a dynamic library. The receiver must already support the chosen Rerun
view class; transmitting components does not install a custom renderer.

Rerun types are optional dependencies, enabled by the `rerun` feature. The protocol
uses `RrdPayload { bytes }`; RRD headers carry the format version.
No representation is computed or logged
on the robot's task execution path, and existing Copper logs need no conversion.
This initial adapter logs one entity per value; `AsComponents` can combine batches
from multiple archetypes on that entity. The wire format is currently pinned to
Rerun 0.43, matching Timetraveler. Representation APIs are experimental.

Start with the [soft-arm example](../../../examples/cu_rerun_soft_arm) to build a
custom representation from tendon measurements. `just` runs the Copper app with
live Rerun visualization and Copper logging; `just debug` serves that recording to
Timetraveler using the same representation.

See `cu-ahrs`'s `rerun` feature for working message and task-state implementations,
and `examples/cu_flight_controller`'s `resim-mcu-debug` recipe to try them.
