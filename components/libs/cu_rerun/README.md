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
The `representation` adapter logs one entity per value; `AsComponents` can combine
batches from multiple archetypes on that entity. The wire format is currently pinned to
Rerun 0.43, matching Timetraveler. Representation APIs are experimental.

Start with the [soft-arm example](../../../examples/cu_rerun_soft_arm) to build a
custom representation from tendon measurements and open it directly in Rerun with
`just`. Its optional attribute also makes that representation available to the debug API.

See `cu-ahrs`'s `rerun` feature for working message and task-state implementations,
and `examples/cu_flight_controller`'s `resim-mcu-debug` recipe to try them.


## Composed representations

Use `composite(Self::compose)` to arrange existing representations. This API is
experimental and requires a viewer supporting native blueprint RRD in the composite RPC response.

```rust
use bevy_reflect::Reflect;
use cu_rerun::{Composition, field, horizontal};

#[derive(Reflect)]
#[reflect(@cu_rerun::composite(Self::compose))]
struct Stereo {
    left: Image,
    right: Image,
}

impl Stereo {
    fn compose(&self) -> Composition<'_> {
        horizontal([field("left", &self.left), field("right", &self.right)])
    }
}
```

Here `Image` is a reflected type with its own `representation` adapter. Existing
AHRS payload and debug-state adapters work unchanged as composition leaves.
`horizontal` and `vertical` build native Rerun containers. `container(kind, children)`
accepts Rerun's `ContainerKind`, including `Tabs` and `Grid`. `overlay` builds one
native view selecting all child entities; children must use the same view class
and compatible coordinates. Compositions can nest. Blueprints are viewer defaults;
the user can rearrange panes.

`field` borrows a reflected field and uses its field name as its stable path
segment. Names must be unique across layout groups at the same entity root.
For computed data use `entity::<cu_rerun::views::Spatial3DView>("axes", archetype)`;
it owns the supplied `AsComponents` value for the debug request. Computed entities
are not Rust fields and cannot be selected with `field_path`.

An unannotated struct, tuple, or enum is searched recursively for represented
children. Options retain paths across absence and return; enum paths include the
variant and field. Scalars remain available through the ordinary inspector;
plotting a scalar in a composition requires a representation adapter or an
explicit `TimeSeriesView` entity. Opaque handles require a typed adapter; this
API does not dereference arbitrary handles or traverse pixel buffers.

`representation.read` accepts an optional `field_path` array, such as `["right"]`,
to request a field independently of its parent's composite override. Its entity
path is the requested root plus that field path. Enum paths include their active
variant, for example `["right", "Some", "0"]`.

Composite responses carry `composition: CompositionSnapshot`: a flat `entities`
list and an optional `blueprint` recording. Each entity contains its full path,
view class, and RRD bytes. `BlueprintPayload` carries native Rerun blueprint RRD;
there is no Copper layout enum in the protocol. Nested blueprints are merged into
one blueprint store, retaining their native container and view components. The
single-entity response keeps its `bytes` field. A descriptor's optional `view_class`
is absent for composite/discovery roots.

Every successful composition is a complete snapshot of its owned subtree at the
requested CopperList and time. A viewer must replace that subtree at that sample,
including clearing children omitted by the new snapshot, before inserting the
leaf data. Empty snapshots clear the subtree; failed or unrequested reads do not.
Use the same replacement semantics for seeking and replay cache restoration.
Blueprint recordings contain no activation command. Apply them when the user adds
a composite with (+), preserving their existing layout during subsequent samples
and replay. Timetraveler must implement blueprint import and parent/child (+)
actions separately.
