# A custom Rerun representation: a soft robot arm

Nine tendon lengths describe a three-section flexible arm. Rerun can plot those
numbers, but it cannot infer the arm's shape. This example implements Rerun's
`AsComponents` for a Copper payload to reconstruct the backbone, tendons,
orientation rings, section joints, and tip axes.

![The soft arm playing in the native Rerun viewer](preview.png)

## Run

From this directory:

```sh
just
```

This opens the matching Rerun 0.43 viewer, embedded in the executable. No separate
Rerun CLI or Copper debugger is needed. The 12-second gesture loops automatically;
press **Space** to pause or play, or scrub the `motion` timeline. Pink, green, and
blue curves are the three tendons; the thick curve is the backbone.

The first build includes the native viewer and can take a few minutes. A graphical
desktop is required to open it. To write the same recording without opening a window:

```sh
just save /tmp/soft-arm.rrd
```

## Adapt it to your type

1. Keep domain data in the payload. [`SoftArm`](src/lib.rs) stores three
   `TendonSection`s, each containing three `Length`s and a routing radius. It
   implements Copper's normal payload traits; no geometry is stored in messages.
2. Implement [`AsComponents`](src/representation.rs). Here the tendon lengths
   become constant-curvature sections, composed from base to tip. Native
   `LineStrips3D`, `Points3D`, and `Arrows3D` batches are combined on one entity.
   Replace this calculation with your own representation.
3. Log it directly, as [`main.rs`](src/main.rs) does:

   ```rust
   rec.log("world/soft_arm", &arm)?;
   ```

For Copper's debug API, attach the optional reflection attribute to the same type:

```rust
#[derive(Debug, Clone, Default, Serialize, Deserialize, Encode, Decode, Reflect)]
#[cfg_attr(feature = "rerun", reflect(@cu_rerun::representation::<Self, cu_rerun::views::Spatial3DView>()))]
pub struct SoftArm {
    pub sections: [TendonSection; 3],
}
```

The `rerun` feature enables `cu-rerun`, `cu29/reflect`, and the representation
module. The `viewer` feature adds the desktop viewer and demo binary. Both are on
by default here; `cargo check -p cu-rerun-soft-arm --no-default-features` builds
just the payload. See the complete wiring in [`Cargo.toml`](Cargo.toml).

In an application that uses this payload, enable the payload's `rerun` feature
and `cu29/rerun` in the replay/debug server. Copper discovers the attribute through
the existing reflected type, calls the same `AsComponents`, and sends the Rerun
components to Timetraveler. There is no second renderer or separate registration
list. For task state, attach it to the projected state type exposed through
`with_debug_state` / `debug_state_type_path`; see [cu-rerun](../../components/libs/cu_rerun).
This demo itself starts no Copper runtime or debug server.

The representation uses existing Rerun visualizers. A custom renderer would still
need to be installed in the receiving viewer; the attribute does not ship renderer
code. The test in `representation.rs` verifies that local logging and the debug
adapter encode identical component arrays.

## Geometry model

The three tendons are equally spaced around each section. Their mean length gives
the backbone length; their differences determine the bend direction and angle.
Section frames are composed along the arm, so bending a lower section moves every
section above it. Equal tendon lengths produce a straight section.

This follows Open Continuum Robotics' [tendon-driven constant-curvature model](https://www.opencontinuumrobotics.com/101/2022/12/09/tdcr-cc-model.html)
and [arc kinematics](https://www.opencontinuumrobotics.com/101/2022/12/02/cc-kinematics.html).
It assumes continuous tendon routing and no twist. It is a geometric illustration,
not a force solver: gravity, friction, tendon slack, and collisions are not modeled.
