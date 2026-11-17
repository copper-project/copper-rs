# Copper Spatial payloads

A set for components for spatial representation like poses, transforms etc...
It is made to be used with the Copper framework (cu29-copper)

## Rerun geometry

Enable `rerun` to display unit-typed `Point2`/`Point3`, their SoA batches, and
axis-aligned `BBox` values in Copper's debugger or with `RecordingStream::log`.
Meter coordinates support both f32 and f64 units; 2D pixel coordinates support
u32 and i32. Other coordinate types remain reflectable without a custom view.

Generate a recording with `cargo run -p cu-spatial-payloads --features rerun
--example rerun_geometry -- geometry.rrd`, then open it with `rerun geometry.rrd`.

Enable `rerun` for visible X/Y/Z axes on `Transform3D<f32>` and
`Transform3D<f64>` (including the `Pose` aliases). The axes follow the existing
transform and use one-unit local vectors. Copper's debugger discovers the
representation through reflection; other scalar types retain reflection without
advertising a representation. With `rerun`, transforms reflect as structs with
hidden storage; without it they retain opaque reflection.

Record an example with `cargo run -p cu-spatial-payloads --features rerun --example rerun_pose -- poses.rrd`,
then open `poses.rrd` in Rerun.
