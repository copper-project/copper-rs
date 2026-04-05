# cu-logviz-demo

This example generates a short Copper log containing:
- `CuImage<Vec<u8>>`
- `PointCloudSoa<64>`
- `Transform3D<f32>`
- `ImuPayload`

Then you can visualize it via the app-specific logviz binary.

## Run

```bash
# Generate a log
just run

# Visualize it in Rerun
just logviz

# Visualize it in Rerun with the custom frame tree (map/base_link/...)
just logviz-custom

# Save the default view to an .rrd file
just save-rrd

# Inspect the log and export stats
just fsck
just log-stats

# Render the config DAG annotated with the measured log stats
just dag-logstats
```

All targets accept a custom log path, for example:

```bash
just run log=/tmp/logviz_demo.copper
just logviz log=/tmp/logviz_demo.copper
```
