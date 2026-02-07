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
cargo run -p cu-logviz-demo --bin cu-logviz-demo
# Visualize it in Rerun
cargo run -p cu-logviz-demo --bin cu-logviz-demo-logviz --features logviz -- logs/logviz_demo.copper --spawn

# Visualize it in Rerun with the custom frame tree (map/base_link/...)
cargo run -p cu-logviz-demo --bin cu-logviz-demo-logviz-custom --features logviz -- logs/logviz_demo.copper --spawn

# Or save to an .rrd file
cargo run -p cu-logviz-demo --bin cu-logviz-demo-logviz --features logviz -- logs/logviz_demo.copper --save out.rrd
```

You can override the output path by setting `LOGVIZ_DEMO_LOG`.
