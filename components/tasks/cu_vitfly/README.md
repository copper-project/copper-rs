# cu-vitfly

Standalone Rust inference for the pretrained ViTFly ViT+LSTM depth policy,
implemented with Candle. ROS and the original Python repository are not runtime
dependencies. On first use, the F32 Safetensors weights are downloaded from
Copper's CDN and cached under `weights/`; later runs reuse the local file.

## Tensor contract

The low-level API intentionally keeps the trained model's tensor contract:

- depth: `[1, 1, 60, 90]`, F32
- desired velocity: `[1, 1]`, F32
- attitude quaternion: `[1, 4]`, F32, scalar-first `[w, x, y, z]`
- recurrent input: optional hidden and cell tensors, each `[3, 128]`, or the
  typed state returned by the preceding call
- prediction: `[1, 3]`, F32

The low-level model remains available for parity work. For applications, the
crate also exports `VitFlyTask`, a redistributable Copper task with this standard
message contract:

- inputs: `cu_zed::ZedDepthMap`, `cu_ahrs::AhrsPose`, and
  `cu29::units::si::f32::Velocity`
- output: `cu_vitfly::VitFlyVelocity`, an XYZ array of unit-safe Copper
  velocities in `[forward, left, up]` order

The task resizes ZED depth in meters to 60x90 with bilinear filtering, normalizes
it using `max_depth_m` (12.5 m by default), replaces invalid samples with
`invalid_depth` (0.8 by default), converts AHRS Euler angles to the model's
scalar-first quaternion, and preserves the LSTM state across frames and Copper
keyframes.

Task configuration accepts `device` (`"cuda"` or `"cpu"`), `cuda_ordinal`,
`max_depth_m`, and `invalid_depth`. CPU is the default backend. Enable the `cuda` feature for CUDA inference;
CUDA builds default to the CUDA task device.

## Follow the port step by step

The parity test runs two deterministic frames. The second frame consumes the
first frame's three-layer LSTM state. For every frame it compares PyTorch and
Candle after patch merging, each attention residual, each convolutional FFN
residual, each layer normalization, both encoder outputs, pixel shuffle,
bilinear upsampling, decoder concatenation, decoder convolution and linear
projection, metadata concatenation, every LSTM hidden and cell state, and the
final three-value prediction.

Run the complete CPU validation with:

```text
just test
```

Run the standalone recurrent example with:

```text
just sample
```

The CPU backend uses Candle's native kernels. On this machine, compiling with
`RUSTFLAGS="-C target-cpu=native"` enables all locally available instructions,
including AVX-512, while leaving the crate portable by default.

To print the maximum absolute error at every traced layer while following the
port, run:

```text
cargo test -p cu-vitfly --release pytorch_layer_by_layer_cpu_parity -- --nocapture
```

Run the CUDA parity test with:

```text
just cuda-test
```

The sample can run on the same CUDA backend with:

```text
just cuda-sample
```

For a synchronized one-frame latency check, use the `benchmark` example with
the same optional feature and `--cuda` argument.

## Reproducing the embedded artifacts

The runtime downloads the released checkpoint automatically. Re-exporting is
only needed when the upstream checkpoint changes:

```text
python tools/export_from_pytorch.py --upstream ../vitfly
```

The exporter materializes the two spectral-normalized weights, writes the model
as F32 Safetensors, and regenerates the independent PyTorch parity fixtures.
