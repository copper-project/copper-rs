# cu-image-aligner

## Overview

This example demonstrates how to align two image streams using `cu-aligner` while keeping image data in pooled buffers (`CuHostMemoryPool<Vec<u8>>`). It generates synthetic grayscale images, stamps them with `tov`, aligns them in time, and prints a small summary from the aligned window.

### What It Does

- Creates two source tasks that allocate image buffers from a host pool.
- Fills each buffer with a deterministic pattern based on a per-source base value and sequence.
- Uses `cu-aligner` to align the two `CuImage<Vec<u8>>` streams.
- Emits a summary line showing aligned window sizes and a sample pixel.

## Prerequisites

- Rust stable toolchain
- Any hardware or services referenced by this example

## Run

From the workspace root:

```bash
cargo run -p cu-image-aligner
```

The run is bounded by `ITERATIONS` in `examples/cu_image_aligner/src/main.rs`.

## Expected Output

### Output

You should see output similar to:

```
Aligned window: left_len=2 right_len=1 left_seq=0 left_pixel0=10 right_seq=0 right_pixel0=200
Aligned window: left_len=4 right_len=4 left_seq=3 left_pixel0=13 right_seq=3 right_pixel0=203
```

Meaning of each field:
- `left_len` / `right_len`: how many frames from each stream fell inside the current alignment window (capped by `OUTPUT_CAP`, currently 4).
- `left_seq` / `right_seq`: the sequence number of the *first* frame in each aligned window slice.
- `left_pixel0` / `right_pixel0`: the first pixel value (index 0) from those first frames. These values reflect the synthetic pattern seeded by `base_value` (10 for left, 200 for right) and are a quick sanity check for content.

`left_len` and `right_len` grow until they reach `OUTPUT_CAP`.

## Links

- Example path: `examples/cu_image_aligner`
- Build/deploy guide: <https://copper-project.github.io/copper-rs/Build-and-Deploy-a-Copper-Application>

## Additional Notes

All wiring lives in `examples/cu_image_aligner/copperconfig.ron`.

Source config keys:
- `width`, `height`: image dimensions
- `base_value`: starting pixel value for the synthetic pattern
- `pool_id`: name for the host memory pool
- `pool_size`: number of pooled buffers (must be greater than `BUFFER_CAP` in `tasks.rs`)
- `tov_offset_ms`: offset applied to `tov` in milliseconds

Aligner config keys:
- `target_alignment_window_ms`: size of the alignment window
- `stale_data_horizon_ms`: how far back to retain data

### Tuning

- Adjust `BUFFER_CAP` and `OUTPUT_CAP` in `examples/cu_image_aligner/src/tasks.rs`.
- Increase `pool_size` if you see buffer acquisition failures.
- Increase `ITERATIONS` in `examples/cu_image_aligner/src/main.rs` for longer runs.

### Troubleshooting

- `pool_size must be greater than buffer capacity`: bump `pool_size` above `BUFFER_CAP`.
- If you see logger shutdown warnings, they are emitted during log teardown and can be ignored for this example.
