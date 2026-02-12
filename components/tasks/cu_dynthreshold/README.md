# cu-dynthreshold

## Overview

### Theory of operation

The task computes a dynamic threshold for an image using the mean-based adaptive thresholding algorithm.

The algorithm computes the threshold for each pixel based on the mean pixel values in a
block around it.

The block radius is a parameter of the algorithm and is set in the configuration.

## Interface

Consumes input payload(s) and emits transformed output payload(s) each process cycle.

## Configuration

In the ron config file:

Add the parameters of the expected image size and the block radius for the thresholding algorithm.

```RON
(
    tasks: [
        (
            id: "thres",
            type: "cu_dynthreshold::DynThreshold",
            config: {"width": 1920, "height": 1080, "block_radius": 100}
        ),
    ]
)

```

As an input it takes a gstreamer buffer as a grey image 8 bits per pixel.
For example the beginning of an NV12 buffer works directly (provided you give it the correct width and height in the
config).

As an output it will produce a CuImage with 0 or 255 values for each pixel.

## Usage

Add the task type to your app registry and connect it in the mission graph.

## Compatibility

Supports `std`/`no_std` according to crate features and dependencies.

## Links

- Crate path: `components/tasks/cu_dynthreshold`
- docs.rs: <https://docs.rs/cu-dynthreshold>
