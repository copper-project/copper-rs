# cu-apriltag

## Overview

`cu-apriltag` is a Copper `tasks` crate.

## Interface

### Input

- `CuImage` (required): The CuImage to detect the apriltag on. It should be 8bits and with high contrast.

### Output

- `Pose`: The pose of the detected apriltag in the camera frame of reference (4x4 matrix).

## Configuration

In the Ron file you need to specify the tag family, the tag size and the camera intrinsics.:

```RON
    tasks: [
        (
            id: "pos",
            type: "cu_apriltag::AprilTags",
            config: {
            "tag_family": "tag16h5",
            "tag_size": 0.14,
            "fx": 1513.93,
            "fy": 1513.93,
            "cx": 946.84,
            "cy": 557.819,
            },
        ),
    ]
```

## Usage

Add the task type to your app registry and connect it in the mission graph.

## Compatibility

Supports `std`/`no_std` according to crate features and dependencies.

## Links

- Crate path: `components/tasks/cu_apriltag`
- docs.rs: <https://docs.rs/cu-apriltag>

## Additional Notes

### Apriltag detection

Allows to detect and get the pose out of CuImages.
They need to be 8bits and with high contrast (see the dynamic thresholding task).
