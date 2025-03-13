## Apriltag detection

Allows to detect and get the pose out of CuImages.
They need to be 8bits and with high contrast (see the dynamic thresholding task).

### Configuration

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

### Input

- `CuImage` (required): The CuImage to detect the apriltag on. It should be 8bits and with high contrast.

### Output

- `Pose`: The pose of the detected apriltag in the camera frame of reference (4x4 matrix).


