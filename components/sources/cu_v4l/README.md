# Video4Linux capture driver for Copper (V4L2)

## Compatibility

It should work with any v4l device (USB, MIPI etc.).

## Usage

Add the driver like any other source in Copper:

```RON
    tasks: [
        (
            id: "src",
            type: "cu_v4l::V4l",
            params: {
                device: 0, // /dev/video0
                width: 3280,
                height: 2160,
                fps: 30,  // frames per second, fractions are not supported yet.
                fourcc: "NV12", // format of the image
                buffers: 4, // How many images copper is able to keep in memory
                timeout_ms: 500, // How long should we wait for a new image
            },
        ),
    ]
```

When you connect this driver to the rest of the system you need to use the `cu_sensor_payloads::CuImage` message type.

```RON
    cnx: [
        (src: "src",  dst: "dst",   msg: "cu_sensor_payloads::CuImage"),
    ],
```