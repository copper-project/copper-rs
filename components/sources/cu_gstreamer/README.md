# GStreamer Appsink Source for Copper

## Overview

`cu-gstreamer` is a Copper `sources` crate.

## Interface

Publishes source payloads into the Copper graph on each runtime tick.

## Configuration

Use `config` keys and `resources` bindings for hardware/driver handles.

## Usage

Add the driver like any other source in Copper:

```RON
    tasks: [
        (
            id: "src",
            type: "cu_gstreamer::CuDefaultGStreamer", // the default is a pool of 8 images, you can define your own type to change this pool size.
            config: {   // a gstreamer pipeline example that takes a mjpeg webcam and converts it to NV12. IMPORTANT: the appsink has to be named "copper"
                "pipeline": "v4l2src device=/dev/video0 ! image/jpeg, width=1920, height=1080 ! jpegdec ! videoconvert ! video/x-raw, format=NV12 ! appsink name=copper",
                "caps": "video/x-raw, format=NV12, width=1920, height=1080",  // this is what the Copper source will accept, here we just match the pipeline we define.
            },
        ),
     ],
```

When you connect this driver to the rest of the system you need to use the `cu_gstreamer::CuGstBuffer` message type.

```RON
    cnx: [
        (src: "src",  dst: "dst",   msg: "cu_gstreamer::CuGstBuffer"),
    ],
```

You can see a small test application in the `tests/` folder on github.

## Compatibility

It should be broad, any buffer you can send to a gstreamer Sink should be able to be handled on the Copper Side.

## Links

- Crate path: `components/sources/cu_gstreamer`
- docs.rs: <https://docs.rs/cu-gstreamer>
