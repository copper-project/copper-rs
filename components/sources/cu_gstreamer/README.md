# GStreamer Appsink Source for Copper

## Compatibility

It should be broad, any buffer you can send to a gstreamer Sink should be able to be handled on the Copper Side.

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
                "frame_mode": "fifo",  // optional: "fifo" (default) or "newest"
            },
        ),
     ],
```

### Frame mode and dropped-frame tracking

The optional `"frame_mode"` parameter controls how buffered frames are drained on each `process()` tick:

- `"fifo"` (default): delivers buffered frames in oldest-first order. When the ring buffer is full, incoming frames overwrite the oldest queued frame and increment the dropped-frame counter.
- `"newest"`: delivers the most recent buffered frame and clears any older queued frames, counting both ring overwrites and discarded stale frames as dropped.

Dropped frames can be inspected via `CuGStreamer::dropped_frames()` (and through `Reflect`). The counter resets on `start()` and is preserved across `stop()`. While the drop count is non-zero, `process()` also reports `dropped: <count>` in the output message metadata status.

When you connect this driver to the rest of the system you need to use the `cu_gstreamer::CuGstBuffer` message type.

```RON
    cnx: [
        (src: "src",  dst: "dst",   msg: "cu_gstreamer::CuGstBuffer"),
    ],
```

You can see a small test application in the `tests/` folder on github.

### RTSP credentials from the environment

The [RTSP example](examples/rtsp.rs) builds the camera URL from `CAMERA_URL`
(e.g. `rtsp://camera.local/live`), `CAMERA_USER`, and `CAMERA_PASSWORD`.
It sets `.save_effective_config(false)` to omit the configuration snapshot from
log metadata. Replay that depends on runtime overrides needs a separately supplied config.

With those variables exported, run from this component's directory:

```sh
cargo run --features gst --example rtsp
```
