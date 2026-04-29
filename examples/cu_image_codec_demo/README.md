# cu-image-codec-demo

This example generates one synthetic RGB image stream and logs it twice:

- once through the PNG log codec
- once through the FFV1 log codec
- using pooled host image buffers so the source behaves like a real Copper image producer

That keeps image codec experiments separate from viewer/debugger tooling,
rather than mixing them with log codec benchmarking.

## Run

```bash
just run
just fsck
just log-stats
```
