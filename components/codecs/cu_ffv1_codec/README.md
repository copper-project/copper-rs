# cu-ffv1-codec

FFV1 log codec for Copper image payloads.

The real backend is behind the crate feature `ffmpeg` so the workspace can still
build on hosts that do not have FFmpeg development libraries installed.
