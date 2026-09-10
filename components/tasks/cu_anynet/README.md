# cu-anynet

Three-stage stereo-depth inference using Candle, exposed as the Copper
`cu_anynet::AnyNetStereo` anytime task. The base stage publishes an initial depth
map; two refinement stages improve it as the runtime grants more execution time.

Connect a `cu_anynet::StereoPair` containing rectified `RGB3` images to the task.
Both images must have matching, nonzero dimensions divisible by 16.
The output, `cu_anynet::StereoDepth`, contains a depth map in meters, the stage
number, and the result quality.

Set these task configuration keys in `copperconfig.ron`:

- `focal_px`: positive camera focal length in pixels (required).
- `baseline_m`: positive stereo baseline in meters (required).
- `weights`: path to converted AnyNet safetensors weights. Omitting this uses
  random weights for smoke testing.
- `max_disp`: nonzero disparity search range divisible by 16 (default: 192).
- `device`: `"cpu"` (default) or `"cuda"`; CUDA requires the `cuda` crate feature.
- `normalization`: `"imagenet"` (default) or `"raw"`, matching the training data
  normalization of the checkpoint.
- `stage_snapshots`: retain separate stage results for offline comparison
  (default: false). Each retained result copies the depth buffer.

See the repository's `examples/cu_anytime_anynet` application for graph wiring
and the `components/tasks/cu_anynet/tools` directory for checkpoint conversion.

The model architecture is ported from AnyNet (Wang et al., ICRA 2019).
The upstream MIT notice is included in `LICENSE-AnyNet`.
