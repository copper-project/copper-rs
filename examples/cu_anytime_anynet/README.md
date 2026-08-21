# AnyNet anytime stereo depth

This demo runs the first three stages of [AnyNet](https://github.com/mileyan/AnyNet)
as a Copper `CuAnytimeTask`. `anynet-fg` refines in the foreground and can be
recorded/resimulated; `anynet-bg` keeps the 5 Hz graph responsive while Candle
inference runs on a background worker.

The default command downloads and converts the pretrained KITTI checkpoint if
needed, then runs the foreground demo on a moving synthetic stereo pair. The
background variant uses the same checkpoint:

```sh
just
just bg
```

To test only the task-graph wiring, use random weights explicitly:

```sh
just smoketest
```

## What the viewer shows

The 2D panes colormap **disparity in pixels** (turbo, pinned to a shared
0–96 px range) rather than metric depth — the jet-style look of the AnyNet
paper's figures. This is deliberate: a road scene spreads disparity almost
uniformly over the colormap, while a linear 0.5–60 m depth range crams most of
the image into a few color steps and makes every stage look identical.

Refinement rewrites the published depth map in place, so by the time the sink
runs, only the stage the budget actually reached still exists. Both configs
therefore set `config: {"stage_snapshots": true}` on the `anynet` node, which
makes the task keep a detached copy of what each stage published, plus the
elapsed time at which it landed. The demo sends a blueprint laying those out
like the paper's architecture figure:

- Top row: `world/cam/image` and `stage/stage1..3` — the left frame, then the
  *same* frame's disparity after each stage, coarse to fine. Stage 1 comes from
  1/16-scale features and looks visibly blurry; each refinement sharpens it.
- `result/disparity` — the anytime result as the graph consumed it, and
  `world/cam/depth`, its metric back-projection in the 3D view (a `Pinhole`
  sits on `world/cam`).
- `delta/stage2..3` — the absolute per-pixel disparity correction each
  refinement applied, as a grayscale residual map (the paper's "Residual 2/3").
  Two consecutive disparity panes can look near-identical to the eye; this is
  where the refinement is pinpointed.
- `anytime/*` — stage reached, quality, the elapsed time at which each stage
  landed, and the total job latency from the message metadata. This is the
  latency-versus-quality trade-off the anytime policy is making.

### Measured quality: 3-px error against KITTI ground truth

The `quality` curve plots the configured per-stage labels (0.49/0.70/1.00),
not accuracy. To *measure* the improvement, give the viewer node the same
`data_dir` as the stereo node: when the directory contains KITTI's
`disp_occ_0/` ground truth, the viewer computes the benchmark's 3-pixel error
(share of labeled pixels off by more than 3 px *and* more than 5%) for every
stage snapshot and logs it under `anytime/error`, with per-stage rows in the
summary panel. Predictions are rescaled to native KITTI pixel units before
comparison, so the 3 px threshold keeps its standard meaning. Only `_10`
frames carry labels, so every other frame contributes points. Expect the
curves to drop stage by stage — with the mirror checkpoint below at 640×192,
typically ~30% → ~26% → ~15%.

### Watching the refinement happen

Every entity is stamped on two timelines. `copperlist` steps one finished
frame at a time and is what resim compares. `time` stamps each stage snapshot
at the instant it actually published inside the job. Select the `time`
timeline in Rerun and press play: the `anytime result` pane and the 3D view
show stage 1's coarse sketch partway through each frame, then visibly sharpen
as stages 2 and 3 land — the anytime behavior replayed live.

At 1× playback that replay is a blink: the whole refinement window spans a few
tens of milliseconds per frame. The viewer's `time_dilation` config (default
1.0) stretches every step of the `time` timeline; the demo config sets 20.0,
so pressing play shows each stage for a comfortable fraction of a second.
Dilated steps are capped at 1 s so the base-compute wait and the idle stretch
between jobs do not become multi-second freezes on playback — without the cap
the animation would visibly stall between frames. Alternatively, lower Rerun's
playback rate instead of dilating.

Dilation assumes stamps arrive in time order, which only the foreground binary
guarantees. In background mode a result lags the frame delivered beside it, so
leave `time_dilation` at 1.0 there: stamps then map to their true times and the
lagged stages still replay correctly.

Snapshots cost one depth copy per quantum, so `stage_snapshots` defaults to off
and production graphs should leave it off. Without it the stage panes fall back
to a latest-at view: each pane holds the most recent frame that *ended* at that
stage, which is a different frame per pane.

### Smoke mode has nothing to show

In `just smoketest`, every pane is a single flat color and the residuals are
exactly zero. That is correct, not a broken viewer. Random weights make the cost
volume uniform, so the disparity regression returns the midpoint of the search
range for every pixel: a constant 88 px, which is a constant 4.427 m at the
demo configs' KITTI focal length and baseline. All three stages produce that same constant,
so the stage panes are identical and `|d₂−d₁|` and `|d₃−d₂|` are 0.00 px.

The summary panel reports this directly — `disparity: flat at 88.0 px` and
`residual 1 to 2: max 0.00 px` — because a flat map, an all-zero residual,
and an empty pane all render as the same flat color. Load trained weights to see
the panes diverge; the stage-quality labels 0.49, 0.70, and 1.00 are policy
values, not measured accuracy.

### Making the budget bite

With the default 800 ms budget the job converges at roughly 150 ms on a modern
desktop CPU, so every frame reaches stage 3 and the stage and quality curves sit
flat at 3 and 1.00. That is the policy working, not failing: nothing forced it to
stop early.

Read the per-stage times off the `latency (ms)` view and set `time_budget_ms`
just under the stage you want to cut off. Measured on one desktop CPU at
640×192, where stage 1 landed at ~85 ms, stage 2 at ~110 ms, and stage 3 at
~150 ms:

| `time_budget_ms` | published stage |
| --- | --- |
| 90 | always 1 |
| 100 | mostly 1, sometimes 2 |
| 130 | always 2 |
| 800 (default) | always 3 |

These are machine-specific — take the numbers from your own latency view.

Each interactive run starts a fresh Rerun viewer and selects a free port when
the default port is already occupied. The installed viewer must match the
workspace's Rerun SDK version.

For headless runs, add `"rrd": "anynet.rrd"` to the viewer node's config; the
blueprint travels with the `.rrd`. The `anynet` node requires `focal_px` and
`baseline_m` — the demo configs ship KITTI's 721.5 and 0.54. The viewer node
accepts the same two keys (defaulting to those KITTI values) — both must match
the `anynet` node, since they convert the published metric depth back to the
disparity the panes colormap and calibrate the 3D view — plus `disp_max_px` (default 96.0,
the far end of the pinned disparity colormap) and `depth_max_m` (default 60.0,
the far end of the 3D view's depth colormap).
To use KITTI 2015, download the
[stereo 2015 data set](https://www.cvlibs.net/datasets/kitti/eval_scene_flow.php?benchmark=stereo)
(free registration required; `data_scene_flow.zip`, ~2 GB) and add
`config: {"data_dir": "/path/to/KITTI/training"}` to the stereo node, pointing
at the unpacked `training/` directory. It must contain matching `image_2/` and
`image_3/` files; frames are resized to 640×192. Give the viewer node the same `data_dir`
to enable the measured 3-px error curves (needs `disp_occ_0/` next to the image
directories). Because 1242-wide KITTI frames are resized to 640, set
`focal_px` to `721.5 × 640 / 1242 ≈ 371.7` on both the `anynet` and `viewer`
nodes if you want the 3D view metrically scaled; the disparity panes and error
metric are unaffected by this choice as long as the two nodes agree.

The pretrained-model link in the AnyNet README (Google Drive) is dead as of
2026. A working mirror lives in the
[Stereo-3D-Detection](https://github.com/AmrElsersy/Stereo-3D-Detection) fork's
Drive folder; its `kitti2015.tar` matches the reference architecture exactly.
Checkpoint conversion requires `torch`, `safetensors`, and `gdown` in the
active Python environment. Install those dependencies using your preferred
environment manager, then run:

```sh
just weights
```

This writes the converted checkpoint to
`target/cu_anytime_anynet/anynet-kitti.safetensors`. The normal `just`, `just
fg`, `just bg`, and `just cuda` recipes run this setup automatically when the
converted checkpoint is missing.

**Normalization caveat:** that mirror was trained on raw 0..255 pixels, not
the `/255` ImageNet normalization of the original AnyNet dataloader (its first
BatchNorm's running mean sits in the thousands). The `anynet` node's
`normalization` config selects the input scaling: `"imagenet"` (default,
matches the original protocol) or `"raw"` (required for this mirror). A
mismatch is not an error anywhere — every stage just regresses confidently
wrong disparities, with 3-px error pinned near 100%.

Add the `weights` and `normalization` keys to the `anynet` node's existing
`config` block, keeping the required calibration:

```ron
config: {
    "focal_px": 721.5,
    "baseline_m": 0.54,
    "stage_snapshots": true,
    "weights": "/path/to/copper-rs/target/cu_anytime_anynet/anynet-kitti.safetensors",
    "normalization": "raw",
},
```

A tighter `time_budget_ms` publishes earlier stages; for example, a
10 ms foreground budget normally publishes stage 1 only.

CUDA builds use Candle/cudarc:

```sh
just cuda
```

Jetson/aarch64 CUDA support must be verified on the target; CPU is the fallback.
Floating-point replay can differ across CPU and CUDA devices.

The background config intentionally sets `enable_task_logging: false`. A
background anytime task cannot freeze safely while a multi-second job is live,
so keyframes would fail. Consequently the background binary has Rerun output
but no unified Copper log or resimulation. The foreground binary supports the
normal unified-log and resim workflow. Its 8 MiB CopperList sections are sized
to hold the full synthetic stereo pair, the published depth map, and the three
stage snapshots used for replay at 640×192. Higher `width`/`height` on the
stereo node grows every copperlist with the square of the resolution, and one
copperlist outgrowing a section is a hard failure — scale `section_size_mib`
(and `slab_size_mib` together with the binary's matching `SLAB_SIZE`) with it.

The architecture is ported from AnyNet (Wang et al., ICRA 2019) under its MIT
license. The optional SPN sharpening stage and its custom CUDA operator are not
included.
