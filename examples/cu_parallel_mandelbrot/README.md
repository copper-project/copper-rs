# cu-parallel-mandelbrot

This example is a synthetic Copper graph built to expose `parallel-rt` behavior on a deterministic, stateful, compute-bound workload.

It is not pretending Mandelbrot is a robotics task. The point is to isolate scheduler behavior from device IO, clocks, driver jitter, and transport noise while still exercising the same runtime property that matters in robot graphs: stateful stages must stay deterministic even when multiple CopperLists are in flight.

## What The Graph Does

- `src` emits one `(frame, stripe)` work item per CopperList
- `band_*` tasks advance one stripe through the Mandelbrot iteration pipeline
- `frames` assembles completed stripes into a full `CuImage<Vec<u8>>`
- `image_drain` keeps the `log_only` mission terminal
- `viewer_sink` assembles and displays the live zoom directly in the `viewer_live` mission

The hot stripe payload is handle-backed, so the graph stresses the scheduler and compute stages instead of spending its time copying stripe buffers around.

The example also enables Copper's terminal monitor. When you run it in a real terminal, use:
- `2` or `DAG` for the graph view
- `3` or `LAT` for per-stage latency stats
- `4` for CopperList bandwidth
- `5` for memory pools
- `q` to quit

## Where To Tweak It

If you want to change parameters, start here:

- [copperconfig.ron](/home/gbin/projects/copper/copper-rs.checkpoints/examples/cu_parallel_mandelbrot/copperconfig.ron)
  This is the main control surface: image size, `stripe_rows`, frame count, zoom step, number of iteration bands, per-band work split, logging, monitor, and `copperlist_count`.
- [tasks.rs](/home/gbin/projects/copper/copper-rs.checkpoints/examples/cu_parallel_mandelbrot/src/tasks.rs)
  Source, compute bands, frame assembler, viewer sink, and the TUI status text all live here.
- [payloads.rs](/home/gbin/projects/copper/copper-rs.checkpoints/examples/cu_parallel_mandelbrot/src/payloads.rs)
  The handle-backed in-flight stripe payload lives here.
- [lib.rs](/home/gbin/projects/copper/copper-rs.checkpoints/examples/cu_parallel_mandelbrot/src/lib.rs)
  Mission runners, logger setup, and run summaries live here.
- [justfile](/home/gbin/projects/copper/copper-rs.checkpoints/examples/cu_parallel_mandelbrot/justfile)
  Entry points for serial, parallel, monitor, viewer, logreader, and DAG rendering live here.

The most important knobs for packing more CPU are:
- the number of `band_*` tasks
- the work assigned to each band
- `logging.copperlist_count`
- the source `pool_slots`

Those values need to move together. More in-flight CopperLists without a wider graph just creates idle workers. More graph stages without enough in-flight CopperLists leaves the pipeline empty.

## Running It

Run these from [examples/cu_parallel_mandelbrot](/home/gbin/projects/copper/copper-rs.checkpoints/examples/cu_parallel_mandelbrot):

- `just parallel`
- `just monitor-parallel`
- `just serial`
- `just viewer-parallel`
- `just dag`
- `just logreader`
- `just fsck`

Use the justfile itself for the full recipe list and exact command lines.

## What To Watch

- Throughput: compare `just serial` vs `just parallel`
- Determinism: the band tasks and assembler check strict stripe order and fail if mutable state is observed out of order
- Monitor feedback: the DAG view shows per-stage status text while the latency view shows per-stage timing
- Logs: only completed frame images are logged; intermediate stripe traffic is intentionally not logged

Logs are written under [logs](/home/gbin/projects/copper/copper-rs.checkpoints/examples/cu_parallel_mandelbrot/logs).
