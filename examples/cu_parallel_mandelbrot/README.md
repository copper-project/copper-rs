# cu-parallel-mandelbrot

This example is a synthetic Copper graph built to exercise `PlannerKind::Pipeline` on a deterministic, compute-bound workload. The `parallel-rt` feature enables the executor, while the feature-gated `pipeline.ron` selects it.

It is not pretending Mandelbrot is a robotics task. The point is to isolate scheduler behavior from device IO, clocks, driver jitter, and transport noise while exercising concurrent stateless transforms between stateful source and output stages.

## What The Graph Does

- `src` emits one `(frame, stripe)` work item per CopperList
- `band_*` stateless tasks advance one stripe through the Mandelbrot iteration pipeline
- `frames` assembles completed stripes into a full `CuImage<Vec<u8>>`
- `image_drain` keeps the `log_only` mission terminal
- `viewer_sink` assembles and displays the live zoom directly in the `viewer_live` mission

The hot stripe payload is handle-backed, so the graph stresses the scheduler and compute stages instead of spending its time copying stripe buffers around. On x86-64, each band runtime-dispatches to an eight-lane AVX2/FMA kernel when the processor supports it and otherwise uses the scalar kernel.

The example also enables Copper's terminal monitor. When you run it in a real terminal, use:
- `2` or `DAG` for the graph view
- `3` or `LAT` for per-stage latency stats
- `4` for CopperList bandwidth
- `5` for memory pools
- `q` to quit

## Where To Tweak It

If you want to change parameters, start here:

- [copperconfig.ron](copperconfig.ron)
  This is the main control surface: image size, `stripe_rows`, frame count, zoom step, number of iteration bands, per-band work split, logging, monitor, and `copperlist_count`.
- [tasks.rs](src/tasks.rs)
  Source, compute bands, frame assembler, viewer sink, and the TUI status text all live here.
- [payloads.rs](src/payloads.rs)
  The handle-backed in-flight stripe payload lives here.
- [lib.rs](src/lib.rs)
  Mission runners, logger setup, and run summaries live here.
- [justfile](justfile)
  Entry points for serial, parallel, monitor, viewer, logreader, and graph rendering live here.

The most important knobs for packing more CPU are:
- the number of `band_*` tasks
- the work assigned to each band
- `logging.copperlist_count`
- `runtime.planner.config.max_in_flight` in `pipeline.ron`
- the source `pool_slots`

Those values need to move together. More in-flight CopperLists without a wider graph just creates idle workers. More graph stages without enough in-flight CopperLists leaves the pipeline empty.

## Running It

Run these from this example directory:

- `just parallel`
- `just monitor-parallel`
- `just serial`
- `just viewer-parallel`
- `just graph-view copperconfig.ron graph.svg log_only`
- `just logreader`
- `just fsck`

Use the justfile itself for the full recipe list and exact command lines.

## Profile-Guided Scheduling Fixture

Mandelbrot is the controlled PGS fixture. Its deterministic, compute-bound graph
makes prediction errors and scheduling regressions easy to reproduce; it is not
intended as representative robot performance evidence.

Run the complete naive-to-measured workflow from this directory:

1. `just pgs-baseline` records the existing serial placement.
2. `just pgs-optimize candidates=3` writes ranked plans, prepared configs,
   predictions, and SVGs under `target/pgs/`.
3. `just pgs-candidate candidate=1` rebuilds against
   `target/pgs/selected.config.ron`, records it, and renders observed SVGs.
4. `just pgs-measure candidates=1` compares predicted and measured runs and
   renders the measured schedule SVGs.

Every PGS input is an explicit CLI argument or file. The four Just recipes are
one-line aliases for the Rust executables.

## What To Watch

- Throughput: compare `just serial` vs `just parallel`
- Determinism: the assembler checks strict stripe order; the bands keep all mutable iteration state in each stripe and can process different CopperLists concurrently
- Monitor feedback: the DAG view shows per-stage status text while the latency view shows per-stage timing
- Logs: only completed frame images are logged; intermediate stripe traffic is intentionally not logged

Logs are written under [logs](logs/).
