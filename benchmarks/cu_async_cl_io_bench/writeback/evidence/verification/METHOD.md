# SD-card timing experiments

All trials ran on an x86-64 CachyOS host with kernel `7.3.6-3-cachyos`, 4096-byte pages, an ext4 SD partition and a Realtek `0bda:0306` USB reader at 5 Gbit/s. The reader exposes a 119.2 GiB device and does not identify the card model. Release builds use rustc 1.100.0. The default mmap flush flag is `MS_SYNC`, with the additional `mmap-fsync` feature off. Benchmark logs use fresh temporary directories on the card. Results go to host storage. The card runs one job at a time. Building and syscall tracing run separately.

| Revision | Commit |
| --- | --- |
| Base | `cfa498792e36bd84e5012c4b166c1815db8c4dfc` |
| PR 1 | `748be92278f` |
| PR 2 | `1f4659fc7f673ae793bb8342275325920aee8b33` |

## Application examples

The PID graph uses the measurement source, `GenericPIDTask` and output sink from `cu_reflect_demo` in a live generated runtime. The source varies within the PID cutoff and also emits a fixed 2048-byte sample array. Each run offers 30,000 CLs at 1 kHz. It uses two CL slots, 16 MiB slabs, 1 MiB sections and a keyframe every 1000 ticks. The camera graph runs `cu_image_aligner` with two 640×480 grayscale sources and the original alignment window, pooled image buffers and sink. Each run offers 300 CLs at 20 Hz with two slots, 64 MiB slabs, 8 MiB sections and the initial keyframe. The sink's original console output is redirected to the host. Timestamp-based alignment changes the number of images in a CL when one release starts late, so the camera graph's logged bytes per CL vary across runs.

Both examples run inline and with `cu29/async-cl-io`. The revision order rotates across three fresh repetitions. Pacing uses absolute release times from a single monotonic clock. A late tick runs immediately and the loop catches up to its original schedule. Tick duration covers `run_one_iteration`. Release lateness measures actual start minus scheduled start. A deadline miss means completion after the following scheduled release. Start-to-start jitter measures the absolute difference between consecutive starts and the nominal period, excluding the first tick of each run. Results pool observations within a configuration and also report each repetition. These are wall-clock measurements with the host scheduler. Every result file ends with counts from decoding the closed log, strict increasing CL IDs, keyframes belonging to recovered CLs and successful clean close.

Partition counters record sectors read and written across each complete example trial, including recovery and temporary file cleanup. A sector is 512 bytes. The partition can also see filesystem metadata traffic and writeback from a preceding trial. These counters cannot attribute individual card writes to a logger request.

The linear reference uses Linux `O_DIRECT` and one outstanding 1 MiB write at a time into a regular file on the same mount. Each of three fresh 2 GiB files is preallocated and synced before timing. Aligned writes cycle through 16 MiB of random data prepared in memory. The throughput denominator includes all writes and the final `fdatasync`. Per-write and 256 MiB-window timings are recorded separately. The benchmark deletes only its own temporary file after each trial. Rates in MB/s use decimal megabytes.

The exploratory 1 s variant changes only `MAX_AGE` in PR 2, with the size threshold fixed at 1 MiB. Its separate comparison alternates the two builds within three repetitions of each example and output mode.

## Fixed-payload and rollover trials

`profile/run.py` runs 1,200 offers of 750,000 payload bytes at 20 Hz. Normal trials cover two and eight slots in inline and async modes, with 256 MiB slabs and 8 MiB sections. Rollover trials use two slots, 4 MiB slabs, 1536 KiB CL sections and eight structured entries on each tick. The benchmark's printed section-size argument and its separate background-record counter do not reflect those two overlay settings. Each case runs base, PR 1 and PR 2 in fixed order for three repetitions. The 80-tick spatial passes run separately and instrument logical stores. Page high-water estimates count distinct dirty host pages including writes still in flight. Stored per-thread timing samples reach host storage after shutdown. No strace runs in the timing trials.

The `flush` span times the mmap adapter's request. Counts and requested bytes measure application writeback submissions, including repeated ranges. For each run the metrics include submission, record, flush and tick distributions, drain time, overlapping overruns, page coverage, request order and completion inversions. Standalone two-stream strace diagrams use their own 8 KiB slab geometry and exercise metadata publication, crossing a slab, cleanup and close. They illustrate affected ranges. The real-card runs measure their timing consequences.

## Reproduction

From the evidence directory, set `COPPER_REPO` to a Copper checkout containing the pinned commits. Set `CARD_DIR` to a writable directory on the mounted SD filesystem. Build with a separate Cargo target and build directory for each pinned revision. The three fixed-payload build inputs are checkouts at the listed commits.

```sh
for r in base pr1 pr2; do
  CARGO_TARGET_DIR="$PWD/cache/$r" CARGO_BUILD_BUILD_DIR="$PWD/cache/$r" \
    python3 profile/build.py "$PWD/checkouts/$r" "profile/$r"
done
python3 profile/run.py "$CARD_DIR" profile/sd-results
python3 profile/report.py profile/sd-results
python3 profile/jitter.py profile/sd-results
python3 profile/group.py profile/sd-results
python3 verification/build.py
python3 verification/run-image.py "$CARD_DIR" pid
python3 verification/run-image.py "$CARD_DIR" image
python3 verification/run-age.py "$CARD_DIR"
python3 verification/linear-write.py "$CARD_DIR"
python3 verification/bandwidth.py
python3 verification/report.py
python3 verification/plot-jitter.py
```
