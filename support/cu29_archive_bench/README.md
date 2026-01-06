# cu29-archive-bench

Offline benchmark tool for Copper log archival evaluation. This initial version targets the
balancebot schema defined in `examples/cu_rp_balancebot/copperconfig.ron` and emits columnar
streams plus a `manifest.json` with zstd baselines.

## Requirements

- `cargo` and the repo toolchain
- `python3`
- `zli` on `PATH`: installation from [OpenZL](https://openzl.org/getting-started/quick-start/)

## Step 1: Generate Columnar Streams + Zstd Baselines

```bash
cargo run -p cu29-archive-bench -- \
  --log examples/cu_rp_balancebot/logs/balance_0.copper \
  --out-dir /tmp/cu29-archive-bench \
  --zstd-level 3,19
```

Outputs:
- `/tmp/cu29-archive-bench/structured/*.bin` and `/tmp/cu29-archive-bench/copperlist/*.bin`
- `/tmp/cu29-archive-bench/manifest.json`

Notes:
- Pass a slab (`balance_0.copper`) or base name (`balance.copper`); the tool normalizes the base.
- Raw slab sizes are inflated by preallocation, so compare OpenZL against section or columnar sizes.

## Step 2: OpenZL Untrained Evaluation

```bash
python3 support/cu29_archive_bench/scripts/openzl_eval.py \
  --base-dir /tmp/cu29-archive-bench
```

Outputs:
- `/tmp/cu29-archive-bench/openzl/` (compressed streams)
- `/tmp/cu29-archive-bench/openzl_results.json`

## Step 3: OpenZL Trained Evaluation

```bash
python3 support/cu29_archive_bench/scripts/openzl_train_eval.py \
  --base-dir /tmp/cu29-archive-bench
```

Outputs:
- `/tmp/cu29-archive-bench/openzl_trained/` (trained compressors)
- `/tmp/cu29-archive-bench/openzl_trained_compressed/` (compressed streams)
- `/tmp/cu29-archive-bench/openzl_trained_results.json`

Training notes:
- Per-stream training can overfit small inputs; this is an upper-bound indicator.
- If a profile is not trainable (ex: `le-i32`), the script falls back to untrained compression.

## Sample Results (balance_0.copper)

From a local run with `examples/cu_rp_balancebot/logs/balance_0.copper`:

- Section raw bytes: StructuredLogLine `10,989`, CopperList `49,044`
- Columnar raw total: `90,975` bytes
- Zstd19 total (columnar): `19,162` bytes
- OpenZL untrained total (columnar): `25,121` bytes (about 1.31x zstd19)
- OpenZL trained total (columnar): `10,666` bytes (about 0.56x zstd19)
- Fallback streams during training: 1 (`railpos.payload.ticks.i32`)

These numbers will vary with dataset size and content.
