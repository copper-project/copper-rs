# BalanceBot: this is a full Copper demo robot

with:

- a physical robot implementation
- a simulation implementation
- a resimulation demoing the deterministic replay
- a log export

## To run the simulation

```bash
$ cd examples/cu_rp_balancebot
$ cargo run --release 
```

See the UI help for the navigation.

To debug the game engine side you can add a perf overlay with:

```bash
cargo run --release --features perf-ui
```

## To run the resimulation

(you need at least a log in `logs` for example from a simulation run).

```bash
$ cd examples/cu_rp_balancebot
$ cargo run --bin balancebot_resim --release
```

It will recreate the logs from only the inputs of the previous run in `logs/balancebot_resim*.copper`.

## To run on the real robot

You will need to cross compile for Arm:

```bash
cargo build --target armv7-unknown-linux-musleabihf --release --no-default-features
```

Be sure you save your log string index:

```bash
cp -rv ../../target/armv7-unknown-linux-musleabihf/release/cu29_log_index .  # or anywhere you want
```

Deploy on the target:

```bash
scp ../../target/armv7-unknown-linux-musleabihf/release/balancebot copperconfig.ron copper7:copper/  # change to match your target
```

## To export logs

```bash
$ cd examples/cu_rp_balancebot
$ cargo run --bin balancebot-logreader --release
```

