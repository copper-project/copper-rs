# Self-describing logs

Run `just` to record ten wheel samples in `logs/wheel.copper`, inspect the catalog,
and validate the payloads. Run `just check` for the startup integration tests.

The application keeps its payloads in `src/payloads.rs` and enables
`cu29/self-describing-logs`. Its ordinary `Encode` derives supply the descriptions.
The generated builder serializes and compresses the reachable schemas at startup,
then writes one static section beside application metadata before resources.
All compiled missions share that section and its schema graph; matching append
reuses it. Start, stop and recording iterations reuse the startup metadata. The recording loop uses the usual
native CopperList encoding.

```rust,ignore
let app = Application::builder()
    .with_log_path("logs/wheel.copper", Some(32 * 1024 * 1024))?
    .build()?;
```

```sh
just catalog --export-format ron > catalog.ron
just catalog --export-format json > catalog.json
just extract --export-format jsonl > samples.jsonl
just extract --export-format csv > samples.csv
just fsck
```

Each CopperList contains two captured wheel payloads; the sink retains its metadata.
The catalog carries storage units for distance, speed and time. Startup catalog
serialization and Heatshrink compression use bounded working memory and support
`no_std`. Offline readers allocate the decoded value trees.

See [self-describing logs](../../doc/self-describing-logs.md) for external payloads,
manual encoding recipes, format details, and registration-free Rust/Python readers.
