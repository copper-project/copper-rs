# LibAFL + Copper prototype

Standalone trait/injection probe. Uses real `CuMsg<Imu>`, Bevy reflection, and
`RobotClock::mock`; the controller is a small model, not a generated Copper app.
No runtime or application trait changes. LibAFL is only a dependency here.

From the repository root:

```bash
CARGO_TARGET_DIR="$PWD/target/fuzz-design/libafl-probe-build" \
  cargo run --manifest-path doc/design/fuzzing-prototype/Cargo.toml
```

Tested with Rust 1.99.0 and pinned LibAFL 0.16.1. The first run downloads/builds
dependencies. A successful run prints `probe passed`; assertion failures exit
unsuccessfully. Builds and the generated `scenario.case` stay under root `target/`.
This is its own Cargo workspace, outside normal Copper workspace checks.

Read [src/main.rs](src/main.rs) in order:

| Code | Purpose |
| --- | --- |
| `Imu` / `Scenario` / `EngineInput` | User payload; shared scenario contract; private LibAFL adapter |
| `TemporalMutator` | LibAFL changes the hold duration and scalar replacement |
| `PropertyFeedback` | Saves logic failures even when the executor returns normally |
| `inject_scalar` / `run_scenario` | Local reflected injection; 10ms cycles; stale input must stop motors within two cycles |
| `main` | Runs LibAFL, saves/reloads a failing scenario, verifies the corrected model passes |

The planted bug accepts a present payload without checking its age. Mutation
uses a deterministic duration sweep; corpus feedback accepts every non-failing
candidate. There is no coverage guidance, shrinking, GUI, or campaign checkpoint
yet. Counts are diagnostic, not a discovery-speed benchmark. The final saved
case is replayed against both broken and corrected behavior.

`Scenario` represents shared data; it does not implement LibAFL traits. The
private `EngineInput` wrapper does. Production campaign/case formats must be
public and versioned; this probe's `Input::to_file` persistence is not that API.

`LANDING` comments identify proposed production homes (new crate names):

| Layer | Home |
| --- | --- |
| Public data/RPC contract | `core/cu29_fuzz_schema`; portable selectors, not resolved field indices |
| Open host engine / LibAFL internals | `core/cu29_fuzz`; depends on runtime, never the reverse |
| Generic injection access | Existing `cu29_runtime` / `cu29_derive`; no LibAFL dependency |
| CLI / worker API | Thin client + service around that engine |
| Paid GUI | `../cu-timetraveler`; authoring and investigation |
