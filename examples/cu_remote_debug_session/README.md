# cu_remote_debug_session

This example demonstrates Copper's remote debug API over Zenoh by running a complete end-to-end session that exercises the main RPC endpoints.

It does two things in one run:

1. Records a deterministic Copper log from a small simulated pipeline.
2. Starts a remote debug server, connects a client, and runs API calls for session, navigation, timeline, schema, state, watch, health, and shutdown flows.

## What This Example Builds

The runtime graph is defined in `copperconfig.ron`:

- `CounterSrc`: emits `CounterMsg { value }` with an incrementing counter.
- `Accumulator`: consumes `CounterMsg`, maintains `sum`, emits `AccumMsg { sum }`.
- `SpySink`: consumes `AccumMsg` and stores the latest sum.

All tasks are configured with `run_in_sim: true` for deterministic replay.

## Files

- `src/main.rs`: full demo logic (log recording + remote debug server/client session).
- `copperconfig.ron`: runtime task graph and logging config.
- `Cargo.toml`: package and dependency setup (`cu29` with `reflect` and `remote-debug` features).
- `build.rs`: exports `LOG_INDEX_DIR` for Copper logging internals.

## Prerequisites

- Rust toolchain installed (`cargo`, `rustc`).
- Build from the workspace that contains this example (`copper-rs.remote_debug_api`).
- A writable `logs/` directory in the current working directory (tracked with `logs/.keep`).

## Run

Recommended (from this directory):

```bash
cd examples/cu_remote_debug_session
cargo run
```

If successful, output ends with:

```text
Remote debug API demo completed successfully (all endpoints exercised).
```

## What Gets Exercised

The client uses `RemoteDebugZenohClient` with `WireCodec::Cbor` and the server uses `RemoteDebugZenohServer` under:

`copper/examples/cu_remote_debug_session/debug/v1`

Session and control:

- `session.open`
- `session.capabilities`
- `session.cancel`
- `session.close`
- `admin.stop`

Navigation and replay:

- `nav.seek` (CL and timestamp targets)
- `nav.run_until`
- `nav.step`
- `nav.replay`

Timeline:

- `timeline.get_cursor`
- `timeline.get_cl`
- `timeline.list`

Schema and type info:

- `schema.get_stack`
- `schema.list_types`
- `schema.get_type`
- `schema.get_payload_map`

State inspection:

- `state.inspect`
- `state.read`
- `state.search`
- `state.watch.open`
- `state.watch.close`

Health:

- `health.ping`
- `health.stats`

## Generated Artifacts

Running the demo writes:

- `logs/cu_remote_debug_session_0.copper`: original recorded log.
- `logs/cu_remote_debug_session_replay_0.copper`: replay/debug session output log.

Older files with prefix `cu_remote_debug_session` are cleaned at start of each run.

## Notes On Behavior

- The demo records 128 iterations at 10 ms simulated increments.
- The replay callback injects recorded source output and lets runtime tasks update internal state.
- Errors from RPC calls are normalized through `call_ok(...)` and returned as `CuError`.

## Troubleshooting

- `No such file or directory ... logs/...copper`:
  restore the tracked `logs/` directory (for example, `git checkout -- logs/.keep`).
- No success line printed:
  inspect earlier stderr for the first failing RPC (`RPC <method> failed: ...`).
