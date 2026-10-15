# cu-zenoh-bridge-demo

This demo starts two Copper apps that exchange ping/pong messages over Zenoh on three channels,
using different wire formats per channel (bincode, json, cbor).

## Run

Terminal 1:
```bash
cargo run -p cu-zenoh-bridge-demo --bin zenoh-pong -- --iterations 40 --instance-id 2
```

Terminal 2:
```bash
cargo run -p cu-zenoh-bridge-demo --bin zenoh-ping -- --iterations 20 --instance-id 1
```

`zenohd` is optional for local peer-to-peer runs, but you can start it for routed setups:
```bash
zenohd
```

You should see pong logs tagged with `pong-bincode`, `pong-json`, and `pong-cbor` in the ping app.

Logs are written under `examples/cu_zenoh_bridge_demo/logs/` by default. You can override the
path with `--log <path>`.

## Talk To The Demo From Python

The bridge sends and receives bare payloads, so a non-Copper process can take the place of the
ping app. Install `eclipse-zenoh`, start the pong app, then run:

```python
import json, time, zenoh

def main() -> None:
    with zenoh.open(zenoh.Config()) as session:
        # Keep a reference: the subscriber is undeclared when dropped.
        sub = session.declare_subscriber(
            "demo/pong/json",
            lambda sample: print("pong:", json.loads(sample.payload.to_bytes())),
        )
        for seq in range(10):
            session.put("demo/ping/json", json.dumps({"seq": seq, "note": f"python#{seq}"}))
            time.sleep(0.2)
        time.sleep(1.0)

if __name__ == "__main__":
    main()
```

Each ping is plain JSON matching `Ping { seq, note }`, and each reply prints as plain JSON matching
`Pong { seq, reply }`. The Copper metadata travels in the Zenoh attachment, which the script
ignores.

### Preserve a third-party measurement timestamp

Without an attachment, Copper stamps local arrival time. A third-party publisher can attach its
measurement time using the same standard-bincode metadata layout. It must already share a usable
execution epoch and time scale with the Copper receiver (v0 PTP adapters normalize to TAI), and
have assigned `subsystem_code` and `instance_id` values plus a meaningful source cycle identifier.
Do not use process uptime or unconverted `time.time_ns()` (UTC) as a synchronized ToV.

```python
def bincode_uint(value: int, bits: int) -> bytes:
    if not 0 <= value < (1 << bits):
        raise ValueError("integer outside field range")
    if value < 251:
        return bytes([value])
    for marker, width in [(251, 2), (252, 4), (253, 8)]:
        if value < (1 << (8 * width)):
            return bytes([marker]) + value.to_bytes(width, "little")
    raise ValueError("integer outside u64 range")


def measurement_attachment(subsystem: int, instance: int, cycle: int, tov_ns: int) -> bytes:
    return (
        bincode_uint(subsystem, 16)
        + bincode_uint(instance, 32)
        + bincode_uint(cycle, 64)
        + b"\x01"  # Tov::Time enum discriminant
        + bincode_uint(tov_ns, 64)
        + b"\x01"  # payload present (including a zero-byte bincode unit payload)
    )


# In the session loop, with measurement_tai_ns from your synchronized acquisition clock:
# session.put("demo/ping/json", json.dumps({"seq": seq, "note": "ptp"}),
#             attachment=measurement_attachment(subsystem, instance, seq, measurement_tai_ns))
```

`Tov::Range` uses discriminant `2` followed by start and end nanoseconds; `Tov::None` uses `0`
with no time fields. The metadata has no layout version byte. Peers must agree on this layout.

Both sides use Zenoh's default multicast discovery. On networks where multicast is blocked, run a
`zenohd` router and point every process at it, e.g. with
`{ mode: "client", connect: { endpoints: ["tcp/[::1]:7447"] } }` as `zenoh_config_json` in both demo
configs and `zenoh.Config.from_json5(...)` with the same string in Python.

## Validate The Strict Multi-Copper Config Layer

This example also includes `multi_copper.ron`, a strict umbrella config that models the ping and
pong apps as two explicit Copper subsystems connected through bridge channels.

```bash
just graph multi_copper.ron graph.svg
cargo run -p cu-zenoh-bridge-demo --bin validate-multi-config
```

## Inspect Recorded Bridge Provenance

After running both apps, inspect the recorded bridge RX provenance in the Copper logs:

```bash
cargo run -p cu-zenoh-bridge-demo --bin inspect-ping-provenance
cargo run -p cu-zenoh-bridge-demo --bin inspect-pong-provenance
```

The inspectors print bridge RX slots with the remote `{subsystem_code, instance_id, cl_id}` that
arrived over Zenoh and was persisted in the local CopperList log.
