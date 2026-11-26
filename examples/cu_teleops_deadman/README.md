# Loopback deadman

Run the robot and twin on one host. Press Enter at the red button prompt.

```sh
cargo run -p cu-teleops-deadman --features demo
```

Add `-- --auto` to press once as soon as the twin sees the robot armed.

1. The twin samples the button state once per observed robot CopperList and sends a permit while it is released. The robot source drops input more than 32 iterations old. The robot starts safe and arms only until 150 ms after the permit's observed robot tick. Silence lets the permit expire.
2. Pressing Enter sends a stamped stop on the UDP return link. The robot switches to safe in its next iteration after admission.
3. The twin waits for the first FEC frame that confirms this press. The robot records the press timestamp and its armed-to-safe transition time, then repeats that state in subsequent frames. A watchdog expiry cannot satisfy the measurement.

The first interval starts when the console samples the press and ends when the robot task trips. The second ends at the twin receiver’s timestamp for the first matching confirmation it delivers. These processes share a monotonic time base in this single-process example. Results include OS scheduling and socket latency. The example saves both Copper logs under `logs/`.

The graph and watchdog are in `copperconfig.ron` and `src/policy.rs`. The FEC link settings are in `fec.ron`.
