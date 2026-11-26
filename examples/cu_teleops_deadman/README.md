# Loopback deadman

Run the robot and twin on one host. Press Enter at the red button prompt.

```sh
cargo run -p cu-teleops-deadman --features demo
```

Add `-- --auto` to press once as soon as the twin sees the robot armed.

1. The twin sends a fresh permit while the button is released. The robot starts safe and arms only until 150 ms after the permit's observed robot tick. Silence lets the permit expire.
2. Pressing Enter sends a stamped stop on the UDP return link. The robot switches to safe in its next iteration after admission.
3. The twin reads that iteration over FEC. It checks that the preceding frame was armed and the returned frame contains both the press and the safe state.

The first interval starts when the console samples the press and ends at completion of the robot safety task. The second ends at the twin receiver's timestamp for that CopperList. These processes share a monotonic time base in this single-process example. Results include OS scheduling and socket latency. The example saves both Copper logs under `logs/`.

The graph and watchdog are in `copperconfig.ron` and `src/policy.rs`. The FEC link settings are in `fec.ron`.
