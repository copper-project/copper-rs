# Release notes

## 1.3.1

This patch fixes runtime shutdown, concurrent logging, image payloads, and project scaffolding while preserving Copper 1.3 APIs, configuration, and v2 log encapsulation.

### Fixes

- Failed or panicking iterations release their unfinished CopperList before shutdown. Completed output still drains, partial output stays unpublished, and the application can restart. ([#1531](https://github.com/copper-project/copper-rs/pull/1531), [#1533](https://github.com/copper-project/copper-rs/pull/1533))
- Concurrent log sections retain their ownership generations during allocation and rollover. Free capacity remains available while another encoder runs, and stale streams rebind without treating an encoding error as a revoked section. This includes the logger fix already merged into the release branch. ([#1532](https://github.com/copper-project/copper-rs/pull/1532), [#1535](https://github.com/copper-project/copper-rs/pull/1535))
- Shared-memory images implement `Clone` and `Default` without requiring the underlying storage to implement those traits; cloning retains the buffer handle. ([#1525](https://github.com/copper-project/copper-rs/pull/1525))
- Bevy's demo keeps the runtime on the main thread, including when asynchronous CopperList I/O is enabled. ([#1528](https://github.com/copper-project/copper-rs/pull/1528))
- The published workspace template includes its graph/schedule viewer helper. This fix is already available in `cargo-cunew` 1.3.1. ([#1505](https://github.com/copper-project/copper-rs/pull/1505))
- Lossless recording fixtures drain asynchronous output, Windows UDP integration fixtures use delivery settings suitable for CI, and RRT* satisfies newer Clippy checks. ([#1534](https://github.com/copper-project/copper-rs/pull/1534), [#1504](https://github.com/copper-project/copper-rs/pull/1504), [#1506](https://github.com/copper-project/copper-rs/pull/1506))

### Packages

The following packages advance to **1.3.1**: `cu29`, `cu29-runtime`, `cu29-derive`, `cu29-unifiedlog`, `cu-sensor-payloads`, `cu29-logstream-udp`, and `cu-rrt-star`. `cargo-cunew` 1.3.1 was already published. Other package versions remain unchanged. `cu29` signals this maintenance release without republishing every crate.

### Upgrading

Use `cu29 = "~1.3.1"`, then run `cargo update`. Update directly selected affected components to `~1.3.1` as needed. The generated runtime requires `cu29-runtime` and `cu29-derive` 1.3.1 or newer within the 1.3 line. Update applications, recording peers, and replay/export tools together when they share payload types. The 1.3 log encodings remain unchanged.
