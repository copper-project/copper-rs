# cu29

## Overview

`cu29` is the main Copper prelude crate for building deterministic robotics applications.
It re-exports runtime, derive macros, logging, clock, and value crates so applications can depend on a single entrypoint.

## Usage

Add the crate to your application:

```bash
cargo add cu29
```

Then use Copper macros and task traits from the `cu29` prelude in your app crate.

## Compatibility

- Default feature set targets `std`.
- `defmt` supports embedded text logging.
- `logviz` enables log visualization helpers.
- `reflect` and `remote-debug` enable runtime reflection/remote debug paths.
- `rtsan` enables runtime thread-sanitizer support.

## Links

- docs.rs: <https://docs.rs/cu29>
- Runtime overview: <https://copper-project.github.io/copper-rs/Copper-Runtime-Overview>
- Workspace root: [`README.md`](../../README.md)
- Crate path: `core/cu29`
- docs.rs: <https://docs.rs/cu29>
