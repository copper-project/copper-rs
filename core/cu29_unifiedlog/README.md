# cu29-unifiedlog

Unified binary logging primitives used by Copper.

This crate provides the core data structures and I/O abstractions for Copper's
task-data and text-log stream format. It can be used independently if you need
the same log container format in another project.

## Features

- `std` (default): enables memory-mapped file logging backend.
- `compact` (default): favors compact log layout.

## Format compatibility

Unified log format version 2 records CopperLists as `id` followed by `msgs`.
Their lifecycle `state` is runtime bookkeeping and is omitted from binary,
Serde, Python, and remote-debug output. A decoded list reconstructs
`BeingSerialized` internally; this value is not recorded history.

Version 1 logs require a reader built against the older layout. Updated readers
reject incompatible container versions rather than interpreting the state byte
as message data. Compressed metadata retains its existing byte format: timestamp
deltas and metadata backreferences use the selective `cu_bincode::Uleb128`
wrapper, while ordinary integers keep their existing bincode encoding.
