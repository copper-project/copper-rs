# cu29-unifiedlog

Unified binary logging primitives used by Copper.

This crate provides the core data structures and I/O abstractions for Copper's
task-data and text-log stream format. It can be used independently if you need
the same log container format in another project.

## Features

- `std` (default): enables memory-mapped file logging backend.
- `compact` (default): favors compact log layout.

