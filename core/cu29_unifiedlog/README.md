# cu29-unifiedlog

Unified binary logging primitives used by Copper.

This crate provides the core data structures and I/O abstractions for Copper's
task-data and text-log stream format. It can be used independently if you need
the same log container format in another project.

## Features

- `std` (default): enables memory-mapped file logging backend.
- `compact` (default): favors compact log layout.

## Encapsulation and application content

`UNIFIED_LOG_FORMAT_VERSION` is **1**, unchanged since Copper's original format.
It versions only the file/section encapsulation and layout. **Never bump it for
changes to encoded section content**: CopperLists, payloads, keyframes, and codecs
are decoded by the logreader built for the exact application version that wrote
them. The encapsulation version cannot establish content compatibility.

CopperLists record `id` followed by `msgs`. Their lifecycle `state` is runtime
bookkeeping and is omitted from binary, Serde, Python, and remote-debug output.
A decoded list reconstructs `BeingSerialized` internally; this is not recorded
history. Compressed metadata retains its byte format with the selective
`cu_bincode::Uleb128` wrapper; ordinary integers retain their bincode encoding.
