# cu29-unifiedlog

Unified binary logging primitives used by Copper.

This crate provides the core data structures and I/O abstractions for Copper's
task-data and text-log stream format. It can be used independently if you need
the same log container format in another project.

## Features

- `std` (default): enables memory-mapped file logging backend.
- `compact` (default): favors compact log layout.

## Encapsulation and application content

`UNIFIED_LOG_FORMAT_VERSION` is **2** (unreleased). Static application metadata
precedes a byte-addressed chain of rotating sections. Each section carries its
construction run, instance and numeric mission identity.
It versions only the file/section encapsulation and layout. **Never bump it for
changes to encoded section content**: CopperLists, payloads, keyframes, and codecs
are decoded by the logreader built for the exact application version that wrote
them. The encapsulation version cannot establish content compatibility.

CopperLists record `id` followed by `msgs`. Their lifecycle `state` is runtime
bookkeeping and is omitted from binary, Serde, Python, and remote-debug output.
A decoded list reconstructs `BeingSerialized` internally; this is not recorded
history. Compressed metadata retains its byte format with the selective
`cu_bincode::Uleb128` wrapper; ordinary integers retain their bincode encoding.

## Capacity and append

The mmap builder accepts `capacity(bytes, CapacityPolicy::{Grow, OverwriteOldest,
StopWhenFull})`; `rollover(bytes)` selects `OverwriteOldest`. SD/eMMC uses the same
policies within its partition. Slabs are backing allocations: a section can span
several files, and readers follow byte links across ring wrap.

Generated builders seal static `ApplicationMetadata` before resources or runtime
streams. Applications that enable self-description supply one compressed catalog
in the adjacent static section. Both bodies are compared when sharing or appending
to a logger. Append requires a clean close; mismatched metadata fails before any
write. Capacity policies may change on append, while retained storage cannot be
shrunk.

Independent streams use run ID zero and seal an empty metadata region when data
begins. Idle partial streams are sealed during reclamation and rebind on their
next write. A write in progress blocks reclamation. The allocator preallocates
64 concurrent section leases; one mmap section can span up to 64 backing files.
Choose backing allocation sizes appropriate to the largest section.

Native entries retain one encoding pass. Failed entries leave committed `used`
and the writer cursor unchanged. Inspection reads committed bytes of incomplete
logs; an open section or unclean main header prevents append.

Run `just metadata-rollover-check` from the workspace root to check both adapters,
reader discovery, generated lifecycle behavior, embedded compilation and replay.
