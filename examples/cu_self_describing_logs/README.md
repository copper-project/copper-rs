# Embedded payload decode catalogs

Run `just` here to build a compressed catalog on the host, embed it as a Rust static
byte array, and record it alongside ten wheel samples in `logs/wheel.copper`.
Run `just check` to verify the complete build/startup/offline-read workflow.

The `payloads` crate is shared between the host build script and application.
`gen_cumsgs!("copperconfig.ron")` supplies `cumsgs::value_decode_catalog()` using
the generated slot order. The build script passes that catalog to
`cu29_build::catalog::write_value_decode_catalog`, which writes `catalog.rs` into
Cargo's `OUT_DIR`. The application includes that source and supplies
`VALUE_DECODE_CATALOG` through `.with_value_decode_catalog(...)`.

Each application construction copies the prepared bytes once into a closed
`UnifiedLogType::ValueDecodeCatalog` section. The recording loop retains the
native CopperList encoding. The catalog reader produces shared wire/schema
information without linking the original payload crate; the tests also check
native payload decoding through that loaded description.

Version 1 uses a 10-byte header: eight-byte magic `CUVDCAT\0`, followed by a
little-endian `u16` version. Bincode standard encoding and Brotli compression are
fixed by that version. The unified section's `used` field supplies the envelope
length. Offline decompression has a fixed 16 MiB limit.

## Compression measurement

The wheel catalog contains units, time, arrays, sequences, an enum, config and
repeated payload slots. Its uncompressed bincode body is **1,585 bytes**. Measured
on 2026-10-02, with the same 10-byte header added to each candidate:

| Compressor | Compressed body | Embedded envelope |
| --- | ---: | ---: |
| Brotli quality 11, window 24 | 543 bytes | **553 bytes** |
| Zstd level 22 | 649 bytes | 659 bytes |
| XZ preset 9 extreme | 684 bytes | 694 bytes |

Brotli gives the smallest result in this comparison. Compression runs only during
the host build. The catalog's layout byte differs between compact and flat
CopperList encoding, so the flat build can differ slightly in compressed size.

The next integration step supplies standalone CopperList metadata interpretation
and CLI/Python export of complete recorded CopperLists.
