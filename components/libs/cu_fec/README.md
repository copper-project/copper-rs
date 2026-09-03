# cu-fec

Real-time and one-way streams often cannot wait for a retransmission when a packet is
lost. Forward erasure correction (FEC) adds repair symbols to the stream so a receiver
can reconstruct missing source symbols from the packets that do arrive.

`cu-fec` provides transport-independent FEC for fixed-size symbols. It implements the
systematic sliding-window random linear code from [RFC 8681] over GF(2) and GF(2^8).
Unlike a block code, a sliding-window code can emit repairs continuously as new data
arrives, making it useful when recovery latency must remain bounded.

Typical uses include:

- telemetry and logs over one-way or intermittent radio links;
- live sensor, audio, or video data sent over lossy UDP;
- Wi-Fi or cellular streams where timely recovery matters more than retransmission;
- embedded and bare-metal systems that require fixed memory use.

The crate was developed within the [Copper] umbrella project, a robotics runtime that
needed deterministic log streaming over lossy links. It is deliberately standalone:
it has no Copper dependencies, types, or protocol assumptions and can be used by any
Rust project.

`cu-fec` is dependency-free, `no_std`, and allocator-free. Runtime configuration
selects the active symbol and window sizes, while const generic parameters impose
hard memory limits.

## Quick start

```rust
use cu_fec::{
    DensityThreshold, EncodingSymbolId, Field, RepairParameters, RlcConfig,
    RlcDecoder, RlcEncoder,
};

const SYMBOL_CAPACITY: usize = 1200;
const WINDOW_CAPACITY: usize = 64;
const EQUATION_CAPACITY: usize = 64;

let config = RlcConfig::new(4, 16, Field::Gf256)?;
let mut encoder = RlcEncoder::<SYMBOL_CAPACITY, WINDOW_CAPACITY>::new(
    config,
    EncodingSymbolId::new(0),
)?;
let mut decoder = RlcDecoder::<
    SYMBOL_CAPACITY,
    WINDOW_CAPACITY,
    EQUATION_CAPACITY,
>::new(config, 16)?;

let esi = encoder.push_source(&[1, 2, 3, 4])?;
let _ = decoder.receive_source(esi, &[1, 2, 3, 4])?;

let mut repair = [0_u8; SYMBOL_CAPACITY];
let id = encoder.encode_repair(
    RepairParameters::new(7, DensityThreshold::FULL),
    &mut repair[..config.symbol_size()],
)?;
let _ = decoder.receive_repair(id, &repair[..config.symbol_size()])?;
# Ok::<(), cu_fec::Error>(())
```

The codec operates on symbols only. Packet integrity, fragmentation, ADU framing,
pacing, transport, and authentication belong to the integrating protocol. RFC 8681
source and repair payload-ID helpers are included for interoperable framing.

FEC recovers erasures; it does not identify corrupted or malicious packets. Check
packet integrity first, then pass only valid source and repair symbols to the decoder.

The crate's original Rust code is Apache-2.0 licensed. The TinyMT32 implementation
is translated from the [reference C implementation in RFC 8682, Section 2.1] and
retains the IETF Trust's Revised BSD terms in `LICENSE-BSD-3-Clause`.

## Verification

From the repository root:

```text
cargo test -p cu-fec
cargo check -p cu-fec --no-default-features
cargo bench -p cu-fec --bench rlc
```

[Copper]: https://github.com/copper-project/copper-rs
[RFC 8681]: https://www.rfc-editor.org/rfc/rfc8681.html
[reference C implementation in RFC 8682, Section 2.1]: https://www.rfc-editor.org/rfc/rfc8682.html#section-2.1
