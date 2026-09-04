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

## What recovery to expect

A received repair symbol contributes at most one independent equation. If `L` source
symbols are missing, the decoder needs at least `L` innovative repair symbols whose
windows cover those losses. Duplicate repairs, repairs lost in transit, and repairs
whose windows do not cover a missing source do not help recover it.

For repairs covering the same source window, these are useful rules of thumb:

| Coding parameters | Received repairs needed for `L` losses | Practical expectation |
| --- | ---: | --- |
| GF(2^8), `DensityThreshold::FULL` | `L` | About 99.6% full-rank probability; there is no loss margin. |
| GF(2^8), `DensityThreshold::FULL` | `L + 1` | Above 99.998% full-rank probability, assuming distinct repair keys. This is the recommended baseline. |
| GF(2), threshold `7` (about 50% density) | `L + 4` | About 94% full-rank probability for moderate `L`. |
| GF(2), threshold `7` | `L + 8` | About 99.6% full-rank probability for moderate `L`, at the cost of substantially more redundancy. |
| GF(2), `DensityThreshold::FULL` | — | Every coefficient is one. Repairs for an identical window repeat the same equation, so this normally handles only one erasure in that window. |

The probabilities are random-matrix approximations, not delivery guarantees. RFC
8681 derives coefficients deterministically from the repair key, so use a different
key for each repair. Sliding windows overlap rather than forming perfect blocks;
packet timing and window coverage therefore matter as much as the raw repair count.

For example, suppose 32 source symbols are each carried by one UDP datagram and the
receiver gets four distinct GF(2^8), full-density repairs covering all 32 symbols:

- three missing source datagrams should be recoverable with one repair to spare;
- four missing source datagrams are usually recoverable, but have no margin if a
  repair is lost or dependent;
- five missing source datagrams cannot be recovered from those four repairs;
- if two repair datagrams are also lost, only two source losses can be corrected.

## Starting profiles

These are deliberately conservative starting points. Measure the actual link and
adjust them using recorded loss bursts rather than relying only on average loss.

| Link and loss pattern | Symbol size | Window | Repair cadence | Field and density | Design target |
| --- | ---: | ---: | ---: | --- | --- |
| Ethernet/Wi-Fi, below about 2% loss | 1,024–1,200 bytes | 64 | 1 repair per 16 sources (6.25%) | GF(2^8), full | Isolated one- or two-packet losses. |
| UDP telemetry, around 5% loss with short bursts | 1,024–1,200 bytes | 64 | 1 per 8 (12.5%) | GF(2^8), full | Bursts of roughly three to five packets. |
| Intermittent radio, around 10% loss or longer bursts | 256–512 bytes | 64–96 | 1 per 4 (25%) | GF(2^8), full | Bursts of roughly eight to twelve packets, after trace-based validation. |
| CPU-constrained link where extra bandwidth is acceptable | 256–512 bytes | 32–64 | Start at 1 per 4 | GF(2), threshold `7` | Keep four to eight more received repairs than losses and validate the exact workload. |

As a concrete telemetry example, at 100 source datagrams per second a 64-symbol
window retains about 640 ms of data. Sending one repair after every eight sources
adds 12.5% FEC overhead and produces a repair every 80 ms. Recovering a three-packet
burst with one spare repair may require up to four covering repairs, or roughly
320 ms when no earlier repair is useful, which fits inside that window.

## Choosing the parameters

- `symbol_size`: If one symbol maps to one UDP datagram, start around 1,200 bytes on
  a normal Ethernet path so the symbol, FEC payload ID, application framing, UDP,
  and IP headers remain below the path MTU. Radios often work better at 256–512
  bytes. Avoid IP fragmentation.
- `window_symbols`: This is the protection horizon. At `P` source packets per
  second, a window of `W` symbols retains approximately `W / P` seconds. It must
  cover the longest loss burst plus the time needed to receive enough repairs.
- Repair cadence: One repair every `N` source symbols adds `100 / N` percent FEC
  traffic before transport headers. Redundancy must exceed the observed loss rate
  with room for burstiness and lost repair packets.
- `Field::Gf256` with `DensityThreshold::FULL`: Use this first. It gives much more
  reliable rank with little repair-count overhead.
- `Field::Gf2`: Consider it when encode CPU matters more than bandwidth. Threshold
  `7` makes each coefficient nonzero with probability 8/16. For thresholds `0..=14`,
  the nonzero probability is `(threshold + 1) / 16`; very sparse settings need
  careful workload-specific testing.
- Equation capacity: Set the active and const-generic equation capacities above the
  largest simultaneous loss count you intend to recover. For a five-loss target,
  eight is a reasonable GF(2^8) starting point; use more if repairs can accumulate
  across a larger unresolved window.

Const-generic capacities determine the allocated object size even when runtime
limits are smaller. Encoder storage is approximately
`MAX_SYMBOL_SIZE * MAX_WINDOW_SYMBOLS`. The decoder additionally stores about
`MAX_EQUATIONS * (MAX_WINDOW_SYMBOLS + MAX_SYMBOL_SIZE)` bytes of equation data.
Encoding work grows approximately with the active `symbol_size * window_symbols`,
so increasing the window is not free. The included benchmark prints exact encoder
and decoder sizes for its selected capacities.

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
