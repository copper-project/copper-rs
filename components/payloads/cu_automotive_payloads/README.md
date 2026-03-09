# cu_automotive_payloads

Zero-copy, deterministic payload types for the Copper automotive protocol stack.

This crate defines the shared message types used across CAN, ISO-TP, UDS, and SOME/IP layers. All types implement `CuMsgPayload` bounds (Clone, Debug, Encode/Decode, Serialize/Deserialize, Reflect) and are designed for the copper-rs hot path — no heap allocation, fixed-size buffers.

## Modules

| Module | Description |
|--------|-------------|
| `can` | CAN 2.0B / CAN FD frame types — `CanFrame`, `CanFdFrame`, `CanId`, `CanFlags`, `CanFrameBatch` |
| `isotp` | ISO 15765-2 (ISO-TP) transport types — `IsotpPdu` (4095-byte buffer), `FlowControlParams`, addressing modes |
| `uds` | ISO 14229 UDS diagnostic types — `UdsRequest`, `UdsResponse`, `UdsSessionType`, `Nrc`, `Did` |
| `someip` | AUTOSAR SOME/IP types — `SomeIpHeader`, `SomeIpMessage` (1400-byte payload), wire format, service discovery |

## Usage

```toml
[dependencies]
cu-automotive-payloads = { path = "../../components/payloads/cu_automotive_payloads" }
```

```rust
use cu_automotive_payloads::can::{CanFrame, CanId};
use cu_automotive_payloads::isotp::IsotpPdu;
use cu_automotive_payloads::uds::{UdsRequest, UdsResponse};
use cu_automotive_payloads::someip::SomeIpMessage;
```

## Design Principles

- **Fixed-size buffers** — All payloads use `[u8; N]` arrays with a length field, no `Vec<u8>`.
- **Copy semantics** — Most payloads derive `Copy` for zero-cost CopperList transfer.
- **serde-big-array** — Arrays > 32 bytes use `#[serde(with = "BigArray")]` for serialization.
- **Wire format fidelity** — `to_bytes()`/`from_bytes()` methods match the on-wire protocol encoding.
