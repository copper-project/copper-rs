# cu_isotp

ISO 15765-2 (ISO-TP) transport layer codec for the Copper runtime.

Implements segmentation and reassembly of multi-frame ISO-TP messages over CAN. Handles Single Frame, First Frame, Consecutive Frame, and Flow Control frame types with full state machine management.

## Task

| Task | Trait | Description |
|------|-------|-------------|
| `IsotpCodec` | `CuTask` | Bidirectional ISO-TP codec — reassembles RX, segments TX |

### I/O Types

- **Input:** `(CanFrame, IsotpPdu)` — CAN frame from network + ISO-TP PDU from upper layer
- **Output:** `(CanFrame, IsotpPdu)` — CAN frame to transmit + reassembled ISO-TP PDU to upper layer

## Configuration (RON)

```ron
(
    id: "isotp",
    type: "cu_isotp::IsotpCodec",
    config: {
        "tx_id": 0x641,
        "rx_id": 0x642,
        "block_size": 0,
        "st_min_ms": 10,
    },
),
```

## Protocol Features

- Single Frame (≤7 bytes) — immediate transfer
- First Frame + Consecutive Frames — segmented transfer for messages up to 4095 bytes
- Flow Control — block size and separation time management
- Normal and Extended addressing modes
