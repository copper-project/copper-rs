# Production Automotive Stack on Copper-rs: Implementation Plan

> **Scope:** Full implementation of CAN (ISO 11898), UDS (ISO 14229), and SOME/IP (AUTOSAR) stacks
> as copper-rs crates, production-grade, targeting both Linux (x86-64/aarch64) and embedded
> bare-metal (ARM Cortex-M / RISC-V). Read `research.md` first for copper internals.

---

## Table of Contents

1. [Guiding Principles](#1-guiding-principles)
2. [Repository Structure](#2-repository-structure)
3. [Shared Infrastructure (Phase 0)](#3-shared-infrastructure-phase-0)
4. [CAN Stack (Phase 1)](#4-can-stack-phase-1)
5. [UDS Stack (Phase 2)](#5-uds-stack-phase-2)
6. [SOME/IP Stack (Phase 3)](#6-someip-stack-phase-3)
7. [Cross-Cutting Concerns](#7-cross-cutting-concerns)
8. [Embedded Deployment Strategy](#8-embedded-deployment-strategy)
9. [Testing and Validation Strategy](#9-testing-and-validation-strategy)
10. [Detailed Task Breakdown](#10-detailed-task-breakdown)

---

## 1. Guiding Principles

These are the **non-negotiable constraints** derived from copper's design philosophy:

### 1.1 Stay Within Copper's Paradigm

- Every protocol component is a **copper task, sink, source, or bridge** — never a raw thread
  managed outside the copper runtime
- All configuration lives in **RON config files** with `ComponentConfig` parameters
- All inter-task data flows through **CopperList** — no side channels (Mutex<>, channels, globals)
- Use the **Resources system** for hardware handles (SocketCAN fd, UDS transport socket, etc.)
- Background tasks (session managers, heartbeat senders) use copper's `background: true` mechanism

### 1.2 Zero Allocation in the Hot Path

- Preallocate frame buffers, PDU pools, session tables at `new()` time
- Use `heapless::Vec`, fixed-size arrays, or copper's SOA derive for frame batches
- Never call `Box::new()`, `Vec::push()` (unbounded), or `HashMap::insert()` in `process()`
- Use `#[derive(Soa)]` for bulk frame handling (e.g., CAN burst reads)

### 1.3 Copper Timestamp Semantics

- Set `msg.tov = Tov::Time(hardware_rx_timestamp)` — the hardware reception time, not process time
- The `process_time` in metadata is auto-filled by the runtime; do not touch it
- For request-response protocols (UDS), set tov on the response to the time the response was
  received from the ECU, not the time copper processed it

### 1.4 Payload Requirements

All message payload types must implement `CuMsgPayload`:
```rust
#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct CanFrame { ... }
```
This means: default-constructable (empty/invalid frame for pre-allocation), serializable, cloneable.

### 1.5 Error Handling

- Tasks return `CuResult<()>` — use `CuError::new("message")` and `.add_cause("context")`
- Fatal bus errors (bus-off, lost arbitration flood) should return `Err` to stop the runtime
- Recoverable frame errors (CRC, stuffing) should log via `CuLog` and return `Ok(())` with
  no payload set on the output message

### 1.6 Config-Driven Everything

Never hardcode:
- CAN interface name (`vcan0`, `/dev/can0`)
- CAN bitrate / FD data rate
- UDS target address, logical address, P2 timeout
- SOME/IP service ID, instance ID, port numbers

All go in `ComponentConfig` (RON config `config:` field of the node).

---

## 2. Repository Structure

Create a new workspace (or add to copper-rs workspace as a sub-workspace):

```
automotive/
  Cargo.toml                         ← workspace root

  # Shared foundation
  cu_automotive_base/                ← common types, CAN frame, PDU types, error codes
    src/
      lib.rs
      can_frame.rs                   ← CanFrame, CanFdFrame, CanFilter payloads
      pdu.rs                         ← PDU type for ISO 15765-2 / DoIP
      error_codes.rs                 ← automotive error codes (NRC, ...)
      addressing.rs                  ← CAN addressing modes, logical/physical addresses

  # CAN stack
  cu_can/                            ← core CAN source/sink tasks
    src/
      lib.rs
      source.rs                      ← CanSource (reads from socketcan or HW register)
      sink.rs                        ← CanSink (writes to socketcan or HW register)
      filter.rs                      ← CanFilter task (software frame filtering)
      mux.rs                         ← CanMux task (multi-channel multiplexer)
      decoder.rs                     ← CanSignalDecoder task (DBC/ARXML signal extraction)

  cu_can_fd/                         ← CAN FD extensions
    src/
      lib.rs
      source.rs                      ← CanFdSource
      sink.rs                        ← CanFdSink

  cu_can_nm/                         ← CAN Network Management (AUTOSAR NM / ISO 11898-6)
    src/
      lib.rs
      nm_task.rs                     ← background NM state machine task

  # ISO 15765-2: CAN Transport Layer
  cu_isotp/                          ← ISO-TP framing (segmentation / reassembly)
    src/
      lib.rs
      segmenter.rs                   ← CanFrame + PDU → IsotpFrame (TX path)
      reassembler.rs                 ← IsotpFrame → PDU (RX path, handles CF/FF/SF/FC)
      flow_control.rs                ← FC frame generation and BS/STmin enforcement

  # UDS stack
  cu_uds/                            ← ISO 14229-1 UDS services
    src/
      lib.rs
      transport.rs                   ← UDS transport (ISO-TP or DoIP wrapper tasks)
      dispatcher.rs                  ← UdsRequest → service routing task
      session.rs                     ← background session/security state machine
      services/
        read_data.rs                 ← 0x22 ReadDataByIdentifier
        write_data.rs                ← 0x2E WriteDataByIdentifier
        routine_control.rs           ← 0x31 RoutineControl
        ecu_reset.rs                 ← 0x11 EcuReset
        security_access.rs           ← 0x27 SecurityAccess
        diagnostic_session.rs        ← 0x10 DiagnosticSessionControl
        download_upload.rs           ← 0x34/0x35/0x36/0x37 transfer
        dtc.rs                       ← 0x19 ReadDTCInformation
        communication_control.rs     ← 0x28 CommunicationControl
      client.rs                      ← UDS client (tester) implementation
      server.rs                      ← UDS server (ECU) implementation

  cu_uds_doip/                       ← DoIP transport (ISO 13400) for UDS over Ethernet
    src/
      lib.rs
      source.rs                      ← DoIP Rx source task
      sink.rs                        ← DoIP Tx sink task

  # SOME/IP stack
  cu_someip/                         ← SOME/IP core
    src/
      lib.rs
      header.rs                      ← SOME/IP header serialization (16-byte fixed header)
      method.rs                      ← Method call (request/response) task pair
      event.rs                       ← Event publisher/subscriber tasks
      field.rs                       ← Field getter/setter/notifier tasks

  cu_someip_sd/                      ← SOME/IP Service Discovery
    src/
      lib.rs
      sd_task.rs                     ← background SD state machine
      registry.rs                    ← service/instance registry

  cu_someip_tp/                      ← SOME/IP-TP (fragmentation for large payloads)
    src/
      lib.rs

  # Example applications
  examples/
    can_loopback/                    ← CAN source → decoder → sink (vcan0 loopback)
    uds_tester/                      ← UDS client scanning an ECU
    uds_server/                      ← Simulated ECU with UDS services
    someip_service/                  ← SOME/IP service offering a method and an event
    someip_client/                   ← SOME/IP client consuming that service
    full_automotive_node/            ← CAN + UDS + SOME/IP all running together
```

---

## 3. Shared Infrastructure (Phase 0)

This phase must be complete before any protocol implementation begins.

### 3.1 cu_automotive_base: Core Types

#### CanFrame payload

```rust
#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct CanFrame {
    pub id: CanId,           // 11-bit or 29-bit (extended)
    pub dlc: u8,             // Data Length Code (0-8)
    pub data: [u8; 8],       // Payload (fixed-size — no alloc)
    pub flags: CanFlags,     // RTR, ERROR, EXTENDED flags
}

#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct CanFdFrame {
    pub id: CanId,
    pub dlc: u8,             // DLC maps to 0-64 bytes via CAN FD table
    pub data: [u8; 64],      // Max CAN FD payload
    pub flags: CanFdFlags,   // BRS, ESI flags
}

#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub enum CanId {
    #[default]
    Standard(u16),    // 11-bit, 0x000-0x7FF
    Extended(u32),    // 29-bit, 0x00000000-0x1FFFFFFF
}
```

**Important:** Fixed-size arrays (`[u8; 8]`, `[u8; 64]`) are essential for `Default` and no-alloc.

#### SOA for bulk CAN frames

```rust
#[derive(Soa)]
pub struct CanFrame { ... }
// Generates CanFrameSoa<N> for batched frame handling
```

Use `CuMsg<CanFrameSoa<64>>` for high-throughput buses (8000+ fps on 1 Mbit/s CAN).

#### PDU type for ISO-TP / DoIP

```rust
#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct Pdu {
    pub source_addr: u16,
    pub target_addr: u16,
    pub addressing_mode: AddressingMode,
    pub data: heapless::Vec<u8, 4096>,   // max UDS PDU size
}
```

`heapless::Vec` provides fixed-capacity allocation without heap; add `heapless` as dependency.

#### UDS NRC (Negative Response Code) type

```rust
#[derive(Default, Debug, Clone, Copy, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
#[repr(u8)]
pub enum Nrc {
    #[default]
    PositiveResponse = 0x00,
    GeneralReject = 0x10,
    ServiceNotSupported = 0x11,
    SubFunctionNotSupported = 0x12,
    IncorrectMessageLengthOrInvalidFormat = 0x13,
    ResponseTooLong = 0x14,
    BusyRepeatRequest = 0x21,
    ConditionsNotCorrect = 0x22,
    RequestSequenceError = 0x24,
    RequestOutOfRange = 0x31,
    SecurityAccessDenied = 0x33,
    InvalidKey = 0x35,
    ExceededNumberOfAttempts = 0x36,
    RequiredTimeDelayNotExpired = 0x37,
    UploadDownloadNotAccepted = 0x70,
    TransferDataSuspended = 0x71,
    GeneralProgrammingFailure = 0x72,
    WrongBlockSequenceCounter = 0x73,
    RequestCorrectlyReceivedResponsePending = 0x78,
    SubFunctionNotSupportedInActiveSession = 0x7E,
    ServiceNotSupportedInActiveSession = 0x7F,
}
```

---

## 4. CAN Stack (Phase 1)

### 4.1 CanSource Task

The CAN source reads frames from the OS (socketcan on Linux) or hardware registers (embedded).

```rust
pub struct CanSource {
    interface: String,       // from config: "vcan0", "can0"
    #[cfg(feature = "std")]
    socket: socketcan::CANSocket,
    filters: Vec<CanFilter>, // hardware filters configured at start
}

impl<'cl> CuSrcTask<'cl> for CanSource {
    type Output = CuMsg<'cl, CanFrame>;
    type Resources<'a> = ();

    fn new(config: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        let iface = config.get::<String>("interface")?.unwrap_or("can0");
        let bitrate = config.get::<u32>("bitrate_bps")?.unwrap_or(500_000);
        // open socketcan socket, configure bitrate, set non-blocking
    }

    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        // Non-blocking read attempt (or epoll wait with short timeout)
        // Store pending frame in self.pending
        Ok(())
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        if let Some(frame) = self.pending.take() {
            new_msg.tov = Tov::Time(self.hw_timestamp);  // use kernel SO_TIMESTAMPING
            new_msg.set_payload(frame);
        }
        Ok(())
    }
}
```

**Hardware timestamps:** Use Linux `SO_TIMESTAMPING` with `SOF_TIMESTAMPING_RX_HARDWARE` to
get the NIC-level receive timestamp. This is the correct `Tov` value.

**Embedded:** On bare-metal, the source reads from the CAN peripheral's receive FIFO (e.g.,
STM32 bxCAN FIFO0/FIFO1). The copper `Resources` system wraps the peripheral handle.

**Batching for high-throughput:** For 1 Mbit/s CAN with many frames per cycle, use:
```rust
type Output = CuMsg<'cl, CanFrameSoa<64>>;
// Read up to 64 frames per cycle in preprocess(), deliver batch in process()
```

### 4.2 CanSink Task

```rust
pub struct CanSink {
    #[cfg(feature = "std")]
    socket: socketcan::CANSocket,
}

impl<'cl> CuSinkTask<'cl> for CanSink {
    type Input = CuMsg<'cl, CanFrame>;
    type Resources<'a> = ();

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        if let Some(frame) = input.payload() {
            self.socket.write_frame(frame)?;
        }
        Ok(())
    }
}
```

### 4.3 CanFilter Task (software filtering)

```rust
// Regular task: filters CanFrame by ID range/mask
impl<'cl> CuTask<'cl> for CanFilter {
    type Input = CuMsg<'cl, CanFrame>;
    type Output = CuMsg<'cl, CanFrame>;

    fn process(&mut self, _clock: &RobotClock, input: Self::Input, output: Self::Output) -> CuResult<()> {
        if let Some(frame) = input.payload() {
            if self.passes(frame) {
                output.set_payload(frame.clone());
                output.tov = input.tov;  // preserve hardware timestamp
            }
        }
        Ok(())
    }
}
```

### 4.4 CanSignalDecoder Task (DBC-based)

```rust
// Decodes raw CanFrame into application-level signals
// DBC/ARXML is parsed at new() time and stored as a lookup table
pub struct CanSignalDecoder {
    db: candb::Database,  // loaded from DBC file path in config
}

impl<'cl> CuTask<'cl> for CanSignalDecoder {
    type Input = CuMsg<'cl, CanFrame>;
    type Output = CuMsg<'cl, DecodedSignals>;
    // process(): decode frame.data bytes into signal values per DBC definition
}
```

`DecodedSignals` is a fixed-size map: `[(SignalId, f64); MAX_SIGNALS]` (no HashMap in hot path).

### 4.5 CAN Network Management (background task)

CAN NM (AUTOSAR NM / ISO 11898-6) requires periodic NM frames and state machine management:

```rust
// Marked background: true in config
pub struct CanNmTask {
    state: NmState,  // BusSleep | PrepareBusSleep | Normal | Ready | Repeat...
    nm_timer: CuInstant,
}

impl<'cl> CuTask<'cl> for CanNmTask {
    type Input = CuMsg<'cl, CanFrame>;   // incoming NM frames
    type Output = CuMsg<'cl, CanFrame>;  // outgoing NM frames
    // process(): run NM state machine, produce NM frames on schedule
}
```

### 4.6 Example RON Config (CAN pipeline)

```ron
(
    tasks: [
        (
            id: "can_rx",
            type: "cu_can::CanSource",
            config: { "interface": "can0", "bitrate_bps": 500000 },
        ),
        (
            id: "can_filter",
            type: "cu_can::CanFilter",
            config: { "pass_ids": [0x100, 0x200, 0x7DF] },
        ),
        (
            id: "signal_decoder",
            type: "cu_can::CanSignalDecoder",
            config: { "dbc_path": "/etc/robot/vehicle.dbc" },
        ),
        (
            id: "app_logic",
            type: "myapp::VehicleControl",
        ),
        (
            id: "can_tx",
            type: "cu_can::CanSink",
            config: { "interface": "can0" },
        ),
    ],
    cnx: [
        (src: "can_rx",       dst: "can_filter",    msg: "cu_automotive_base::CanFrame"),
        (src: "can_filter",   dst: "signal_decoder", msg: "cu_automotive_base::CanFrame"),
        (src: "signal_decoder", dst: "app_logic",   msg: "cu_can::DecodedSignals"),
        (src: "app_logic",    dst: "can_tx",         msg: "cu_automotive_base::CanFrame"),
    ],
)
```

---

## 5. UDS Stack (Phase 2)

UDS requires Phase 1 (CAN ISO-TP) to be complete. Build in this order:
`cu_isotp` → `cu_uds` transport layer → `cu_uds` services → client/server

### 5.1 ISO-TP (ISO 15765-2) — cu_isotp

ISO-TP segments large PDUs (up to 4095 bytes) into CAN frames using:
- **SF** (Single Frame): PDU fits in one CAN frame
- **FF** (First Frame): start of multi-frame PDU
- **CF** (Consecutive Frame): continuation frames
- **FC** (Flow Control): receiver throttles sender

#### IsotpRx Task (reassembler — stateful)

```rust
pub struct IsotpRx {
    state: IsotpRxState,             // Idle | Receiving { expected_sn, buffer }
    rx_buffer: [u8; 4096],           // preallocated reassembly buffer
    rx_len: usize,
    target_addr: u8,                 // physical/functional address filter
    addressing_mode: AddressingMode, // Normal | Extended | Mixed
}

pub enum IsotpRxState {
    Idle,
    Receiving { expected_sn: u8, received: usize, total: usize },
}

impl<'cl> CuTask<'cl> for IsotpRx {
    type Input = CuMsg<'cl, CanFrame>;
    type Output = CuMsg<'cl, Pdu>;

    fn process(&mut self, clock: &RobotClock, input: Self::Input, output: Self::Output) -> CuResult<()> {
        if let Some(frame) = input.payload() {
            match self.classify_frame(frame) {
                FrameType::SF => {
                    // Direct output — single frame PDU
                    output.set_payload(self.decode_sf(frame)?);
                }
                FrameType::FF => {
                    // Start reassembly, send FC via separate output
                    self.start_reassembly(frame)?;
                }
                FrameType::CF => {
                    // Continue reassembly
                    if let Some(pdu) = self.continue_reassembly(frame)? {
                        output.set_payload(pdu);
                    }
                }
                FrameType::FC => {
                    // Not expected on RX path (this is for TX flow control)
                }
            }
        }
        Ok(())
    }
}
```

#### IsotpTx Task (segmenter)

```rust
pub struct IsotpTx {
    tx_buffer: [u8; 4096],
    stmin: CuDuration,   // minimum inter-frame time from FC
    block_size: u8,      // from FC
    state: IsotpTxState,
}

impl<'cl> CuTask<'cl> for IsotpTx {
    type Input = CuMsg<'cl, Pdu>;
    type Output = CuMsg<'cl, CanFrame>;
    // Segments PDU into SF/FF/CF frames, respects STmin and BS from FC
}
```

**Multi-frame handling:** ISO-TP TX is inherently multi-cycle (one CF per copper cycle, with
STmin enforcement). Use the `background: true` flag or a stateful task that emits one frame
per cycle and tracks remaining frames in its state.

### 5.2 UDS Session/Security Background Task

UDS sessions have timing requirements (P2, P2*, S3 timers) that must run continuously:

```rust
// background: true in config
pub struct UdsSession {
    session_type: SessionType,      // Default | Programming | Extended
    security_level: u8,             // 0 = locked, 1+ = unlocked level
    s3_timer: CuInstant,            // session keep-alive timer
    p2_timer: Option<CuInstant>,    // response pending timer
    seed: [u8; 4],                  // security access seed
}

impl<'cl> CuTask<'cl> for UdsSession {
    type Input = CuMsg<'cl, UdsRequest>;
    type Output = CuMsg<'cl, UdsSessionState>;
    // Manages session timeouts, transitions, security state
    // Emits current UdsSessionState every cycle for service tasks to observe
}
```

### 5.3 UDS Request/Response Message Types

```rust
#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct UdsRequest {
    pub service_id: u8,
    pub sub_function: Option<u8>,
    pub data: heapless::Vec<u8, 4095>,
    pub addr: u16,    // target ECU logical address
}

#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct UdsResponse {
    pub service_id: u8,    // positive: 0x40 + request SID; negative: 0x7F
    pub nrc: Option<Nrc>,  // None for positive responses
    pub data: heapless::Vec<u8, 4095>,
}
```

### 5.4 UDS Service Handlers (as copper tasks)

Each UDS service is a separate copper regular task:

```rust
// 0x22 ReadDataByIdentifier
pub struct ReadDataByIdentifier {
    data_store: [(u16, heapless::Vec<u8, 255>); MAX_DIDS],  // DID → value store
}

impl<'cl> CuTask<'cl> for ReadDataByIdentifier {
    type Input = input_msg!('cl, UdsRequest, UdsSessionState);
    type Output = CuMsg<'cl, UdsResponse>;

    fn process(&mut self, clock, input, output) -> CuResult<()> {
        let (req, session) = input;
        if let Some(req) = req.payload() {
            if req.service_id != 0x22 { return Ok(()); }
            if session.payload().map(|s| s.session_type) == Some(SessionType::Default) {
                // Check if DID allowed in default session
            }
            // Look up DID in data_store, build positive response
            output.set_payload(UdsResponse { ... });
        }
        Ok(())
    }
}
```

### 5.5 UDS RON Config Example

```ron
(
    tasks: [
        // CAN transport layer
        (id: "can_rx",   type: "cu_can::CanSource",   config: { "interface": "can0" }),
        (id: "isotp_rx", type: "cu_isotp::IsotpRx",   config: { "target_addr": "0x7E8" }),
        (id: "isotp_tx", type: "cu_isotp::IsotpTx",   config: { "source_addr": "0x7E8" }),
        (id: "can_tx",   type: "cu_can::CanSink",      config: { "interface": "can0" }),

        // UDS session management (background)
        (
            id: "uds_session",
            type: "cu_uds::UdsSession",
            background: true,
            config: { "p2_timeout_ms": 50, "s3_timeout_ms": 5000 },
        ),

        // UDS service handlers
        (id: "uds_rdbi",  type: "cu_uds::ReadDataByIdentifier"),
        (id: "uds_wdbi",  type: "cu_uds::WriteDataByIdentifier"),
        (id: "uds_rc",    type: "cu_uds::RoutineControl"),
        (id: "uds_reset", type: "cu_uds::EcuReset"),
        (id: "uds_sa",    type: "cu_uds::SecurityAccess"),
    ],
    cnx: [
        (src: "can_rx",      dst: "isotp_rx",   msg: "cu_automotive_base::CanFrame"),
        (src: "isotp_rx",    dst: "uds_rdbi",   msg: "cu_uds::UdsRequest"),
        (src: "isotp_rx",    dst: "uds_wdbi",   msg: "cu_uds::UdsRequest"),
        (src: "isotp_rx",    dst: "uds_rc",     msg: "cu_uds::UdsRequest"),
        (src: "uds_session", dst: "uds_rdbi",   msg: "cu_uds::UdsSessionState"),
        (src: "uds_session", dst: "uds_wdbi",   msg: "cu_uds::UdsSessionState"),
        (src: "uds_rdbi",    dst: "isotp_tx",   msg: "cu_uds::UdsResponse"),
        (src: "isotp_tx",    dst: "can_tx",     msg: "cu_automotive_base::CanFrame"),
    ],
)
```

---

## 6. SOME/IP Stack (Phase 3)

SOME/IP targets Ethernet-based automotive networks. Requires Ethernet sockets (std) or
raw Ethernet frames (embedded with MAC hardware). Build in this order:
`cu_someip` header + serialization → methods → events → fields → `cu_someip_sd`

### 6.1 SOME/IP Header

The fixed 16-byte SOME/IP header must be zero-copy serializable:

```rust
#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct SomeIpHeader {
    pub service_id: u16,
    pub method_id: u16,   // or event_id (MSB=1 for events)
    pub length: u32,      // total length of header + payload
    pub client_id: u16,
    pub session_id: u16,
    pub protocol_version: u8,  // always 0x01
    pub interface_version: u8,
    pub message_type: MessageType,  // REQUEST | REQUEST_NO_RETURN | NOTIFICATION | RESPONSE | ERROR
    pub return_code: ReturnCode,
}

#[derive(Default, Debug, Clone, Copy, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
#[repr(u8)]
pub enum MessageType {
    #[default]
    Request = 0x00,
    RequestNoReturn = 0x01,
    Notification = 0x02,
    Response = 0x80,
    Error = 0x81,
}
```

### 6.2 SOME/IP Message Payload Type

```rust
#[derive(Default, Debug, Clone, bincode::Encode, bincode::Decode, Serialize, Deserialize)]
pub struct SomeIpMessage {
    pub header: SomeIpHeader,
    pub payload: heapless::Vec<u8, 65519>,  // max SOME/IP payload (65535 - 16 header)
}
```

For production, consider a fixed-size payload enum keyed on (service_id, method_id) to avoid
even the heapless vec — generate per-service types from ARXML/FIDL.

### 6.3 SOME/IP Source/Sink Tasks

```rust
// Reads UDP datagrams and emits SOME/IP messages
pub struct SomeIpUdpSource {
    socket: std::net::UdpSocket,
    rx_buf: [u8; 65535],
}

impl<'cl> CuSrcTask<'cl> for SomeIpUdpSource {
    type Output = CuMsg<'cl, SomeIpMessage>;
    // preprocess: non-blocking recv_from into rx_buf
    // process: parse header + payload, set tov to kernel timestamp
}

// Sends SOME/IP messages over UDP
pub struct SomeIpUdpSink {
    socket: std::net::UdpSocket,
    remote_addr: SocketAddr,
}

impl<'cl> CuSinkTask<'cl> for SomeIpUdpSink {
    type Input = CuMsg<'cl, SomeIpMessage>;
    // process: serialize header + payload, sendto remote
}
```

### 6.4 SOME/IP Method Handler Task

```rust
// Generic method handler — one per service/method pair
pub struct SomeIpMethodHandler<Req, Resp>
where Req: SomeIpSerializable, Resp: SomeIpSerializable
{
    service_id: u16,
    method_id: u16,
    handler: Box<dyn Fn(&Req) -> CuResult<Resp> + Send>,  // registered at new()
}

impl<'cl, Req, Resp> CuTask<'cl> for SomeIpMethodHandler<Req, Resp> {
    type Input = CuMsg<'cl, SomeIpMessage>;   // raw SOME/IP message
    type Output = CuMsg<'cl, SomeIpMessage>;  // SOME/IP response

    fn process(&mut self, _clock, input, output) -> CuResult<()> {
        if let Some(msg) = input.payload() {
            if msg.header.service_id != self.service_id { return Ok(()); }
            if msg.header.method_id != self.method_id { return Ok(()); }
            let req: Req = Req::deserialize(&msg.payload)?;
            let resp = (self.handler)(&req)?;
            output.set_payload(SomeIpMessage {
                header: SomeIpHeader {
                    service_id: self.service_id,
                    method_id: self.method_id,
                    message_type: MessageType::Response,
                    session_id: msg.header.session_id,  // echo
                    ..Default::default()
                },
                payload: resp.serialize()?,
            });
        }
        Ok(())
    }
}
```

### 6.5 SOME/IP Service Discovery (background task)

SD is a broadcast protocol (UDP multicast 239.0.0.0:30490) requiring:
- Offer Service / Find Service / Subscribe EventGroup / SubscribeAck messages
- TTL-based service expiration
- State machine: Down → Ready → Initial Wait → Repetition → Main Phase

```rust
// background: true in config
pub struct SomeIpSdTask {
    offered_services: heapless::Vec<ServiceEntry, 32>,
    subscriptions: heapless::Vec<EventgroupSubscription, 64>,
    state: SdState,
    multicast_socket: std::net::UdpSocket,
}

impl<'cl> CuTask<'cl> for SomeIpSdTask {
    type Input = CuMsg<'cl, SomeIpMessage>;    // incoming SD messages
    type Output = CuMsg<'cl, SdRegistryUpdate>; // service availability updates
}
```

### 6.6 SOME/IP RON Config Example

```ron
(
    tasks: [
        (
            id: "someip_rx",
            type: "cu_someip::SomeIpUdpSource",
            config: {
                "bind_addr": "0.0.0.0:30501",
                "multicast_group": "239.0.0.1",
            },
        ),
        (
            id: "vehicle_speed_handler",
            type: "myapp::VehicleSpeedService",
            config: { "service_id": "0x0123", "method_id": "0x0001" },
        ),
        (
            id: "someip_tx",
            type: "cu_someip::SomeIpUdpSink",
            config: { "remote_addr": "10.0.0.1:30501" },
        ),
        (
            id: "someip_sd",
            type: "cu_someip_sd::SomeIpSdTask",
            background: true,
            config: {
                "multicast_addr": "239.0.0.0:30490",
                "offer_services": [{ "service_id": "0x0123", "instance_id": "0x0001" }],
            },
        ),
    ],
    cnx: [
        (src: "someip_rx",              dst: "vehicle_speed_handler", msg: "cu_someip::SomeIpMessage"),
        (src: "vehicle_speed_handler",  dst: "someip_tx",             msg: "cu_someip::SomeIpMessage"),
    ],
)
```

---

## 7. Cross-Cutting Concerns

### 7.1 Unified Diagnostic Monitor

Build a `cu_automotive_monitor` implementing `CuMonitor` that:
- Tracks CAN bus error counters (TEC/REC) each cycle
- Detects missing frames (timestamp gap > expected period)
- Reports UDS NRC error rates
- Integrates with `cu_safetymon` for fault escalation

### 7.2 DTC (Diagnostic Trouble Code) Management

DTCs span the entire stack. Build a shared `DtcStore` resource (accessed via Resources system):
```rust
pub struct DtcStore {
    entries: [(DtcCode, DtcStatus); MAX_DTCS],  // fixed-size, no alloc
    count: usize,
}
```
Register as a `ResourceBundle` so CAN NM, UDS 0x19, and safety monitors all share it.

### 7.3 Automotive Time Synchronization

For SOME/IP and multi-ECU systems, time synchronization is critical:
- **Linux**: Use PTP (IEEE 1588) via `linuxptp` / `phc2sys` — copper clock reads
  `CLOCK_TAI` or hardware PTP clock
- **Embedded**: Implement a `TimeSyncTask` that consumes SOME/IP time sync messages and
  updates `RobotClock` via the callback mechanism

### 7.4 Serialization for SOME/IP Payloads

AUTOSAR SOME/IP uses **SOME/IP serialization** (big-endian, no length prefixes for fixed types).
Build a `cu_someip_serde` crate with `#[derive(SomeIpSerialize, SomeIpDeserialize)]`:
- Fixed-size types: direct byte cast (big-endian)
- Variable-length strings/arrays: 4-byte length prefix
- This is distinct from copper's bincode used for the unified log

### 7.5 Code Generation from ARXML/FIDL

Long-term, build a `cu_gen_arxml` tool (separate binary) that:
- Reads AUTOSAR ARXML / FIBEX / VSS (Vehicle Signal Specification)
- Generates: Rust payload structs, DBC decoder tables, SOME/IP method handler stubs, RON config
- Outputs ready-to-compile copper task implementations

---

## 8. Embedded Deployment Strategy

### 8.1 Feature Flags Design

Each crate should have:
```toml
[features]
default = ["std"]
std = ["cu29/std", "socketcan", ...]
embedded = ["cu29/no_std", "embedded-can", ...]
```

### 8.2 HAL Abstraction for CAN

Use the `embedded-can` crate trait (`embedded_can::Frame`, `embedded_can::nb::Can`) as the
hardware abstraction layer:
```rust
// In CanSource, behind feature flag:
#[cfg(feature = "embedded")]
use embedded_can::nb::Can;

pub struct EmbeddedCanSource<C: Can> {
    can: C,
    // C is the MCU-specific CAN peripheral (bxCAN, FDCAN, etc.)
}
```

### 8.3 Supported Embedded Targets

| MCU | CAN Hardware | Notes |
|-----|-------------|-------|
| STM32F4/F7/H7 | bxCAN / FDCAN | via `stm32f4xx-hal`, `fdcan` crate |
| STM32G0/G4 | FDCAN | CAN FD support |
| RP2040 / RP2350 | MCP2515 (SPI) | External CAN controller |
| NXP S32K3xx | FlexCAN / CANFD | Automotive-grade MCU |
| Renesas RH850 | RSCAN | Future target |

### 8.4 Memory Budget (Embedded)

Typical allocation at startup for a minimal CAN+UDS node:

| Component | Memory |
|-----------|--------|
| CopperList ring buffer (N=2) | 2 × sizeof(P) |
| ISO-TP RX reassembly buffer | 4 KB |
| UDS service data store | 8 KB |
| Unified log (if std) | 10 MB |
| Task state total | ~16 KB |
| **Total (no_std)** | **~32 KB heap + flash** |

RP2350 has 512 KB flash and 520 KB SRAM — ample for CAN+UDS.

---

## 9. Testing and Validation Strategy

### 9.1 Unit Testing (offline, no hardware)

- Test all ISO-TP frame assembly/disassembly with known byte sequences
- Test UDS service handlers with pre-built `UdsRequest` messages
- Test SOME/IP header serialize/deserialize round-trips
- Use copper's **simulation mode** (`sim_mode = true`) for integration tests

### 9.2 Integration Testing (vcan / virtual network)

```bash
# Create virtual CAN interface
modprobe vcan
ip link add dev vcan0 type vcan
ip link set up vcan0

# Run copper CAN loopback example against vcan0
cargo run --example can_loopback
```

For SOME/IP: use `localhost` UDP loopback with two copper instances.

### 9.3 Hardware-in-the-Loop Testing

- Use the copper **replay** system: record a CAN session with the unified log, replay it
  through the pipeline deterministically
- Compare output CopperLists between replay runs to verify determinism
- Use `cu_safetymon` to detect timing regressions

### 9.4 Compliance Testing

- **CAN**: Use CANoe (Vector) or cananalyst+ with CAPL scripts for ISO 11898 compliance
- **ISO-TP**: Test with `isotpsend` / `isotprecv` Linux tools against the copper stack
- **UDS**: Use `udsoncan` Python library as a reference client against the copper UDS server
- **SOME/IP**: Use `vsomeip` reference implementation for interoperability testing

---

## 10. Detailed Task Breakdown

### Phase 0: Shared Infrastructure

| Task | Description | Dependencies |
|------|-------------|--------------|
| `cu_automotive_base` | `CanFrame`, `CanFdFrame`, `Pdu`, `Nrc`, addressing types | copper-rs |
| Cargo workspace setup | Workspace Cargo.toml, CI, feature flags | — |
| `heapless` integration | Verify heapless Vec in CuMsgPayload across no_std targets | cu_automotive_base |

### Phase 1: CAN

| Task | Description | Dependencies |
|------|-------------|--------------|
| `cu_can::CanSource` (std) | socketcan-based, SO_TIMESTAMPING | cu_automotive_base |
| `cu_can::CanSink` (std) | socketcan write | cu_automotive_base |
| `cu_can::CanSource` (embedded) | embedded-can HAL | cu_automotive_base |
| `cu_can::CanSink` (embedded) | embedded-can HAL | cu_automotive_base |
| `cu_can::CanFilter` | software filter task | cu_can source/sink |
| `cu_can::CanSignalDecoder` | DBC parsing + signal extraction | cu_can source |
| `cu_can_fd` extension | CAN FD source/sink | cu_can |
| `cu_can_nm` | NM background task | cu_can |
| CAN loopback example | vcan0 end-to-end | all above |

### Phase 2: UDS

| Task | Description | Dependencies |
|------|-------------|--------------|
| `cu_isotp::IsotpRx` | ISO-TP reassembler | cu_can |
| `cu_isotp::IsotpTx` | ISO-TP segmenter | cu_can |
| `cu_uds::UdsSession` (background) | session/security state machine | cu_isotp |
| `cu_uds::ReadDataByIdentifier` | 0x22 service | cu_uds session |
| `cu_uds::WriteDataByIdentifier` | 0x2E service | cu_uds session |
| `cu_uds::DiagnosticSessionControl` | 0x10 service | cu_uds session |
| `cu_uds::SecurityAccess` | 0x27 service | cu_uds session |
| `cu_uds::EcuReset` | 0x11 service | cu_uds session |
| `cu_uds::RoutineControl` | 0x31 service | cu_uds session |
| `cu_uds::ReadDTCInformation` | 0x19 service | DtcStore resource |
| `cu_uds::RequestDownload/Upload` | 0x34-0x37 transfer | cu_uds session |
| `cu_uds_doip` | DoIP transport (Ethernet UDS) | cu_someip base |
| UDS server example | Simulated ECU | all above |
| UDS client example | Tester/scanner | all above |

### Phase 3: SOME/IP

| Task | Description | Dependencies |
|------|-------------|--------------|
| `cu_someip` header types | `SomeIpHeader`, `SomeIpMessage`, enums | cu_automotive_base |
| SOME/IP serialization | `SomeIpSerialize`/`Deserialize` derive | cu_someip |
| `cu_someip::SomeIpUdpSource` | UDP rx → SomeIpMessage | cu_someip |
| `cu_someip::SomeIpUdpSink` | SomeIpMessage → UDP tx | cu_someip |
| `cu_someip::MethodHandler` | generic method dispatch task | cu_someip |
| `cu_someip::EventPublisher` | event notification task | cu_someip |
| `cu_someip::EventSubscriber` | event receive task | cu_someip |
| `cu_someip_sd::SdTask` | SD background state machine | cu_someip |
| SOME/IP method example | client/server pair | all above |
| SOME/IP event example | publisher/subscriber pair | all above |
| Time sync task | PTP/SOME/IP time sync → RobotClock | cu_someip |
| ARXML code generator | Generate Rust types from ARXML | all above |

### Cross-Cutting

| Task | Description |
|------|-------------|
| `cu_automotive_monitor` | `CuMonitor` for bus health + DTC tracking |
| `DtcStore` resource bundle | Shared DTC storage across stack |
| Embedded CI targets | Cross-compile + QEMU smoke tests for Cortex-M |
| vsomeip interop test | SOME/IP interop with reference implementation |
| CANoe-compatible test scripts | CAN/UDS validation |

---

*This plan was authored based on deep research of copper-rs internals documented in `research.md`.
Read `research.md` before implementing any phase to understand copper's paradigms and constraints.*
