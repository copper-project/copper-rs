//! # cu_isotp — ISO 15765-2 Transport Protocol for CAN
//!
//! Provides the [`IsotpCodec`] task that performs segmentation (TX) and
//! reassembly (RX) of multi-frame ISO-TP PDUs over CAN. This is the
//! transport layer between raw CAN frames and UDS (or any higher layer).
//!
//! The codec operates as a [`CuTask`] with two inputs (CAN RX + upper-layer TX)
//! and two outputs (CAN TX + upper-layer RX), though in practice it is wired
//! as a single-input single-output pair between CanSource → IsotpCodec → UDS.
//!
//! ## ISO-TP Frame Types
//! - **Single Frame (SF)**: payload ≤ 7 bytes, one CAN frame
//! - **First Frame (FF)**: begins a multi-frame transfer, carries length + first 6 bytes
//! - **Consecutive Frame (CF)**: carries subsequent 7-byte chunks
//! - **Flow Control (FC)**: receiver → sender, controls burst size and timing
//!
//! ## Configuration (RON)
//! ```ron
//! (id: "isotp", type: "cu_isotp::IsotpCodec", config: {
//!     "tx_id": 0x641,
//!     "rx_id": 0x642,
//!     "block_size": 0,
//!     "st_min_ms": 10
//! })
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

use cu29::prelude::*;
use cu_automotive_payloads::{
    isotp::{
        FlowControlParams, FlowStatus, IsotpAddressingMode, IsotpFrameType, IsotpPdu,
        ISOTP_MAX_PDU_SIZE,
    },
    CanFrame, CanId,
};

/// Maximum consecutive frames before waiting for the next flow control.
const DEFAULT_BLOCK_SIZE: u8 = 0; // 0 = no limit
/// Default separation time minimum in ms.
const DEFAULT_ST_MIN_MS: u8 = 10;

// ---------------------------------------------------------------------------
// RX state machine
// ---------------------------------------------------------------------------

/// State of ISO-TP reassembly (receiving side).
#[derive(Clone, Debug, Default)]
enum RxState {
    #[default]
    Idle,
    /// Receiving consecutive frames; accumulating into buffer.
    Receiving {
        expected_len: usize,
        received: usize,
        next_sn: u8,
        buffer: [u8; ISOTP_MAX_PDU_SIZE],
    },
}

// ---------------------------------------------------------------------------
// TX state machine
// ---------------------------------------------------------------------------

/// State of ISO-TP segmentation (sending side).
#[derive(Clone, Debug, Default)]
enum TxState {
    #[default]
    Idle,
    /// Transmitting consecutive frames from a buffered PDU.
    Sending {
        buffer: [u8; ISOTP_MAX_PDU_SIZE],
        total_len: usize,
        offset: usize,
        next_sn: u8,
        block_remaining: u8,
        bs: u8,
        waiting_fc: bool,
    },
}

// ---------------------------------------------------------------------------
// IsotpCodec — the main CuTask
// ---------------------------------------------------------------------------

/// ISO-TP segmentation/reassembly task.
///
/// **Inputs**: CAN frames from CanSource (to reassemble), upper-layer PDUs to segment.
/// **Outputs**: CAN frames to CanSink (segmented), reassembled PDUs to upper layer.
///
/// In the simplest wiring:
///   `CanSource → IsotpCodec → UdsServer`  (RX path)
///   `UdsServer → IsotpCodec → CanSink`    (TX path)
///
/// For Copper's DAG, we model this as:
///   Input: `(CuMsg<CanFrame>, CuMsg<IsotpPdu>)`
///   Output: `(CuMsg<CanFrame>, CuMsg<IsotpPdu>)`
/// where input.0 = CAN RX frames, input.1 = upper layer TX PDUs
///       output.0 = CAN TX frames, output.1 = reassembled PDU to upper layer
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct IsotpCodec {
    /// CAN ID we transmit on.
    tx_can_id: CanId,
    /// CAN ID we accept (filter) for reassembly.
    rx_can_id: CanId,
    /// Flow control parameters we advertise.
    fc_params: FlowControlParams,
    /// Addressing mode.
    #[allow(dead_code)]
    addressing: IsotpAddressingMode,
    /// RX state machine.
    rx_state: RxState,
    /// TX state machine.
    tx_state: TxState,
}

impl Freezable for IsotpCodec {}

impl IsotpCodec {
    /// Build a Single Frame CAN payload for data ≤ 7 bytes.
    fn build_single_frame(data: &[u8]) -> CanFrame {
        let mut frame = CanFrame::default();
        frame.dlc = (data.len() + 1).min(8) as u8;
        frame.data[0] = data.len() as u8; // PCI: SF with length
        let copy = data.len().min(7);
        frame.data[1..1 + copy].copy_from_slice(&data[..copy]);
        frame
    }

    /// Build a First Frame for multi-frame transfer.
    fn build_first_frame(data: &[u8]) -> CanFrame {
        let total_len = data.len();
        let mut frame = CanFrame::default();
        frame.dlc = 8;
        // PCI: type=1 (FF), length in 12 bits
        frame.data[0] = 0x10 | ((total_len >> 8) & 0x0F) as u8;
        frame.data[1] = (total_len & 0xFF) as u8;
        let copy = data.len().min(6);
        frame.data[2..2 + copy].copy_from_slice(&data[..copy]);
        frame
    }

    /// Build a Consecutive Frame.
    fn build_consecutive_frame(data: &[u8], sn: u8) -> CanFrame {
        let mut frame = CanFrame::default();
        let copy = data.len().min(7);
        frame.dlc = (copy + 1) as u8;
        frame.data[0] = 0x20 | (sn & 0x0F);
        frame.data[1..1 + copy].copy_from_slice(&data[..copy]);
        frame
    }

    /// Build a Flow Control frame.
    fn build_flow_control(fs: FlowStatus, bs: u8, st_min: u8) -> CanFrame {
        let mut frame = CanFrame::default();
        frame.dlc = 3;
        frame.data[0] = 0x30 | (fs as u8);
        frame.data[1] = bs;
        frame.data[2] = st_min;
        frame
    }

    /// Decode the PCI (Protocol Control Information) from a CAN frame.
    fn frame_type(data: &[u8]) -> IsotpFrameType {
        if data.is_empty() {
            return IsotpFrameType::SingleFrame;
        }
        match data[0] >> 4 {
            0 => IsotpFrameType::SingleFrame,
            1 => IsotpFrameType::FirstFrame,
            2 => IsotpFrameType::ConsecutiveFrame,
            3 => IsotpFrameType::FlowControl,
            _ => IsotpFrameType::SingleFrame,
        }
    }

    /// Handle an incoming CAN frame on the RX path.
    /// Returns Some(IsotpPdu) when a complete message has been reassembled.
    fn handle_rx(&mut self, frame: &CanFrame) -> (Option<IsotpPdu>, Option<CanFrame>) {
        let data = &frame.data[..frame.dlc as usize];
        match Self::frame_type(data) {
            IsotpFrameType::SingleFrame => {
                let sf_len = (data[0] & 0x0F) as usize;
                if sf_len == 0 || sf_len > 7 || data.len() < 1 + sf_len {
                    return (None, None);
                }
                let mut pdu = IsotpPdu::default();
                pdu.data[..sf_len].copy_from_slice(&data[1..1 + sf_len]);
                pdu.len = sf_len as u16;
                pdu.addressing_mode = self.addressing;
                self.rx_state = RxState::Idle;
                (Some(pdu), None)
            }
            IsotpFrameType::FirstFrame => {
                let total_len =
                    (((data[0] & 0x0F) as usize) << 8) | (data[1] as usize);
                if total_len > ISOTP_MAX_PDU_SIZE || data.len() < 2 {
                    return (None, None);
                }
                let first_bytes = data.len().min(8) - 2;
                let copy = first_bytes.min(total_len);
                let mut buf = [0u8; ISOTP_MAX_PDU_SIZE];
                buf[..copy].copy_from_slice(&data[2..2 + copy]);
                self.rx_state = RxState::Receiving {
                    expected_len: total_len,
                    received: copy,
                    next_sn: 1,
                    buffer: buf,
                };
                // Send FC
                let fc = Self::build_flow_control(
                    FlowStatus::ContinueToSend,
                    self.fc_params.block_size,
                    self.fc_params.st_min,
                );
                (None, Some(fc))
            }
            IsotpFrameType::ConsecutiveFrame => {
                if let RxState::Receiving {
                    ref expected_len,
                    ref mut received,
                    ref mut next_sn,
                    ref mut buffer,
                } = self.rx_state
                {
                    let sn = data[0] & 0x0F;
                    if sn != *next_sn & 0x0F {
                        self.rx_state = RxState::Idle;
                        return (None, None); // sequence error
                    }
                    let payload_bytes = (data.len() - 1).min(7);
                    let remaining = expected_len - *received;
                    let copy = payload_bytes.min(remaining);
                    if *received + copy <= ISOTP_MAX_PDU_SIZE {
                        buffer[*received..*received + copy]
                            .copy_from_slice(&data[1..1 + copy]);
                    }
                    *received += copy;
                    *next_sn = next_sn.wrapping_add(1);

                    if *received >= *expected_len {
                        let mut pdu = IsotpPdu::default();
                        let final_len = (*expected_len).min(ISOTP_MAX_PDU_SIZE);
                        pdu.data[..final_len].copy_from_slice(&buffer[..final_len]);
                        pdu.len = final_len as u16;
                        pdu.addressing_mode = self.addressing;
                        self.rx_state = RxState::Idle;
                        return (Some(pdu), None);
                    }
                }
                (None, None)
            }
            IsotpFrameType::FlowControl => {
                // This is relevant to the TX state machine
                if let TxState::Sending {
                    ref mut waiting_fc,
                    ref mut bs,
                    ref mut block_remaining,
                    ..
                } = self.tx_state
                {
                    if data.len() >= 3 {
                        let fs = data[0] & 0x0F;
                        if fs == 0 {
                            // ContinueToSend
                            *waiting_fc = false;
                            *bs = data[1];
                            *block_remaining = data[1];
                        }
                        // fs == 1 → Wait (stay in waiting_fc)
                        // fs == 2 → Overflow/abort → reset
                        if fs == 2 {
                            self.tx_state = TxState::Idle;
                        }
                    }
                }
                (None, None)
            }
        }
    }

    /// Continue sending CFs from the TX buffer.
    /// Returns the next CAN frame to send, if any.
    fn tx_next_frame(&mut self) -> Option<CanFrame> {
        if let TxState::Sending {
            ref buffer,
            total_len,
            ref mut offset,
            ref mut next_sn,
            ref mut block_remaining,
            bs,
            ref mut waiting_fc,
        } = self.tx_state
        {
            if *waiting_fc {
                return None;
            }
            if *offset >= total_len {
                self.tx_state = TxState::Idle;
                return None;
            }
            let remaining = total_len - *offset;
            let chunk = remaining.min(7);
            let frame = Self::build_consecutive_frame(
                &buffer[*offset..*offset + chunk],
                *next_sn,
            );
            *offset += chunk;
            *next_sn = next_sn.wrapping_add(1);

            if bs > 0 {
                *block_remaining = block_remaining.saturating_sub(1);
                if *block_remaining == 0 && *offset < total_len {
                    *waiting_fc = true;
                    *block_remaining = bs;
                }
            }

            if *offset >= total_len {
                // Transmission complete — we'll reset on next call
            }
            Some(frame)
        } else {
            None
        }
    }

    /// Begin a new segmented transmission of an ISO-TP PDU.
    fn start_tx(&mut self, pdu: &IsotpPdu) -> Option<CanFrame> {
        let len = pdu.len as usize;
        if len == 0 {
            return None;
        }
        if len <= 7 {
            // Single frame
            Some(Self::build_single_frame(&pdu.data[..len]))
        } else {
            // Multi-frame: send FF, then wait for FC
            let frame = Self::build_first_frame(&pdu.data[..len]);
            let mut buf = [0u8; ISOTP_MAX_PDU_SIZE];
            buf[..len].copy_from_slice(&pdu.data[..len]);
            self.tx_state = TxState::Sending {
                buffer: buf,
                total_len: len,
                offset: 6, // first 6 bytes already sent in FF
                next_sn: 1,
                block_remaining: 0,
                bs: 0,
                waiting_fc: true,
            };
            Some(frame)
        }
    }
}

impl CuTask for IsotpCodec {
    type Resources<'r> = ();
    type Input<'m> = input_msg!('m, CanFrame, IsotpPdu);
    type Output<'m> = output_msg!(CanFrame, IsotpPdu);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let (tx_id, rx_id, bs, st_min) = match config {
            Some(cfg) => {
                let tx = cfg.get::<i64>("tx_id")?.unwrap_or(0x641) as u32;
                let rx = cfg.get::<i64>("rx_id")?.unwrap_or(0x642) as u32;
                let bs = cfg.get::<i64>("block_size")?.unwrap_or(0) as u8;
                let st = cfg.get::<i64>("st_min_ms")?.unwrap_or(10) as u8;
                (tx, rx, bs, st)
            }
            None => (0x641, 0x642, DEFAULT_BLOCK_SIZE, DEFAULT_ST_MIN_MS),
        };
        Ok(Self {
            tx_can_id: CanId::Standard(tx_id.min(0x7FF) as u16),
            rx_can_id: CanId::Standard(rx_id.min(0x7FF) as u16),
            fc_params: FlowControlParams {
                status: FlowStatus::ContinueToSend,
                block_size: bs,
                st_min: st_min,
            },
            addressing: IsotpAddressingMode::Normal,
            rx_state: RxState::Idle,
            tx_state: TxState::Idle,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        let (can_input, isotp_input) = input;
        let (can_output, isotp_output) = output;

        // --- RX path: CAN frame → reassembled ISO-TP PDU ---
        if let Some(frame) = can_input.payload() {
            // Filter by expected RX CAN ID
            if frame.id == self.rx_can_id || self.rx_can_id.raw() == 0 {
                let (maybe_pdu, maybe_fc_frame) = self.handle_rx(frame);
                if let Some(pdu) = maybe_pdu {
                    isotp_output.set_payload(pdu);
                    isotp_output.tov = Tov::Time(ctx.now());
                }
                if let Some(mut fc) = maybe_fc_frame {
                    fc.id = self.tx_can_id;
                    can_output.set_payload(fc);
                    can_output.tov = Tov::Time(ctx.now());
                    return Ok(());
                }
            }
        }

        // --- TX path: upper-layer PDU → segmented CAN frames ---
        if let Some(pdu) = isotp_input.payload() {
            if let Some(mut frame) = self.start_tx(pdu) {
                frame.id = self.tx_can_id;
                can_output.set_payload(frame);
                can_output.tov = Tov::Time(ctx.now());
                return Ok(());
            }
        }

        // Continue any in-progress multi-frame TX
        if let Some(mut frame) = self.tx_next_frame() {
            frame.id = self.tx_can_id;
            can_output.set_payload(frame);
            can_output.tov = Tov::Time(ctx.now());
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_codec() -> IsotpCodec {
        IsotpCodec {
            tx_can_id: CanId::Standard(0x641),
            rx_can_id: CanId::Standard(0x642),
            fc_params: FlowControlParams {
                status: FlowStatus::ContinueToSend,
                block_size: 0,
                st_min: 10,
            },
            addressing: IsotpAddressingMode::Normal,
            rx_state: RxState::Idle,
            tx_state: TxState::Idle,
        }
    }

    #[test]
    fn single_frame_round_trip() {
        let data = [0x10, 0x01]; // UDS DiagnosticSessionControl
        let pdu = IsotpPdu::from_data(&data);
        assert!(pdu.len <= 7);

        let frame = IsotpCodec::build_single_frame(&pdu.data[..pdu.len as usize]);
        assert_eq!(frame.data[0] & 0xF0, 0x00); // SF
        assert_eq!(frame.data[0] & 0x0F, 2); // length
        assert_eq!(frame.data[1], 0x10);
        assert_eq!(frame.data[2], 0x01);
    }

    #[test]
    fn multi_frame_segmentation() {
        let mut codec = make_codec();
        // Create a 20-byte PDU
        let mut data = [0u8; 20];
        for (i, b) in data.iter_mut().enumerate() {
            *b = i as u8;
        }
        let pdu = IsotpPdu::from_data(&data);

        let ff = codec.start_tx(&pdu).unwrap();
        // Should be a First Frame
        assert_eq!(ff.data[0] >> 4, 1); // FF
        let announced_len = (((ff.data[0] & 0x0F) as usize) << 8) | ff.data[1] as usize;
        assert_eq!(announced_len, 20);

        // Simulate FC (CTS, BS=0, STmin=0)
        let mut fc_frame = CanFrame::default();
        fc_frame.dlc = 3;
        fc_frame.data[0] = 0x30; // FC CTS
        fc_frame.data[1] = 0; // BS=0 (no limit)
        fc_frame.data[2] = 0; // STmin=0
        fc_frame.id = CanId::Standard(0x642);
        codec.handle_rx(&fc_frame);

        // Now we should be able to get CFs
        let cf1 = codec.tx_next_frame().unwrap();
        assert_eq!(cf1.data[0] >> 4, 2); // CF
        assert_eq!(cf1.data[0] & 0x0F, 1); // SN=1

        let cf2 = codec.tx_next_frame().unwrap();
        assert_eq!(cf2.data[0] & 0x0F, 2); // SN=2
    }

    #[test]
    fn reassembly_single_frame() {
        let mut codec = make_codec();
        let mut frame = CanFrame::default();
        frame.dlc = 4;
        frame.data[0] = 0x03; // SF, len=3
        frame.data[1] = 0x7F;
        frame.data[2] = 0x10;
        frame.data[3] = 0x22;
        frame.id = CanId::Standard(0x642);

        let (pdu, _fc) = codec.handle_rx(&frame);
        let pdu = pdu.unwrap();
        assert_eq!(pdu.len, 3);
        assert_eq!(pdu.data[0], 0x7F);
        assert_eq!(pdu.data[1], 0x10);
        assert_eq!(pdu.data[2], 0x22);
    }
}
