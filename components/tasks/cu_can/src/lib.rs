//! # cu_can — CAN Bus Tasks for Copper
//!
//! Provides [`CanSource`], [`CanSink`], and [`CanFilter`] copper tasks for
//! interfacing with CAN bus networks via Linux SocketCAN (std) or mock mode.
//!
//! ## Configuration (RON)
//!
//! ```ron
//! (id: "can_rx", type: "cu_can::CanSource", config: { "interface": "vcan0" })
//! (id: "can_tx", type: "cu_can::CanSink",   config: { "interface": "vcan0" })
//! (id: "filter", type: "cu_can::CanFilter",  config: { "accept_id": 0x100 })
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

mod socketcan;

use cu29::prelude::*;
use cu_automotive_payloads::{CanFrame, CanId};

// Re-export payload types for convenient use in RON configs
pub use cu_automotive_payloads::CanFrame as CanFramePayload;

// ---------------------------------------------------------------------------
// CanSource — reads CAN frames from SocketCAN (or mock)
// ---------------------------------------------------------------------------

/// CAN bus source task that reads frames from a SocketCAN interface.
///
/// In mock mode, produces synthetic toggling frames for testing.
///
/// # Config
/// - `interface` (String): SocketCAN interface name, e.g. `"vcan0"`, `"can0"`. Default: `"vcan0"`.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct CanSource {
    #[allow(dead_code)]
    interface: alloc::string::String,
    #[cfg(all(target_os = "linux", not(feature = "mock")))]
    fd: i32,
    #[cfg(feature = "mock")]
    mock_counter: u32,
    pending_frame: Option<CanFrame>,
}

impl Freezable for CanSource {}

impl CuSrcTask for CanSource {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(CanFrame);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let interface = match config {
            Some(cfg) => cfg
                .get::<alloc::string::String>("interface")?
                .unwrap_or_else(|| alloc::string::String::from("vcan0")),
            None => alloc::string::String::from("vcan0"),
        };

        #[cfg(all(target_os = "linux", not(feature = "mock")))]
        let fd = socketcan::open_can_socket(&interface)?;

        Ok(Self {
            interface,
            #[cfg(all(target_os = "linux", not(feature = "mock")))]
            fd,
            #[cfg(feature = "mock")]
            mock_counter: 0,
            pending_frame: None,
        })
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        Ok(())
    }

    fn preprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
        // Non-blocking read in preprocess to keep process() fast
        #[cfg(all(target_os = "linux", not(feature = "mock")))]
        {
            self.pending_frame = socketcan::read_frame_nonblocking(self.fd);
        }

        #[cfg(feature = "mock")]
        {
            self.mock_counter = self.mock_counter.wrapping_add(1);
            self.pending_frame = Some(CanFrame::new(
                CanId::Standard((self.mock_counter % 0x800) as u16),
                &self.mock_counter.to_le_bytes()[..4],
            ));
        }
        Ok(())
    }

    fn process(&mut self, ctx: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        if let Some(frame) = self.pending_frame.take() {
            output.set_payload(frame);
            output.tov = Tov::Time(ctx.now());
        }
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        #[cfg(all(target_os = "linux", not(feature = "mock")))]
        socketcan::close_socket(self.fd);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// CanSink — writes CAN frames to SocketCAN (or mock)
// ---------------------------------------------------------------------------

/// CAN bus sink task that transmits frames to a SocketCAN interface.
///
/// # Config
/// - `interface` (String): SocketCAN interface name. Default: `"vcan0"`.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct CanSink {
    #[allow(dead_code)]
    #[allow(dead_code)]
    interface: alloc::string::String,
    #[cfg(all(target_os = "linux", not(feature = "mock")))]
    fd: i32,
    #[cfg(feature = "mock")]
    tx_count: u64,
}

impl Freezable for CanSink {}

impl CuSinkTask for CanSink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CanFrame);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let interface = match config {
            Some(cfg) => cfg
                .get::<alloc::string::String>("interface")?
                .unwrap_or_else(|| alloc::string::String::from("vcan0")),
            None => alloc::string::String::from("vcan0"),
        };

        #[cfg(all(target_os = "linux", not(feature = "mock")))]
        let fd = socketcan::open_can_socket(&interface)?;

        Ok(Self {
            interface,
            #[cfg(all(target_os = "linux", not(feature = "mock")))]
            fd,
            #[cfg(feature = "mock")]
            tx_count: 0,
        })
    }

    fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(_frame) = input.payload() {
            #[cfg(all(target_os = "linux", not(feature = "mock")))]
            socketcan::write_frame(self.fd, _frame)?;

            #[cfg(feature = "mock")]
            {
                self.tx_count += 1;
            }
        }
        Ok(())
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        #[cfg(all(target_os = "linux", not(feature = "mock")))]
        socketcan::close_socket(self.fd);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// CanFilter — software ID filter
// ---------------------------------------------------------------------------

/// CAN frame filter that passes or drops frames based on their identifier.
///
/// # Config
/// - `accept_id` (i64): If set, only frames with this exact CAN ID pass through.
/// - `accept_mask` (i64): If set with `accept_id`, applies a mask: `(frame.id & mask) == accept_id`.
///   Default mask: `0x7FF` (standard ID match).
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct CanFilter {
    accept_id: Option<u32>,
    accept_mask: u32,
}

impl Freezable for CanFilter {}

impl CuTask for CanFilter {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(CanFrame);
    type Output<'m> = output_msg!(CanFrame);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let accept_id = match config {
            Some(cfg) => cfg.get::<i64>("accept_id")?.map(|v| v as u32),
            None => None,
        };
        let accept_mask = match config {
            Some(cfg) => cfg.get::<i64>("accept_mask")?.unwrap_or(0x7FF) as u32,
            None => 0x7FF,
        };
        Ok(Self {
            accept_id,
            accept_mask,
        })
    }

    fn process(
        &mut self,
        _ctx: &CuContext,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        if let Some(frame) = input.payload() {
            let pass = match self.accept_id {
                Some(id) => (frame.id.raw() & self.accept_mask) == id,
                None => true, // no filter configured → pass all
            };
            if pass {
                output.set_payload(*frame);
                output.tov = input.tov;
            }
        }
        Ok(())
    }
}
