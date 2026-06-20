#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "rp2350", feature = "stm32h7"))]
compile_error!("Enable only one cu-bdshot feature: rp2350 or stm32h7.");
#[cfg(all(
    not(any(feature = "rp2350", feature = "stm32h7")),
    not(feature = "messages-only")
))]
compile_error!("Enable one cu-bdshot feature: rp2350 or stm32h7 (or messages-only for host use).");

#[cfg(any(feature = "rp2350", feature = "stm32h7"))]
pub mod board;
#[cfg(any(feature = "rp2350", feature = "stm32h7"))]
pub mod bridge;
pub mod messages;

#[cfg(any(feature = "rp2350", feature = "stm32h7"))]
mod decode;
#[cfg(feature = "stm32h7")]
mod dshot;
#[cfg(feature = "rp2350")]
mod esc_channel;

#[cfg(feature = "rp2350")]
pub use board::{
    Rp2350Board, Rp2350BoardConfig, Rp2350BoardProvider, Rp2350BoardResources,
    register_rp2350_board,
};
#[cfg(feature = "stm32h7")]
pub use board::{
    Stm32H7Board, Stm32H7BoardProvider, Stm32H7BoardResources, register_stm32h7_board,
};
#[cfg(any(feature = "rp2350", feature = "stm32h7"))]
pub use bridge::{BdshotBoardProvider, CuBdshotBridge};
pub use messages::{DShotTelemetry, EscCommand, EscTelemetry};

#[cfg(feature = "rp2350")]
pub type RpBdshotBridge = CuBdshotBridge<Rp2350BoardProvider>;
#[cfg(feature = "stm32h7")]
pub type Stm32BdshotBridge = CuBdshotBridge<Stm32H7BoardProvider>;
