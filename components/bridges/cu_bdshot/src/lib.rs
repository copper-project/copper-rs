#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "rp2350", feature = "stm32h7"))]
compile_error!("Enable only one cu-bdshot feature: rp2350 or stm32h7.");
#[cfg(all(
    not(any(feature = "rp2350", feature = "stm32h7")),
    not(feature = "messages-only")
))]
compile_error!("Enable one cu-bdshot feature: rp2350 or stm32h7 (or messages-only for host use).");

#[cfg(not(feature = "messages-only"))]
pub mod board;
#[cfg(not(feature = "messages-only"))]
pub mod bridge;
pub mod messages;

#[cfg(not(feature = "messages-only"))]
mod decode;
#[cfg(all(feature = "stm32h7", not(feature = "messages-only")))]
mod dshot;
#[cfg(all(feature = "rp2350", not(feature = "messages-only")))]
mod esc_channel;

#[cfg(all(feature = "rp2350", not(feature = "messages-only")))]
pub use board::{
    Rp2350Board, Rp2350BoardConfig, Rp2350BoardProvider, Rp2350BoardResources,
    register_rp2350_board,
};
#[cfg(all(feature = "stm32h7", not(feature = "messages-only")))]
pub use board::{
    Stm32H7Board, Stm32H7BoardProvider, Stm32H7BoardResources, register_stm32h7_board,
};
#[cfg(not(feature = "messages-only"))]
pub use bridge::{BdshotBoardProvider, CuBdshotBridge};
pub use messages::{DShotTelemetry, EscCommand, EscTelemetry};

#[cfg(all(feature = "rp2350", not(feature = "messages-only")))]
pub type RpBdshotBridge = CuBdshotBridge<Rp2350BoardProvider>;
#[cfg(all(feature = "stm32h7", not(feature = "messages-only")))]
pub type Stm32BdshotBridge = CuBdshotBridge<Stm32H7BoardProvider>;
