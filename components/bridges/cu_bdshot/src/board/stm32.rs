use crate::board::BdshotBoard;
use crate::bridge::BdshotBoardProvider;
use crate::messages::DShotTelemetry;
use cu29::CuResult;

/// Placeholder STM32 implementation until the hardware backend is wired up.
pub struct Stm32BdshotBoard;

impl BdshotBoard for Stm32BdshotBoard {
    const CHANNEL_COUNT: usize = 4;

    fn exchange(&mut self, _channel: usize, _frame: u32) -> Option<DShotTelemetry> {
        // TODO: implement STM32 BDShot exchange when hardware support lands.
        None
    }

    fn delay(&mut self, _micros: u32) {
        // TODO: replace with a proper microsecond delay for STM32 targets.
        core::hint::spin_loop();
    }
}

pub struct Stm32BoardProvider;

impl BdshotBoardProvider for Stm32BoardProvider {
    type Board = Stm32BdshotBoard;

    fn create_board() -> CuResult<Self::Board> {
        Ok(Stm32BdshotBoard)
    }
}
