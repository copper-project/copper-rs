use hal::dma::TransferSize;
use hal::pio::Running;
use hal::pio::StateMachine;
use hal::pio::{PIOExt, Rx, StateMachineIndex, Tx, ValidStateMachine};
use rp235x_hal as hal;

pub struct EscChannel<P, SM, TxSize, RxSize>
where
    P: PIOExt,
    SM: StateMachineIndex,
    (P, SM): ValidStateMachine,
    TxSize: TransferSize,
    RxSize: TransferSize,
{
    pub tx: Tx<(P, SM), TxSize>,
    pub rx: Rx<(P, SM), RxSize>,
    pub run: StateMachine<(P, SM), Running>,
}

impl<P, SM, TxSize, RxSize> EscChannel<P, SM, TxSize, RxSize>
where
    P: PIOExt,
    SM: StateMachineIndex,
    (P, SM): ValidStateMachine,
    TxSize: TransferSize,
    RxSize: TransferSize,
{
    pub fn is_full(&self) -> bool {
        self.tx.is_full()
    }

    pub fn write(&mut self, v: u32) -> bool {
        self.tx.write(v)
    }

    pub fn read(&mut self) -> Option<u32> {
        self.rx.read()
    }

    pub fn restart(&mut self) {
        self.run.restart()
    }
}

/// Helper to DRY on the initialization of each channel for each ESC
#[macro_export]
macro_rules! build_ch {
    ($prog:expr, $sm:ident, $pin:expr, $d:expr, $f:expr) => {{
        let (sm, rx, tx) = PIOBuilder::from_installed_program(unsafe { $prog.share() })
            .set_pins($pin, 1)
            .out_pins($pin, 1)
            .out_shift_direction(hal::pio::ShiftDirection::Left)
            .in_shift_direction(hal::pio::ShiftDirection::Left)
            .in_pin_base($pin)
            .jmp_pin($pin)
            .clock_divisor_fixed_point($d, $f)
            .autopush(true)
            // A telemetry response is encoded over 21 bits (Then distilled to
            // 16 bits accounting for encoding and redundancy)
            .push_threshold(21)
            .side_set_pin_base($pin)
            .build($sm);
        EscChannel {
            tx,
            rx,
            run: sm.start(),
        }
    }};
}
