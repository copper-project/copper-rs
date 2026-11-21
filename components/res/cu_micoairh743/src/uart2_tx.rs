//! UART2 TX FIFO. DMA1 reads contiguous spans from AXI SRAM while MSP appends
//! to the free tail. Transfer completion starts the next span, even when the
//! application has no more packets to send.
use core::cell::RefCell;
use cortex_m::{interrupt::Mutex, peripheral::NVIC};
use stm32h7xx_hal::{
    dma::{
        CurrentBuffer, DmaDirection, MemoryToPeripheral,
        dma::{DmaConfig, StreamX, StreamsTuple},
        traits::{DoubleBufferedStream, Stream, TargetAddress},
    },
    pac::{self, interrupt},
    rcc::rec::Dma1,
    serial::Serial,
};

use super::{SerialPortError, Uart2Port};

const CAPACITY: usize = 512;
// The board maps .bss to AXI SRAM and runs with the data cache disabled.
static mut BYTES: [u8; CAPACITY] = [0; CAPACITY];
static TX: Mutex<RefCell<Option<Tx>>> = Mutex::new(RefCell::new(None));

struct Tx {
    stream: StreamX<pac::DMA1, 0>,
    head: usize,
    used: usize,
    active: usize,
    failed: bool,
}

pub(super) fn init(serial: &mut Serial<pac::USART2>, dma: pac::DMA1, clock: Dma1) {
    type Peripheral = Serial<pac::USART2>;
    let mut stream = StreamsTuple::new(dma, clock).0;
    stream.disable();
    stream.clear_interrupts();
    stream.apply_config(
        DmaConfig::default()
            .memory_increment(true)
            .transfer_complete_interrupt(true)
            .transfer_error_interrupt(true)
            .direct_mode_error_interrupt(true),
    );
    stream.set_direction(DmaDirection::MemoryToPeripheral);
    stream
        .set_request_line(<Peripheral as TargetAddress<MemoryToPeripheral>>::REQUEST_LINE.unwrap());
    stream.set_trbuff(<Peripheral as TargetAddress<MemoryToPeripheral>>::TRBUFF);
    // SAFETY: only this DMA stream writes to the USART2 transmit register.
    unsafe { stream.set_peripheral_address(TargetAddress::<MemoryToPeripheral>::address(serial)) };
    serial.enable_dma_tx();
    cortex_m::interrupt::free(|cs| {
        TX.borrow(cs).replace(Some(Tx {
            stream,
            head: 0,
            used: 0,
            active: 0,
            failed: false,
        }))
    });
    NVIC::unpend(pac::Interrupt::DMA1_STR0);
    // SAFETY: the handler and the producer serialize access to TX.
    unsafe { NVIC::unmask(pac::Interrupt::DMA1_STR0) };
}

impl Tx {
    fn start(&mut self) {
        if self.active != 0 || self.used == 0 {
            return;
        }
        self.active = self.used.min(CAPACITY - self.head);
        let address = (&raw const BYTES).cast::<u8>().wrapping_add(self.head);
        self.stream.set_number_of_transfers(self.active as u16);
        cortex_m::asm::dmb();
        // SAFETY: the active span stays occupied until DMA signals completion.
        unsafe {
            self.stream
                .set_memory_address(CurrentBuffer::Buffer0, address as usize);
            self.stream.enable();
        }
    }

    fn service(&mut self) {
        // SAFETY: read-only status for our exclusively owned stream.
        let status = unsafe { &*pac::DMA1::ptr() }.lisr.read();
        if status.teif0().bit() || status.dmeif0().bit() {
            self.stream.disable();
            self.failed = true;
        } else if status.tcif0().bit() {
            self.head = (self.head + self.active) % CAPACITY;
            self.used -= self.active;
            self.active = 0;
        } else {
            return;
        }
        self.stream.clear_interrupts();
        if !self.failed {
            self.start();
        }
    }

    fn enqueue(&mut self, bytes: &[u8]) -> Result<usize, SerialPortError> {
        if self.used == CAPACITY {
            self.service();
        }
        if self.failed {
            return Err(SerialPortError::Other);
        }
        let tail = (self.head + self.used) % CAPACITY;
        let n = bytes.len().min(CAPACITY - self.used).min(CAPACITY - tail);
        // SAFETY: this free span is disjoint from the span owned by DMA.
        unsafe {
            (&raw mut BYTES)
                .cast::<u8>()
                .add(tail)
                .copy_from_nonoverlapping(bytes.as_ptr(), n)
        };
        self.used += n;
        self.start();
        Ok(n)
    }
}

#[interrupt]
fn DMA1_STR0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(tx) = TX.borrow(cs).borrow_mut().as_mut() {
            tx.service();
        }
    });
}

impl embedded_io::Write for Uart2Port {
    fn write(&mut self, bytes: &[u8]) -> Result<usize, SerialPortError> {
        if bytes.is_empty() {
            return Ok(0);
        }
        loop {
            let n = cortex_m::interrupt::free(|cs| {
                TX.borrow(cs).borrow_mut().as_mut().unwrap().enqueue(bytes)
            })?;
            if n != 0 {
                return Ok(n);
            }
        }
    }

    fn flush(&mut self) -> Result<(), SerialPortError> {
        loop {
            let empty = cortex_m::interrupt::free(|cs| {
                let mut state = TX.borrow(cs).borrow_mut();
                let tx = state.as_mut().unwrap();
                tx.service();
                if tx.failed {
                    Err(SerialPortError::Other)
                } else {
                    Ok(tx.used == 0)
                }
            })?;
            if empty && self.inner.inner().isr.read().tc().bit() {
                return Ok(());
            }
        }
    }
}
