//! Cooperative SDMMC1 IDMA transfers. Only the storage worker owns the peripheral.
use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering, compiler_fence},
};
use cortex_m::peripheral::{DWT, SCB};
use cu_sdlogger::sdmmc::{Block, BlockCount, BlockDevice, BlockIdx};
use spin::Mutex;
use stm32h7xx_hal::{
    pac,
    sdmmc::{SdCard, Sdmmc},
};

// The resource bundle runs the H743 at 400 MHz. Limit storage work between
// service calls, including chains of individually short sector operations.
const SERVICE_CYCLES: u32 = 240_000;
const TIMEOUT_CYCLES: u32 = 400_000_000;
static IDLE: Mutex<Option<fn()>> = Mutex::new(None);
static LAST_SERVICE: AtomicU32 = AtomicU32::new(0);

/// Run storage work with a thread-mode callback while SDMMC is busy.
/// The callback may enqueue records. The worker retains exclusive card ownership.
pub fn service_with(work: impl FnOnce(), idle: fn()) {
    *IDLE.lock() = Some(idle);
    LAST_SERVICE.store(DWT::cycle_count(), Ordering::Relaxed);
    work();
    *IDLE.lock() = None;
}

fn service_if_due(now: u32) {
    if now.wrapping_sub(LAST_SERVICE.load(Ordering::Relaxed)) >= SERVICE_CYCLES {
        let idle = *IDLE.lock();
        if let Some(idle) = idle {
            idle();
        }
        LAST_SERVICE.store(DWT::cycle_count(), Ordering::Relaxed);
    }
}

struct Deadline(u32);
impl Deadline {
    fn new() -> Self {
        let now = DWT::cycle_count();
        service_if_due(now);
        Self(now)
    }
    fn pending(&self) -> Result<(), ()> {
        let now = DWT::cycle_count();
        if now.wrapping_sub(self.0) > TIMEOUT_CYCLES {
            return Err(());
        }
        service_if_due(now);
        Ok(())
    }
}

#[repr(align(32))]
struct Buffer([u8; 8192]);
// This static lives in AXI SRAM. The board enables I-cache only.
// Own the staging buffer through the entire command, including error recovery.
static BUFFER: Mutex<Buffer> = Mutex::new(Buffer([0; 8192]));
pub struct Device(RefCell<Card>);
struct Card {
    host: Sdmmc<pac::SDMMC1, SdCard>,
    faulted: bool,
    cache: SCB,
}
impl Device {
    pub(super) fn new(host: Sdmmc<pac::SDMMC1, SdCard>, cache: SCB) -> Self {
        Self(RefCell::new(Card {
            host,
            faulted: false,
            cache,
        }))
    }
}

impl Card {
    fn stop_after_error(&mut self) {
        // SDMMC1 owns this IDMA channel. Reset it before releasing the staging
        // buffer even if the card timed out halfway through a transfer.
        cortex_m::interrupt::free(|_| {
            // SAFETY: Only the SDMMC1 reset bit is changed, with interrupts masked
            // across each read-modify-write of the shared RCC register.
            let rcc = unsafe { &*pac::RCC::ptr() };
            rcc.ahb3rstr.modify(|_, w| w.sdmmc1rst().set_bit());
            cortex_m::asm::dsb();
            rcc.ahb3rstr.modify(|_, w| w.sdmmc1rst().clear_bit());
            cortex_m::asm::dsb();
        });
        self.faulted = true;
    }

    fn command(
        &self,
        index: u8,
        arg: u32,
        transfer: bool,
        deadline: &mut Deadline,
    ) -> Result<u32, ()> {
        let regs = self.host.inner();
        regs.icr.write(|w| {
            w.cmdrendc()
                .set_bit()
                .ccrcfailc()
                .set_bit()
                .ctimeoutc()
                .set_bit()
        });
        regs.argr.write(|w| unsafe { w.cmdarg().bits(arg) });
        regs.cmdr.write(|w| unsafe {
            w.cmdindex()
                .bits(index)
                .waitresp()
                .bits(1)
                .cmdtrans()
                .bit(transfer)
                .cmdstop()
                .bit(index == 12)
                .cpsmen()
                .set_bit()
        });
        loop {
            let flags = regs.star.read();
            if flags.ccrcfail().bit() || flags.ctimeout().bit() {
                return Err(());
            }
            if flags.cmdrend().bit() {
                let status = regs.resp1r.read().bits();
                // R1 errors: address, erase, write protection, CRC, illegal command and ECC.
                if status & 0xfdff_e008 != 0 {
                    return Err(());
                }
                return Ok(status);
            }
            deadline.pending()?;
        }
    }

    fn transfer(&mut self, write: bool, first: u32, data: &mut [u8]) -> Result<(), ()> {
        let card = self.host.card().map_err(|_| ())?;
        let rca = u32::from(card.rca.address()) << 16;
        let addr = if card.ocr.high_capacity() {
            first
        } else {
            first.checked_mul(512).ok_or(())?
        };
        let mut deadline = Deadline::new();
        let regs = self.host.inner();
        while regs.star.read().dpsmact().bit() || regs.star.read().cpsmact().bit() {
            deadline.pending()?;
        }
        regs.icr.write(|w| {
            w.dataendc()
                .set_bit()
                .dbckendc()
                .set_bit()
                .dtimeoutc()
                .set_bit()
                .dcrcfailc()
                .set_bit()
                .txunderrc()
                .set_bit()
                .rxoverrc()
                .set_bit()
                .idmatec()
                .set_bit()
                .idmabtcc()
                .set_bit()
                .dabortc()
                .set_bit()
                .busyd0endc()
                .set_bit()
        });
        regs.dtimer.write(|w| unsafe { w.bits(25_000_000) });
        regs.dlenr
            .write(|w| unsafe { w.datalength().bits(data.len() as u32) });
        regs.dctrl
            .write(|w| unsafe { w.dblocksize().bits(9).dtdir().bit(!write) });
        regs.idmabase0r
            .write(|w| unsafe { w.idmabase0().bits(data.as_mut_ptr() as u32) });
        if SCB::dcache_enabled() {
            // The staging buffer is exclusively owned and covers whole cache lines.
            if write {
                self.cache.clean_dcache_by_slice(data);
            } else {
                self.cache
                    .clean_invalidate_dcache_by_address(data.as_ptr() as usize, data.len());
            }
        }
        compiler_fence(Ordering::Release);
        cortex_m::asm::dsb();
        regs.idmactrlr.write(|w| w.idmaen().set_bit());
        let multi = data.len() > 512;
        let cmd = match (write, multi) {
            (false, false) => 17,
            (false, true) => 18,
            (true, false) => 24,
            (true, true) => 25,
        };
        self.command(cmd, addr, true, &mut deadline)?;
        loop {
            let flags = regs.star.read();
            if flags.dtimeout().bit()
                || flags.dcrcfail().bit()
                || flags.txunderr().bit()
                || flags.rxoverr().bit()
                || flags.idmate().bit()
            {
                return Err(());
            }
            if flags.dataend().bit() {
                break;
            }
            deadline.pending()?;
        }
        regs.idmactrlr.write(|w| w.idmaen().clear_bit());
        cortex_m::asm::dsb();
        if !write && SCB::dcache_enabled() {
            // SAFETY: IDMA has completed. This aligned, whole-line buffer has no
            // other CPU users and its only writer during the transfer was IDMA.
            unsafe {
                self.cache.invalidate_dcache_by_slice(data);
            }
        }
        compiler_fence(Ordering::Acquire);
        if multi {
            self.command(12, 0, false, &mut deadline)?;
        }
        if write || multi {
            loop {
                let response = self.command(13, rca, false, &mut deadline)?;
                if response & (1 << 8) != 0 && (response >> 9) & 15 == 4 {
                    break;
                }
                deadline.pending()?;
            }
        }
        Ok(())
    }
}
impl BlockDevice for Device {
    type Error = ();
    fn num_blocks(&self) -> Result<BlockCount, ()> {
        Ok(BlockCount(
            (self.0.borrow().host.card().map_err(|_| ())?.size() / 512) as u32,
        ))
    }
    fn write(&self, blocks: &[Block], start: BlockIdx) -> Result<(), ()> {
        let mut card = self.0.borrow_mut();
        if card.faulted {
            return Err(());
        }
        let mut buffer = BUFFER.lock();
        for (i, batch) in blocks.chunks(16).enumerate() {
            let bytes = &mut buffer.0[..batch.len() * 512];
            for (dst, src) in bytes.as_chunks_mut::<512>().0.iter_mut().zip(batch) {
                *dst = src.contents;
            }
            if let Err(e) = card.transfer(true, start.0 + (16 * i) as u32, bytes) {
                card.stop_after_error();
                return Err(e);
            }
        }
        Ok(())
    }
    fn read(&self, blocks: &mut [Block], start: BlockIdx, _: &str) -> Result<(), ()> {
        let mut card = self.0.borrow_mut();
        if card.faulted {
            return Err(());
        }
        let mut buffer = BUFFER.lock();
        for (i, batch) in blocks.chunks_mut(16).enumerate() {
            let bytes = &mut buffer.0[..batch.len() * 512];
            if let Err(e) = card.transfer(false, start.0 + (16 * i) as u32, bytes) {
                card.stop_after_error();
                return Err(e);
            }
            for (dst, src) in batch.iter_mut().zip(bytes.as_chunks::<512>().0) {
                dst.contents = *src;
            }
        }
        Ok(())
    }
}
