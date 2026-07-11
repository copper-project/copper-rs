// Source: ARMv7-R performance monitor cycle counter (`PMCCNTR`) through CP15.
// Guarantee: 32-bit per-core PMU counter after `initialize()`, extended in
// software with rollover tracking. This backend enables the PMU cycle counter
// with the divide-by-64 mode, so calibration must convert raw ticks to time.
// It assumes reads are frequent enough to observe each 32-bit wrap and that the
// runtime stays on a CPU whose PMU counter is coherent for the application.
use portable_atomic::{AtomicU32, AtomicU64, Ordering};

static LAST_LOW: AtomicU32 = AtomicU32::new(0);
static HIGH: AtomicU64 = AtomicU64::new(0);

pub fn initialize() {
    // SAFETY: This backend is selected only for bare-metal ARM targets with the
    // Copper Cortex-R cfg. PMU control is a privileged CPU-local operation and
    // initialization is expected to run before normal runtime use.
    unsafe {
        let mut pmcr: u32;
        core::arch::asm!(
            "mrc p15, 0, {pmcr}, c9, c12, 0",
            pmcr = out(reg) pmcr,
            options(nomem, nostack, preserves_flags)
        );

        // Enable the PMU, reset counters, reset the cycle counter, and divide
        // the cycle counter by 64 to make 32-bit rollover manageable.
        pmcr |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
        core::arch::asm!(
            "mcr p15, 0, {pmcr}, c9, c12, 0",
            pmcr = in(reg) pmcr,
            options(nomem, nostack, preserves_flags)
        );

        let cycle_counter_enable: u32 = 1 << 31;
        core::arch::asm!(
            "mcr p15, 0, {cycle_counter_enable}, c9, c12, 1",
            cycle_counter_enable = in(reg) cycle_counter_enable,
            options(nomem, nostack, preserves_flags)
        );

        core::arch::asm!("isb", options(nomem, nostack, preserves_flags));
    }

    LAST_LOW.store(read_pmccntr(), Ordering::Relaxed);
    HIGH.store(0, Ordering::Relaxed);
}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    let low = read_pmccntr();
    let previous = LAST_LOW.swap(low, Ordering::Relaxed);

    if low < previous {
        HIGH.fetch_add(1u64 << 32, Ordering::Relaxed);
    }

    HIGH.load(Ordering::Relaxed) | low as u64
}

#[inline(always)]
fn read_pmccntr() -> u32 {
    let value: u32;

    // SAFETY: Reading PMCCNTR is a CPU-local counter read. The selected backend
    // enables the counter during `initialize()`.
    unsafe {
        core::arch::asm!(
            "mrc p15, 0, {value}, c9, c13, 0",
            value = out(reg) value,
            options(nomem, nostack, preserves_flags)
        );
    }

    value
}
