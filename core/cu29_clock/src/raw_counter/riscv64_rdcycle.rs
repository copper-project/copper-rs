// Source: RISC-V `cycle` CSR.
// Guarantee: monotonically increasing cycle counter on the current hart. RISC-V
// does not universally guarantee cross-hart synchronization or stable frequency,
// so platforms using this backend must provide those properties or keep the
// runtime pinned to a suitable hart.
pub fn initialize() {}

#[inline(always)]
pub fn read_raw_counter() -> u64 {
    let counter: u64;
    // SAFETY: Reading the cycle counter register is a side-effect-free CPU instruction.
    unsafe {
        core::arch::asm!("rdcycle {}", out(reg) counter);
    }
    counter
}
