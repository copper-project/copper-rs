// Raw counter backend selection lives here so the public clock implementation
// does not grow target-specific cfg routing. Each backend exports:
//
// - `initialize()`
// - `read_raw_counter() -> u64`
//
// Backend files document the clock source and the guarantees Copper relies on.
#[cfg_attr(target_arch = "aarch64", path = "aarch64_cntvct.rs")]
#[cfg_attr(all(target_os = "none", target_arch = "arm"), path = "cortex_m_dwt.rs")]
#[cfg_attr(target_arch = "riscv64", path = "riscv64_rdcycle.rs")]
#[cfg_attr(
    all(feature = "std", target_arch = "wasm32", target_os = "unknown"),
    path = "wasm_instant.rs"
)]
#[cfg_attr(target_arch = "x86_64", path = "x86_64_rdtsc.rs")]
#[cfg_attr(
    not(any(
        target_arch = "x86_64",
        target_arch = "aarch64",
        all(target_os = "none", target_arch = "arm"),
        target_arch = "riscv64",
        all(feature = "std", target_arch = "wasm32", target_os = "unknown")
    )),
    path = "std_system_time.rs"
)]
mod selected;

pub use selected::*;
