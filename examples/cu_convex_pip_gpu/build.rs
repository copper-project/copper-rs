use spirv_builder::{MetadataPrintout, SpirvBuilder};

fn main() {
    // Allow building with a stable toolchain by skipping rust-gpu's toolchain check.
    std::env::set_var("RUSTGPU_SKIP_TOOLCHAIN_CHECK", "1");
    let result = SpirvBuilder::new("shader", "spirv-unknown-vulkan1.2")
        .print_metadata(MetadataPrintout::None)
        .build()
        .expect("Failed to build shader");
    println!(
        "cargo:rustc-env=KERNEL_SPV={}",
        result.module.unwrap_single().to_str().unwrap()
    );
}
