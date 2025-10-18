fn main() {
    println!(
        "\n\n\n# To build the firmware:\n\
         cargo build-arm\n\
         cargo build-riscv\n\
         \n\
         # To run the firmware (requires a connected probe):\n\
         cargo run-arm\n\
         cargo run-riscv\n\
         \n\
         # To build the log reader host tool:\n\
         cargo build-logreader\n"
    );
}
