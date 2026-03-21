fn main() {
    if let Err(err) = cu_runtime_matrix::run_bench_cli() {
        eprintln!("cu-runtime-matrix failed: {err}");
        std::process::exit(1);
    }
}
