use std::path::PathBuf;

/// `cu-autoware [iterations] [log base]`.
fn main() {
    let mut args = std::env::args().skip(1);
    let iterations = args
        .next()
        .map(|arg| arg.parse().expect("iteration count must be a number"))
        .unwrap_or(cu_autoware::ITERATIONS);
    cu_autoware::run(iterations, args.next().map(PathBuf::from));
}
