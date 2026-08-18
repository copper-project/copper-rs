/// Override the iteration count with the first argument.
fn main() {
    let iterations = std::env::args()
        .nth(1)
        .map(|arg| arg.parse().expect("iteration count must be a number"))
        .unwrap_or(cu_autoware::ITERATIONS);
    cu_autoware::run(iterations);
}
