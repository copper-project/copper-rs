use copper_log::debug;
fn main() {
    let a = 2;
    let b = 3;
    // debug!("Hello, world! {} {}", a, b);
    debug!("Hello, world!");
    debug!("Hello, world2!", a);
}
