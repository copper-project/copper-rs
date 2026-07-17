use cfg_aliases::cfg_aliases;
fn main() {
    cu29_build::setup();
    cfg_aliases! {
        hardware: { all(target_os = "linux", not(feature = "mock")) },
        mock: { any(not(target_os = "linux"), feature = "mock") },
    }
}
