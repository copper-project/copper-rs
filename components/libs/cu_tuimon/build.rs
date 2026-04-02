fn main() {
    cfg_aliases::cfg_aliases! {
        browser:  { all(target_family = "wasm", target_os = "unknown") },
        native:  { not(browser) },
    };
}
