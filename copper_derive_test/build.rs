fn main() {
    let config_path = "copperconfig.ron";
    println!("cargo:rerun-if-changed={}", config_path);
}
