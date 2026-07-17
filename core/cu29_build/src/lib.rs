#![doc = include_str!("../README.md")]

/// Environment variable used to pass the consuming crate's Cargo features to
/// Copper's procedural macros and generated runtime.
pub const COPPER_CFG_FEATURES_ENV: &str = "COPPER_CFG_FEATURES";

/// Emit the standard Cargo build-script configuration required by Copper.
pub fn setup() {
    println!(
        "cargo::rustc-env=LOG_INDEX_DIR={}",
        std::env::var("OUT_DIR").expect("Cargo must provide OUT_DIR")
    );

    let mut features: Vec<_> = std::env::var("CARGO_CFG_FEATURE")
        .unwrap_or_default()
        .split(',')
        .filter(|feature| !feature.is_empty())
        .map(str::to_owned)
        .collect();
    features.sort_unstable();
    features.dedup();

    println!(
        "cargo::rustc-env={COPPER_CFG_FEATURES_ENV}={}",
        features.join(",")
    );
}

#[cfg(test)]
mod tests {
    use super::COPPER_CFG_FEATURES_ENV;

    #[test]
    fn feature_environment_name_is_stable() {
        assert_eq!(COPPER_CFG_FEATURES_ENV, "COPPER_CFG_FEATURES");
    }
}
