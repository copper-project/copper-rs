//! A missing nested description must be reported at the encoded field.
#![cfg(feature = "self-describing-logs")]

use std::path::Path;
use std::path::PathBuf;
use ui_test::Config;
use ui_test::custom_flags::edition::Edition;
use ui_test::custom_flags::rustfix::RustfixMode;
use ui_test::dependencies::DependencyBuilder;

#[test]
fn test_missing_nested_description() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let target_dir = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("../../target"));
    let mut dependencies = DependencyBuilder {
        crate_manifest_path: root.join("tests/dependencies/Cargo.toml"),
        // Lockfiles are generated locally and are not committed in this workspace.
        bless_lockfile: true,
        ..DependencyBuilder::default()
    };
    dependencies.program.out_dir_flag = None;

    let mut config = Config::rustc(root.join("tests/ui"));
    config.out_dir = target_dir.join("ui/value");
    config.output_conflict_handling = ui_test::ignore_output_conflict;
    let defaults = config.comment_defaults.base();
    defaults.custom.remove("edition");
    defaults.custom.remove("rustfix");
    defaults.add_custom("edition", Edition("2024".into()));
    defaults.add_custom("rustfix", RustfixMode::Disabled);
    defaults.add_custom("dependencies", dependencies);

    ui_test::run_tests(config).expect("value compile tests failed");
}
