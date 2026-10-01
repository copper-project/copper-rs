//! A missing nested recipe must be reported at the encoded field.
#![cfg(feature = "self-describing")]
#[test]
fn test_missing_nested_recipe() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/missing_nested_recipe.rs");
    tests.compile_fail("tests/ui/missing_codec_recipe.rs");
}
