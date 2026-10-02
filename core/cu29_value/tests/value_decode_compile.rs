//! A missing nested description must be reported at the encoded field.
#![cfg(feature = "self-describing-logs")]
#[test]
fn test_missing_nested_description() {
    let tests = trybuild::TestCases::new();
    tests.compile_fail("tests/ui/missing_nested_description.rs");
    tests.compile_fail("tests/ui/missing_codec_description.rs");
}
