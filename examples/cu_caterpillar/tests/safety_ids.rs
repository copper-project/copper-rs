use std::env;
use std::path::PathBuf;

fn main() {
    let out = parse_out_path();
    cu_caterpillar::determinism::link_safety_ids();
    let index = cu29::safety::collect_package_index(env!("CARGO_PKG_NAME"));
    cu29::safety::write_package_index_json(&out, &index).expect("failed to write safety ID dump");
}

fn parse_out_path() -> PathBuf {
    let mut args = env::args().skip(1);
    let flag = args.next().expect("expected '--out <path>'");
    let path = args.next().expect("expected '--out <path>'");
    assert_eq!(flag, "--out", "expected '--out <path>'");
    assert!(
        args.next().is_none(),
        "unexpected extra arguments, expected only '--out <path>'",
    );
    PathBuf::from(path)
}
