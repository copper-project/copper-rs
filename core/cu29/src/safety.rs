use serde::Serialize;
use std::fs;
use std::path::Path;

pub use inventory;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct SafetyCheckRef {
    pub check_id: &'static str,
    pub requirement_id: &'static str,
    pub kind: &'static str,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct SafetyCaseRef {
    pub package: &'static str,
    pub case_id: &'static str,
    pub function: &'static str,
    pub module_path: &'static str,
    pub file: &'static str,
    pub checks: &'static [SafetyCheckRef],
}

inventory::collect!(SafetyCaseRef);

#[derive(Debug, Clone, Serialize)]
pub struct PackageSafetyIndex {
    pub package: String,
    pub cases: Vec<CollectedSafetyCase>,
}

#[derive(Debug, Clone, Serialize)]
pub struct CollectedSafetyCase {
    pub package: String,
    pub case_id: String,
    pub function: String,
    pub test_name: String,
    pub file: String,
    pub checks: Vec<CollectedSafetyCheck>,
}

#[derive(Debug, Clone, Serialize)]
pub struct CollectedSafetyCheck {
    pub check_id: String,
    pub requirement_id: String,
    pub kind: String,
}

pub fn collect_package_index(package: &str) -> PackageSafetyIndex {
    let mut cases: Vec<_> = inventory::iter::<SafetyCaseRef>
        .into_iter()
        .filter(|case_ref| case_ref.package == package)
        .map(|case_ref| CollectedSafetyCase {
            package: case_ref.package.to_string(),
            case_id: case_ref.case_id.to_string(),
            function: case_ref.function.to_string(),
            test_name: libtest_name(case_ref.module_path, case_ref.function),
            file: case_ref.file.to_string(),
            checks: case_ref
                .checks
                .iter()
                .map(|check| CollectedSafetyCheck {
                    check_id: check.check_id.to_string(),
                    requirement_id: check.requirement_id.to_string(),
                    kind: check.kind.to_string(),
                })
                .collect(),
        })
        .collect();

    cases.sort_by(|left, right| left.case_id.cmp(&right.case_id));
    for case in &mut cases {
        case.checks
            .sort_by(|left, right| left.check_id.cmp(&right.check_id));
    }

    PackageSafetyIndex {
        package: package.to_string(),
        cases,
    }
}

pub fn write_package_index_json(
    path: impl AsRef<Path>,
    index: &PackageSafetyIndex,
) -> std::io::Result<()> {
    let path = path.as_ref();
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let json = serde_json::to_vec_pretty(index).expect("safety index should serialize");
    fs::write(path, json)
}

fn libtest_name(module_path: &str, function: &str) -> String {
    let mut segments = module_path.split("::");
    let _crate_name = segments.next();
    let rest: Vec<_> = segments.collect();
    if rest.is_empty() {
        function.to_string()
    } else {
        format!("{}::{}", rest.join("::"), function)
    }
}
