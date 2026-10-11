use std::fs;
use std::path::PathBuf;
use std::process::Command;

#[cfg(unix)]
#[test]
fn generates_project_without_user_env() {
    let tempdir = tempfile::tempdir().expect("tempdir");
    let home = tempdir.path().join("home");
    let xdg = tempdir.path().join("xdg");
    fs::create_dir_all(&home).expect("home dir");
    fs::create_dir_all(&xdg).expect("xdg dir");

    let project = tempdir.path().join("hello-copper");
    let copper_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("repo root");

    let output = Command::new(env!("CARGO_BIN_EXE_cargo-cunew"))
        .arg(&project)
        .arg("--template")
        .arg("project")
        .arg("--source")
        .arg("local")
        .arg("--copper-root")
        .arg(&copper_root)
        .current_dir(tempdir.path())
        .env("HOME", &home)
        .env("XDG_CONFIG_HOME", &xdg)
        .env("GIT_CONFIG_GLOBAL", home.join("gitconfig"))
        .env("GIT_CONFIG_NOSYSTEM", "1")
        .env_remove("USER")
        .env_remove("USERNAME")
        .env_remove("CARGO_NAME")
        .env_remove("GIT_AUTHOR_NAME")
        .env_remove("GIT_COMMITTER_NAME")
        .env_remove("NAME")
        .env_remove("CARGO_EMAIL")
        .env_remove("GIT_AUTHOR_EMAIL")
        .env_remove("GIT_COMMITTER_EMAIL")
        .env_remove("EMAIL")
        .output()
        .expect("run cargo-cunew");

    assert!(
        output.status.success(),
        "stdout:\n{}\n\nstderr:\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(project.join("Cargo.toml").is_file());
}
