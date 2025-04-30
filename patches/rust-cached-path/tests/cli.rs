use assert_cmd::prelude::*; // Add methods on commands
use predicates::prelude::*; // Used for writing assertions
use std::fs;
use std::io::Read;
use std::path::PathBuf;
use std::process::Command; // Run programs
use tempfile::tempdir;

#[test]
fn file_doesnt_exist() -> Result<(), Box<dyn std::error::Error>> {
    let mut cmd = Command::cargo_bin("cached-path")?;
    let cache_dir = tempdir().unwrap().path().to_owned();

    cmd.arg("--dir")
        .arg(cache_dir.to_str().unwrap())
        .arg("test/file/doesnt/exist");
    cmd.assert()
        .failure()
        .stderr(predicate::str::contains("file does not exist"));

    Ok(())
}

#[test]
fn test_remote_file() -> Result<(), Box<dyn std::error::Error>> {
    let mut cmd = Command::cargo_bin("cached-path")?;
    let cache_dir = tempdir().unwrap().path().to_owned();

    cmd.arg("--dir")
        .arg(cache_dir.to_str().unwrap())
        .arg("https://raw.githubusercontent.com/epwalsh/rust-cached-path/main/test_fixtures/utf-8_sample/utf-8_sample.txt");
    let result = cmd.assert().success();
    let output = result.get_output();
    let mut stdout = String::from_utf8(output.stdout.clone()).unwrap();
    // remove newline at the end.
    stdout.pop();
    let path = PathBuf::from(stdout);
    println!("{:?}", path);
    assert!(path.is_file());

    // Ensure cached version exactly matches local version.
    let mut cached_file = fs::File::open(&path)?;
    let mut cached_contents = String::new();
    cached_file.read_to_string(&mut cached_contents)?;

    let local_path: PathBuf = [".", "test_fixtures", "utf-8_sample", "utf-8_sample.txt"]
        .iter()
        .collect();
    assert!(local_path.is_file());
    let mut local_file = fs::File::open(local_path)?;
    let mut local_contents = String::new();
    local_file.read_to_string(&mut local_contents)?;

    // On Windows, git will automatically convert '\n' line-endings to '\r\n'.
    // So we change those back.
    let local_contents = local_contents.replace("\r\n", "\n");

    assert_eq!(local_contents, cached_contents);

    Ok(())
}

#[test]
fn test_extract_remote_file() -> Result<(), Box<dyn std::error::Error>> {
    let mut cmd = Command::cargo_bin("cached-path")?;
    let cache_dir = tempdir().unwrap().path().to_owned();

    cmd.arg("--dir")
        .arg(cache_dir.to_str().unwrap())
        .arg("--extract").arg(
            "https://raw.githubusercontent.com/epwalsh/rust-cached-path/main/test_fixtures/utf-8_sample/archives/utf-8.tar.gz"
        );
    let result = cmd.assert().success();
    let output = result.get_output();
    let mut stdout = String::from_utf8(output.stdout.clone()).unwrap();
    // remove newline at the end.
    stdout.pop();
    let path = PathBuf::from(stdout);
    println!("{:?}", path);
    assert!(path.is_dir());
    assert!(path.join("dummy.txt").is_file());
    assert!(path.join("folder").is_dir());
    assert!(path.join("folder").join("utf-8_sample.txt").is_file());

    Ok(())
}

#[test]
fn test_extract_local_file() -> Result<(), Box<dyn std::error::Error>> {
    let mut cmd = Command::cargo_bin("cached-path")?;
    let cache_dir = tempdir().unwrap().path().to_owned();

    cmd.arg("--dir")
        .arg(cache_dir.to_str().unwrap())
        .arg("--extract")
        .arg("test_fixtures/utf-8_sample/archives/utf-8.tar.gz");
    let result = cmd.assert().success();
    let output = result.get_output();
    let mut stdout = String::from_utf8(output.stdout.clone()).unwrap();
    // remove newline at the end.
    stdout.pop();
    let path = PathBuf::from(stdout);
    println!("{:?}", path);
    assert!(path.is_dir());
    assert!(path.join("dummy.txt").is_file());
    assert!(path.join("folder").is_dir());
    assert!(path.join("folder").join("utf-8_sample.txt").is_file());

    Ok(())
}
