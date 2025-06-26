use std::{
    fs,
    path::PathBuf,
    process::{Command, Stdio},
};

fn main() {
    let main_rs = PathBuf::from("src/main.rs");
    if !main_rs.exists() {
        eprintln!("[cubuild] Error: src/main.rs not found in current directory");
        std::process::exit(1);
    }

    let backup = main_rs.with_extension("rs.bak");

    println!("[cubuild] Backing up main.rs to {}", backup.display());

    if let Err(code) = try_main(&main_rs, &backup) {
        cleanup_and_exit(&backup, &main_rs, code);
    } else {
        cleanup_and_exit(&backup, &main_rs, 0);
    }
}

fn try_main(main_rs: &PathBuf, backup: &PathBuf) -> Result<(), i32> {
    fs::copy(main_rs, backup).expect("Failed to backup main.rs");

    println!("[cubuild] Expanding the macros... ");
    let output = Command::new("cargo")
        .env("RUSTFLAGS", "--cfg feature=\"cu29/macro_debug\"")
        .arg("check")
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .output()
        .expect("Failed to run cargo build with macro_debug");

    let stderr = String::from_utf8_lossy(&output.stderr);
    let expanded = match extract_expansion(&stderr) {
        Some(e) => e,
        None => {
            eprintln!("[cubuild] Could not extract macro expansion from output:");
            eprintln!("{}", stderr);
            return Err(1);
        }
    };
    println!("[cubuild] Reinjecting the macros... ");

    let original = fs::read_to_string(backup).expect("Failed to read main.rs");
    let patched = inject_generated_code(&original, &expanded);
    fs::write(main_rs, patched).expect("Failed to write patched main.rs");

    println!("[cubuild] Recompiling with the injected macros... ");
    let status = Command::new("cargo")
        .env("RUSTFLAGS", "--cfg feature=\"cu29/macro_debug\"")
        .arg("check")
        .status()
        .expect("cargo check failed");

    if !status.success() {
        return Err(status.code().unwrap_or(1));
    }

    Ok(())
}

fn cleanup_and_exit(backup: &PathBuf, main_rs: &PathBuf, code: i32) -> ! {
    if backup.exists() {
        println!("[cubuild] Restoring original main.rs from backup");
        let _ = fs::rename(backup, main_rs);
    }
    std::process::exit(code);
}

const START_TAG: &str = "===    Gen. Runtime ===";
const END_TAG: &str = "=== === === === === ===";

fn extract_expansion(stderr: &str) -> Option<String> {
    let start = stderr.find(START_TAG)? + START_TAG.len();
    let end = stderr[start..].find(END_TAG)? + start;
    stderr.get(start..end).map(|s| s.trim().to_string())
}

fn inject_generated_code(original: &str, generated: &str) -> String {
    let mut output = String::new();
    let mut in_macro_block = false;
    for line in original.lines() {
        if line.contains("#[copper_runtime") {
            in_macro_block = true;
            continue;
        }
        if in_macro_block && line.contains("struct") {
            output.push_str(generated);
            output.push('\n');
            in_macro_block = false;
            continue;
        }
        if !in_macro_block {
            output.push_str(line);
            output.push('\n');
        }
    }
    output
}
