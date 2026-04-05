use std::env;
use std::ffi::OsStr;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=src/local_shim.cpp");
    println!("cargo:rerun-if-changed=vendor/zed-c-api/CMakeLists.txt");
    println!("cargo:rerun-if-changed=vendor/zed-c-api/include/sl/c_api/zed_interface.h");
    println!("cargo:rerun-if-changed=vendor/zed-c-api/include/sl/c_api/types_c.h");
    println!("cargo:rerun-if-changed=vendor/zed-c-api/src/zed_interface.cpp");
    println!("cargo:rerun-if-changed=vendor/zed-c-api/src/ZEDController.cpp");
    println!("cargo:rerun-if-changed=/usr/local/zed");
    println!("cargo:rerun-if-changed=/opt/zed-sdk");
    println!("cargo:rerun-if-changed=/usr/lib/cmake/zed/zed-config.cmake");
    println!("cargo:rerun-if-env-changed=ZED_DIR");
    println!("cargo:rustc-check-cfg=cfg(zed_sdk_sys_has_native)");

    if env::var("CARGO_CFG_TARGET_OS").ok().as_deref() != Some("linux") {
        println!("cargo:warning=zed-sdk-sys currently only auto-detects native libraries on Linux");
        return;
    }

    if let Some(installed_dir) = find_installed_wrapper() {
        emit_native_link(&installed_dir);
        build_local_shim().unwrap_or_else(|err| panic!("failed to build local ZED shim: {err}"));
        println!("cargo:rustc-cfg=zed_sdk_sys_has_native");
        return;
    }

    match try_build_vendored_wrapper() {
        Ok(built_dir) => {
            emit_native_link(&built_dir);
            build_local_shim()
                .unwrap_or_else(|err| panic!("failed to build local ZED shim: {err}"));
            println!("cargo:rustc-cfg=zed_sdk_sys_has_native");
        }
        Err(err) => {
            println!("cargo:warning=zed-sdk-sys native wrapper unavailable: {err}");
        }
    }
}

fn emit_native_link(lib_dir: &Path) {
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=dylib=sl_zed_c");

    if let Some(sdk_root) = find_sdk_root() {
        let sdk_lib_dir = sdk_root.join("lib");
        if sdk_lib_dir.exists() {
            println!("cargo:rustc-link-search=native={}", sdk_lib_dir.display());
        }
    }
}

fn build_local_shim() -> Result<(), String> {
    let sdk_root = find_sdk_root().ok_or_else(|| {
        "expected the ZED SDK under /usr/local/zed or /opt/zed-sdk; install the Stereolabs SDK first"
            .to_owned()
    })?;
    let manifest_dir =
        PathBuf::from(env::var("CARGO_MANIFEST_DIR").map_err(|err| err.to_string())?);
    let shim_path = manifest_dir.join("src/local_shim.cpp");
    if !shim_path.exists() {
        return Err(format!(
            "missing local shim source at {}",
            shim_path.display()
        ));
    }

    let mut build = cc::Build::new();
    build
        .cpp(true)
        .file(&shim_path)
        .include(sdk_root.join("include"))
        .flag_if_supported("-std=c++17");

    if let Some(cuda_include) = find_cuda_include() {
        build.include(cuda_include);
    }

    build.compile("zed_sdk_sys_local_shim");

    println!("cargo:rustc-link-lib=dylib=sl_zed");
    println!("cargo:rustc-link-lib=dylib=stdc++");

    Ok(())
}

fn find_installed_wrapper() -> Option<PathBuf> {
    let candidates = [
        "/usr/local/zed/lib/libsl_zed_c.so",
        "/usr/local/zed/lib/libsl_zed_c.so.1",
        "/opt/zed-sdk/lib/libsl_zed_c.so",
        "/opt/zed-sdk/lib/libsl_zed_c.so.1",
    ];

    candidates
        .iter()
        .map(Path::new)
        .find(|path| path.exists())
        .and_then(Path::parent)
        .map(Path::to_path_buf)
}

fn find_sdk_root() -> Option<PathBuf> {
    let candidates = ["/usr/local/zed", "/opt/zed-sdk"];
    candidates
        .iter()
        .map(Path::new)
        .find(|path| path.join("include").exists() && path.join("lib").exists())
        .map(Path::to_path_buf)
}

fn find_zed_cmake_dir() -> Option<PathBuf> {
    if let Ok(path) = env::var("ZED_DIR") {
        let candidate = PathBuf::from(path);
        if candidate.join("zed-config.cmake").exists() || candidate.join("ZEDConfig.cmake").exists()
        {
            return Some(candidate);
        }
    }

    let candidates = ["/usr/lib/cmake/zed", "/opt/zed-sdk", "/usr/local/zed"];
    candidates
        .iter()
        .map(Path::new)
        .find(|path| {
            path.join("zed-config.cmake").exists() || path.join("ZEDConfig.cmake").exists()
        })
        .map(Path::to_path_buf)
}

fn find_cuda_include() -> Option<PathBuf> {
    let candidates = [
        "/usr/local/cuda/include",
        "/opt/cuda/include",
        "/usr/include",
    ];
    candidates
        .iter()
        .map(Path::new)
        .find(|path| path.join("cuda_runtime.h").exists())
        .map(Path::to_path_buf)
}

fn try_build_vendored_wrapper() -> Result<PathBuf, String> {
    let manifest_dir =
        PathBuf::from(env::var("CARGO_MANIFEST_DIR").map_err(|err| err.to_string())?);
    let vendor_dir = manifest_dir.join("vendor/zed-c-api");
    let cmake_lists = vendor_dir.join("CMakeLists.txt");
    if !cmake_lists.exists() {
        return Err(
            "missing vendored zed-c-api sources; run `git submodule update --init --recursive`"
                .into(),
        );
    }

    let sdk_root = find_sdk_root().ok_or_else(|| {
        "expected the ZED SDK under /usr/local/zed or /opt/zed-sdk; install the Stereolabs SDK first"
            .to_owned()
    })?;

    ensure_command_exists("cmake")?;

    let out_dir = PathBuf::from(env::var("OUT_DIR").map_err(|err| err.to_string())?);
    let build_dir = out_dir.join("zed-c-api-build");
    fs::create_dir_all(&build_dir).map_err(|err| err.to_string())?;

    let mut configure = Command::new("cmake");
    configure
        .arg("-S")
        .arg(&vendor_dir)
        .arg("-B")
        .arg(&build_dir)
        .arg("-DCMAKE_POLICY_VERSION_MINIMUM=3.5")
        .arg("-DCMAKE_BUILD_TYPE=Release")
        .arg(format!("-DCMAKE_PREFIX_PATH={}", sdk_root.display()))
        .arg(format!(
            "-DCMAKE_LIBRARY_PATH={}",
            sdk_root.join("lib").display()
        ))
        .arg(format!(
            "-DCMAKE_INCLUDE_PATH={}",
            sdk_root.join("include").display()
        ))
        .arg(format!(
            "-DCMAKE_BUILD_RPATH={}",
            sdk_root.join("lib").display()
        ));

    if let Some(zed_cmake_dir) = find_zed_cmake_dir() {
        configure.arg(format!("-DZED_DIR={}", zed_cmake_dir.display()));
    }

    run(&mut configure, "configure vendored zed-c-api")?;

    run(
        Command::new("cmake")
            .arg("--build")
            .arg(&build_dir)
            .arg("--parallel"),
        "build vendored zed-c-api",
    )?;

    find_library_dir(&build_dir, "libsl_zed_c.so").ok_or_else(|| {
        format!(
            "built vendored zed-c-api but could not locate libsl_zed_c.so under {}",
            build_dir.display()
        )
    })
}

fn ensure_command_exists(cmd: &str) -> Result<(), String> {
    let status = Command::new(cmd)
        .arg("--version")
        .status()
        .map_err(|_| format!("required command `{cmd}` was not found"))?;
    if status.success() {
        Ok(())
    } else {
        Err(format!("required command `{cmd}` is not usable"))
    }
}

fn run(command: &mut Command, context: &str) -> Result<(), String> {
    let debug = format!("{command:?}");
    let output = command
        .output()
        .map_err(|err| format!("failed to {context}: {err}"))?;
    if output.status.success() {
        return Ok(());
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    Err(format!(
        "{context} failed with command `{debug}`\nstdout:\n{stdout}\nstderr:\n{stderr}"
    ))
}

fn find_library_dir(root: &Path, needle: &str) -> Option<PathBuf> {
    let mut stack = vec![root.to_path_buf()];
    while let Some(dir) = stack.pop() {
        let entries = fs::read_dir(&dir).ok()?;
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
                continue;
            }

            if path.file_name() == Some(OsStr::new(needle)) {
                return path.parent().map(Path::to_path_buf);
            }
        }
    }
    None
}
