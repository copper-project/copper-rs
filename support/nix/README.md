# Copper-rs Development Environment

Nix flake providing a cross-platform Rust development environment with optional CUDA support.

## Prerequisites

- [Nix](https://nixos.org/download.html) package manager installed
    ```
    sh <(curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install) --daemon
    ```
- For CUDA support: NVIDIA GPU with appropriate drivers installed

## Quick Start

```bash
# Setup experimental feature
mkdir -p ~/.conf/nix && echo "experimental-features = nix-command flakes" >> ~/.conf/nix/nix.conf

# Setup flake
nix flake update

# Basic development environment
nix develop

# With CUDA support (Linux only)
nix develop #cuda
```

The environment automatically:

- Sets up Rust toolchain with essential components
- Configures all dependencies (GStreamer, LLVM, OpenSSL, system libraries)
- Navigates to project root and sets feature flags
- Displays platform and CUDA status

## Features

- **Cross-platform**: Linux and macOS support
- **CUDA support**: GPU acceleration on Linux systems
- **Pre-configured**: Comprehensive feature flags and environment variables
- **Complete toolchain**: Latest stable Rust with rust-analyzer, clippy, rustfmt

## Dependencies

**All platforms**: Rust toolchain, GStreamer, LLVM/Clang, OpenSSL, pkg-config
**Linux**: udev, libpcap, mold, optional CUDA toolkit
**macOS**: Apple frameworks (Security, CoreFoundation), libiconv

## Usage

```bash
# Build with pre-configured features
cargo build --workspace $FEATURES_FLAG

# Run tests
cargo nextest run --workspace $FEATURES_FLAG
```

## Environment Variables

- `FEATURES_FLAG`: Pre-configured feature set including `macro_debug,mock,perf-ui,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode` (plus `cuda` when enabled)
- `LLVM_CONFIG`, `LIBCLANG_PATH`: LLVM/Clang configuration
- Platform-specific library paths automatically configured

## Troubleshooting

**CUDA not accessible (Linux)**: Ensure NVIDIA drivers are installed, check `/dev/nvidia*` permissions
**macOS OpenSSL issues**: PKG_CONFIG_PATH is automatically configured
**LLVM library issues**: Symlinks created automatically in `$HOME/.nix-llvm-libs`

## Notes

- [BETA] CUDA support requires Linux with NVIDIA drivers.
- Uses LLVM 14 on Linux, latest on macOS
- Automatically navigates to `../..` (project root) on shell entry
- For order nix version on Linux, you can also you `nix-shell` and `nix-shell --arg withCuda true``
