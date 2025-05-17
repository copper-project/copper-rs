# Copper-rs NIX Development Environment

This repository contains the Nix configuration for setting up a comprehensive nix development environment for the Copper-rs project. The environment includes all necessary dependencies and tools for building and testing the project, with optional CUDA support.

This setup has been tested on Linux (x86_64).

## Prerequisites

- [Nix](https://nixos.org/download.html) package manager installed
    ```
    sh <(curl --proto '=https' --tlsv1.2 -L https://nixos.org/nix/install) --daemon
    ```
- For CUDA support: NVIDIA GPU with appropriate drivers installed

## Quick Start

### Basic Development Environment

To enter the development environment without CUDA support:

```bash
cd support/nix
nix-shell
```

This will set up a lightweight environment with all necessary dependencies for building and testing Copper-rs.

### Development Environment with CUDA Support

To enable CUDA support for GPU acceleration:

```bash
cd support/nix
nix-shell --arg withCuda true
```

This adds CUDA libraries and configures the environment for GPU-accelerated development.

## Environment Features

The development environment includes:

- **Rust Toolchain**: Stable Rust with components like rust-analyzer, clippy, and rustfmt
- **LLVM Dependencies**: Version 14 with proper configuration for bindgen
- **GStreamer**: Complete suite of GStreamer libraries and plugins
- **System Dependencies**: Common libraries like libpcap, udev, glib, openssl
- **Development Tools**: cargo-nextest, cargo-generate, and other useful tools
- **Optional CUDA Support**: NVIDIA CUDA toolkit and related libraries (when enabled)

## Feature Flags

When the environment is activated, it sets the `FEATURES_FLAG` environment variable with the following features:

```
macro_debug,mock,perf-ui,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode
```

When CUDA is enabled, the `cuda` feature is also added to this list.

## Working Directory

The shell automatically navigates to the project root directory (two levels up from the `support/nix` directory) when started.

## CUDA Configuration

When CUDA support is enabled:

1. CUDA toolkit and NVIDIA drivers are added to the environment
2. Environment variables like `CUDA_PATH` are configured appropriately
3. Library paths are set up to find CUDA libraries
4. The shell checks for available NVIDIA GPUs and displays information about them

## Troubleshooting

### LLVM Library Issues

If you encounter issues with LLVM libraries, the environment creates a symlink in `$HOME/.nix-llvm-libs` to ensure compatibility with bindgen and other tools that require specific library versions.

### CUDA Detection

If CUDA is enabled but your GPU isn't detected, verify that:
1. You have a compatible NVIDIA GPU
2. The NVIDIA drivers are properly installed
3. The `nvidia-smi` command works outside the Nix shell

### Unfree Packages

CUDA and NVIDIA drivers are marked as "unfree" in Nix. The shell configuration handles this automatically, but if you get permission errors, you may need to:

```bash
export NIXPKGS_ALLOW_UNFREE=1
```

before running `nix-shell`.

## Customization

The development environment can be customized by editing the `shell.nix` file. You can:

- Add additional packages to `buildInputs`
- Modify feature flags in the `shellHook`
- Change LLVM or CUDA versions
- Add new environment variables or configurations

