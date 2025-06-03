{
  description = "Copper-rs development environment with cross-platform support and optional CUDA";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, rust-overlay }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
          config = { allowUnfree = true; };
        };

        # Platform detection
        isLinux = pkgs.stdenv.isLinux;
        isDarwin = pkgs.stdenv.isDarwin;

        # Use stable rust with specific components
        rustWithComponents = pkgs.rust-bin.stable.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" "clippy" "rustfmt" ];
        };
        
        # Choose LLVM version based on platform
        # Use latest LLVM on macOS to avoid version conflicts, LLVM 14 on Linux for consistency
        llvmPackages = if isDarwin then pkgs.llvmPackages else pkgs.llvmPackages_14;
        
        # Create a derivation that creates a symlink to the LLVM shared libraries (Linux only)
        llvmLibs = if isLinux then pkgs.symlinkJoin {
          name = "llvm-libs";
          paths = with llvmPackages; [ libllvm libclang clang ];
        } else null;

        # Function to create shell with optional CUDA support
        mkDevShell = withCuda: let
          # CUDA is only available on Linux
          effectiveCuda = withCuda && isLinux;
          
          # Platform-specific packages
          linuxOnlyPackages = with pkgs; pkgs.lib.optionals isLinux [
            # Linux-specific system dependencies
            udev
            libpcap
            mold
            
            # NVIDIA/CUDA packages (only if CUDA enabled)
          ] ++ (pkgs.lib.optionals effectiveCuda [
            cudatoolkit
            linuxPackages.nvidia_x11
            libGL
            libGLU
          ]);

          # macOS-specific packages
          darwinOnlyPackages = with pkgs; pkgs.lib.optionals isDarwin [
            # macOS-specific dependencies
            darwin.apple_sdk.frameworks.Security
            darwin.apple_sdk.frameworks.CoreFoundation
            darwin.apple_sdk.frameworks.SystemConfiguration
            libiconv
            
            # Network capture library (macOS version)
            libpcap
          ];
          
          # Cross-platform packages
          commonPackages = with pkgs; [
            # System dependencies (available on both platforms)
            pkg-config
            glib
            openssl

            # LLVM dependencies (version varies by platform)
            llvmPackages.clang
            llvmPackages.libclang
            llvmPackages.libllvm

            # GStreamer dependencies (available on both platforms)
            gst_all_1.gstreamer
            gst_all_1.gst-plugins-base
            gst_all_1.gst-plugins-good
            gst_all_1.gst-plugins-bad
            gst_all_1.gst-plugins-ugly
            gst_all_1.gst-devtools
            
            # Rust and tooling
            rustWithComponents
            cargo-nextest
            cargo-generate
          ];

          # Define feature flag string based on CUDA availability
          cudaFeatureFlag = if effectiveCuda then ",cuda" else "";
          
          # Platform-specific library paths
          linuxLibraryPaths = with pkgs; pkgs.lib.optionals isLinux ([
            # Linux-specific libraries
            libpcap
            udev
            glib
            openssl
            
            # GStreamer libraries
            gst_all_1.gstreamer
            gst_all_1.gst-plugins-base
            
            # LLVM libraries (LLVM 14 on Linux)
            llvmPackages.libllvm
            llvmPackages.libclang
            llvmPackages.clang
            "${llvmPackages.libllvm}/lib"
            "${llvmPackages.clang}/lib"
          ] ++ (if llvmLibs != null then [ "${llvmLibs}/lib" ] else [])
            ++ (pkgs.lib.optionals effectiveCuda [
              cudatoolkit
              linuxPackages.nvidia_x11
              libGL
            ]));

          # macOS library paths
          darwinLibraryPaths = with pkgs; pkgs.lib.optionals isDarwin [
            # macOS-specific libraries
            glib
            openssl
            libiconv
            libpcap
            
            # GStreamer libraries
            gst_all_1.gstreamer
            gst_all_1.gst-plugins-base
            
            # LLVM libraries (latest LLVM on macOS)
            llvmPackages.libllvm
            llvmPackages.libclang
            llvmPackages.clang
          ];
          
          # Combine all library paths
          allLibraryPaths = linuxLibraryPaths ++ darwinLibraryPaths;

          # Platform-specific shell hooks
          linuxShellHook = if isLinux then ''
            # Linux-specific LLVM library symlink setup
            mkdir -p $HOME/.nix-llvm-libs
            ln -sf ${llvmPackages.libllvm}/lib/libLLVM-14.so $HOME/.nix-llvm-libs/libLLVM-14.so.1 || true
            export LD_LIBRARY_PATH=$HOME/.nix-llvm-libs:${pkgs.lib.makeLibraryPath allLibraryPaths}:$LD_LIBRARY_PATH
            
            ${if effectiveCuda then ''
              # CUDA configuration (Linux only)
              export CUDA_PATH=${pkgs.cudatoolkit}
              export EXTRA_LDFLAGS="-L/lib -L${pkgs.linuxPackages.nvidia_x11}/lib"
              export EXTRA_CCFLAGS="-I/usr/include"
              
              # Check if device files exist but have permission issues
              if [ -e /dev/nvidia0 ]; then
                echo "NVIDIA devices exist but may have permission issues."
                echo "Please use sudo nvidia-smi to check if GPU is detected"

                # Attempt to fix permissions if user has sudo access
                if command -v sudo &> /dev/null && sudo -n true 2>/dev/null; then
                  sudo nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
                fi
              else
                echo "Warning: No NVIDIA GPU devices detected (/dev/nvidia*)."
                echo "Make sure NVIDIA drivers are properly installed on your system."
              fi
            '' else ""}
          '' else "";

          darwinShellHook = if isDarwin then ''
            # macOS-specific library path setup
            export DYLD_LIBRARY_PATH=${pkgs.lib.makeLibraryPath allLibraryPaths}:$DYLD_LIBRARY_PATH
            
            # macOS-specific OpenSSL configuration
            export PKG_CONFIG_PATH="${pkgs.openssl.dev}/lib/pkgconfig:$PKG_CONFIG_PATH"
            
            # Ensure consistent LLVM/Clang usage on macOS
            export CC="${llvmPackages.clang}/bin/clang"
            export CXX="${llvmPackages.clang}/bin/clang++"
            
            # macOS-specific pcap library path
            export LIBPCAP_LIBDIR="${pkgs.libpcap}/lib"
            export PKG_CONFIG_PATH="${pkgs.libpcap}/lib/pkgconfig:$PKG_CONFIG_PATH"
          '' else "";

          # Platform name for display
          platformName = if isDarwin then "macOS" else "Linux";
          cudaStatus = if effectiveCuda then " with CUDA support" else 
                      if withCuda && isDarwin then " (CUDA not available on macOS)" else "";
        in
        pkgs.mkShell {
          name = "copper-rs-dev-${platformName}${if effectiveCuda then "-cuda" else ""}-env";
          
          buildInputs = commonPackages ++ linuxOnlyPackages ++ darwinOnlyPackages;

          # Environment variables and shell hook
          shellHook = ''
            # Change to parent directory (../..): Navigate up two levels
            cd ../..
            
            # Cargo configuration
            export CARGO_TERM_COLOR=always
            export FEATURES_FLAG="--features macro_debug,mock,perf-ui,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode${cudaFeatureFlag}"
            
            # LLVM configuration (platform-specific versions)
            export LLVM_CONFIG=${llvmPackages.llvm}/bin/llvm-config
            export LIBCLANG_PATH="${llvmPackages.libclang.lib}/lib"
            
            # Platform-specific configurations
            ${linuxShellHook}
            ${darwinShellHook}
            
            # Bindgen configuration (platform-specific LLVM versions)
            export BINDGEN_EXTRA_CLANG_ARGS="-I${llvmPackages.libclang.lib}/lib/clang/${llvmPackages.libclang.version}/include -I${pkgs.glib.dev}/include/glib-2.0 -I${pkgs.glib.out}/lib/glib-2.0/include"
            
            # Display current directory and environment status
            echo "Copper-rs development environment activated on ${platformName}${cudaStatus}!"
            echo "Working directory: $(pwd)"
            ${if withCuda && isDarwin then ''
              echo "Note: CUDA support is only available on Linux systems."
            '' else ""}
          '';
        };
      in
      {
        # Default development shell (without CUDA)
        devShells.default = mkDevShell false;
        
        # CUDA-enabled development shell (Linux only)
        devShells.cuda = mkDevShell true;
        
        # Legacy compatibility
        devShell = mkDevShell false;
      }
    );
}
