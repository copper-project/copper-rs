{ 
  pkgs ? import <nixpkgs> { config = { allowUnfree = true; }; },
  withCuda ? false  # Parameter to enable/disable CUDA support
}:

let
  # Specify the rust version we want to use
  rustOverlay = import (builtins.fetchTarball "https://github.com/oxalica/rust-overlay/archive/master.tar.gz");
  pkgsWithRustOverlay = import <nixpkgs> { 
    overlays = [ rustOverlay ];
    config = { allowUnfree = true; };
  };

  # Use stable rust with specific components
  rustWithComponents = pkgsWithRustOverlay.rust-bin.stable.latest.default.override {
    extensions = [ "rust-src" "rust-analyzer" "clippy" "rustfmt" ];
  };
  
  # Create a derivation that creates a symlink to the LLVM shared libraries
  llvmLibs = pkgs.symlinkJoin {
    name = "llvm-libs";
    paths = with pkgs.llvmPackages_14; [ libllvm libclang clang ];
  };
  
  # CUDA-specific packages
  cudaPackages = with pkgs; if withCuda then [
    cudatoolkit
    linuxPackages.nvidia_x11 # nvidia driver 570.133.20
    libGL
    libGLU
  ] else [];
  
  # Define feature flag string based on CUDA availability
  cudaFeatureFlag = if withCuda then ",cuda" else "";
  
  # Basic library paths
  baseLibraryPaths = with pkgs; [
    # System libraries
    libpcap
    udev
    glib
    openssl
    
    # GStreamer libraries
    gst_all_1.gstreamer
    gst_all_1.gst-plugins-base
    
    # LLVM libraries
    llvmPackages_14.libllvm
    llvmPackages_14.libclang
    llvmPackages_14.clang
    "${llvmPackages_14.libllvm}/lib"
    "${llvmPackages_14.clang}/lib"
    "${llvmLibs}/lib"
  ];
  
  # CUDA library paths
  cudaLibraryPaths = with pkgs; if withCuda then [
    cudatoolkit
    linuxPackages.nvidia_x11
    libGL
  ] else [];
  
  # Combine library paths
  allLibraryPaths = baseLibraryPaths ++ cudaLibraryPaths;
in
pkgs.mkShell {
  name = if withCuda then "copper-rs-dev-cuda-env" else "copper-rs-dev-env";
  
  buildInputs = with pkgs; [
    # System dependencies
    pkg-config
    udev
    libpcap
    glib
    openssl
    mold

    # LLVM dependencies
    clang
    libclang
    llvmPackages_14.clang
    llvmPackages_14.libclang
    llvmPackages_14.libllvm

    # GStreamer dependencies
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
  ] ++ cudaPackages;  # Conditionally add CUDA packages

  # Environment variables
  shellHook = ''
    # Change to parent directory (../..): Navigate up two levels
    cd ../..
    
    # Cargo configuration
    export CARGO_TERM_COLOR=always
    export FEATURES_FLAG="--features macro_debug,mock,perf-ui,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode${cudaFeatureFlag}"
    
    # LLVM configuration
    export LLVM_CONFIG=${pkgs.llvmPackages_14.llvm}/bin/llvm-config
    export LIBCLANG_PATH="${pkgs.llvmPackages_14.libclang.lib}/lib"
    
    # Create directory with symlink for libLLVM-14.so.1
    mkdir -p $HOME/.nix-llvm-libs
    ln -sf ${pkgs.llvmPackages_14.libllvm}/lib/libLLVM-14.so $HOME/.nix-llvm-libs/libLLVM-14.so.1 || true
    export LD_LIBRARY_PATH=$HOME/.nix-llvm-libs:${pkgs.lib.makeLibraryPath allLibraryPaths}:$LD_LIBRARY_PATH
    
    # Bindgen configuration
    export BINDGEN_EXTRA_CLANG_ARGS="-I${pkgs.llvmPackages_14.libclang.lib}/lib/clang/${pkgs.llvmPackages_14.libclang.version}/include -I${pkgs.glib.dev}/include/glib-2.0 -I${pkgs.glib.out}/lib/glib-2.0/include"
    
    # CUDA configuration if enabled
    ${if withCuda then ''
      # CUDA configuration (following nixos.wiki/wiki/CUDA recommendations)
      export CUDA_PATH=${pkgs.cudatoolkit}
      export EXTRA_LDFLAGS="-L/lib -L${pkgs.linuxPackages.nvidia_x11}/lib"
      export EXTRA_CCFLAGS="-I/usr/include"
      
      # Check if device files exist but have permission issues
      if [ -e /dev/nvidia0 ]; then
        echo "NVIDIA devices exist but may have permission issues."
        echo "Please use sudo nvidia-smi to check if GPU is detected"

        # Attempt to fix permissions if user has sudo access
        if command -v sudo &> /dev/null && sudo -n true 2>/dev/null; then
          # echo "Attempting to fix NVIDIA device permissions (one-time)..."
          # sudo chmod 666 /dev/nvidia* 2>/dev/null
          sudo nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
        fi
      else
        echo "Warning: No NVIDIA GPU devices detected (/dev/nvidia*)."
        echo "Make sure NVIDIA drivers are properly installed on your system."
      fi
    '' else ""}
    
    # Display current directory and environment status
    ${if withCuda then ''
      echo "Copper-rs development environment activated with CUDA support!"
    '' else ''
      echo "Copper-rs development environment activated!"
    ''}
    echo "Working directory: $(pwd)"
  '';
}
