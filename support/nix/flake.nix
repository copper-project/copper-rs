# flake.nix
{
  description = "Copper-rs development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };
        
        # Use stable rust with the components we need
        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" "clippy" "rustfmt" ];
        };

        # Define common dependencies
        commonDeps = with pkgs; [
          # System dependencies
          pkg-config
          udev
          libpcap
          glib
          openssl
          clang
          
          # GStreamer dependencies
          gst_all_1.gstreamer
          gst_all_1.gst-plugins-base
          gst_all_1.gst-plugins-good
          gst_all_1.gst-plugins-bad
          gst_all_1.gst-plugins-ugly
          gst_all_1.gst-devtools
        ];

        # Define platform-specific dependencies
        linuxDeps = with pkgs; lib.optionals pkgs.stdenv.isLinux [
          # Linux-specific dependencies
        ];
        
        macDeps = with pkgs; lib.optionals pkgs.stdenv.isDarwin [
          # macOS-specific dependencies
          darwin.apple_sdk.frameworks.CoreServices
          darwin.apple_sdk.frameworks.CoreFoundation
        ];
        
        # Environment variables
        commonEnv = {
          CARGO_TERM_COLOR = "always";
          FEATURES_FLAG = "--features macro_debug,mock,perf-ui,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode";
        };
        
        # CI-specific environment settings
        ciEnv = commonEnv // {
          # CI-specific environment variables
        };
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = [
            # Rust and tooling
            rustToolchain
            pkgs.cargo-nextest
            pkgs.cargo-generate
          ] ++ commonDeps ++ linuxDeps ++ macDeps;
          
          inherit (commonEnv) CARGO_TERM_COLOR FEATURES_FLAG;
          
          # Ensure libraries are discoverable
          LD_LIBRARY_PATH = with pkgs; lib.makeLibraryPath [
            libpcap
            udev
            glib
            openssl
            gst_all_1.gstreamer
            gst_all_1.gst-plugins-base
          ];
          
          # Additional shell hook for setup
          shellHook = ''
            echo "Copper-rs development environment activated!"
          '';
        };
        
        # Define a CI shell specifically for GitHub Actions
        devShells.ci = pkgs.mkShell {
          buildInputs = [
            # Rust and tooling for CI
            rustToolchain
            pkgs.cargo-nextest
            pkgs.cargo-generate
          ] ++ commonDeps ++ linuxDeps;
          
          inherit (ciEnv) CARGO_TERM_COLOR FEATURES_FLAG;
          
          # Ensure libraries are discoverable
          LD_LIBRARY_PATH = with pkgs; lib.makeLibraryPath [
            libpcap
            udev
            glib
            openssl
            gst_all_1.gstreamer
            gst_all_1.gst-plugins-base
          ];
        };
        
        # CUDA-enabled development shell
        devShells.cuda = pkgs.mkShell {
          buildInputs = [
            # Rust and tooling
            rustToolchain
            pkgs.cargo-nextest
            pkgs.cargo-generate
            
            # CUDA toolkit
            pkgs.cudaPackages.cudatoolkit
          ] ++ commonDeps ++ linuxDeps;
          
          # Enable all features including CUDA
          CARGO_TERM_COLOR = "always";
          FEATURES_FLAG = "--all-features";
          
          # CUDA-specific environment variables
          CUDA_PATH = "${pkgs.cudaPackages.cudatoolkit}";
          
          # Ensure libraries are discoverable
          LD_LIBRARY_PATH = with pkgs; lib.makeLibraryPath ([
            libpcap
            udev
            glib
            openssl
            gst_all_1.gstreamer
            gst_all_1.gst-plugins-base
            cudaPackages.cudatoolkit
          ]);
          
          shellHook = ''
            echo "Copper-rs CUDA development environment activated!"
          '';
        };
      }
    );
}
