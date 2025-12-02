# CI-aligned helpers mirroring .github/workflows/general.yml
BASE_FEATURES := "macro_debug,mock,perf-ui,image,kornia,gst,faer,nalgebra,glam,debug_pane,bincode,log-level-debug"
WINDOWS_BASE_FEATURES := "macro_debug,mock,perf-ui,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode"
export ROOT := `git rev-parse --show-toplevel`
EMBEDDED_EXCLUDES := shell('python3 $1/support/ci/embedded_crates.py excludes', ROOT)

# Run the main CI flow for the current host OS. Call like `just ci debug`.
# Available modes: debug | release | cuda-release.
ci mode="debug":
	#!/usr/bin/env bash
	set -euo pipefail

	mode="{{mode}}"
	release_flag=""
	if [ "$mode" = "release" ] || [ "$mode" = "cuda-release" ]; then
		release_flag="--release"
	fi

	os=$(uname -s)
	if [ "$os" = "Linux" ]; then
		if [ "$mode" = "cuda-release" ]; then
			features="--features {{BASE_FEATURES}},python,cuda"
		else
			features="--features {{BASE_FEATURES}},python"
		fi
	elif [ "$os" = "Darwin" ]; then
		if [ "$mode" = "cuda-release" ]; then
			features="--features {{BASE_FEATURES}},cuda"
		else
			features="--features {{BASE_FEATURES}}"
		fi
	else
		if [ "$mode" = "cuda-release" ]; then
			features="--features {{WINDOWS_BASE_FEATURES}},cuda"
		else
			features="--features {{WINDOWS_BASE_FEATURES}}"
		fi
	fi

	cargo +stable clippy ${release_flag} --workspace --all-targets {{EMBEDDED_EXCLUDES}} -- --deny warnings
	cargo +stable clippy ${release_flag} --workspace --all-targets ${features} {{EMBEDDED_EXCLUDES}} -- --deny warnings
	cargo +stable build ${release_flag} --workspace --all-targets ${features} {{EMBEDDED_EXCLUDES}}

	if [ "$mode" = "debug" ]; then
		cargo +stable test --doc --workspace {{EMBEDDED_EXCLUDES}}
	fi

	cargo +stable nextest run ${release_flag} --all-targets --workspace {{EMBEDDED_EXCLUDES}}
	cargo +stable nextest run ${release_flag} --all-targets --workspace ${features} {{EMBEDDED_EXCLUDES}}

	if [ "$mode" = "debug" ]; then
		cargo +stable install cargo-generate
		rm -rf templates/test_project
		cd templates
		cargo +stable generate -p cu_full --name test_project --destination . -d copper_source=local -d copper_root_path=../.. --silent
		cd test_project
		cargo +stable build
	fi

# Formatting and typo checks mirroring the dedicated workflow job.
lint:
	just fmt-check
	just typos

# Formatting check only
fmt-check:
	cargo +stable fmt --all -- --check

# Typo check only
typos:
	typos -c .config/_typos.toml

# Embedded-specific CI flow.
embedded-ci:
	cargo +stable build --no-default-features
	cargo +stable nextest run --no-default-features
	python3 support/ci/embedded_crates.py run --action clippy
	python3 support/ci/embedded_crates.py run --action build
	cd examples/cu_rp2350_skeleton && cargo +stable clippy
	cd examples/cu_rp2350_skeleton && cargo +stable build-arm
	cd examples/cu_rp2350_skeleton && cargo +stable build-logreader

# Project-specific helpers now live in per-directory justfiles under examples/, components/, and support/.
