# CI-aligned helpers mirroring .github/workflows/general.yml
BASE_FEATURES := "mock,perf-ui,image,kornia,gst,faer,nalgebra,glam,debug_pane,bincode,log-level-debug"
WINDOWS_BASE_FEATURES := "mock,perf-ui,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode"
export ROOT := `git rev-parse --show-toplevel`
EMBEDDED_EXCLUDES := shell('python3 $1/support/ci/embedded_crates.py excludes', ROOT)

default:
	just std-ci


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

ci:
  act -W .github/workflows/general.yml -j Unit-Tests --matrix os:ubuntu-latest --matrix mode:debug -P ubuntu-latest=ghcr.io/catthehacker/ubuntu:act-latest

nostd-ci: lint
	cargo +stable build --no-default-features
	cargo +stable nextest run --no-default-features
	python3 support/ci/embedded_crates.py run --action clippy
	python3 support/ci/embedded_crates.py run --action build
	cd examples/cu_rp2350_skeleton && cargo +stable clippy
	cd examples/cu_rp2350_skeleton && cargo +stable build-arm
	cd examples/cu_rp2350_skeleton && cargo +stable build-logreader

# Std-specific CI flow (local, CI-aligned). Use mode=release or mode=cuda-release as needed.
std-ci mode="debug": lint
	#!/usr/bin/env bash
	set -euo pipefail

	mode="{{mode}}"
	release_flag=""
	if [[ "$mode" == "release" || "$mode" == "cuda-release" ]]; then
		release_flag="--release"
	fi

	os="$(uname -s || true)"
	features="{{BASE_FEATURES}}"
	case "$os" in
		Linux*) features="{{BASE_FEATURES}},python" ;;
		Darwin*) features="{{BASE_FEATURES}}" ;;
		MINGW*|MSYS*|CYGWIN*|Windows_NT) features="{{WINDOWS_BASE_FEATURES}}" ;;
		*) features="{{BASE_FEATURES}}" ;;
	esac
	if [[ "$mode" == "cuda-release" ]]; then
		features="${features},cuda"
	fi
	features_flag="--features $features"
	embedded_excludes="{{EMBEDDED_EXCLUDES}}"

	cargo +stable clippy $release_flag --workspace --all-targets $embedded_excludes -- --deny warnings
	cargo +stable clippy $release_flag --workspace --all-targets $features_flag $embedded_excludes -- --deny warnings
	cargo +stable build $release_flag --workspace --all-targets $features_flag $embedded_excludes

	if [[ "$mode" == "debug" ]]; then
		cargo +stable test --doc --workspace $embedded_excludes --quiet
	fi

	cargo +stable nextest run $release_flag --all-targets --workspace $embedded_excludes
	cargo +stable nextest run $release_flag --all-targets --workspace $features_flag $embedded_excludes

	if [[ "$mode" == "debug" ]]; then
		if ! cargo +stable generate --version >/dev/null 2>&1; then
			cargo +stable install cargo-generate
		fi
		(
			cd templates
			cargo +stable generate -p cu_full --name test_project --destination . -d copper_source=local -d copper_root_path=../.. --silent
		)
		(
			cd templates/test_project
			cargo +stable build
		)
	fi

# Project-specific helpers now live in per-directory justfiles under examples/, components/, and support/.

# Render copperconfig.ron from the current working directory.
dag mission="":
	#!/usr/bin/env bash
	set -euo pipefail

	invocation_dir="{{invocation_directory()}}"
	cfg_path="${invocation_dir}/copperconfig.ron"
	if [[ ! -f "$cfg_path" ]]; then
		echo "No copperconfig.ron found in ${invocation_dir}" >&2
		exit 1
	fi

	cd "{{ROOT}}"
	mission_value="{{mission}}"
	if [[ "$mission_value" == mission=* ]]; then
		mission_value="${mission_value#mission=}"
	fi
	mission_arg=()
	if [[ -n "$mission_value" ]]; then
		mission_arg=(--mission "$mission_value")
	fi
	cargo run -p cu29-runtime --bin cu29-rendercfg -- "${mission_arg[@]}" --open "$cfg_path"
