# CI-aligned helpers mirroring .github/workflows/general.yml
BASE_FEATURES := "macro_debug,mock,perf-ui,image,kornia,gst,faer,nalgebra,glam,debug_pane,bincode,log-level-debug"
WINDOWS_BASE_FEATURES := "macro_debug,mock,perf-ui,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode"
export ROOT := `git rev-parse --show-toplevel`
EMBEDDED_EXCLUDES := shell('python3 $1/support/ci/embedded_crates.py excludes', ROOT)

ci:
  act -W .github/workflows/general.yml -j Unit-Tests --matrix os:ubuntu-latest --matrix mode:debug -P ubuntu-latest=ghcr.io/catthehacker/ubuntu:act-latest

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
