# CI-aligned helpers mirroring .github/workflows/general.yml
BASE_FEATURES := "mock,cu-sensor-payloads/image,kornia,gst,faer,nalgebra,glam,debug_pane,bincode,log-level-debug"
WINDOWS_BASE_FEATURES := "mock,cu-sensor-payloads/image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode"
MSRV := "1.95.0"
PUBLIC_API_VERSION := "0.51.0"
PUBLIC_API_TOOLCHAIN := "nightly"
export ROOT := justfile_directory()
WORKSPACE_EXCLUDES := shell('python3 $1/support/ci/workspace_excludes.py excludes --toolchain stable', ROOT)
PREK_FMT_FIX_HOOKS := "trailing-whitespace mixed-line-ending"
PREK_FMT_CHECK_HOOKS := "trailing-whitespace check-merge-conflict detect-private-key check-case-conflict check-added-large-files check-yaml check-json check-xml check-symlinks mixed-line-ending"
PREK_FMT_CI_HOOKS := "trailing-whitespace check-merge-conflict detect-private-key check-case-conflict check-added-large-files check-yaml check-json check-xml check-symlinks mixed-line-ending typos check-rust check-ron check-toml"

# Default to the local PR-check workflow.
default:
	just pr-check

# Local PR pipeline: format, lint, then tests.
pr-check:
	just fmt
	just lint
	just api-check
	just test

# Formatting, typo, and clippy checks.
lint:
	just fmt-check
	just typos
	just clippy-std
	just clippy-nostd

# Formatting and CI-aligned file-hygiene check only
fmt-check: check-format-tools
	@cargo +stable fmt --all -- --check
	@git ls-files -z '*.toml' | xargs -0 -r env RUST_LOG=warn taplo format --check
	@bash support/ci/check_ron_format.sh
	@prek run --all-files {{PREK_FMT_CHECK_HOOKS}}

# Apply formatting plus auto-fixable CI hygiene hooks, then verify the full CI fmt gate.
fmt: check-format-tools
	@cargo +stable fmt --all
	@git ls-files -z '*.toml' | xargs -0 -r env RUST_LOG=warn taplo format >/dev/null
	@git ls-files -z '*.ron' ':!examples/modular_config_example/motors.ron' | xargs -0 -r -n 1 fmtron --input>/dev/null
	@find . -type f -name '*.ron.bak' -delete
	@bash -lc 'set -euo pipefail; prek run --all-files {{PREK_FMT_FIX_HOOKS}} || prek run --all-files {{PREK_FMT_FIX_HOOKS}}'
	@prek run --all-files {{PREK_FMT_CI_HOOKS}}

# Ensure the tools needed by fmt/fmt-check are installed.
check-format-tools:
	#!/usr/bin/env bash
	set -euo pipefail

	missing=0
	if ! command -v taplo >/dev/null 2>&1; then
		echo "Missing taplo (taplo-cli). Install with: cargo install --locked taplo-cli"
		missing=1
	fi
	if ! command -v fmtron >/dev/null 2>&1; then
		echo "Missing fmtron. Install with: cargo install --locked fmtron"
		missing=1
	fi
	if ! command -v prek >/dev/null 2>&1; then
		echo "Missing prek. Install with: cargo install --locked prek"
		missing=1
	fi
	if [[ "$missing" -ne 0 ]]; then
		exit 1
	fi

# Typo check only
typos:
	typos -c .config/_typos.toml

# Ensure public API snapshot tooling is available.
check-public-api:
	#!/usr/bin/env bash
	set -euo pipefail

	if ! rustup run {{PUBLIC_API_TOOLCHAIN}} rustc --version >/dev/null 2>&1; then
		echo "Missing Rust {{PUBLIC_API_TOOLCHAIN}} toolchain required by cargo-public-api. Install with: rustup toolchain install {{PUBLIC_API_TOOLCHAIN}} --profile minimal"
		exit 1
	fi

	if ! cargo +stable public-api --version 2>/dev/null | grep -qx "cargo-public-api {{PUBLIC_API_VERSION}}"; then
		echo "Missing cargo-public-api {{PUBLIC_API_VERSION}}. Install with: cargo install --locked cargo-public-api --version {{PUBLIC_API_VERSION}}"
		exit 1
	fi

# Check the V1 public API snapshot baseline.
api-check: check-public-api
	@support/ci/check_public_api.sh

# Refresh the V1 public API snapshot baseline after an intentional API change.
api-update: check-public-api
	@UPDATE_PUBLIC_API=1 support/ci/check_public_api.sh

# Std clippy checks aligned with reusable unit-test workflow defaults.
clippy-std:
	#!/usr/bin/env bash
	set -euo pipefail

	os="$(uname -s || true)"
	features="{{BASE_FEATURES}}"
	case "$os" in
		Linux*) features="{{BASE_FEATURES}},python" ;;
		Darwin*) features="{{BASE_FEATURES}},python" ;;
		MINGW*|MSYS*|CYGWIN*|Windows_NT) features="{{WINDOWS_BASE_FEATURES}}" ;;
		*) features="{{BASE_FEATURES}}" ;;
	esac
	features_flag="--features $features"
	workspace_excludes="{{WORKSPACE_EXCLUDES}}"

	cargo +stable clippy --workspace --all-targets $workspace_excludes -- --deny warnings
	cargo +stable clippy --workspace --all-targets $features_flag $workspace_excludes -- --deny warnings

# Run the Unit-Tests job locally via act (debug/ubuntu matrix).
ci:
  act -W .github/workflows/general.yml -j Unit-Tests --matrix os:ubuntu-latest --matrix mode:debug -P ubuntu-latest=ghcr.io/catthehacker/ubuntu:act-latest

# Host target detection for cross-platform logreader builds
host_target := `rustc +stable -vV | sed -n 's/host: //p'`

# no_std/embedded clippy checks mirroring the embedded workflow.
clippy-nostd:
	cargo +stable clippy --no-default-features
	python3 support/ci/embedded_crates.py run --action clippy --toolchain stable
	cd examples/cu_rp2350_skeleton && cargo +stable clippy --target thumbv8m.main-none-eabihf --bin cu-blinky --features firmware
	cd examples/cu_rp2350_skeleton && cargo +stable clippy --no-default-features --features host --bins --target={{host_target}}

# Run std and no_std unit tests.
test:
	#!/usr/bin/env bash
	set -euo pipefail

	cargo +stable nextest run --all-targets --workspace {{WORKSPACE_EXCLUDES}}
	cargo +stable nextest run --no-default-features

# Check the large examples from the former extra-examples repo.
check-extra-examples: check-extra-examples-host check-extra-examples-embedded

# Host checks for large examples. These intentionally stay out of default workspace CI.
check-extra-examples-host:
	cargo +stable check -p cu-rp-balancebot --all-targets
	cargo +stable check -p cu-flight-controller --all-targets --features textlogs
	cargo +stable check -p cu-human-pose --all-targets
	cargo +stable check -p cu-feetech-demo --all-targets
	cargo +stable check -p cu-gnss-ublox-demo --all-targets --features logexport

# Embedded checks for large examples. These mirror the old satellite CI.
check-extra-examples-embedded:
	cargo +stable check -p cu-elrs-bdshot-demo --target thumbv8m.main-none-eabihf
	cargo +stable check -p cu-flight-controller --target thumbv7em-none-eabihf --no-default-features --features firmware,textlogs --bin quad

# Check the benchmarks from the former benchmarks repo.
check-benchmarks:
	cargo +stable check -p cu-dorabench --all-targets
	cargo +stable check -p cu-async-cl-io-bench --all-targets
	cargo +stable check -p cu-zenoh-bridge-bench --all-targets
	cargo +stable check --manifest-path benchmarks/dora_caterpillar/Cargo.toml --all-targets
	cargo +stable check --manifest-path benchmarks/horus_caterpillar/Cargo.toml --all-targets

# Ensure the tools needed by coverage are installed.
check-coverage-tools:
	#!/usr/bin/env bash
	set -euo pipefail

	missing=0
	if ! cargo +stable llvm-cov --version >/dev/null 2>&1; then
		echo "Missing cargo-llvm-cov. Install with: cargo install --locked cargo-llvm-cov"
		missing=1
	fi
	if ! cargo +stable nextest --version >/dev/null 2>&1; then
		echo "Missing cargo-nextest. Install with: cargo install --locked cargo-nextest"
		missing=1
	fi
	if ! rustup component list --toolchain stable | grep -Eq '^llvm-tools(-preview|-.*-unknown-.*)? .*installed'; then
		echo "Missing llvm-tools-preview for stable. Install with: rustup component add llvm-tools-preview --toolchain stable"
		missing=1
	fi
	if [[ "$missing" -ne 0 ]]; then
		exit 1
	fi

# Generate the Linux std coverage report used by CI.
coverage: check-coverage-tools
	support/ci/run_coverage.sh

# Verify the declared minimum supported Rust version.
msrv-check:
	#!/usr/bin/env bash
	set -euo pipefail

	cargo +{{MSRV}} check --workspace --all-targets {{WORKSPACE_EXCLUDES}}
	cargo +{{MSRV}} check --no-default-features

# Run the no_std/embedded CI flow locally.
nostd-ci:
	just fmt-check
	just typos
	cargo +stable build --no-default-features
	cargo +stable nextest run --no-default-features
	python3 support/ci/embedded_crates.py run --action clippy --toolchain stable
	python3 support/ci/embedded_crates.py run --action build --toolchain stable
	cd examples/cu_rp2350_skeleton && cargo +stable clippy --target thumbv8m.main-none-eabihf --bin cu-blinky --features firmware
	cd examples/cu_rp2350_skeleton && cargo +stable clippy --no-default-features --features host --bins --target={{host_target}}
	cd examples/cu_rp2350_skeleton && cargo +stable build-arm
	cd examples/cu_rp2350_skeleton && cargo +stable build --target={{host_target}} --no-default-features --features host --bin blinky-logreader

# Std-specific CI flow (local, CI-aligned). Use mode=release or mode=cuda-release as needed.
std-ci mode="debug":
	#!/usr/bin/env bash
	set -euo pipefail
	just fmt-check
	just typos

	mode="{{mode}}"
	release_flag=""
	if [[ "$mode" == "release" || "$mode" == "cuda-release" ]]; then
		release_flag="--release"
	fi

	os="$(uname -s || true)"
	features="{{BASE_FEATURES}}"
	case "$os" in
		Linux*) features="{{BASE_FEATURES}},python" ;;
		Darwin*) features="{{BASE_FEATURES}},python" ;;
		MINGW*|MSYS*|CYGWIN*|Windows_NT) features="{{WINDOWS_BASE_FEATURES}}" ;;
		*) features="{{BASE_FEATURES}}" ;;
	esac
	if [[ "$mode" == "cuda-release" ]]; then
		features="${features},cuda"
	fi
	features_flag="--features $features"
	workspace_excludes="{{WORKSPACE_EXCLUDES}}"

	cargo +stable clippy $release_flag --workspace --all-targets $workspace_excludes -- --deny warnings
	cargo +stable clippy $release_flag --workspace --all-targets $features_flag $workspace_excludes -- --deny warnings
	cargo +stable build $release_flag --workspace --all-targets $features_flag $workspace_excludes

	if [[ "$mode" == "debug" ]]; then
		cargo +stable test --doc --workspace $workspace_excludes --quiet
	fi

	cargo +stable nextest run $release_flag --all-targets --workspace $workspace_excludes
	cargo +stable nextest run $release_flag --all-targets --workspace $features_flag $workspace_excludes

	RAYON_NUM_THREADS=1 COPPER_DETERMINISM_ITERS=256 COPPER_DETERMINISM_DT_TICKS=1000 \
		cargo +stable test $release_flag -p cu-caterpillar --features determinism_ci \
		-- determinism_record_and_resim --test-threads=1

	if [[ "$mode" == "debug" ]]; then
		rm -rf support/cargo_cunew/templates/test_project support/cargo_cunew/templates/test_workspace
		cargo +stable run -p cargo-cunew -- support/cargo_cunew/templates/test_project --template project --source local --copper-root .
		cargo +stable run -p cargo-cunew -- support/cargo_cunew/templates/test_workspace --template workspace --source local --copper-root .
		(
			cd support/cargo_cunew/templates/test_project
			cargo +stable build
		)
		(
			cd support/cargo_cunew/templates/test_workspace
			cargo +stable build
		)
	fi

# Proc-macro expansion helpers (cargo-expand).
check-expand:
	#!/usr/bin/env bash
	set -euo pipefail

	if ! cargo +stable expand --version >/dev/null 2>&1; then
		echo "Missing cargo-expand. Install with: cargo install cargo-expand"
		exit 1
	fi

# Inspect expanded output of cu29-deriva in the target
expand-runtime pkg bin features="": check-expand
	#!/usr/bin/env bash
	# Usage: just expand-runtime pkg=<crate> bin=<bin> [features=feat1,feat2]
	set -euo pipefail

	features_flag=""
	if [[ -n "{{features}}" ]]; then
	    features_flag="--features {{features}}"
	fi

	cargo +stable expand -p "{{pkg}}" --bin "{{bin}}" $features_flag

# expand macro of cu29-soa-derive test target
expand-soa test="proctest": check-expand
	# Usage: just expand-soa [test=proctest]
	cargo +stable expand -p cu29-soa-derive --test "{{test}}"

# Run RTSan on a single app (defaults to cu-caterpillar).
# RTSan reports violations to stderr; tweak RTSAN_OPTIONS if you need to keep running.
rtsan-smoke pkg="cu-caterpillar" bin="cu-caterpillar" args="" options="halt_on_error=false":
	#!/usr/bin/env bash
	set -euo pipefail
	RTSAN_ENABLE=1 RTSAN_OPTIONS="{{options}}" \
		cargo run --profile screaming -p "{{pkg}}" --features rtsan --bin "{{bin}}" -- {{args}}

# Project-specific helpers now live in per-directory justfiles under examples/, components/, support/, and catalog/.

# Install hidden desktop entries so Wayland can map sim app_ids to the Copper icon.
install-sim-desktop-entries:
	#!/usr/bin/env bash
	set -euo pipefail
	bash support/linux/install_sim_desktop_entries.sh "{{ROOT}}"

# Build and open the generated wiki + API docs locally.
docs:
	#!/usr/bin/env bash
	set -euo pipefail

	if ! command -v mkdocs >/dev/null; then
		echo "mkdocs not found. Install it with: python3 -m pip install --upgrade pip mkdocs" >&2
		exit 1
	fi

	RUSTDOCFLAGS="--enable-index-page -Zunstable-options" cargo +nightly doc --no-deps
	python3 support/ci/wiki_site.py
	mkdocs build -f build/wiki/mkdocs.yml

	mkdir -p build/wiki/site/api
	cp -R target/doc/* build/wiki/site/api/

	site_path="build/wiki/site/index.html"
	if [[ "$(uname -s)" == "Darwin" ]]; then
		open "$site_path"
	elif command -v xdg-open >/dev/null; then
		xdg-open "$site_path"
	elif command -v python3 >/dev/null; then
		python3 -m webbrowser "file://${PWD}/${site_path}"
	else
		echo "Open ${site_path} in your browser."
	fi

# Partition/format an SD card for Cu29 logging (destroys data).
mkpartition dev:
	#!/usr/bin/env bash
	set -euo pipefail

	DEV="{{dev}}"
	[[ -b "$DEV" ]] || { echo "not a block device: $DEV"; exit 1; }

	for t in sgdisk mkfs.vfat lsblk wipefs udevadm; do
	  command -v "$t" >/dev/null || { echo "missing tool: $t"; exit 1; }
	done

	read -r -p "ERASE ALL DATA on $DEV? type 'YES' to continue: " ans
	[[ "$ans" == "YES" ]] || { echo "aborted"; exit 1; }

	lsblk -ln "$DEV" -o NAME,MOUNTPOINTS | while read -r name mp; do
	  [[ -n "${mp:-}" ]] && sudo umount -f "/dev/$name" || true
	done

	sudo wipefs -a "$DEV" || true
	sudo sgdisk --zap-all "$DEV"

	CU29_GUID="29A2E0C9-0000-4C75-9229-000000000029"
	sudo sgdisk -og "$DEV"
	sudo sgdisk \
	  -n1:1MiB:+1MiB  -t1:0700                                -c1:"COPPERCFG" \
	  -n2:0:0         -t2:${CU29_GUID}                        -c2:"Cu29" \
	  "$DEV"

	sudo partprobe "$DEV" || true
	sudo udevadm settle || true
	sleep 0.5

	if [[ "$DEV" =~ [0-9]$ ]]; then P1="${DEV}p1"; P2="${DEV}p2"; else P1="${DEV}1"; P2="${DEV}2"; fi
	[[ -b "$P1" && -b "$P2" ]] || { echo "partition nodes not found: $P1 $P2"; exit 1; }

	sudo mkfs.vfat -F 12 -n COPPERCFG "$P1"

	mnt=$(mktemp -d)
	trap 'sudo umount "$mnt" 2>/dev/null || true; rmdir "$mnt" 2>/dev/null || true' EXIT
	sudo mount "$P1" "$mnt"
	sudo tee "$mnt/README-COPPER.txt" >/dev/null <<-'TXT'
	This SD card uses Copper (Cu29) raw binary logging.

	Partition layout (GPT):
	  - Partition 1 (FAT12, label COPPERCFG): configuration + this notice
	  - Partition 2 (type GUID 29A2E0C9-0000-4C75-9229-000000000029, label Cu29): Copper raw log storage

	To read logs, use the Copper log reader.
	Do NOT format or mount partition 2 — it is not a filesystem.
	TXT
	sync
	sudo umount "$mnt"
	rmdir "$mnt"
	trap - EXIT

	lsblk -o NAME,SIZE,TYPE,FSTYPE,PTTYPE,PARTLABEL,PARTTYPE,LABEL "$DEV"
	echo "Done. Copper raw partition is $P2 (GPT type ${CU29_GUID}, PARTLABEL=Cu29)."

# Extract the Cu29 raw log partition into a .copper file.
extract-log dev out="logs/embedded_0.copper":
	#!/usr/bin/env bash
	set -euo pipefail

	DEV="{{dev}}"
	OUT="{{out}}"
	invocation_dir="{{invocation_directory()}}"
	[[ -b "$DEV" ]] || { echo "not a block device: $DEV"; exit 1; }

	for t in lsblk dd; do
	  command -v "$t" >/dev/null || { echo "missing tool: $t"; exit 1; }
	done

	CU29_GUID="29A2E0C9-0000-4C75-9229-000000000029"
	dev_type="$(lsblk -no TYPE "$DEV" 2>/dev/null | head -n1 || true)"

	PART="$DEV"
	if [[ "$dev_type" == "disk" ]]; then
	  PART="$(lsblk -lnpo NAME,PARTLABEL,PARTTYPE "$DEV" | awk -v guid="$CU29_GUID" '($2=="Cu29") || (toupper($3)==toupper(guid)) {print $1; exit}')"
	  [[ -n "${PART:-}" ]] || { echo "Cu29 partition not found on $DEV"; exit 1; }
	fi

	if [[ "$dev_type" == "part" ]]; then
	  part_label="$(lsblk -no PARTLABEL "$PART" 2>/dev/null | head -n1 || true)"
	  part_type="$(lsblk -no PARTTYPE "$PART" 2>/dev/null | head -n1 || true)"
	  part_label="${part_label:-}"
	  part_type="${part_type:-}"
	  if [[ "$part_label" != "Cu29" && "${part_type^^}" != "${CU29_GUID}" ]]; then
	    echo "Warning: $PART does not look like a Cu29 partition (label=$part_label type=$part_type)" >&2
	  fi
	fi

	if [[ "$OUT" != /* ]]; then
	  OUT="${invocation_dir}/${OUT}"
	fi

	echo "Reading Cu29 partition $PART -> $OUT"
	sudo dd if="$PART" of="$OUT" bs=4M status=progress conv=fsync

# Render the current Copper config from the working directory.
# Prefers `copperconfig.ron`, and falls back to `multi_copper.ron` for distributed demos.
dag mission="":
	#!/usr/bin/env bash
	set -euo pipefail

	invocation_dir="{{invocation_directory()}}"
	cfg_path="${invocation_dir}/copperconfig.ron"
	if [[ ! -f "$cfg_path" ]]; then
		cfg_path="${invocation_dir}/multi_copper.ron"
		if [[ ! -f "$cfg_path" ]]; then
			echo "No copperconfig.ron or multi_copper.ron found in ${invocation_dir}" >&2
			exit 1
		fi
	fi

	cd "{{ROOT}}"
	mission_value="{{mission}}"
	if [[ -n "$mission_value" ]]; then
		cargo run -p cu29-runtime --bin cu29-rendercfg -- --mission "$mission_value" --open "$cfg_path"
	else
		cargo run -p cu29-runtime --bin cu29-rendercfg -- --open "$cfg_path"
	fi

# Helpers for managing git worktrees for different branches.
wt branch:
  #!/usr/bin/env bash
  set -euo pipefail

  name="$(basename "{{branch}}")"
  dir="$(realpath ../copper-rs.${name})"
  echo "Adding worktree for branch '{{branch}}' at ${dir}"
  if [[ -e "${dir}" ]]; then
    echo "Worktree already exists at ${dir}"
  elif git show-ref --verify --quiet "refs/heads/{{branch}}"; then
    git worktree add "${dir}" "{{branch}}"
  elif git show-ref --verify --quiet "refs/remotes/origin/{{branch}}"; then
    git worktree add -b "{{branch}}" "${dir}" "origin/{{branch}}"
  else
    git worktree add -b "{{branch}}" "${dir}"
  fi
  if [[ -n "${ZELLIJ:-}" ]]; then
    zellij action new-tab --name "${name}"
    zellij action write-chars "cd ${dir};reset"
    zellij action write 13
  fi
