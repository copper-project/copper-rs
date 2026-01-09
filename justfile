# CI-aligned helpers mirroring .github/workflows/general.yml
BASE_FEATURES := "mock,image,kornia,gst,faer,nalgebra,glam,debug_pane,bincode,log-level-debug"
WINDOWS_BASE_FEATURES := "mock,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode"
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

# Host target detection for cross-platform logreader builds
host_target := `rustc -vV | sed -n 's/host: //p'`

nostd-ci: lint
	cargo +stable build --no-default-features
	cargo +stable nextest run --no-default-features
	python3 support/ci/embedded_crates.py run --action clippy
	python3 support/ci/embedded_crates.py run --action build
	cd examples/cu_rp2350_skeleton && cargo +stable clippy
	cd examples/cu_rp2350_skeleton && cargo +stable build-arm
	cd examples/cu_rp2350_skeleton && cargo +stable build --target={{host_target}} --no-default-features --features host --bin blinky-logreader

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
			rm -rf test_project test_workspace
			cargo +stable generate -p cu_project --name test_project --destination . -d copper_source=local -d copper_root_path=../.. --silent
			cargo +stable generate -p cu_full --name test_workspace --destination . -d copper_source=local -d copper_root_path=../.. --silent
		)
		(
			cd templates/test_project
			cargo +stable build
		)
		(
			cd templates/test_workspace
			cargo +stable build
		)
	fi

# Run RTSan on a single app (defaults to cu-caterpillar).
# RTSan reports violations to stderr; tweak RTSAN_OPTIONS if you need to keep running.
rtsan-smoke pkg="cu-caterpillar" bin="cu-caterpillar" args="" options="halt_on_error=false":
	#!/usr/bin/env bash
	set -euo pipefail
	RTSAN_ENABLE=1 RTSAN_OPTIONS="{{options}}" \
		cargo run -p "{{pkg}}" --features rtsan --bin "{{bin}}" -- {{args}}

# Project-specific helpers now live in per-directory justfiles under examples/, components/, and support/.

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
	mission_arg=()
	if [[ -n "$mission_value" ]]; then
		mission_arg=(--mission "$mission_value")
	fi
	cargo run -p cu29-runtime --bin cu29-rendercfg -- "${mission_arg[@]}" --open "$cfg_path"

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
