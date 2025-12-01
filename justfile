# CI-aligned helpers mirroring .github/workflows/general.yml
BASE_FEATURES := "macro_debug,mock,perf-ui,image,kornia,gst,faer,nalgebra,glam,debug_pane,bincode,log-level-debug"
WINDOWS_BASE_FEATURES := "macro_debug,mock,perf-ui,image,kornia,python,gst,faer,nalgebra,glam,debug_pane,bincode"
EMBEDDED_EXCLUDES := `python3 support/ci/embedded_crates.py excludes`

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

# Example and support helpers migrated from shell scripts.
mk-cu29-sdcard dev:
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

profile-standardlog:
	#!/usr/bin/env bash
	set -euo pipefail
	CARGO_PROFILE_RELEASE_DEBUG=true cargo flamegraph --bin standardlog_perf

profile-structlog:
	#!/usr/bin/env bash
	set -euo pipefail
	CARGO_PROFILE_RELEASE_DEBUG=true cargo flamegraph --bin structlog_perf

ros-caterpillar-run:
	#!/usr/bin/env bash
	set -euo pipefail
	cd examples/ros_caterpillar
	rm -rf build/ install/ log/
	colcon build
	source install/setup.bash
	export ROS_DOMAIN_ID=42
	echo "Starting all nodes..."
	ros2 run gpio_caterpillar propagate_node gpio_node_1 flip_topic_1 flip_topic_2 &
	ros2 run gpio_caterpillar actuation_node gpio_node_1 4 &
	ros2 run gpio_caterpillar propagate_node gpio_node_2 flip_topic_2 flip_topic_3 &
	ros2 run gpio_caterpillar actuation_node gpio_node_2 17 &
	ros2 run gpio_caterpillar propagate_node gpio_node_3 flip_topic_3 flip_topic_4 &
	ros2 run gpio_caterpillar actuation_node gpio_node_3 27 &
	ros2 run gpio_caterpillar propagate_node gpio_node_4 flip_topic_4 flip_topic_5 &
	ros2 run gpio_caterpillar actuation_node gpio_node_4 22 &
	ros2 run gpio_caterpillar propagate_node gpio_node_5 flip_topic_5 flip_topic_6 &
	ros2 run gpio_caterpillar actuation_node gpio_node_5 5 &
	ros2 run gpio_caterpillar propagate_node gpio_node_6 flip_topic_6 flip_topic_7 &
	ros2 run gpio_caterpillar actuation_node gpio_node_6 6 &
	ros2 run gpio_caterpillar propagate_node gpio_node_7 flip_topic_7 flip_topic_8 &
	ros2 run gpio_caterpillar actuation_node gpio_node_7 19 &
	ros2 run gpio_caterpillar propagate_node gpio_node_8 flip_topic_8 flip_topic_9 &
	ros2 run gpio_caterpillar actuation_node gpio_node_8 26 &
	sleep 2
	ros2 run gpio_caterpillar flip_node &
	echo "All nodes started"
	wait

ros-zenoh-caterpillar-run:
	#!/usr/bin/env bash
	set -euo pipefail
	cd examples/ros_zenoh_caterpillar
	rm -rf build/ install/ log/
	colcon build
	source install/setup.bash
	export ROS_DOMAIN_ID=42
	export RMW_IMPLEMENTATION=rmw_zenoh_cpp
	ros2 run rmw_zenoh_cpp rmw_zenohd &
	read -n1 -r -p "Press any key to continue..."
	echo
	echo "Starting all nodes..."
	ros2 run gpio_caterpillar propagate_node gpio_node_1 flip_topic_1 flip_topic_2 &
	ros2 run gpio_caterpillar actuation_node gpio_node_1 4 &
	ros2 run gpio_caterpillar propagate_node gpio_node_2 flip_topic_2 flip_topic_3 &
	ros2 run gpio_caterpillar actuation_node gpio_node_2 17 &
	ros2 run gpio_caterpillar propagate_node gpio_node_3 flip_topic_3 flip_topic_4 &
	ros2 run gpio_caterpillar actuation_node gpio_node_3 27 &
	ros2 run gpio_caterpillar propagate_node gpio_node_4 flip_topic_4 flip_topic_5 &
	ros2 run gpio_caterpillar actuation_node gpio_node_4 22 &
	ros2 run gpio_caterpillar propagate_node gpio_node_5 flip_topic_5 flip_topic_6 &
	ros2 run gpio_caterpillar actuation_node gpio_node_5 5 &
	ros2 run gpio_caterpillar propagate_node gpio_node_6 flip_topic_6 flip_topic_7 &
	ros2 run gpio_caterpillar actuation_node gpio_node_6 6 &
	ros2 run gpio_caterpillar propagate_node gpio_node_7 flip_topic_7 flip_topic_8 &
	ros2 run gpio_caterpillar actuation_node gpio_node_7 19 &
	ros2 run gpio_caterpillar propagate_node gpio_node_8 flip_topic_8 flip_topic_9 &
	ros2 run gpio_caterpillar actuation_node gpio_node_8 26 &
	sleep 2
	ros2 run gpio_caterpillar flip_node &
	echo "All nodes started"
	wait

balancebot-dump-text-logs:
	#!/usr/bin/env bash
	set -euo pipefail
	cd examples/cu_rp_balancebot
	cargo run --bin balancebot-logreader -- logs/balance.copper extract-text-log ../../target/debug/cu29_log_index/strings.bin

balancebot-fsck:
	#!/usr/bin/env bash
	set -euo pipefail
	cd examples/cu_rp_balancebot
	cargo run --bin balancebot-logreader -- logs/balance.copper fsck

balancebot-set-pwm-permissions:
	#!/usr/bin/env bash
	set -euo pipefail
	for pwm in /sys/class/pwm/pwmchip0/pwm*/; do
		chmod 0660 "${pwm}enable"
		chmod 0660 "${pwm}duty_cycle"
		chmod 0660 "${pwm}period"
		chmod 0660 "${pwm}polarity"
		chown root:wheel "${pwm}enable"
		chown root:wheel "${pwm}duty_cycle"
		chown root:wheel "${pwm}period"
		chown root:wheel "${pwm}polarity"
	done
	chmod 0660 /sys/class/pwm/pwmchip0/unexport
	chown root:wheel /sys/class/pwm/pwmchip0/unexport

cross-riscv64-deploy:
	#!/usr/bin/env bash
	set -euo pipefail
	cross build --target riscv64gc-unknown-linux-gnu --release
	find target/riscv64gc-unknown-linux-gnu/release -maxdepth 1 -type f -executable -exec scp -r {} gbin@copperv:copper/ \;
	scp copper_derive_test/copperconfig.ron gbin@copperv:copper

caterpillar-copperlist:
	#!/usr/bin/env bash
	set -euo pipefail
	cd examples/cu_caterpillar
	RUST_BACKTRACE=1 cargo run --bin logreader /tmp/caterpillar.copper extract-copperlist

caterpillar-rendercfg:
	#!/usr/bin/env bash
	set -euo pipefail
	cd examples/cu_caterpillar
	../../target/release/copper-rendercfg copperconfig.ron --open

caterpillar-logreader:
	#!/usr/bin/env bash
	set -euo pipefail
	cd examples/cu_caterpillar
	RUST_BACKTRACE=1 cargo run --bin cu-caterpillar-logreader logs/caterpillar.copper extract-log ../../target/debug/copper_log_index

caterpillar-resim:
	#!/usr/bin/env bash
	set -euo pipefail
	cd examples/cu_caterpillar
	RUST_BACKTRACE=1 cargo run --bin cu-caterpillar-resim logs/caterpillar.copper

caterpillar-cross-armv7:
	#!/usr/bin/env bash
	set -euo pipefail
	cd examples/cu_caterpillar
	cross build --target armv7-unknown-linux-gnueabihf --release
	scp ../../target/armv7-unknown-linux-gnueabihf/release/cu-caterpillar copper7:copper

elrs-bdshot-attach:
	#!/usr/bin/env bash
	set -euo pipefail
	cd examples/cu_elrs_bdshot_demo/support
	probe-rs attach ../../target/thumbv8m.main-none-eabihf/debug/cu-bdshot-demo

docker-build-env:
	#!/usr/bin/env bash
	set -euo pipefail
	cd support/docker
	docker build -f Dockerfile -t copper-rs-env .

docker-build-cuda-env:
	#!/usr/bin/env bash
	set -euo pipefail
	cd support/docker
	docker build -f Dockerfile_cuda -t copper-rs-cuda-env .

docker-run-env:
	#!/usr/bin/env bash
	set -euo pipefail
	cd support/docker
	docker run -it -entrypoint="" --mount type=bind,source="$(pwd)"/../..,target=/home/copper-rs copper-rs-env

docker-run-cuda-env:
	#!/usr/bin/env bash
	set -euo pipefail
	cd support/docker
	docker run -it -entrypoint="" --mount type=bind,source="$(pwd)"/../..,target=/home/copper-rs copper-rs-cuda-env

ros-rihs01-hashes:
	#!/usr/bin/env bash
	set -euo pipefail
	ROS_DIR="/opt/ros/jazzy/share"
	echo "| Type Name | RIHS01 Hash |"
	echo "|-----------|-------------|"
	find "$ROS_DIR" -name '*.json' -print0 | \
		xargs -0 jq -r '.type_hashes[]? | "\(.type_name) \(.hash_string)"' | \
		sort -u | \
		awk '{ printf "| %s | %s |\n", $1, $2 }'

cross-armv7-deploy:
	#!/usr/bin/env bash
	set -euo pipefail
	cross build --target armv7-unknown-linux-gnueabihf --release
	find target/armv7-unknown-linux-gnueabihf/release -maxdepth 1 -type f -executable -exec scp -r {} gbin@copper7:copper/ \;
	scp copper_derive_test/copperconfig.ron gbin@copper7:copper

deploy-cu-ads7883-tests:
	#!/usr/bin/env bash
	set -euo pipefail
	cd components/sources/cu_ads7883
	EXEC=$(cargo test --message-format=json --target arm-unknown-linux-musleabihf --no-run | jq -r 'select(.executable and .target.kind[] == "test") | .executable')
	echo "This is the executable: $EXEC"
	scp "$EXEC" copper7:testads7883
	scp tests/copperconfig.ron copper7:tests/copperconfig.ron

deploy-cu-sn754410-tests:
	#!/usr/bin/env bash
	set -euo pipefail
	cd components/sinks/cu_rp_sn754410
	EXEC=$(cargo test --message-format=json --target arm-unknown-linux-musleabihf --no-run | jq -r 'select(.executable and .target.kind[] == "test") | .executable')
	echo "This is the executable: $EXEC"
	scp "$EXEC" copper7:testads7883
	scp tests/copperconfig.ron copper7:tests/copperconfig.ron
