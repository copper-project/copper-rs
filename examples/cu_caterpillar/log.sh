#!/bin/bash
RUST_BACKTRACE=1 cargo run --bin logreader /tmp/caterpillar.copper extract-log /home/gbin/projects/copper/copper-project/target/debug/copper_log_index
