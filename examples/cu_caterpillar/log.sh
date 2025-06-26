#!/bin/bash
RUST_BACKTRACE=1 cargo run --bin cu-caterpillar-logreader logs/caterpillar.copper extract-log ../../target/debug/copper_log_index
