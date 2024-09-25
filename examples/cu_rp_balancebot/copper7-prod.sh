#!/bin/bash
set -e

cargo build --target armv7-unknown-linux-musleabihf --release --no-default-features
cp -r ../../target/armv7-unknown-linux-musleabihf/release/cu29_log_index .
# scp ../../target/armv7-unknown-linux-musleabihf/release/balancebot copperconfig.ron copper7:copper/ 
scp ../../target/armv7-unknown-linux-musleabihf/release/balancebot copper7:copper/ 
