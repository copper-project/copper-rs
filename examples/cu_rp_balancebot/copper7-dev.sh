#!/bin/bash
set -e

#cross build --target armv7-unknown-linux-gnueabihf --no-default-features
cargo build --target armv7-unknown-linux-musleabihf --no-default-features
cp -r ../../target/armv7-unknown-linux-musleabihf/debug/cu29_log_index .
// scp ../../target/armv7-unknown-linux-musleabihf/debug/balancebot copperconfig.ron copper7:copper/ 
scp ../../target/armv7-unknown-linux-musleabihf/debug/balancebot copper7:copper/ 
