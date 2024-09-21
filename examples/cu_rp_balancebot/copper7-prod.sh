#!/bin/bash
set -e

cross build --target armv7-unknown-linux-gnueabihf --release --no-default-features
scp ../../target/armv7-unknown-linux-gnueabihf/release/balancebot copperconfig.ron copper7:copper/ 
