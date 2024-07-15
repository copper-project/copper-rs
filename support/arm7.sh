#!/bin/bash
cross build --target armv7-unknown-linux-gnueabihf --release
find target/armv7-unknown-linux-gnueabihf/release -maxdepth 1 -type f -executable -exec scp -r {} gbin@copper7:copper/ \;
scp copper_derive_test/copperconfig.ron gbin@copper7:copper

