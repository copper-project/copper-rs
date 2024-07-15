#!/bin/bash
cross build --target riscv64gc-unknown-linux-gnu --release
find target/riscv64gc-unknown-linux-gnu/release -maxdepth 1 -type f -executable -exec scp -r {} gbin@copperv:copper/ \;
scp copper_derive_test/copperconfig.ron gbin@copperv:copper

