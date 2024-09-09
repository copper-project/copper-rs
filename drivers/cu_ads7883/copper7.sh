#!/bin/bash
# cargo build --release --target arm-unknown-linux-musleabihf
EXEC=$(cargo test --message-format=json --target arm-unknown-linux-musleabihf --no-run | jq -r 'select(.executable and .target.kind[] == "test") | .executable')
echo 'This is the executable:' $EXEC
scp $EXEC copper7:ads7883test
scp -r /home/gbin/projects/copper/copper-rs/target/arm-unknown-linux-musleabihf/debug/cu29_log_index copper7:
