rm logs/*.copper
scp "copper7:copper/logs/*.copper" logs
cargo run --bin balancebot-logreader
