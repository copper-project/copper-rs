#!/bin/bash
EXEC=$(cargo test --message-format=json --target arm-unknown-linux-musleabihf --no-run | jq -r 'select(.executable and .target.kind[] == "test") | .executable')
echo 'This is the executable:' $EXEC
scp $EXEC copper7:testads7883
scp tests/copperconfig.ron copper7:tests/copperconfig.ron
