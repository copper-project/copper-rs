## Cu-dorabench: let's drag race vs dora-rs

This is to compare apple to apple with dora-rs another rust middleware for robotics.

This mimics the implementation of https://github.com/dora-rs/dora-benchmark

Usage: `cargo run -r --profile screaming`

Warning: say goodbye to your disk space, we are logging to disk 40MiB at every message as fast as your storage allows us to do.

The payload in this example is a tiny newtype around `CuHandle<Vec<u8>>`.
Copper's BW monitor and exported payload size stats now measure payload bytes
from the actual encode path, and `CuHandle` contributes its backing buffer size
to that measurement. That keeps this wrapper honest without forcing the wider
payload ecosystem to add another mandatory trait impl.
