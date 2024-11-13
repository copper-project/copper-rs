# cu_iceoryx2

This demonstrates how to use Copper to interface with the Iceoryx2 shared memory communication middleware.

It is useful for Copper users who want to use Iceoryx to communicate with external applications.

You can start the 2 processes in separate terminals:

```sh
# Terminal 1
cargo run --bin upstream

# Terminal 2
cargo run --bin downstream
```

And you should see messages from the caterpillar example (just booleans) getting passed from one copper process to the
other.

