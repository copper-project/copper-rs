# Distributed Resim Demo

This example is a heavier companion to `cu_zenoh_bridge_demo`. It is meant to
exercise the distributed replay stack rather than to stay minimal.

Topology:

- `scout -> plan -> control -> scout`
- two identical robot instances distinguished only by `instance_id`
- each subsystem runs as its own Copper process and writes its own `.copper` log

Useful commands:

- `just scout-1`
- `just scout-2`
- `just plan-1`
- `just plan-2`
- `just control-1`
- `just control-2`
- `just record`
- `just resim`
- `just check`

`just check` starts all six processes, records logs under `logs/record/`, then
replays the discovered fleet into `logs/replay/` and compares the original and
replayed CopperList/keyframe streams for every `(instance_id, subsystem_id)`.
