# Instance Overrides Demo

This example shows how a multi-Copper deployment can keep a static subsystem graph in
`robot_base.ron` while applying per-instance task config overlays from:

```text
instances/<instance_id>/<subsystem_id>.ron
```

## Layout

```text
multi_copper.ron
robot_base.ron
instances/
  17/robot.ron
  42/robot.ron
```

The `multi_copper.ron` file declares:

```ron
instance_overrides_root: "instances"
```

At runtime, Copper resolves the effective config for subsystem `robot` and instance `17`
from:

```text
instances/17/robot.ron
```

## Run

From this directory:

```bash
cargo run -- --instance-id 17
```

Or from the workspace root:

```bash
cargo run --manifest-path examples/cu_instance_overrides_demo/Cargo.toml -- --instance-id 42
```

The program prints the effective task config before building the app, then runs one Copper
iteration with that same `instance_id`.
