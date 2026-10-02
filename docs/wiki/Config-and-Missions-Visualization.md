In order to visualize a configuration and the mission you can use this command:

```bash
cd examples/cu_bridge_test
just dag
```

It should produce a file in the current directory called `output.svg` containing a graph like this with all the missions:
<img width="949" height="1777" alt="image" src="https://github.com/user-attachments/assets/f7781f70-fde3-42a6-a1dc-fd9bfff390b1" />

You can also list the missions from the config or only render a specific mission check `--help` for details:
```
$ cargo run -p cu29-runtime --bin cu29-rendercfg -- --help
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.15s
     Running `target/debug/cu29-rendercfg --help`
Copper Runtime Runtime crate. Copper is an engine for robotics.

Usage: cu29-rendercfg [OPTIONS] <CONFIG>

Arguments:
  <CONFIG>  Config file name

Options:
      --mission <MISSION>  Mission id to render (omit to render every mission)
      --list-missions      List missions contained in the configuration and exit
      --open               Open the SVG in the default system viewer
  -h, --help               Print help
  -V, --version            Print version
```
