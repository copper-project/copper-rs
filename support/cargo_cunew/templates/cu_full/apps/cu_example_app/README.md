This is the primary Copper application for the workspace (`cu_example_app`). Local tasks and messages live under `src/`.

The app ships with three host-side binaries:

- `cu_example_app`: normal runtime execution.
- `cu_example_app-logreader`: offline log export and inspection.
- `cu_example_app-resim`: replay target that can run once or expose the remote debug API with `--debug-base`.
