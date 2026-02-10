## Lewansoul Serial Bus driver for Copper

This enables the communication with the Lewansoul bus servos as a Sink task on Copper.

`Lewansoul` can consume an owned serial resource via
`resources: { serial: \"<bundle>.serial\" }` from
`cu_linux_resources::LinuxResources`.

See the crate [cu29](https://crates.io/crates/cu29) for more information about the Copper project.
