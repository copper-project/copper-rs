## WitMotion WT901 driver for Copper

This enables the communication with a WitMotion WT901 over I2C a Source task on Copper.

On Linux, WT901 can consume an owned I2C resource (`i2c`) from
`cu_linux_resources::LinuxResources`. If no resource binding is provided, it
falls back to opening `i2c_bus` from task config (default: `/dev/i2c-9`).

See the crate cu29 for more information about the Copper project.
