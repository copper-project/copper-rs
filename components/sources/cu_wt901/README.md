## WitMotion WT901 driver for Copper

This enables the communication with a WitMotion WT901 over I2C a Source task on Copper.

On Linux, WT901 consumes an owned I2C resource (`i2c`) from
`cu_linux_resources::LinuxResources`, for example:
`resources: { i2c: "<bundle>.i2c1" }`.

See the crate cu29 for more information about the Copper project.
