## Raspberry GPIO driver for Copper

This enables the communication with the GPIO pins of a raspberry Pi as a Sink task on Copper.

`RPGpio` can take an owned GPIO pin resource via `resources: { pin: \"<bundle>.gpio_out0\" }`
from `cu_linux_resources::LinuxResources`.

See the crate [cu29](https://crates.io/crates/cu29) for more information about the Copper project.
