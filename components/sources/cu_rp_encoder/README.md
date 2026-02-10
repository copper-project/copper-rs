# Raspberry Pi based encoder driver for Copper

This driver is for the Raspberry Pi based encoder driver for Copper.

## Compatibility

Any encoder with a base clock + a direction trigger.

## Usage

On Linux hardware, provide both GPIO inputs through resource bindings:

```ron
resources: [
    (
        id: "enc_clk",
        provider: "cu_linux_resources::LinuxResources",
        config: { "gpio_in_pin": 17 },
    ),
    (
        id: "enc_dat",
        provider: "cu_linux_resources::LinuxResources",
        config: { "gpio_in_pin": 18 },
    ),
],
tasks: [
    (
        id: "src",
        type: "cu_rp_encoder::Encoder",
        resources: {
            clk_pin: "enc_clk.gpio_in",
            dat_pin: "enc_dat.gpio_in",
        },
    ),
]
```

In `mock` mode, pins can be supplied through task config:

```ron
    tasks: [
        (
            id: "src",
            type: "cu_rp_encoder::Encoder",
            params: {
                clk_pin: 17,
                dat_pin: 18,
            },
        ),
    ]
```

The `clk_pin` is the pin for the clock signal and the `dat_pin` is the pin for the direction signal.

When you connect this driver to the rest of the system you need to use the `cu_rp_encoder::EncoderMsg` message type.

```ron
    cnx: [
        (src: "src",  dst: "dst",   msg: "cu_rp_encoder::EncoderMsg"),
    ],
```

It has been tested with a Hall effect encoder like this one:

<img alt="The encoder" src="doc/encoder.jpg">
