# Raspberry Pi based encoder driver for Copper

## Overview

This driver is for the Raspberry Pi based encoder driver for Copper.

## Interface

Publishes source payloads into the Copper graph on each runtime tick.

## Configuration

Use `config` keys and `resources` bindings for hardware/driver handles.

## Usage

On Linux hardware, provide both GPIO inputs through resource bindings:

```ron
resources: [
    (
        id: "enc",
        provider: "cu_linux_resources::LinuxResources",
        config: {
            "gpio_in0_pin": 17,
            "gpio_in1_pin": 18,
        },
    ),
],
tasks: [
    (
        id: "src",
        type: "cu_rp_encoder::Encoder",
        resources: {
            clk_pin: "enc.gpio_in0",
            dat_pin: "enc.gpio_in1",
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

## Compatibility

Any encoder with a base clock + a direction trigger.

## Links

- Crate path: `components/sources/cu_rp_encoder`
- docs.rs: <https://docs.rs/cu-rp-encoder>
