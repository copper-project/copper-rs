# cu-zenoh-sink

## Overview

### This is an outgoing bridge towards Zenoh

Note: This sink is deprecated in favor of the bidirectional `cu-zenoh-bridge` component.

It allows you to send Copper messages to external systems via [Zenoh](https://zenoh.io/).

## Interface

Consumes upstream payloads and performs external side effects (actuation, IO, or export).

## Configuration

### Config

zenoh_config_file: Zenoh [configuration json file](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5) (optional).
topic: the name of the topic to publish the messages in accordance with the [key expressions rules](https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Key%20Expressions.md).

Example in your Copper configuration file:

```RON
    tasks: [
        (
            id: "zenohsink",
            type: "cu_zenoh::CustomZenohSink",
            config: {
                "zenoh_config_file": "/home/cam/dev_ext/zenoh/DEFAULT_CONFIG.json5",
                "topic": "copper/output"
            },
        ),
   ]
```

See the crate [cu29](https://crates.io/crates/cu29) for more information about the Copper project.

## Usage

Wire upstream producers to this sink in `copperconfig.ron` and set required config keys.

## Compatibility

Depends on output transport/hardware availability and selected Cargo features.

## Links

- Crate path: `components/sinks/cu_zenoh_sink`
- docs.rs: <https://docs.rs/cu-zenoh-sink>
