## This is an incoming bridge from Zenoh

It allows you to receive messages from external systems via [Zenoh](https://zenoh.io/).

### Config

zenoh_config_file: Zenoh [configuration json file](https://github.com/eclipse-zenoh/zenoh/blob/main/DEFAULT_CONFIG.json5) (optional).
topic: the name of the topic to subscribe to, in accordance with the [key expressions rules](https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Key%20Expressions.md).

Example in your Copper configuration file:

```RON
    tasks: [
        (
            id: "zenohsrc",
            type: "cu_zenoh::CustomZenohSrc",
            config: {
                "zenoh_config_file": "/home/cam/dev_ext/zenoh/DEFAULT_CONFIG.json5",
                "topic": "copper/input"
            },
        ),
   ]
```

See the crate [cu29](https://crates.io/crates/cu29) for more information about the Copper project.
