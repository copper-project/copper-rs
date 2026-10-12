# Linux PTP setup

The [application example](../README.md) consumes a local or network PTP reference.
These instructions cover preparing that reference with LinuxPTP (`ptp4l`,
`pmc`, and `phc2sys`) and identifying hardware with `ethtool`.
Run commands from the repository root and replace `enp1s0` with your interface.

## Follow an existing grandmaster

This is the second step after the standalone demo. On each participating machine,
set `"allow_local_master": false` in the resource's `config`, and point `device`
at that machine's PHC. Both machines must follow the same grandmaster and domain.
Use `cargo run -p cu-clock-sync` with the configured services; run `just` from
this example's directory for the local reference.

Identify the interface's PTP hardware clock (PHC):

```sh
ethtool -T enp1s0
ls -l /dev/ptp*
```

Set `device` in `copperconfig.ron` to the reported PHC, such as `/dev/ptp0`.
Give your application user read access through device permissions or udev rules.
Start ptp4l in hardware-timestamped client mode:

```sh
sudo ptp4l -H -s -i enp1s0 -m
```

The app must be able to write to `/var/run/ptp4l` to send GET requests. With
linuxptp's default socket mode, give your user's group access from another terminal:

```sh
sudo chgrp "$(id -g)" /var/run/ptp4l
```

For a persistent service, configure its socket group through your service setup.
Use `"management": "/var/run/ptp4l"` in Copper's resource configuration.
The read-only `ptp4lro` socket rejects PHC identity and timestamping-mode queries.

Wait for synchronization to the grandmaster. Confirm the selected port and time
status through the default GET-only management socket:

```sh
pmc -u -b 0 -i /tmp/cu-clock-sync-pmc -s /var/run/ptp4lro 'GET PORT_DATA_SET' 'GET TIME_STATUS_NP'
```

Then run `cargo run -p cu-clock-sync`. The example assumes domain 0 and port 1;
set the resource's `domain` and `port` for a different service configuration.
The ptp4l options and socket defaults are documented in the
[ptp4l manual](https://www.linuxptp.org/documentation/ptp4l/).

## Local hardware grandmaster

Follow the [standalone demo instructions](../README.md#quick-start).
No second machine is needed: phc2sys disciplines the PHC from system UTC and
gets the UTC/PTP offset from ptp4l. See the
[phc2sys manual](https://www.linuxptp.org/documentation/phc2sys/).

## Software PTP without a PHC

Use the same application with a software reference. Start a local
software-timestamped grandmaster on a test network:

```sh
sudo ptp4l -S -i enp1s0 --serverOnly 1 -m
```

Give your user's group access to `/var/run/ptp4l` as described above, and wait
for the port to reach MASTER. Replace only the PTP resource declaration in
`copperconfig.ron` with:

```ron
resources: [(
    id: "ptp",
    provider: "cu_ptp::LinuxSystemPtpBundle",
    config: {
        "management": "/var/run/ptp4l",
        "reference_error_ns": 5000000,
        "utc_offset_seconds": 37,
        "allow_local_master": true,
    },
)],
```

Supply the known TAI−UTC offset for your experiment instead of assuming 37.
The provider reads system UTC and adds this offset to expose TAI. The example's
5 ms upstream bound and 20 ms total budget are starting values, not accuracy
guarantees. Run `cargo run -p cu-clock-sync`; the task code stays the same.

This experiment demonstrates a common epoch on one machine. Software ptp4l uses
the system clock for timestamps; keep its synchronization service stable.
A system clock step or reference change causes Copper to stop and require
reacquisition.
