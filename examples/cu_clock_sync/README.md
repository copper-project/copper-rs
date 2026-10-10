# A local PTP reference for Copper

Run the mock first from the repository root:

```sh
cargo run -p cu-clock-sync --bin clock-sync-mock
```

It runs for about five seconds, stamps messages against a simulated reference
with a 20 ppm rate difference, and prints the final clock quality. Its Copper
log is in this example's own `logs/` directory. Read the recorded timestamps:

```sh
cargo run -p cu-clock-sync --bin clock-sync-logreader -- \
  examples/cu_clock_sync/logs/mock.copper extract-copperlists
```

## Software PTP on one Linux machine

Install LinuxPTP (`ptp4l`, `pmc`, `phc2sys`) and `ethtool` using your distribution's
package manager. Choose an active Ethernet interface using `ip link`; substitute
its name for `enp1s0` below. Use a test network: this setup advertises the local
machine as a PTP grandmaster.

1. Start a software-timestamped local grandmaster in another terminal:

   ```sh
   sudo ptp4l -S -i enp1s0 -f examples/cu_clock_sync/ptp4l-local.conf -m
   ```

   Wait for the interface to reach MASTER. The supplied configuration selects
   `serverOnly`, domain 0, and a readable GET-only management socket. A peer is
   optional for this local experiment. Older LinuxPTP versions may call
   `serverOnly` `masterOnly`; use the spelling documented by your installed version.

2. Confirm the service and selected port:

   ```sh
   pmc -u -b 0 -s /var/run/ptp4lro 'GET PORT_DATA_SET' 'GET TIME_STATUS_NP'
   ```

3. Check `software.ron`: the example explicitly permits a local grandmaster and
   supplies `utc_offset_seconds: 37`. Replace 37 with the known TAI−UTC offset
   for your experiment when needed. Software ptp4l uses the system's UTC clock;
   Copper adds this offset to expose TAI. The configured 5 ms upstream error
   and 20 ms total budget are starting values, not an accuracy guarantee.

   ```sh
   cargo run -p cu-clock-sync --features linux-ptp --bin clock-sync-software
   ```

The application can run as your regular user. It reads the system clock and
ptp4l's GET-only socket. This experiment establishes a common epoch; its absolute
accuracy follows your system clock. Keep that clock's synchronization service
stable while the example runs. A system clock step or reference change causes
Copper to stop and require reacquisition.

## Hardware PTP

Identify your interface's PHC and timestamping support:

```sh
ethtool -T enp1s0
ls -l /dev/ptp*
```

Set `linux.ron`'s `device` to the reported PTP hardware clock number, for example
`/dev/ptp0`. Give your application user read access through your system's device
permissions or udev rules.

For a local hardware grandmaster, run these in separate terminals:

```sh
sudo ptp4l -H -i enp1s0 -f examples/cu_clock_sync/ptp4l-local.conf -m
sudo phc2sys -s CLOCK_REALTIME -c /dev/ptp0 -w -m
```

The second command disciplines the PHC from system UTC and obtains the UTC/PTP
offset from ptp4l. Wait for stable offsets before starting Copper:

```sh
cargo run -p cu-clock-sync --features linux-ptp --bin clock-sync-linux
```

To follow an existing grandmaster, run ptp4l with `-H -s` on the appropriate
interface and remove `serverOnly` from its configuration. Set
`allow_local_master` to false in the Copper resource config. ptp4l disciplines
the PHC; Copper only reads it. Match the domain and port between the service and
resource, and measure a suitable `reference_error_ns` for your network.

## Clock health and recordings

Tasks use ordinary `ctx.now()` and `Tov::Time` with the shared reference epoch.
Clock clones expose `sync_status()` with domain/session, phase, drift, sample age
and estimated error. Losing upstream quality enters bounded holdover; exceeding
the error or age limit fails the next iteration. Restarting reacquires the
reference before consumers start.

The unified log's RuntimeLifecycle stream stores correction snapshots before
the first CopperList using each curve. Enable `clock-sync` in replay applications
to restore those snapshots through the recorded replay engine. Replay constructs
resources without starting the parent or opening PTP handles. Sync-enabled logs
use an extended lifecycle format; use an equally enabled logreader.

The v0 runtime uses the Serial planner. Parallel clock correction requires a
coordinated barrier across in-flight CopperLists and is rejected by the builder.

See the upstream manuals for [ptp4l](https://www.linuxptp.org/documentation/ptp4l/),
[pmc](https://www.linuxptp.org/documentation/pmc/) and
[phc2sys](https://www.linuxptp.org/documentation/phc2sys/), and the kernel's
[PTP hardware clock interface](https://docs.kernel.org/driver-api/ptp.html).
