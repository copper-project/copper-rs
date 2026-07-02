# cu_ryuw122_probe

Minimal robot-side Copper example for two REYAX `RYUW122` modules.

It gives you three short flows:

- configure the robot module that runs the Copper source
- configure the second module as the fixed localization anchor
- run the Copper source against the robot serial port and print live ranges

The Copper source runs on the modem that uses the REYAX vendor `ANCHOR` role, because that is
the side that reports distance over UART. The fixed space anchor uses the vendor `TAG` role.

The example keeps the test setup intentionally fixed:

- robot address: `ROBOT001`
- fixed anchor address: `ANCH0001`
- network id: `REYAX123`
- CPIN: `FABC0002EEDCAA90FABC0002EEDCAA90`

## Quick start

From this directory:

```bash
just robot
just anchor
just probe
```

Default serial ports:

- robot / Copper source: `/dev/ttyACM0`
- fixed anchor: `/dev/ttyACM1`

Override them by passing a port explicitly:

```bash
just robot /dev/ttyUSB0
just anchor /dev/ttyUSB1
just probe /dev/ttyUSB0
```

The `probe` command writes a normal Copper log to:

```text
logs/cu_ryuw122_probe.copper
```

If no ranges appear, check that:

- both modules are powered and visible as serial devices
- the second module was configured with `just anchor`
- the robot modem is the port passed to `just probe`
