# Teleops deadman

In this directory, run `just robot` and `just twin` in separate terminals.
The twin header shows the latest reconstructed robot CopperList ID and live
application round-trip latency (RTT). A dash means no probe reply in the last second.
When the twin shows **Robot armed**, click the red **STOP** button or press
Enter. The twin holds **Press to action** and **Press to confirmation** on
screen while stopped. Click the green **RESUME** button to rearm and run
another measurement, or press **Q** to quit. The robot terminal opens Copper’s ConsoleMon
with the live task graph, loop count and task timings. Both recipes use the
workspace’s optimized `screaming` profile.

Start either process first. The robot loop starts immediately. In its DAG,
the moving tick marks loop activity and the safety node shows
a red **SAFEMODE: actuation off** error and emits no output while it waits
for a valid permit. The first valid permit enables simulated
actuation and changes the status to **ARMED: permit active**. STOP latches
**SAFEMODE: red button**. An expired permit latches **SAFEMODE: permit lost**.
In either state the safety node stays red and emits no permit. Reconnecting
the twin shows RESUME. The robot requires an explicit resume for that specific
trip before it accepts ordinary permits again. The loop keeps running to report
its state. This example drives a software watchdog with no physical actuator.

## Two hosts

Both recipes accept a local UDP bind address followed by the other machine's
address. For example, with the robot at 192.168.1.10 and the operator station at
192.168.1.20, run these on their respective machines.

```sh
# Robot
just robot 192.168.1.10:7441 192.168.1.20:7442

# Operator station
just twin 192.168.1.20:7442 192.168.1.10:7441
```

Use a trusted network that permits UDP on these ports. Build both binaries with
`cargo build --profile screaming --bins`, or build `--bin robot` and `--bin twin`
separately for their respective platforms. Deploy both from the same revision
and pass the same two addresses shown above to each executable. Each launch replaces the previous recording for that role at
`logs/robot.copper` or `logs/twin.copper` in its working directory. For a headless
measurement, append `--auto` to the twin executable’s arguments.

## What the latency measures

**Press to action** subtracts the twin’s button timestamp from the robot’s
recorded watchdog transition timestamp. The twin supplies a `RobotClock` to
the teleops sampler and stamps STOP when it reads the click. Every 100 ms the
twin sends its raw send timestamp in a probe. The robot echoes it in the
CopperList alongside the watchdog's receive and reply timestamps. The twin
records receipt on its own clock, subtracts robot processing time from the
round trip, and assumes symmetric path delays to align the two clock midpoints.
The header shows **Clock synced** while the estimated error is within 10 ms.
Clock probes continue while stopped without renewing the actuation permit.
A confirmed press shows one approximate action latency, using the synchronized robot timestamp. If it falls outside the local send-to-confirmation
window, **RTT estimate** adds the click-to-send delay to half the remaining round trip. STOP also works before
clock acquisition and still reports confirmation latency.

**Press to confirmation** measures the time from reading the button to
reading the first reconstructed frame confirming that specific trip in the twin.
Both timestamps come from the local monotonic counter of the same operator
`RobotClock`. This includes the UDP return link, robot processing, telemetry
delivery, replay and polling. The watchdog records the trip in the same CopperList
in which the safety gate rejects actuation. Subsequent frames repeat the trip for confirmation.

While the button is released, the twin sends a permit for each observed
CopperList. The robot uses that list's captured tick as the permit timestamp,
rejects input more than 32 iterations old, and expires a permit after 150 ms.
A press ends the permits and sends one stop. If the packet is lost, the permit
expires and the twin reports missing confirmation. Only a matching button
trip produces a latency result.
