# Teleoperation over the telemetry return link

**Proposal.** Enable `cu29/teleops` on the robot and console. Each operator input enters the robot graph through an ordinary `CuSrcTask`. Customer tasks own safety policy.

## Application API

```rust
#[derive(Teleops)]
pub struct OperatorInputs {
    pub stick: Latest<Pilot>,               // Pilot { held: bool, axes: Axes }
    pub mode: Latched<Mode>,
    pub take_photo: Event<PhotoRequest, 4>, // Four pending clicks
}
```

Fields generate console accessors, inbox resources, source aliases and private wire tags in declaration order. Both builds assume matching application metadata. Compatibility validation is a separate change.

```rust
let (builder, mut operator) = Twin::from_telemetry(PlainUdpRx::new(rx))
    .with_teleops::<OperatorInputs>(return_tx)?;
let (twin, frames) = builder.spawn()?;

// In a UI callback, bind the frame actually on screen before sampling.
let mut input = operator.observing(displayed_copperlist_id);
input.stick().try_send(&Pilot { held: pad.enable(), axes: pad.axes() })?;
input.take_photo().try_send(&PhotoRequest { request_id: click_id })?; // on click
input.mode().set(&Mode::Inspection)?; // or input.mode().clear()

// A headless Copper console can forward a stamped CuMsg<Pilot> directly.
operator.observing(displayed_copperlist_id).stick().try_send_msg(gamepad_msg)?;
```

```rust
let (publish, inboxes) = OperatorInputs::inboxes()?;
let robot = Robot::builder()
    .with_resources(move |cfg| app_resources(cfg, inboxes))
    .with_telemetry_tx(boot_session_id, move |clock| {
        PlainUdpTx::builder(data_tx)
            .with_teleops(return_rx, publish, clock)
            .build()
    }, buffers)
    .build()?;
```

```ron
// One generated source and its application consumer, excerpted from the graph.
tasks: [
    (id: "stick", type: "app::operator_inputs::StickSource", resources: {"inbox": "ops.stick"}),
    (id: "control", type: "app::Control"),
],
cnx: [(src: "stick", dst: "control", msg: "app::Pilot")],
```

| Channel | Graph behavior |
| --- | --- |
| `Latest<Pilot>` | Select the newest sample in the batch for each observed robot CL. Send at most once per field per CL. Axes and enable switch share one ToV. |
| `Latched<Mode>` | Latest `Set` or `Clear` wins. Its ToV dates the transition. Consumers retain `CuLatchedState<Mode>`. |
| `Event<PhotoRequest, 4>` | FIFO, one event per iteration. Drop incoming events when full. For confirmed actions, deduplicate by request ID and publish the result. |

Idle sources emit empty messages. A full console sender returns `WouldBlock`. Success means local submission.

## Which CopperList did the operator see?

| Evidence | Meaning |
| --- | --- |
| `observed_copperlist_id: u64` | Robot frame displayed **when the input was sampled**. Scoped to the robot session. Wait for a frame before sending. |
| `CuMsg.tov` | Sample validity in robot time. Queueing and retries preserve it. |
| Accepting CopperList ID | Robot iteration in which the source emitted the command. Available from the enclosing CopperList. |

The source checks the original observed CL against its actual iteration immediately before emission. Its RON setting `max_lag_cl` is the largest allowed difference (default 1). Future and expired samples are dropped. A newer observed CL supersedes an older pending joystick position. Multiple events and latch transitions within one CL retain their own transport ordering.

For an input sampled on displayed CL 812, submit the newest available reading once, for example with `operator.observing(812).stick().try_send_latest(samples.iter())`. An empty batch emits nothing. A stamped input retains its original validity independently of this CL-based admission check.

## Deadman and replay

Example policy in `Control::process`. Start with `self.manual = None`. Read `now` from a captured robot tick every iteration, including during silence.

```rust
if let Some(pilot) = stick.payload() {
    self.manual = match (pilot.held, stick.tov) {
        (true, Tov::Time(t)) if t <= now => Some((pilot.axes, t + self.hold_time)),
        _ => None,
    };
}
match self.manual {
    Some((axes, until)) if now < until => out.set_payload(self.command(axes)),
    _ => out.set_payload(self.safe_command()),
}
```

With a 150 ms hold time, a sample already 120 ms old grants at most 30 ms of control. Capture source outputs and ToVs. Keyframe the customer's latch, command, deadline and action deduplication state so replay takes the same branch.

## Transport and verification

```text
CUOP | version | robot session | field tag | observed_copperlist_id
     | sequence (events and latch transitions only) | validity (ToV or use observed tick) | payload
```

One reader per endpoint routes `CUFB` to FEC and `CUOP` to typed inboxes. Custom transports can publish into the same inboxes. Use a trusted or authenticated carrier. Validate framing, robot session and channel ordering before publication.

Use preallocated inboxes, bounded polling on host even during telemetry silence, and a fixed packet budget in the threadless `CuTelemetryTx::poll`. [FEC worker](../../core/cu29_logstream/src/worker.rs) · [Plain UDP example](../../examples/cu_udp_telemetry/src/lib.rs)

Verify host and embedded UDP, mixed feedback and commands, and event overflow. Freeze the displayed frame while newer telemetry arrives and check the recorded observed ID. Delay a held sample past expiry, then replay from a mid-hold keyframe and compare the exact safe-mode iteration.
