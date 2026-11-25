# Teleoperation over the telemetry return link

**Proposal.** Give the console a typed sender and the robot an ordinary
`CuSrcTask` for each operator input. A resource connects the source to the
telemetry endpoint. The source is where an asynchronous command becomes an
input to a particular CopperList. A customer task decides what that input may do.

## What the application writes

Enable `cu29/teleops` on both builds and wire a return endpoint for the
application's telemetry driver. FEC feedback is optional. These APIs are proposed.

```rust
// Shared application crate. The 4 is the robot's event queue capacity.
#[derive(Teleops)]
pub struct OperatorInputs {
    pub stick: Latest<Pilot>,               // Pilot { held: bool, axes: Axes }
    pub mode: Latched<Mode>,
    pub take_photo: Event<PhotoRequest, 4>, // PhotoRequest { request_id: u64 }
}

pub type ModeUpdate = CuLatchedStateUpdate<Mode>;
```

Each field generates a typed console accessor, a named inbox resource and a
source alias in `operator_inputs`. Declaration order supplies private wire
tags. Both endpoints assume matching application metadata and the same
declaration. Checking metadata compatibility is a separate change.

```rust
// Works with Twin::builder or Twin::from_telemetry.
let (builder, mut operator) = Twin::from_telemetry(PlainUdpRx::new(rx))
    .with_teleops::<OperatorInputs>(return_tx)?;
let (twin, frames) = builder.spawn()?;

// Fresh UI input. Sample the enable switch and axes together.
operator.stick().try_send(&Pilot { held: pad.enable(), axes: pad.axes() })?;
operator.mode().set(&Mode::Inspection)?;
operator.take_photo().try_send(&PhotoRequest { request_id: click_id })?;
// On a later transition:
operator.mode().clear()?;
```

### Time of validity belongs to the sample

Each submission produces one ordinary graph message. Stick axes and the enable
switch share a `CuMsg<Pilot>` and a deadline. A click has its own
`CuMsg<PhotoRequest>`. A mode update's ToV dates its `Set` or `Clear`
transition. The consumer retains the resulting mode across idle cycles.

```rust
// Alternative input path for a console running its own Copper graph.
// gamepad_msg: &CuMsg<Pilot>, with the ToV assigned by the gamepad source.
operator.stick().try_send_msg(gamepad_msg)?;
```

- `try_send(&payload)`, `set` and `clear` submit fresh samples. Bind their
  time at acquisition, before queueing. With unsynchronized clocks, use a
  robot tick already observed before sampling as a conservative validity start.
- `try_send_msg(&msg)` forwards the payload and its supplied ToV. A time or
  range in a console clock domain requires a known mapping to robot time.
  Reject an unmappable timestamp. Preserve `Tov::None` as unknown and skip
  empty messages. Only the robot runtime supplies execution metadata.
- A full local sender returns `WouldBlock`. Local admission, retransmission
  and robot dequeue never renew a sample's ToV. The control task requires a
  fresh, timed observation to grant manual control.

```rust
// Robot setup, sketched for the threadless plain UDP example.
// app_resources binds the named consumer slots from the graph below.
let (publish, inboxes) = OperatorInputs::inboxes()?;
let robot = Robot::builder()
    .with_resources(move |cfg| app_resources(cfg, inboxes))
    .with_telemetry_tx(boot_session_id, move |clock| {
        let sender = PlainUdpTx::new(data_tx);
        Ok(sender.with_teleops(return_rx, publish, clock))
    }, buffers)
    .build()?;
```

```ron
// Robot graph excerpt. The app's resource bundle publishes the three inboxes.
tasks: [
    (id: "stick", type: "app::operator_inputs::StickSource", resources: {"inbox": "ops.stick"}),
    (id: "mode",  type: "app::operator_inputs::ModeSource",  resources: {"inbox": "ops.mode"}),
    (id: "photo", type: "app::operator_inputs::TakePhotoSource", resources: {"inbox": "ops.take_photo"}),
    (id: "tick", type: "app::RobotTick"),
    (id: "control", type: "app::Control"),
    (id: "camera", type: "app::Camera"),
],
cnx: [
    (src: "stick", dst: "control", msg: "app::Pilot"),
    (src: "tick",  dst: "control", msg: "app::Tick"),
    (src: "mode",  dst: "control", msg: "app::ModeUpdate"),
    (src: "photo", dst: "camera",  msg: "app::PhotoRequest"),
],
```

| Channel | Between robot iterations | Source output on an idle cycle |
| --- | --- | --- |
| Stick | Keep the newest complete sample, including an explicit release | Empty `CuMsg<Pilot>` |
| Mode | Keep the newest `Set` or `Clear`. Skip `NoChange` at submission so it cannot erase a pending transition | Empty message. The consumer retains its `CuLatchedState<Mode>` |
| Photo | Accept newer events in arrival order into a four-entry FIFO. Drop incoming events when full. Consume at most one per cycle | Empty message. A camera fires only on a fresh request |

On a lossy link the console periodically republishes its **desired** mode
(including `Clear`) until the twin shows the applied mode. For a photo that
must be confirmed, expose `PhotoResult { request_id, outcome }` on the robot
graph. Retry with the same application ID and deduplicate at the camera task
over the advertised retry window. A packet sequence only suppresses transport
duplicates. The twin can display the actual decision through existing telemetry.

## Operator presence belongs in the graph

Example customer policy: sample the pad every 40 ms. The robot starts in safe
mode and requires a positive sample less than 150 ms old on **every** iteration.
Release revokes control immediately when observed. Keep the axes and their
expiry together so an unrelated heartbeat cannot authorize a stale setpoint.

```rust
// Inside Control::process. 'now' is a robot-time tick supplied by a captured
// graph input, including on idle cycles. All times use the same clock domain.
if let Some(update) = mode.payload() {
    self.mode.update(update);
}
if let (Some(pilot), Tov::Time(valid_from)) = (stick.payload(), stick.tov) {
    self.manual = pilot.held.then(|| (pilot.axes, valid_from + self.hold_time));
}
match self.manual {
    Some((axes, until)) if now < until => out.set_payload(self.command(axes)),
    _ => out.set_payload(self.safe_command()),
}
```

`hold_time` and re-engagement are application policy. This example permits
re-engagement on the next fresh held sample. An application that requires an
explicit re-arm should carry a separate arm request and retain the trip state.
Freeze the control task's latch, manual command, deadline and any application
deduplication state needed after a keyframe. Capture source payloads **and
their ToVs**. During twin reconstruction or offline replay the recorded source
outputs and tick drive the same timeout, even across a silent interval.

## Where the packets go

Today the FEC worker reads `CUFB` datagrams into a 320-byte report buffer.
The plain UDP example sends native telemetry through `CuTelemetryTx` and
receives it through `CuTelemetryRx`. Its packet codec already runs on
`no_std`. [FEC worker](../../core/cu29_logstream/src/worker.rs) ·
[Transport contracts](../../core/cu29_logstream/src/stream.rs) ·
[Plain UDP](../../examples/cu_udp_telemetry/src/lib.rs)

```text
console UI ── typed command ─┐                       ┌─ CUFB → FeedbackController
twin receiver ─ FEC report ──┴─ return TX → robot RX ┤
                                                    └─ teleops → typed inbox → source → graph
```

One owner reads each physical endpoint and dispatches complete packets before
either decoder runs. Keep the existing report encoding. Give commands a distinct
packet tag and a bounded codec, for example:

```text
CUOP | framing version | robot session | control epoch | generated field tag
     | per-channel sequence | sample validity (robot-time ToV or tick reference) | payload
```

- Bind to one authorized console and robot session. Issue a new control epoch
  on takeover or reconnect. Reject older epochs, unknown field tags,
  old or repeated sequences, oversized frames and trailing bytes before publishing.
  Check the full payload before advancing the sequence. UDP checksums and session
  IDs provide no sender authentication. The deployment supplies an authenticated
  carrier or establishes a trusted point-to-point link.
- For fresh samples using unsynchronized clocks, echo a recent robot-issued freshness reference from
  telemetry, e.g. a captured robot tick identified by CopperList ID. The console
  samples the controls *after* observing that reference. The robot resolves it in
  a bounded history of issued ticks, drops expired or unknown references, and
  puts that tick's robot time in `CuMsg.tov`. Queueing and delayed delivery then
  consume the 150 ms lease. Starting a fresh lease at dequeue would let an old
  held-stick packet revive control. On epoch change flush inboxes and deliver
  revocation to the control graph before admitting the new operator. For this
  example, the application supplies a release sample (`Pilot::default()` with
  `held == false`) and a mode `Clear`. Reserve their delivery for one graph
  iteration before publishing new samples. A disconnected console also expires
  through the ordinary deadline when no explicit disconnect arrives.
- Size the return receive buffer at setup for the largest enabled packet,
  within the carrier MTU. Decode once into bounded typed storage. The source
  takes at most one value into its runtime-owned output slot per iteration.
  On host, use preallocated single-producer/single-consumer handoffs and schedule
  return polling even during telemetry silence. On firmware, service a fixed
  packet budget in the existing between-iteration poll. The next iteration
  observes admitted inputs. Budget traffic and measure worst-case service time
  with maximum packets on the target.
- In the plain UDP example, compose the return reader with the existing sender
  through `CuTelemetryTx::poll`. A custom protocol can decode its own framing
  and publish to the same typed inboxes. For FEC, extend the existing worker's
  receive loop to route both tags with independent processing budgets. Configure
  one return carrier even when only teleops is enabled.

## First implementation slice

1. Generate accessors, source aliases and field tags from `#[derive(Teleops)]`.
   Reject zero event capacities. In `cu29_logstream`, add the paired inbox
   endpoints and bounded return codec. In `cu_udp_telemetry`, exercise one stick channel
   through both host and threadless senders. Compile the protocol for
   `thumbv8m.main-none-eabihf`.
2. Bind the consumer half via task resources and implement `TeleopSource`.
   Check that forwarding a queued `CuMsg` preserves its validity and a click
   cannot refresh the stick. Test loss, duplicate delivery, expiry and re-engagement.
   Replay from a keyframe mid-hold with the live endpoint disconnected. Compare
   the exact CopperList where control returns to safe mode.
3. Add the twin builder handle and shared FEC dispatch. Exercise teleops alone,
   feedback alone, and interleaved commands and reports. Verify event overflow,
   a dropped mode clear followed by retransmission, and old-epoch traffic after
   reconnect. Build the legacy one-way example with the feature disabled.
