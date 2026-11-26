# Clock synchronization over telemetry

A teleops station can use the existing bidirectional telemetry link to learn robot time. We propose an optional probe and reply exchange. The robot stays the time reference. The station disciplines a clock dedicated to capturing operator inputs. An operator can send a command at any point during acquisition or loss of sync. The observed CopperList ID travels with every command. [Teleoperation design](teleops.md)

## One exchange

| Event | Timestamp | Packet |
| --- | --- | --- |
| Station sends a probe on the return link | `t1`, station raw time | Protocol version, robot session ID, probe ID |
| Robot receives the probe | `t2`, robot execution time | |
| Robot sends a reply on the forward link | `t3`, robot execution time | Echoed session and probe ID, `t2`, `t3`, reference domain and origin |
| Station receives the reply | `t4`, station raw time | |

Use a distinct versioned packet prefix for clock traffic. The robot's existing return reader can route probes alongside `CUFB` feedback and `CUOP` commands. On the forward link, route replies at the datagram boundary before FEC decoding. Both directions share their existing socket owner. Service probes periodically, independently of command submission, feedback reports and telemetry captures.

Keep one probe outstanding. Record `t1` at the successful send attempt and capture the other three timestamps as close to transport receive/send as the carrier allows. The station accepts one matching reply before a raw-clock deadline. After timeout it issues a fresh ID and discards late replies. Bind probes to the validated robot session and allocate IDs that stay distinct across station reconnects to that session. Keep request and reply sizes fixed and probe work bounded within the telemetry poll budget. Under backpressure, skip a probe or drop a reply.

## From timestamps to a Copper observation

Pair the midpoint of `[t1, t4]` on the station's raw clock with the midpoint of `[t2, t3]` on the robot's execution clock. The residual round-trip time is `(t4 - t1) - (t3 - t2)`. Subtracting robot turnaround removes time the robot spent holding the request. With unknown path asymmetry, allow half the residual round trip as uncertainty in the midpoint estimate. For example, an 8 ms round trip with 2 ms of robot turnaround leaves 6 ms in transit and up to 3 ms of offset uncertainty before other errors. [Four-timestamp calculation, RFC 5905 §8](https://www.rfc-editor.org/rfc/rfc5905#section-8)

Populate a `ClockObservation` with the two midpoints, the robot's domain and origin, and an uncertainty bound that also covers timestamp precision, capture error and relative clock-rate error during the exchange. Use checked integer arithmetic and round uncertainties up. Discard reversed timestamps, negative residuals, exchanges spanning a reference discontinuity and observations exceeding the configured error budget. The station passes accepted observations to Copper's existing `ClockSync` for drift tracking and holdover. The robot may have a boot-relative clock. Absolute UTC synchronization is unnecessary. [Clock discipline and reference domains](clock-sync.md)

## What a button press records

At the input event, read the station raw time once. Use the same clock snapshot to evaluate sync quality and convert that instant into robot time. Record that ToV when the snapshot belongs to the current session and reports `Locked`, or `Holdover` within the configured age and error budget. During acquisition or expiry, record `Tov::None` and let the robot resolve the observed CopperList's tick. Queueing and retries preserve the captured ToV.

On a robot session change, discard the pending probe and queued commands from the old session, invalidate the sampling reference, and reacquire against the new session. Staleness checks still use the observed CopperList even under lock. Replay uses the ToV that entered the graph. [Teleops admission and replay](teleops.md)

## Verification for implementation

- Use two mock clocks with a known offset and drift. Inject symmetric and asymmetric delays and verify the reported uncertainty covers the known offset error.
- Exercise delayed, duplicate and reordered replies, loss through holdover and expiry, and robot restart. Check the ToV of inputs captured before and after each transition, including a queued input retried after expiry.
- Interleave probes, commands and FEC feedback on loopback, including during telemetry silence. Verify the poll budget, command delivery and observed-CopperList fallback with an unresponsive sync peer.
