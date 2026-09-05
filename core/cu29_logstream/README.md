# Semantic log streaming

`SessionRouter` accepts complete packets from any `CuStreamRx` or through
`receive_datagram`. It discovers sender manifests without out-of-band FEC settings.
Use receiver-local hard limits for sessions, continuous records, startup packets,
completed recovery records, and finite-object decoding.

Startup packets are retained in arrival order up to `max_startup_packets` per
session. Overflow is counted and never implies that missing history was received.
Completed keyframes and anchors use a separate `max_recovery_records` cache;
oldest entries are evicted when full. This is additional to the bounded RaptorQ
object storage. Repetition may recover evicted objects only while the sender
still retains them. These are receiver allocations, outside the real-time path.

The router verifies each anchor against the exact manifest and keyframe record
digests, including the keyframe's CopperList id. Control objects can arrive in
any order. A verified boundary advances ordered delivery without discarding the
active FEC window or records at and after that boundary. Missing prefixes produce
`LateJoin` gaps; an established stream restarting after an outage produces
`AnchorRecovery` gaps. RLC expiration and explicit session finalization also emit
inclusive gaps. Repeated or older anchors cannot rewind delivery.

`SessionEvent::Object` is a raw recovered object, not permission to restore state.
Use `VerifiedAnchor` for that purpose. A consumer failure leaves the event pending;
call `drain_events` before accepting another packet. Call `finish_through` only
when the sender's last CopperList id is known. An unknown tail stays unknown.

## Native archives

With `std`, `NativeArchive<P>` writes one sender/session to a native `.copper` log.
Construct it on the manifest event. The receiver schema is derived from `P`'s
generated `MatchingTasks::get_output_specs()` and compared with the manifest:

```rust,ignore
let archive = NativeArchive::<default::CuStampedDataSet>::new(
    &path, manifest, slab_bytes, section_bytes,
)?;
```

Pass that sender's ordered router events to `accept`. Each CopperList is decoded
once into `CopperList<P>` and returned for an in-process consumer. The original
canonical payload bytes are appended directly to `CopperList` sections; verified
keyframes go into `FrozenTasks`. This requires the matching full-capture native
application codec. Hybrid payload reconstruction and codec conversion are deferred.
The section size must accommodate the largest canonical entry, including manifest
and keyframe objects. Use a fresh output path per session.

The `StreamContinuity` section preserves the canonical manifest, explicit gaps,
verified anchor references, and the receiver's final known boundary. `finish`
closes the archive; it does not assert that the unobserved sender tail is complete.
A failed archive write is terminal for that writer because an event can span more
than one native section.

Ordinary application logreaders read these CopperLists and keyframes.
`cu29_export::stream_continuity_reader` reads provenance and gaps; logreader `fsck`
reports the gap ranges and verified anchors. Generated recorded replay and indexed
state replay reject crossing missing CopperList ids without a matching keyframe.
A later keyframe permits state replay from its boundary; it does not heal history.

Run `just logstream-receiver-check` at the repository root. This exercises startup,
late join, digest rejection, consumer failure, native archive/export over real UDP,
and replay gap handling. Packet pacing, feedback, the two-process demonstrator,
hybrid reconstruction, and dashboard APIs remain separate follow-up work.
