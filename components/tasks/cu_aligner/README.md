## Message alignment

Align messages from different sources using shared-window batches or master-driven
observations. Both preserve complete source messages and metadata.

The following describes window alignment; master alignment is documented below.

### Theory of operation:

```plaintext
        STREAM 1                STREAM 2
           │                       │
  Msg1_S1 ─╯                       │ (discarded)
           │                       │
═══════════╪═══════════════════════╪═══════════ TIME HORIZON ( present - stale_data_horizon_ms )
           │                       │
           │                       │
           │              Msg1_S2 ─┤
  Msg2_S1 ─┤                       │
           │                       │
  Msg3_S1 ─┤              Msg2_S2 ─┤
           │                       │
           │                       │
           │                       │
═══════════╪═══════════════════════╪═══════════ BEGINNING OF ALIGNMENT WINDOW ( t - alignment_window_ms )
           │                       │
           │              Msg3_S2 ─┤
  Msg4_S1 ─┤                       │           <- This part will be in the output at each process call
           │                       │
  Msg5_S1 ─┤              Msg4_S2 ─┤
           │                       │
═══════════╪═══════════════════════╪═══════════ MOST RECENT ALIGNED MESSAGE ( t )
           │                       │
           │              Msg5_S2 ─┤
           │                       │
           │              Msg6_S2 ─┤ (not yet aligned)
```

The timings are taken from the CuMsg::tov field (time of validity).

### Usage

The task is generated entirely out of the `cu_aligner::define_window_task` macro:

```rust,ignore

use cu_aligner::define_window_task;
use cu29::input_msg;
use cu29::output_msg;
use cu29::cutask::Freezable;
use cu29::config::ComponentConfig;
use cu29::CuResult;
use cu29::cutask::CuTask;
use cu29::cutask::CuMsg;

// Defines a task that aligns two streams of messages, one with payloads f32, the other MyPayload (you can use any rust struct that to implement the traits for CuMsgPayload).
define_window_task!(MyAlignerTask,
             0 => { 15, 7, f32 },  // 15 is the Maximum capacity in nb of messages the internal
                                   // buffer structure can hold before they will me discarded,
                                   // 7 is the Maximum size in nb of messages the output (aligned messages of this type)
             1 => { 20, 5, u64 }   // or any CuMsgPayload
            // you can continue with 2 => etc...
            );

```

You defined task will need to be connected with the matching tasks upstream in the Copper config file.
The type of the input will be a tuple of CuMsg (which is what the aligner expects). From the example:
`(CuMsg<f32>, CuMsg<MyPayload>)`.
The type of the output will be a CuMsg of a tuple of CuArrays holding the aligned messages for each stream. From the
example: `CuMsg<(CuArray<CuMsg<f32>, 7>, CuArray<CuMsg<MyPayload>, 5>)>`.

Each array element preserves the complete input message: its optional payload, original
`Tov` (point or range), processing timestamps, status text, and remote origin metadata.
Read values with `sample.payload()` and timing with `sample.tov`; the outer message
metadata describes the aligner itself, not the original sources.

When migrating from payload-only arrays, change downstream input and RON connection
types from `CuArray<T, N>` to `CuArray<CuMsg<T>, N>` and access each value through
`payload()`. This also changes the serialized output schema.

### Performance consideration

Copper by itself never buffers anything to avoid copies but for this aligner has to copy data until it can align it.
It means you will have 1 copy from the input to the internal buffer and 1 copy from the internal buffer to the output.

If your usecase is just to get the latest message from 2 sources, just connect your task to the upstream tasks
but do not use this aligner. It is only useful if the time of arrival from the 2 upstream tasks are too far apart
in terms of tov.

### Window and master tasks

`define_window_task!` names the existing shared-window batching task.
`define_task!` remains a compatibility alias. Both take
`index => { buffer_capacity, output_capacity, Payload }` entries. Multiple task
macros can be used in the same module.

`define_master_task!` generates a different task: each accepted master message
produces at most one observation containing one complete message from each input.
The first macro argument names the generated Rust task type. Inputs must be
numbered consecutively from zero; `master` selects one of those input indices.

```rust
use cu29::prelude::*;
use cu_aligner::define_master_task;

// Substitute your Image, Motion and Pose payload types for these aliases.
type Image = u32;
type Motion = f32;
type Pose = i32;

define_master_task!(ObservationAligner,
    master: 0,
    0 => { 8, Image },   // pending camera capacity
    1 => { 64, Motion }, // retained motion capacity
    2 => { 32, Pose },   // retained pose capacity
);

// Output: CuMsg<(CuMsg<Image>, CuMsg<Motion>, CuMsg<Pose>)>
```

Configure the generated task in RON:

```ron
(
    id: "aligner",
    type: "tasks::ObservationAligner",
    config: {
        "selection": "nearest",
        "max_offset_ms": 20,
        "max_wait_ms": 10,
        "stale_data_horizon_ms": 500,
    },
)
```

The output connection's `msg` type is
`(cu29::prelude::CuMsg<Image>, cu29::prelude::CuMsg<Motion>, cu29::prelude::CuMsg<Pose>)`,
using your payloads' fully qualified paths. The runtime supplies the outer `CuMsg`.

| Setting | Behavior |
| --- | --- |
| `selection` | `nearest` (default) chooses the closest ToV, with ties choosing the earlier sample. `latest_before` chooses the latest sample at or before the master ToV. |
| `max_offset_ms` | Required, inclusive maximum absolute difference from the master ToV. |
| `max_wait_ms` | Required, wait after the master arrives at the aligner. `0` selects immediately. |
| `stale_data_horizon_ms` | Required retention relative to `ctx.now()`. Must cover `max_offset_ms + max_wait_ms`. |

Waiting never blocks `process`: selection happens on the first processing call
at or after the deadline, using samples buffered on that call. It does not promise
the globally nearest future sample. All ToVs must use the same clock domain.

For a camera at 100 ms, buffered motion samples at 91, 99 and 104 ms, and pose
samples at 95 and 103 ms, `nearest` selects camera 100, motion 99, pose 103.
`latest_before` selects camera 100, motion 99, pose 95.

The outer ToV is the master reference; inner messages retain their original ToVs
and all metadata. Consumers can compute offsets from those ToVs without a separate
`sample_time_offsets` payload. The outer ToV does not assert simultaneity.

Every input is required. At the deadline, a missing or out-of-tolerance match drops
the master permanently. A sensor message may serve multiple masters. At most one
observation is emitted per processing call, in master arrival order. Calls without
an observation clear the output payload and ToV.

Inputs without payloads are ignored. Payload-bearing messages must have point
ToVs and monotonically increasing timestamps within each stream. Equal timestamps
are treated as repeated inputs, including repeated masters; decreasing timestamps,
range ToVs and missing ToVs return an error before ingesting any input that cycle.

Buffers have fixed capacities. Overflow evicts the oldest message; master overflow
drops the oldest pending observation. Retention expiry can also discard a pending
master or its matching samples. Size capacities and retention for expected sensor
rates, source latency and the configured wait. Freeze/thaw preserves histories,
pending arrival times and duplicate detection for deterministic replay.
