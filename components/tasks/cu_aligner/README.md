## Aligner to buffer and align data flow from different time horizons

The goal of this Copper component is to align data from different sources.

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

The timings are taken from the CuMsg::metadata.tov field (time of validity).

### Usage

The task is generated entirely out of the `cu_aligner::define_task` macro:

```rust,ignore

use cu_aligner::define_task;
use cu29::input_msg;
use cu29::output_msg;
use cu29::cutask::Freezable;
use cu29::config::ComponentConfig;
use cu29::CuResult;
use cu29::cutask::CuTask;
use cu29::cutask::CuMsg;

// Defines a task that aligns two streams of messages, one with payloads f32, the other MyPayload (you can use any rust struct that to implement the traits for CuMsgPayload).
define_task!(MyAlignerTask, 
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
example: `CuMsg<(CuArray<f32, 7>, CuArray<MyPayload, 5>)>`.

### Performance consideration

Copper by itself never buffers anything to avoid copies but for this aligner has to copy data until it can align it.
It means you will have 1 copy from the input to the internal buffer and 1 copy from the internal buffer to the output.

If your usecase is just to get the latest message from 2 sources, just connect your task to the upstream tasks
but do not use this aligner. It is only useful if the time of arrival from the 2 upstream tasks are too far apart
in terms of tov.
