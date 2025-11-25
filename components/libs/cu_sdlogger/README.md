# cu-sdlogger

Utilities for writing Copper unified logs to an SD/eMMC partition from embedded targets. It builds on `embedded_sdmmc::BlockDevice` and speaks the Cu29 on-disk format used by the runtime log reader.

## What it provides
- `find_copper_partition`: scan the GPT and return the start/length of the Cu29 partition.
- `EMMCLogger`: `UnifiedLogWrite<EMMCSectionStorage<_>>` implementation that lays out the main header and log sections on a block device.
- `EMMCSectionStorage`: section writer that encodes entries with bincode, plus `ForceSyncSend` to share block devices that are not `Sync`.

## Usage
```rust
use cu_sdlogger::{find_copper_partition, EMMCLogger, EMMCSectionStorage, ForceSyncSend};
use cu29::prelude::*;
use embedded_sdmmc::SdCard;

// 1) Wrap your block device if it is not Sync.
let sd = ForceSyncSend::new(SdCard::new(device, timer));

// 2) Locate the Copper partition (GUID 29A2E0C9-0000-4C75-9229-000000000029).
let Some((start, len)) = find_copper_partition(&sd)? else {
    panic!("Copper partition missing on card");
};

// 3) Create the logger and allocate a section for your stream.
let mut logger: EMMCLogger<_> = EMMCLogger::new(sd, start, len)?;
let mut telem = logger.add_section(UnifiedLogType::Telemetry, 32 * 1024)?; // bytes of payload

// 4) Append log entries and flush before shutdown.
telem.append(&my_entry)?;
logger.flush_section(&mut telem);
```

Notes:
- Blocks are assumed to be 512 bytes; the logger errors if it would run out of space.
- `append` and `flush_section` use bincode encoding and update the section header to keep usage counters accurate.
- See `examples/cu_rp2350_skeleton` and `examples/cu_elrs_bdshot_demo` for full RP2350 setups that hand the logger to a `CuApplication`.
