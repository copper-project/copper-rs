//! The single message type of the reference system.

use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize};

/// Message size used by every node of the Autoware reference system.
pub const DATA_LEN: usize = 4096;

/// The 4kB block. Serde has no derive support for arrays this large, so it rides as bytes.
#[derive(Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
#[reflect(opaque)]
pub struct DataBlock(#[serde(with = "serde_bytes")] pub [u8; DATA_LEN]);

impl Default for DataBlock {
    fn default() -> Self {
        Self([0u8; DATA_LEN])
    }
}

#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct RefSample {
    pub data: DataBlock,
    /// Sequence number stamped by the originating sensor. Its sample time is the
    /// envelope `tov`, which every node propagates unchanged.
    pub seq: u64,
}

/// Maximum callbacks executed by one statically fused background region.
pub const REGION_CALLBACK_CAPACITY: usize = 9;

/// One completed independent region. Causal callbacks execute inline on the same worker;
/// their individual durations remain available for benchmark validation.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct RefRegionSample {
    pub data: DataBlock,
    pub seq: u64,
    pub endpoint_ns: u64,
    pub callback_ns: [u64; REGION_CALLBACK_CAPACITY],
}

/// The same block under a second name.
///
/// Copper derives a task's output ports from the *distinct* `msg:` types of its outgoing
/// connections, so a node cannot drive two ports with one payload type. The intersection
/// node needs two, so its second lane gets its own type.
#[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
pub struct RefLaneSample(pub RefSample);

impl From<RefSample> for RefLaneSample {
    fn from(sample: RefSample) -> Self {
        Self(sample)
    }
}

impl AsRef<RefSample> for RefSample {
    fn as_ref(&self) -> &RefSample {
        self
    }
}

impl AsRef<RefSample> for RefLaneSample {
    fn as_ref(&self) -> &RefSample {
        &self.0
    }
}
