use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use crsf::{LinkStatistics, RcChannels};
use cu29::bevy_reflect;
use cu29::reflect::Reflect;
use serde::{Deserialize, Serialize};

/// Copper-friendly wrapper for CRSF RC channel data.
#[derive(Clone, Debug, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct RcChannelsPayload(pub RcChannels);

impl RcChannelsPayload {
    /// Returns immutable access to the inner CRSF structure.
    pub fn inner(&self) -> &RcChannels {
        &self.0
    }

    /// Returns mutable access to the inner CRSF structure.
    pub fn inner_mut(&mut self) -> &mut RcChannels {
        &mut self.0
    }
}

impl From<RcChannels> for RcChannelsPayload {
    fn from(value: RcChannels) -> Self {
        Self(value)
    }
}

impl From<RcChannelsPayload> for RcChannels {
    fn from(value: RcChannelsPayload) -> Self {
        value.0
    }
}

impl Default for RcChannelsPayload {
    fn default() -> Self {
        Self(RcChannels([0; 16]))
    }
}

impl PartialEq for RcChannelsPayload {
    fn eq(&self, other: &Self) -> bool {
        self.0.0 == other.0.0
    }
}

impl Eq for RcChannelsPayload {}

impl Encode for RcChannelsPayload {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        for value in self.0.0.iter() {
            value.encode(encoder)?;
        }
        Ok(())
    }
}

impl Decode<()> for RcChannelsPayload {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        let mut channels = [0u16; 16];
        for slot in channels.iter_mut() {
            *slot = u16::decode(decoder)?;
        }
        Ok(Self(RcChannels(channels)))
    }
}

impl Serialize for RcChannelsPayload {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.inner().0.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for RcChannelsPayload {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let channels = <[u16; 16]>::deserialize(deserializer)?;
        Ok(Self(RcChannels(channels)))
    }
}

/// Copper-friendly wrapper for CRSF link statistics packets.
#[derive(Clone, Debug, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct LinkStatisticsPayload(pub LinkStatistics);

impl LinkStatisticsPayload {
    pub fn inner(&self) -> &LinkStatistics {
        &self.0
    }
}

impl From<LinkStatistics> for LinkStatisticsPayload {
    fn from(value: LinkStatistics) -> Self {
        Self(value)
    }
}

impl From<LinkStatisticsPayload> for LinkStatistics {
    fn from(value: LinkStatisticsPayload) -> Self {
        value.0
    }
}

impl PartialEq for LinkStatisticsPayload {
    fn eq(&self, other: &Self) -> bool {
        let LinkStatisticsPayload(link_a) = self;
        let LinkStatisticsPayload(link_b) = other;
        link_a.uplink_rssi_1 == link_b.uplink_rssi_1
            && link_a.uplink_rssi_2 == link_b.uplink_rssi_2
            && link_a.uplink_link_quality == link_b.uplink_link_quality
            && link_a.uplink_snr == link_b.uplink_snr
            && link_a.active_antenna == link_b.active_antenna
            && link_a.rf_mode == link_b.rf_mode
            && link_a.uplink_tx_power == link_b.uplink_tx_power
            && link_a.downlink_rssi == link_b.downlink_rssi
            && link_a.downlink_link_quality == link_b.downlink_link_quality
            && link_a.downlink_snr == link_b.downlink_snr
    }
}

impl Eq for LinkStatisticsPayload {}

impl Default for LinkStatisticsPayload {
    fn default() -> Self {
        Self(LinkStatistics {
            uplink_rssi_1: 0,
            uplink_rssi_2: 0,
            uplink_link_quality: 0,
            uplink_snr: 0,
            active_antenna: 0,
            rf_mode: 0,
            uplink_tx_power: 0,
            downlink_rssi: 0,
            downlink_link_quality: 0,
            downlink_snr: 0,
        })
    }
}
impl Encode for LinkStatisticsPayload {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        let stats = &self.0;
        stats.uplink_rssi_1.encode(encoder)?;
        stats.uplink_rssi_2.encode(encoder)?;
        stats.uplink_link_quality.encode(encoder)?;
        stats.uplink_snr.encode(encoder)?;
        stats.active_antenna.encode(encoder)?;
        stats.rf_mode.encode(encoder)?;
        stats.uplink_tx_power.encode(encoder)?;
        stats.downlink_rssi.encode(encoder)?;
        stats.downlink_link_quality.encode(encoder)?;
        stats.downlink_snr.encode(encoder)?;
        Ok(())
    }
}

impl Decode<()> for LinkStatisticsPayload {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self(LinkStatistics {
            uplink_rssi_1: u8::decode(decoder)?,
            uplink_rssi_2: u8::decode(decoder)?,
            uplink_link_quality: u8::decode(decoder)?,
            uplink_snr: i8::decode(decoder)?,
            active_antenna: u8::decode(decoder)?,
            rf_mode: u8::decode(decoder)?,
            uplink_tx_power: u8::decode(decoder)?,
            downlink_rssi: u8::decode(decoder)?,
            downlink_link_quality: u8::decode(decoder)?,
            downlink_snr: i8::decode(decoder)?,
        }))
    }
}

impl Serialize for LinkStatisticsPayload {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let stats = &self.0;
        (
            stats.uplink_rssi_1,
            stats.uplink_rssi_2,
            stats.uplink_link_quality,
            stats.uplink_snr,
            stats.active_antenna,
            stats.rf_mode,
            stats.uplink_tx_power,
            stats.downlink_rssi,
            stats.downlink_link_quality,
            stats.downlink_snr,
        )
            .serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for LinkStatisticsPayload {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let (
            uplink_rssi_1,
            uplink_rssi_2,
            uplink_link_quality,
            uplink_snr,
            active_antenna,
            rf_mode,
            uplink_tx_power,
            downlink_rssi,
            downlink_link_quality,
            downlink_snr,
        ) = <(u8, u8, u8, i8, u8, u8, u8, u8, u8, i8)>::deserialize(deserializer)?;
        Ok(Self(LinkStatistics {
            uplink_rssi_1,
            uplink_rssi_2,
            uplink_link_quality,
            uplink_snr,
            active_antenna,
            rf_mode,
            uplink_tx_power,
            downlink_rssi,
            downlink_link_quality,
            downlink_snr,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::{config, decode_from_slice, encode_to_vec};

    #[test]
    fn rc_channels_round_trip() {
        let mut channels = [0u16; 16];
        for (idx, value) in channels.iter_mut().enumerate() {
            *value = (idx as u16) * 42;
        }
        let payload = RcChannelsPayload(RcChannels(channels));
        let cfg = config::standard();

        let encoded = encode_to_vec(&payload, cfg).expect("RC encode");
        let (decoded, consumed): (RcChannelsPayload, usize) =
            decode_from_slice(&encoded, cfg).expect("RC decode");

        assert_eq!(consumed, encoded.len());
        assert_eq!(payload, decoded);
        assert_eq!(decoded.0.0, channels);
    }

    #[test]
    fn link_statistics_round_trip() {
        let stats = LinkStatistics {
            uplink_rssi_1: 100,
            uplink_rssi_2: 98,
            uplink_link_quality: 95,
            uplink_snr: -12,
            active_antenna: 1,
            rf_mode: 2,
            uplink_tx_power: 5,
            downlink_rssi: 110,
            downlink_link_quality: 90,
            downlink_snr: -9,
        };
        let payload = LinkStatisticsPayload(stats);
        let cfg = config::standard();

        let encoded = encode_to_vec(&payload, cfg).expect("Link encode");
        let (decoded, consumed): (LinkStatisticsPayload, usize) =
            decode_from_slice(&encoded, cfg).expect("Link decode");

        assert_eq!(consumed, encoded.len());
        assert_eq!(payload, decoded);
    }
}
