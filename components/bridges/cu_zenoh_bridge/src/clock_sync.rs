//! Versioned epoch metadata following the stable provenance prefix.
use super::CopperBridgeAttachment;
use cu29::clock::sync::ClockDomain;
#[cfg(feature = "clock-sync")]
use cu29::clock::sync::SyncState;
use cu29::prelude::*;

const VERSION: u8 = 1;
#[derive(Clone, Copy, bincode::Encode, bincode::Decode)]
struct Reference {
    version: u8,
    domain: ClockDomain,
    sent_at: CuTime,
    error: CuDuration,
    drift_bound_ppb: u32,
}

#[derive(Default)]
pub(super) struct Attachment {
    pub(super) origin: Option<CuMsgOrigin>,
    reference: Option<Reference>,
}

pub(super) fn encode(origin: CopperBridgeAttachment, ctx: &CuContext) -> CuResult<Vec<u8>> {
    #[cfg(feature = "clock-sync")]
    if let Some(status) = ctx.clock.sync_status() {
        if !matches!(status.state, SyncState::Locked | SyncState::Holdover) {
            return Err(CuError::from("ZenohBridge: synchronized clock is unusable"));
        }
        // One allocation, as in the existing attachment path; no payload copy.
        return bincode::encode_to_vec(
            (
                origin,
                Reference {
                    version: VERSION,
                    domain: status.domain,
                    sent_at: ctx.now(),
                    error: status.estimated_error,
                    drift_bound_ppb: status.drift_bound_ppb,
                },
            ),
            bincode::config::standard(),
        )
        .map_err(|e| CuError::new_with_cause("ZenohBridge: attachment encode failed", e));
    }
    #[cfg(not(feature = "clock-sync"))]
    let _ = ctx;
    bincode::encode_to_vec(origin, bincode::config::standard())
        .map_err(|e| CuError::new_with_cause("ZenohBridge: attachment encode failed", e))
}

pub(super) fn decode(bytes: &[u8]) -> CuResult<Attachment> {
    let (origin, used): (CopperBridgeAttachment, usize) =
        bincode::decode_from_slice(bytes, bincode::config::standard())
            .map_err(|e| CuError::new_with_cause("ZenohBridge: attachment decode failed", e))?;
    let reference = if used == bytes.len() {
        None
    } else {
        let (reference, count): (Reference, usize) =
            bincode::decode_from_slice(&bytes[used..], bincode::config::standard()).map_err(
                |e| CuError::new_with_cause("ZenohBridge: clock attachment decode failed", e),
            )?;
        if reference.version != VERSION
            || used + count != bytes.len()
            || reference.drift_bound_ppb >= 1_000_000_000
        {
            return Err(CuError::from("ZenohBridge: unsupported clock attachment"));
        }
        Some(reference)
    };
    Ok(Attachment {
        origin: Some(CuMsgOrigin {
            subsystem_code: origin.subsystem_code,
            instance_id: origin.instance_id,
            cl_id: origin.cl_id,
        }),
        reference,
    })
}

pub(super) fn validate_tov(tov: &mut Tov, attachment: &Attachment, ctx: &CuContext) {
    #[cfg(feature = "clock-sync")]
    {
        let local = ctx.clock.sync_status();
        if attachment.reference.is_none() && local.is_none() {
            return;
        }
        let valid = match (attachment.reference, local) {
            (Some(sender), Some(local))
                if sender.domain == local.domain
                    && matches!(local.state, SyncState::Locked | SyncState::Holdover) =>
            {
                let now = ctx.now().0;
                let transport_error = (u128::from(now.saturating_sub(sender.sent_at.0))
                    * u128::from(sender.drift_bound_ppb))
                .div_ceil(1_000_000_000);
                let error = u128::from(sender.error.0)
                    + u128::from(local.estimated_error.0)
                    + transport_error;
                let latest = match tov {
                    Tov::None => None,
                    Tov::Time(time) => Some(time.0),
                    Tov::Range(range) if range.start <= range.end => Some(range.end.0),
                    Tov::Range(_) => Some(u64::MAX),
                };
                error <= u128::from(local.max_error.0)
                    && u128::from(sender.sent_at.0) <= u128::from(now) + error
                    && latest.is_none_or(|time| {
                        time < u64::MAX && u128::from(time) <= u128::from(now) + error
                    })
            }
            _ => false,
        };
        if !valid {
            debug!(
                "ZenohBridge: timestamp rejected by clock domain, quality or uncertainty checks"
            );
            *tov = Tov::None;
        }
    }
    #[cfg(not(feature = "clock-sync"))]
    {
        let _ = ctx;
        if attachment.reference.is_some() {
            debug!("ZenohBridge: shared timestamp requires clock-sync receiver support");
            *tov = Tov::None;
        }
    }
}

#[cfg(all(test, feature = "clock-sync"))]
mod tests {
    use super::*;
    use cu29::clock::sync::{ClockObservation, ClockSync, SyncConfig};
    const DOMAIN: ClockDomain = ClockDomain {
        id: 0,
        identity: *b"testroot",
        session: 1,
    };
    fn locked(domain: ClockDomain) -> (CuContext, RobotClockMock, ClockSync) {
        let (clock, mock) = RobotClock::mock();
        let mut sync =
            ClockSync::new(&clock, domain, SyncConfig::new(CuDuration(100_000))).unwrap();
        for raw in [0, 100_000] {
            mock.set_value(raw);
            sync.observe(ClockObservation {
                raw_local: CuInstant::from_nanos(raw),
                parent_ns: 1_000_000 + raw,
                uncertainty: CuDuration(100),
                domain,
            })
            .unwrap();
            sync.update().unwrap();
        }
        (CuContext::from_clock(clock), mock, sync)
    }
    #[test]
    fn range_and_origin_survive_a_matching_epoch_but_not_foreign_or_expired_time() {
        let (ctx, mock, _sync) = locked(DOMAIN);
        let prefix = CopperBridgeAttachment {
            subsystem_code: 2,
            instance_id: 7,
            cl_id: 9,
        };
        let bytes = encode(prefix, &ctx).unwrap();
        let attachment = decode(&bytes).unwrap();
        let range = Tov::Range(CuTimeRange {
            start: CuTime(1_099_000),
            end: CuTime(1_100_000),
        });
        let mut tov = range;
        validate_tov(&mut tov, &attachment, &ctx);
        assert_eq!(tov, range);
        assert_eq!(attachment.origin.as_ref().unwrap().cl_id, 9);
        let (foreign, _, _) = locked(ClockDomain {
            session: 2,
            ..DOMAIN
        });
        validate_tov(&mut tov, &attachment, &foreign);
        assert_eq!(tov, Tov::None);
        tov = range;
        mock.set_value(10_000_000_000);
        validate_tov(&mut tov, &attachment, &ctx);
        assert_eq!(tov, Tov::None);
        assert_eq!(attachment.origin.as_ref().unwrap().instance_id, 7);
        assert!(decode(&bytes[..bytes.len() - 1]).is_err());
        assert_eq!(
            bincode::decode_from_slice::<CopperBridgeAttachment, _>(
                &bytes,
                bincode::config::standard()
            )
            .unwrap()
            .0,
            prefix
        );
    }
}
