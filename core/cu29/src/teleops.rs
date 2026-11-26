//! Operator channel declarations and the source task used by generated aliases.
//!
//! Enable `cu29/teleops` in both applications. Select `cu29/logstream` for a
//! host telemetry worker or `cu29/telemetry` for threadless telemetry service.
//! A custom carrier can use the typed inboxes directly.
//!
//! Use an authenticated or trusted return carrier and matching channel schemas.
//! A successful submission only means that the local carrier accepted the packet.
use crate::prelude::*;
pub use cu29_logstream::teleops::*;

crate::resources!(for <T> where T: Send + Sync + 'static {
    inbox => Owned<Inbox<T>>,
});

/// Drains at most one admitted sample in each graph iteration.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct Source<T: Send + Sync + 'static> {
    max_lag_cl: u64,
    #[reflect(ignore)]
    inbox: Inbox<T>,
}
impl<T: Send + Sync + 'static> Freezable for Source<T> {}
impl<T: CuMsgPayload + Send + Sync + 'static> CuSrcTask for Source<T> {
    type Output<'m> = CuMsg<T>;
    type Resources<'r> = Resources<T>;
    fn new(config: Option<&ComponentConfig>, resources: Resources<T>) -> CuResult<Self> {
        Ok(Self {
            max_lag_cl: config
                .map(|c| c.get::<u64>("max_lag_cl"))
                .transpose()?
                .flatten()
                .unwrap_or(1),
            inbox: resources.inbox.0,
        })
    }
    fn process<'o>(&mut self, ctx: &CuContext, out: &mut Self::Output<'o>) -> CuResult<()> {
        if let Some((id, tov, value)) = self.inbox.take(ctx.cl_id(), self.max_lag_cl) {
            self.inbox.record_emission(id, ctx.cl_id());
            out.set_payload(value);
            out.tov = tov;
        } else {
            out.clear_payload();
            out.tov = Tov::None;
        }
        Ok(())
    }
}
