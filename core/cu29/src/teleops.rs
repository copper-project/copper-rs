//! Operator channel declarations and the source task used by generated aliases.
use crate::prelude::*;
pub use cu29_logstream::teleops::*;

crate::resources!(for <T> where T: Send + Sync + 'static {
    inbox => Owned<Inbox<T>>,
});

/// Drains at most one admitted sample in each graph iteration.
#[derive(Reflect)]
#[reflect(no_field_bounds, from_reflect = false)]
pub struct Source<T: Send + Sync + 'static> {
    #[reflect(ignore)]
    inbox: Inbox<T>,
}
impl<T: Send + Sync + 'static> Freezable for Source<T> {}
impl<T: CuMsgPayload + Send + Sync + 'static> CuSrcTask for Source<T> {
    type Output<'m> = CuMsg<T>;
    type Resources<'r> = Resources<T>;
    fn new(_: Option<&ComponentConfig>, resources: Resources<T>) -> CuResult<Self> {
        Ok(Self {
            inbox: resources.inbox.0,
        })
    }
    fn process<'o>(&mut self, _: &CuContext, out: &mut Self::Output<'o>) -> CuResult<()> {
        if let Some((_, tov, value)) = self.inbox.take() {
            out.set_payload(value);
            out.tov = tov;
        } else {
            out.clear_payload();
            out.tov = Tov::None;
        }
        Ok(())
    }
}
