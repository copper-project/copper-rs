use cu29::prelude::CuContext;
use zenoh::{bytes::ZBytes, config::ZenohId};
use zenoh_ext::ZSerializer;

pub fn encode_attachment(sequence_number: u64, ctx: &CuContext, zid: &ZenohId) -> ZBytes {
    let mut serializer = ZSerializer::new();
    serializer.serialize(sequence_number);
    serializer.serialize(ctx.now().as_nanos());
    serializer.serialize(zid.to_le_bytes());
    serializer.finish()
}
