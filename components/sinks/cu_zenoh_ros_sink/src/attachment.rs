use cu29::clock::RobotClock;
use zenoh::{bytes::ZBytes, config::ZenohId};
use zenoh_ext::ZSerializer;

pub fn encode_attachment(sequence_number: u64, clock: &RobotClock, zid: &ZenohId) -> ZBytes {
    let mut serializer = ZSerializer::new();
    serializer.serialize("sequence_number".to_string());
    serializer.serialize(sequence_number);
    serializer.serialize("source_timestamp".to_string());
    serializer.serialize(clock.now().as_nanos());
    serializer.serialize("source_gid");
    serializer.serialize(zid.to_le_bytes());
    serializer.finish()
}
