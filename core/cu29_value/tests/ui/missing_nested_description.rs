use bincode::Encode;
struct NativeOnly;
impl Encode for NativeOnly {
    fn encode<E: bincode::enc::Encoder>(&self, _: &mut E) -> Result<(), bincode::error::EncodeError> { Ok(()) }
}
#[derive(Encode)] //~ ERROR: /NativeOnly: ValueDecode/
struct Nested { unsupported: NativeOnly }
#[derive(Encode)]
struct Payload { nested: Nested }
fn main() {}
