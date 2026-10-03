use bincode::Encode;
#[derive(Encode)]
struct Payload { compact: bincode::Uleb128<u32> }
fn main() {}
