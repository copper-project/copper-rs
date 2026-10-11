use bincode::Encode;
#[derive(Encode)] //~ ERROR: /Uleb128<u32>: ValueDecode/
struct Payload { compact: bincode::Uleb128<u32> }
fn main() {}
