use cu29_unifiedlog::{LogStream, UnifiedLogger, UnifiedLoggerBuilder};
use cu29_traits::{WriteStream, UnifiedLogType};
use std::{path::Path, sync::{Arc,Mutex}};
fn main() {
    eprintln!("create");
    let UnifiedLogger::Write(writer) = UnifiedLoggerBuilder::new()
        .file_base_name(Path::new("probe.copper")).write(true).create(true)
        .preallocated_size(8192).build().unwrap() else { unreachable!() };
    let writer = Arc::new(Mutex::new(writer));
    eprintln!("open A0");
    let mut a = LogStream::new(UnifiedLogType::CopperList,writer.clone(),4096).unwrap();
    eprintln!("open B");
    let mut b = LogStream::new(UnifiedLogType::StructuredLogLine,writer.clone(),1536).unwrap();
    eprintln!("append A0 and B");
    let sample = [17u8; 2048];
    a.log(&sample).unwrap();
    b.log(&[3u8; 64]).unwrap();
    std::thread::sleep(std::time::Duration::from_millis(125));
    eprintln!("roll A0 to A1");
    a.log(&sample).unwrap();
    eprintln!("roll A1 to A2");
    a.log(&sample).unwrap();
    eprintln!("drop B");
    drop(b);
    eprintln!("drop A2");
    drop(a);
    eprintln!("drop logger");
    drop(writer);
}
