use cu29::bincode::{Decode, Encode};
use cu29::prelude::*;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[derive(Default, Clone, Debug, Encode, Decode)]
struct TestPayload {
    number: u64,
    array: [u8; 10],
    float: f64,
}

fn global_system_fsync() {
    unsafe {
        libc::sync();
    }
}

struct ScopeTimer {
    start: CuDuration,
    msg: String,
    clk: RobotClock,
}

impl ScopeTimer {
    fn new(clk: &RobotClock, msg: &str) -> Self {
        Self {
            start: clk.now(),
            msg: msg.to_string(),
            clk: clk.clone(),
        }
    }
}

impl Drop for ScopeTimer {
    fn drop(&mut self) {
        println!("{}: {}", self.msg, self.clk.now() - self.start);
    }
}

const NB_MSG: usize = 20_000_000;

fn main() {
    let clk = RobotClock::new();
    {
        let _t = ScopeTimer::new(
            &clk,
            "(Initial fsync to get the pending IOs out of the way)",
        );
        global_system_fsync();
    }

    // Copper side.
    let file_path = "./ziooon.copper";
    let mut timings = CuDurationStatistics::new(CuDuration::from(Duration::from_millis(100)));
    let copper_allocations = {
        let _t = ScopeTimer::new(&clk, "Copper: End to End");
        let UnifiedLogger::Write(logger) = UnifiedLoggerBuilder::new()
            .write(true)
            .create(true)
            .file_base_name(Path::new(file_path))
            .preallocated_size(4 * 1024 * 1024 * 1024)
            .build()
            .expect("Failed to create logger")
        else {
            panic!("Failed to create logger")
        };
        let logger = Arc::new(Mutex::new(logger));

        let a = ScopedAllocCounter::new();
        let mut stream = stream_write(logger.clone(), UnifiedLogType::CopperList, 40 * 1024 * 1024);
        {
            for i in 0..NB_MSG {
                let bf = clk.now();
                let test_payload = TestPayload {
                    number: i as u64,
                    array: [(i % 255) as u8; 10],
                    float: i as f64,
                };
                let mut msg = CuMsg::<TestPayload>::new(Some(test_payload));
                msg.metadata.process_time = PartialCuTimeRange {
                    start: CuDuration(22).into(),
                    end: CuDuration(23).into(),
                };
                msg.metadata.tov = CuDuration(19).into();
                stream.log(&msg).unwrap();
                timings.record(clk.now() - bf);
            }
        }
        // here everything will be dropped and a final flush to disk will be done
        a.get_allocated()
    };
    println!("Copper: allocated: {} bytes", copper_allocations);
    println!("Copper: mean jitter: {}", timings.jitter_mean());
    println!("Copper: max jitter: {}", timings.jitter_max());
    {
        // To verify that really, everything was flushed to disk, it should be close to immediate
        let _t = ScopeTimer::new(&clk, "(fsync check after Copper)");
        global_system_fsync();
    }

    let mut timings = CuDurationStatistics::new(CuDuration::from(Duration::from_millis(100)));

    // mcap side
    let mcap_allocations = {
        let _t = ScopeTimer::new(&clk, "Mcap: End to End");
        use mcap::{records::MessageHeader, Channel, Writer};
        use std::{collections::BTreeMap, fs, io::BufWriter};

        let mut out = Writer::new(BufWriter::new(fs::File::create("out.mcap").unwrap())).unwrap();

        let my_channel = Channel {
            topic: String::from("cool stuff"),
            schema: None,
            message_encoding: String::from("application/octet-stream"),
            metadata: BTreeMap::default(),
        };

        let channel_id = out.add_channel(&my_channel).unwrap();
        let mut buffer = [0u8; 1024];

        let a = ScopedAllocCounter::new();
        for i in 0..NB_MSG {
            let bf = clk.now();
            let data = TestPayload {
                number: i as u64,
                array: [(i % 255) as u8; 10],
                float: i as f64,
            };
            // apple to apple with Copper
            let length =
                bincode::encode_into_slice(data, &mut buffer, bincode::config::standard()).unwrap();

            out.write_to_known_channel(
                &MessageHeader {
                    channel_id,
                    sequence: 32,
                    log_time: 23,
                    publish_time: 25,
                },
                &buffer[..length],
            )
            .unwrap();
            timings.record(clk.now() - bf);
        }
        out.finish().unwrap();
        a.get_allocated()
    };
    println!("mcap: allocated: {} bytes", mcap_allocations);
    println!("mcap: mean jitter: {}", timings.jitter_mean());
    println!("mcap: max jitter: {}", timings.jitter_max());
    {
        // To verify that really, everything was flushed to disk, it should be close to immediate
        let _t = ScopeTimer::new(&clk, "(fsync check after Mcap)");
        global_system_fsync();
    }
}
