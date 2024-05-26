use copper_log::debug;
use copper_log_runtime::LoggerRuntime;
use copper_value::to_value;
use serde::Serialize;

fn main() {
    let rt = LoggerRuntime {};
    #[derive(Serialize)]
    struct Test {
        a: i32,
        b: i32,
    }
    let mytuple = (1, "toto", 3.34f64, true, 'a');
    {
        let hop = copper::monitoring::ScopedAllocCounter::new();
        let gigantic_vec = vec![0u8; 1_000_000];
        debug!("Just a string");
        debug!("anonymous param constants {} {}", 3u16, 2u8);
        debug!("named param constants {} {}", a = 3, b = 2);
        debug!("mixed named param constants, {} {} {}", a = 3, 54, b = 2);
        debug!("complex tuple", mytuple);
        debug!("Struct", Test { a: 3, b: 4 });
        debug!("u8", gigantic_vec[999999]);
    }
    debug!(" AFTER CLOSE {} ", "AFTER CLOSE");
    rt.close();
}
