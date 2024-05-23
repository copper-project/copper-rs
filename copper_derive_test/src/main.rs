use copper_derive::copper_runtime;
use copper_log::debug;

#[copper_runtime(config = "copperconfig.ron")]
struct MyApplication {}

fn main() {
    debug!("Application created.");
    let application = MyApplication::new().expect("Failed to create runtime.");
    debug!("End of program.");
}
