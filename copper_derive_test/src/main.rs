use copper_derive::copper_runtime;

#[copper_runtime(config = "copperconfig.ron")]
struct MyRuntime {}

fn main() {
    let runtime = MyRuntime::new().expect("Failed to create runtime.");
    runtime.hello();
}
