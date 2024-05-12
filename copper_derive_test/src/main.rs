use copper_derive::CopperRuntime;

#[CopperRuntime(config = "copperconfig.ron")]
struct MyRuntime {}

fn main() {
    println!("Hello, world!");
    let runtime = MyRuntime{node_instances: (1,2)};
    runtime.hello();
}
