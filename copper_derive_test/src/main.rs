use copper_derive::CopperRuntime;

#[derive(CopperRuntime)]
#[config(file = "copperconfig.ron")]
struct MyRuntime{}

fn main() {
    println!("Hello, world!");
}
