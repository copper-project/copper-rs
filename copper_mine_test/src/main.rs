use copper_mine::CopperRuntime;

#[derive(CopperRuntime)]
#[config(file = "test.ron")]
struct MyRuntime{}

fn main() {
    println!("Hello, world!");
}
