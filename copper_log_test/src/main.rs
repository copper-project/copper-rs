use copper_log::debug;
use copper_value::to_value;
use serde::Serialize;

fn main() {
    #[derive(Serialize)]
    struct Test {
        a: i32,
        b: i32,
    }
    let t = Test { a: 2, b: 3 };

    let v = to_value(t).unwrap();

    println!("{:?}", v);

    let mytuple = (1, "toto", 3.34f64, true, 'a');
    let v = to_value(mytuple).unwrap();
    println!("{:?}", v);
    debug!("Just a string");
    debug!("anonymous param constants {} {}", 3u16, 2u8);
    debug!("named param constants {} {}", a = 3, b = 2);
    debug!("mixed named param constants, {} {} {}", a = 3, 54, b = 2);
    debug!("a tuple {}", mytuple);
}
