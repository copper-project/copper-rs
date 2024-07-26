
#[cfg(test)]
mod tests {
    use cu29_soa::soa;

    #[derive(Debug, Default, PartialEq)]
    #[soa]
    pub struct Xyz {
        x: f32,
        y: f32,
        z: f32,
    }


    /// This is to check if with cargo install cargo-show-asm
    #[test]
    pub fn test_vectorization() {
        println!("Testing vectorization");
        // make a randomly generated Xyz
        let x = rand::random::<f32>();
        let y = rand::random::<f32>();
        let z = rand::random::<f32>();
        let xyz = Xyz { x, y, z };

        let xyzsoa: XyzSoa<8> = XyzSoa::new(xyz);
        assert_eq!(xyzsoa.x(), [x; 8]);

        let xs = xyzsoa.x();
        let ys = xyzsoa.y();
        let zs = xyzsoa.z();

        // add them all
        let sum_v = xs.iter().zip(ys.iter()).zip(zs.iter()).map(|((x, y), z)| x + y + z).collect::<Vec<f32>>();
        let sum =x +y+z;

        assert_eq!(sum_v[..], [sum; 8]);
    }

    #[test]
    #[should_panic]
    fn test_oob() {
        let xyzsoa: XyzSoa<8> = XyzSoa::new(Xyz::default());
        xyzsoa.get(8);
    }
}

