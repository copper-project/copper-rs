#[cfg(test)]
mod tests {
    use bincode::{Decode, Encode};
    use cu29_soa_derive::Soa;
    use serde_derive::Serialize;

    #[derive(Debug, Clone, Default, PartialEq, Soa, Encode, Decode, Serialize)]
    pub struct Xyz {
        x: f32,
        y: f32,
        z: f32,
        i: i32,
    }

    /// This is to check if with cargo install cargo-show-asm
    #[test]
    pub fn test_vectorization() {
        // make a randomly generated Xyz
        let x = rand::random::<f32>();
        let y = rand::random::<f32>();
        let z = rand::random::<f32>();
        let xyz = Xyz { x, y, z, i: 0 };

        let xyzsoa: XyzSoa<8> = XyzSoa::new(xyz);
        assert_eq!(xyzsoa.x(), &[x; 8]);

        let xs = xyzsoa.x();
        let ys = xyzsoa.y();
        let zs = xyzsoa.z();

        // add them all
        let sum_v = xs
            .iter()
            .zip(ys.iter())
            .zip(zs.iter())
            .map(|((x, y), z)| x + y + z)
            .collect::<Vec<f32>>();
        let sum = x + y + z;

        assert_eq!(sum_v[..], [sum; 8]);
    }

    #[test]
    #[should_panic]
    fn test_oob() {
        let xyzsoa: XyzSoa<8> = XyzSoa::new(Xyz::default());
        xyzsoa.get(8);
    }

    #[test]
    fn test_apply_with_simple_distance_to_origin() {
        let mut soa = XyzSoa::<3>::new(Xyz {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            i: 0,
        });

        // Set some example values
        soa.push(Xyz {
            x: 1.0,
            y: 2.0,
            z: 2.0,
            i: 0,
        });
        soa.push(Xyz {
            x: 4.0,
            y: 6.0,
            z: 3.0,
            i: 0,
        });
        soa.push(Xyz {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            i: 0,
        });

        // Create a result array to store the distances
        let mut distances = [0.0; 3];
        let mut i = 0;

        // Apply the operation to compute the distance to the origin
        soa.apply(|x, y, z, intensity| {
            let distance = (x * x + y * y + z * z).sqrt();
            distances[i] = distance;
            i += 1;
            (x, y, z, intensity)
        });

        // Check the results
        assert_eq!(
            distances[0],
            (1.0_f32.powi(2) + 2.0_f32.powi(2) + 2.0_f32.powi(2)).sqrt()
        );
        assert_eq!(
            distances[1],
            (4.0_f32.powi(2) + 6.0_f32.powi(2) + 3.0_f32.powi(2)).sqrt()
        );
        assert_eq!(distances[2], 0.0);
    }

    #[derive(Debug, Clone, Default, PartialEq, Soa, Encode, Decode, Serialize)]
    pub struct Color {
        r: f32,
        g: f32,
        b: f32,
    }

    #[derive(Debug, Clone, Default, PartialEq, Soa, Serialize)]
    pub struct TestStruct {
        x: f32,
        y: f32,
        z: i32,
    }

    #[derive(Debug, Default, PartialEq, Soa, Serialize)]
    pub struct Both {
        xyz: Xyz,
        color: Color,
    }

    #[test]
    fn test_serialization() {
        // Test serialization of XyzSoa
        let mut xyz_soa = XyzSoa::<3>::new(Xyz {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            i: 4,
        });

        xyz_soa.push(Xyz {
            x: 5.0,
            y: 6.0,
            z: 7.0,
            i: 8,
        });

        // Test that we can serialize to JSON using serde_json
        let json_result = serde_json::to_string(&xyz_soa);
        assert!(json_result.is_ok(), "Failed to serialize XyzSoa: {:?}", json_result.err());

        let json = json_result.unwrap();
        println!("Serialized XyzSoa: {}", json);

        // Verify the JSON contains expected fields
        assert!(json.contains("\"len\":"));
        assert!(json.contains("\"x\":"));
        assert!(json.contains("\"y\":"));
        assert!(json.contains("\"z\":"));
        assert!(json.contains("\"i\":"));

        // Test serialization of ColorSoa
        let mut color_soa = ColorSoa::<2>::new(Color {
            r: 0.5,
            g: 0.6,
            b: 0.7,
        });

        color_soa.push(Color {
            r: 0.8,
            g: 0.9,
            b: 1.0,
        });

        let color_json_result = serde_json::to_string(&color_soa);
        assert!(color_json_result.is_ok(), "Failed to serialize ColorSoa: {:?}", color_json_result.err());

        let color_json = color_json_result.unwrap();
        println!("Serialized ColorSoa: {}", color_json);

        // Verify the JSON contains expected fields
        assert!(color_json.contains("\"len\":"));
        assert!(color_json.contains("\"r\":"));
        assert!(color_json.contains("\"g\":"));
        assert!(color_json.contains("\"b\":"));

        // Test serialization of TestStructSoa
        let mut test_soa = TestStructSoa::<5>::new(TestStruct {
            x: 1.0,
            y: 2.0,
            z: 3,
        });

        test_soa.push(TestStruct {
            x: 4.0,
            y: 5.0,
            z: 6,
        });
        test_soa.push(TestStruct {
            x: 7.0,
            y: 8.0,
            z: 9,
        });

        let test_json_result = serde_json::to_string(&test_soa);
        assert!(test_json_result.is_ok(), "Failed to serialize TestStructSoa: {:?}", test_json_result.err());

        let test_json = test_json_result.unwrap();
        println!("Serialized TestStructSoa: {}", test_json);

        // Verify the JSON contains expected fields
        assert!(test_json.contains("\"len\":"));
        assert!(test_json.contains("\"x\":"));
        assert!(test_json.contains("\"y\":"));
        assert!(test_json.contains("\"z\":"));
    }
}
