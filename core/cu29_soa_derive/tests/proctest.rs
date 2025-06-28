#[cfg(test)]
mod tests {
    use cu29::bincode::{Decode, Encode};
    use cu29_schema::{Schema, SchemaType};
    use cu29_soa_derive::Soa;
    #[derive(Debug, Clone, Default, PartialEq, Soa, Encode, Decode)]
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

    #[derive(Debug, Clone, Default, PartialEq, Soa, Encode, Decode)]
    pub struct Color {
        r: f32,
        g: f32,
        b: f32,
    }

    #[derive(Debug, Default, PartialEq, Soa)]
    pub struct Both {
        xyz: Xyz,
        color: Color,
    }

    #[test]
    fn test_soa_schema_implementation() {
        // Test that the SoA type implements Schema
        let schema = XyzSoa::<10>::schema();
        println!("Schema: {:?}", schema);

        // Check that the schema contains the expected fields
        assert!(schema.contains_key("x"));
        assert!(schema.contains_key("y"));
        assert!(schema.contains_key("z"));
        assert!(schema.contains_key("i"));
        assert!(schema.contains_key("len"));

        // Check that the fields are Array types
        if let Some(SchemaType::Array { element_type, size }) = schema.get("x") {
            assert_eq!(*size, 0); // Placeholder for const generic N
            assert_eq!(**element_type, SchemaType::Custom("f32".to_string()));
        } else {
            panic!("Expected x field to be an Array type");
        }

        if let Some(SchemaType::Array { element_type, size }) = schema.get("i") {
            assert_eq!(*size, 0); // Placeholder for const generic N
            assert_eq!(**element_type, SchemaType::Custom("i32".to_string()));
        } else {
            panic!("Expected i field to be an Array type");
        }

        // Test type name
        let type_name = XyzSoa::<10>::type_name();
        assert_eq!(type_name, "XyzSoa<N>");

        // Test schema_type
        let schema_type = XyzSoa::<10>::schema_type();
        if let SchemaType::Struct { name, fields } = schema_type {
            assert_eq!(name, "XyzSoa<N>");
            assert_eq!(fields.len(), 5); // x, y, z, i, len
        } else {
            panic!("Expected schema_type to be a Struct");
        }
    }
}
