#[cfg(test)]
mod tests {
    use bincode::{Decode, Encode, config::standard, decode_from_slice, encode_to_vec};
    use cu29_soa_derive::Soa;
    use serde_derive::{Deserialize, Serialize};

    #[derive(Debug, Clone, Default, PartialEq, Soa, Encode, Decode, Serialize, Deserialize)]
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

    #[derive(Debug, Clone, Default, PartialEq, Soa, Encode, Decode, Serialize, Deserialize)]
    pub struct Color {
        r: f32,
        g: f32,
        b: f32,
    }

    #[derive(Debug, Default, PartialEq, Soa, Serialize, Deserialize)]
    pub struct Both {
        #[soa(nested)]
        xyz: Xyz,
        #[soa(nested)]
        color: Color,
    }

    fn sample_point_cloud<const N: usize>() -> BothSoa<N> {
        let mut point_cloud = BothSoa::<N>::default();

        for i in 0..2 {
            let i_f = i as f32;
            point_cloud.push(Both {
                xyz: Xyz {
                    x: i_f + 1.0,
                    y: i_f + 2.0,
                    z: i_f + 3.0,
                    i,
                },
                color: Color {
                    r: i_f * 0.1,
                    g: 0.5,
                    b: 1.0 - i_f * 0.1,
                },
            });
        }

        point_cloud
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
        assert!(
            json_result.is_ok(),
            "Failed to serialize XyzSoa: {:?}",
            json_result.err()
        );

        let json = json_result.unwrap();
        println!("Serialized XyzSoa: {json}");

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
        assert!(
            color_json_result.is_ok(),
            "Failed to serialize ColorSoa: {:?}",
            color_json_result.err()
        );

        let color_json = color_json_result.unwrap();
        println!("Serialized ColorSoa: {color_json}");

        // Verify the JSON contains expected fields
        assert!(color_json.contains("\"len\":"));
        assert!(color_json.contains("\"r\":"));
        assert!(color_json.contains("\"g\":"));
        assert!(color_json.contains("\"b\":"));
    }

    #[test]
    fn test_both_soa_serde_roundtrip() {
        let point_cloud = sample_point_cloud::<8>();

        let json = serde_json::to_string(&point_cloud).expect("serialize BothSoa");
        let decoded: BothSoa<8> = serde_json::from_str(&json).expect("deserialize BothSoa");

        let len = point_cloud.len();
        assert_eq!(decoded.len(), len);
        for idx in 0..len {
            assert_eq!(decoded.get(idx), point_cloud.get(idx));
        }
    }

    #[test]
    fn test_both_soa_rejects_mismatched_nested_len() {
        let json = r#"{
            "len": 2,
            "xyz": {
                "x": [1.0],
                "y": [2.0],
                "z": [3.0],
                "i": [1]
            },
            "color": {
                "r": [0.1, 0.2],
                "g": [0.3, 0.4],
                "b": [0.5, 0.6]
            }
        }"#;

        let result: Result<BothSoa<4>, _> = serde_json::from_str(json);
        assert!(result.is_err());
    }

    #[test]
    fn test_both_soa_bincode_roundtrip() {
        let point_cloud = sample_point_cloud::<8>();

        let encoded = encode_to_vec(&point_cloud, standard()).expect("encode BothSoa");
        let (decoded, _): (BothSoa<8>, _) =
            decode_from_slice(&encoded, standard()).expect("decode BothSoa");

        let len = point_cloud.len();
        assert_eq!(decoded.len(), len);
        for idx in 0..len {
            assert_eq!(decoded.get(idx), point_cloud.get(idx));
        }
    }

    #[test]
    fn test_both_soa() {
        let mut point_cloud = BothSoa::<1000>::default();

        // Add sample 3D points with colors
        for i in 0..100 {
            let angle = i as f32 * 0.1;
            point_cloud.push(Both {
                xyz: Xyz {
                    x: angle.cos() * 10.0,
                    y: angle.sin() * 10.0,
                    z: i as f32 * 0.1,
                    i,
                },
                color: Color {
                    r: (i as f32 / 100.0),
                    g: 0.5,
                    b: 1.0 - (i as f32 / 100.0),
                },
            });
        }

        assert_eq!(point_cloud.len(), 100);

        let len = point_cloud.len();

        // Spatial-only operations without loading color data
        // Calculate distances from origin using only xyz data (cache-efficient)
        let xyz_data = point_cloud.xyz();
        let xs = xyz_data.x();
        let ys = xyz_data.y();
        let zs = xyz_data.z();
        let distances: Vec<f32> = (0..len)
            .map(|idx| (xs[idx].powi(2) + ys[idx].powi(2) + zs[idx].powi(2)).sqrt())
            .collect();

        assert_eq!(distances.len(), len);
        assert!(distances[0] > 0.0);

        // Color-only operations without loading spatial data
        // Calculate average brightness using only color data
        let color_data = point_cloud.color();
        let rs = color_data.r();
        let gs = color_data.g();
        let bs = color_data.b();
        let avg_brightness: f32 = (0..len)
            .map(|idx| (rs[idx] + gs[idx] + bs[idx]) / 3.0)
            .sum::<f32>()
            / len as f32;

        assert!(avg_brightness > 0.0 && avg_brightness < 1.0);

        // Bulk transformations using apply() method
        // Transform all points by scaling xyz and adjusting color brightness
        point_cloud.apply(|mut xyz, mut color| {
            xyz.x *= 2.0;
            xyz.y *= 2.0;
            xyz.z *= 2.0;

            color.r = (color.r * 1.2).min(1.0);
            color.g = (color.g * 1.2).min(1.0);
            color.b = (color.b * 1.2).min(1.0);

            (xyz, color)
        });

        // Verify transformations applied correctly
        let first_point = point_cloud.get(1);
        let expected_x = (1_f32 * 0.1).cos() * 10.0 * 2.0; // angle=1*0.1, radius=10, scale=2
        assert!((first_point.xyz.x - expected_x).abs() < 0.001);
        assert!(first_point.color.g > 0.5); // Should be brighter

        // Range operations for processing subsets
        // Process only points 10-20 using range accessors
        let xyz_data = point_cloud.xyz();
        let xs = xyz_data.x();
        let subset_centroid_x: f32 = xs[10..20].iter().sum::<f32>() / 10.0;
        assert!(subset_centroid_x != 0.0);

        // Independent field mutations
        // Modify only colors without affecting spatial data
        let color_data_mut = point_cloud.color_mut();
        let reds = color_data_mut.r_mut();
        for red in &mut reds[..len] {
            *red *= 0.8; // Reduce red component
        }

        // Verify only colors changed, not spatial data
        let point_after = point_cloud.get(1);
        assert_eq!(point_after.xyz.x, expected_x); // Spatial unchanged
        assert!(point_after.color.r < first_point.color.r); // Color changed

        // Test serialization works with realistic data
        let json_result = serde_json::to_string(&point_cloud);
        assert!(
            json_result.is_ok(),
            "Failed to serialize realistic point cloud"
        );
    }
}
