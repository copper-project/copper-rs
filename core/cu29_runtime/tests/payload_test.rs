#[cfg(test)]
mod tests {
    use bincode::{config, Decode, Encode};
    use cu29_runtime::payload::CuArrayVec;

    // Test default initialization
    #[test]
    fn test_cuarrayvec_default() {
        let vec: CuArrayVec<u32, 10> = CuArrayVec::default();
        assert_eq!(vec.0.len(), 0);
        assert_eq!(vec.0.capacity(), 10);
    }

    // Test encoding/decoding with basic types
    #[test]
    fn test_cuarrayvec_encode_decode_basic() {
        let mut vec: CuArrayVec<u32, 5> = CuArrayVec::default();

        // Add some elements
        vec.0.push(1);
        vec.0.push(2);
        vec.0.push(3);

        // Encode
        let config = config::standard();
        let encoded = bincode::encode_to_vec(&vec, config).unwrap();

        // Decode
        let (decoded, _): (CuArrayVec<u32, 5>, _) =
            bincode::decode_from_slice(&encoded, config).unwrap();

        // Verify
        assert_eq!(decoded.0.len(), 3);
        assert_eq!(decoded.0[0], 1);
        assert_eq!(decoded.0[1], 2);
        assert_eq!(decoded.0[2], 3);
    }

    // Test decoding with complex types
    #[derive(Debug, Clone, PartialEq, Encode, Decode)]
    struct TestStruct {
        id: u32,
        name: String,
    }

    #[test]
    fn test_cuarrayvec_complex_type() {
        let mut vec: CuArrayVec<TestStruct, 3> = CuArrayVec::default();

        // Add some structs
        vec.0.push(TestStruct {
            id: 1,
            name: "Item 1".to_string(),
        });
        vec.0.push(TestStruct {
            id: 2,
            name: "Item 2".to_string(),
        });

        // Encode
        let config = config::standard();
        let encoded = bincode::encode_to_vec(&vec, config).unwrap();

        // Decode
        let (decoded, _): (CuArrayVec<TestStruct, 3>, _) =
            bincode::decode_from_slice(&encoded, config).unwrap();

        // Verify
        assert_eq!(decoded.0.len(), 2);
        assert_eq!(
            decoded.0[0],
            TestStruct {
                id: 1,
                name: "Item 1".to_string()
            }
        );
        assert_eq!(
            decoded.0[1],
            TestStruct {
                id: 2,
                name: "Item 2".to_string()
            }
        );
    }

    // Test error case: exceeding capacity during decode
    #[test]
    fn test_cuarrayvec_capacity_error() {
        // Create a larger vector than we'll try to decode into
        let mut large_vec: CuArrayVec<u32, 10> = CuArrayVec::default();
        for i in 0..8 {
            large_vec.0.push(i);
        }

        // Encode the large vector
        let config = config::standard();
        let encoded = bincode::encode_to_vec(&large_vec, config).unwrap();

        // Try to decode into a smaller capacity vector - should fail
        let result: Result<(CuArrayVec<u32, 5>, _), _> =
            bincode::decode_from_slice(&encoded, config);

        assert!(result.is_err());

        // Validate the error type
        if let Err(err) = result {
            assert!(matches!(
                err,
                bincode::error::DecodeError::ArrayLengthMismatch { .. }
            ));
        }
    }

    // Test the borrow decoding functionality with borrowed data
    #[test]
    fn test_cuarrayvec_borrow_decode() {
        let mut vec: CuArrayVec<String, 5> = CuArrayVec::default();

        // Add some elements
        vec.0.push("hello".to_string());
        vec.0.push("world".to_string());

        // Encode
        let config = config::standard();
        let encoded = bincode::encode_to_vec(&vec, config).unwrap();

        // Use borrowed decoding
        let (decoded, _) =
            bincode::borrow_decode_from_slice::<CuArrayVec<String, 5>, _>(&encoded, config)
                .unwrap();

        // Verify
        assert_eq!(decoded.0.len(), 2);
        assert_eq!(decoded.0[0], "hello");
        assert_eq!(decoded.0[1], "world");
    }
}
