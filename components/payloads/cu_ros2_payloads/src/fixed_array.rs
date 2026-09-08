//! CDR-correct serde for fixed-size arrays longer than serde's built-in limit.
//!
//! `serde` implements `Serialize`/`Deserialize` for `[T; N]` only up to `N == 32`, which is one
//! short of the 6x6 covariance matrices ROS uses (`float64[36]`). Encoding one as a `Vec` is not
//! an option: in CDR a *fixed* array is written as bare elements, while a *sequence* is prefixed
//! with a `uint32` length, so a `Vec` would insert four bytes that no ROS subscriber expects and
//! shift every field after it.
//!
//! Use it as `#[serde(with = "crate::fixed_array")]` on any `[T; N]` field.

use core::fmt;
use core::marker::PhantomData;
use serde::de::{Deserialize, Deserializer, Error as DeError, SeqAccess, Visitor};
use serde::ser::{Serialize, SerializeTuple, Serializer};

pub fn serialize<S, T, const N: usize>(array: &[T; N], serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
    T: Serialize,
{
    // `serialize_tuple`, not `serialize_seq`: a tuple has a statically known length, which is what
    // makes the CDR backend omit the length prefix.
    let mut tuple = serializer.serialize_tuple(N)?;
    for element in array {
        tuple.serialize_element(element)?;
    }
    tuple.end()
}

struct ArrayVisitor<T, const N: usize>(PhantomData<T>);

impl<'de, T, const N: usize> Visitor<'de> for ArrayVisitor<T, N>
where
    T: Deserialize<'de>,
{
    type Value = [T; N];

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        write!(formatter, "an array of length {N}")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut values = Vec::with_capacity(N);
        for index in 0..N {
            let element = seq
                .next_element()?
                .ok_or_else(|| DeError::invalid_length(index, &self))?;
            values.push(element);
        }
        values
            .try_into()
            .map_err(|_| DeError::invalid_length(N, &self))
    }
}

pub fn deserialize<'de, D, T, const N: usize>(deserializer: D) -> Result<[T; N], D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de>,
{
    deserializer.deserialize_tuple(N, ArrayVisitor::<T, N>(PhantomData))
}

#[cfg(test)]
mod tests {
    use serde::{Deserialize, Serialize};

    #[derive(Debug, PartialEq, Serialize, Deserialize)]
    struct Covariance {
        #[serde(with = "crate::fixed_array")]
        values: [f64; 36],
        // A trailing field so a stray length prefix shows up as a decode failure rather than
        // silently trailing bytes.
        tail: u32,
    }

    #[test]
    fn fixed_array_roundtrips_without_a_length_prefix() {
        let original = Covariance {
            values: core::array::from_fn(|i| i as f64),
            tail: 0xABCD_1234,
        };

        let bytes = cdr::serialize::<_, _, cdr::CdrLe>(&original, cdr::Infinite)
            .expect("cdr encode should succeed");
        // 4-byte CDR encapsulation header, then 36 f64 and one u32, with no sequence length.
        assert_eq!(bytes.len(), 4 + 36 * 8 + 4);

        let decoded: Covariance = cdr::deserialize(bytes.as_slice()).expect("cdr decode");
        assert_eq!(decoded, original);
    }
}
