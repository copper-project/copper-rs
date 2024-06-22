use std::fmt;
use std::marker::PhantomData;

use serde::de::{self, SeqAccess, Visitor};
use serde::ser::SerializeSeq;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// Serialize a fixed 2D array.
pub fn serialize<S, T, const WIDTH: usize, const HEIGHT: usize>(
    array: &[[T; WIDTH]; HEIGHT],
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: Serializer,
    T: Serialize,
{
    let mut seq = serializer.serialize_seq(Some(WIDTH * HEIGHT))?;
    for row in array.iter() {
        for element in row.iter() {
            seq.serialize_element(element)?;
        }
    }
    seq.end()
}

/// Deserialize a fixed 2D array.
pub fn deserialize<'de, D, T, const WIDTH: usize, const HEIGHT: usize>(
    deserializer: D,
) -> Result<[[T; WIDTH]; HEIGHT], D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de> + Default + Copy,
{
    struct ArrayVisitor<T, const WIDTH: usize, const HEIGHT: usize> {
        marker: PhantomData<T>,
    }

    impl<'de, T, const WIDTH: usize, const HEIGHT: usize> Visitor<'de>
        for ArrayVisitor<T, WIDTH, HEIGHT>
    where
        T: Deserialize<'de> + Default + Copy,
    {
        type Value = [[T; WIDTH]; HEIGHT];

        fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
            formatter.write_str("a 2D array")
        }

        fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
        where
            A: SeqAccess<'de>,
        {
            let mut array = [[T::default(); WIDTH]; HEIGHT];
            for row in array.iter_mut() {
                for element in row.iter_mut() {
                    *element = seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                }
            }
            Ok(array)
        }
    }

    deserializer.deserialize_seq(ArrayVisitor::<T, WIDTH, HEIGHT> {
        marker: PhantomData,
    })
}
