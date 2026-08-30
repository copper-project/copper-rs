use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use core::fmt::Debug;
use core::marker::PhantomData;
use cu29::prelude::*;
use cu29::units::si::f32::Length;
use cu29::units::si::length::meter;
use serde::{Deserialize, Serialize, Serializer};

/// Raster layout for a depth map whose samples are physical lengths.
///
/// `stride` is expressed in elements, not bytes, and may be greater than `width`
/// when rows contain padding.
#[derive(
    Default, Debug, Encode, Decode, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Reflect,
)]
pub struct CuDepthMapFormat {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
}

impl CuDepthMapFormat {
    pub fn is_valid(&self) -> bool {
        self.width <= self.stride
    }

    pub fn required_elements(&self) -> usize {
        self.stride as usize * self.height as usize
    }
}

/// Raw invalid value used by a [`CuDepthEncodingDescriptor`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Reflect)]
#[serde(rename_all = "snake_case")]
pub enum CuDepthInvalidValue {
    Unsigned(u64),
    Signed(i64),
    FloatBits(u64),
}

/// Static depth-encoding metadata exposed to debuggers and offline tooling.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Reflect)]
pub struct CuDepthEncodingDescriptor {
    /// Number of native sample units in one meter.
    pub units_per_meter: f32,
    /// Raw value reserved for invalid measurements, when the encoding has one.
    pub invalid: Option<CuDepthInvalidValue>,
}

/// Compile-time description of a depth map's native sample representation.
pub trait CuDepthEncoding: Copy + Debug + Default + Send + Sync + 'static {
    type Sample: ElementType;

    const DESCRIPTOR: CuDepthEncodingDescriptor;
    const SERIALIZED_MAP_SCHEMA: &'static str = GENERIC_DEPTH_MAP_SERIALIZED_SCHEMA;
    const TYPE_PATH: &'static str;
    const SHORT_TYPE_PATH: &'static str;
    const TYPE_IDENT: &'static str;

    /// Converts one native sample into a typed physical length.
    ///
    /// Returns `None` when the raw sample represents an invalid measurement.
    fn decode(sample: Self::Sample) -> Option<Length>;

    /// Converts a typed physical length into one native sample.
    ///
    /// `None` represents an invalid measurement in the target encoding.
    fn encode_sample(sample: Option<Length>) -> Self::Sample;
}

/// Default depth encoding backed directly by unit-safe [`Length`] samples.
#[derive(Debug, Default, Clone, Copy)]
pub struct CuDepthLength;

impl CuDepthEncoding for CuDepthLength {
    type Sample = Length;

    const DESCRIPTOR: CuDepthEncodingDescriptor = CuDepthEncodingDescriptor {
        units_per_meter: 1.0,
        invalid: None,
    };
    const SERIALIZED_MAP_SCHEMA: &'static str = LENGTH_DEPTH_MAP_SERIALIZED_SCHEMA;
    const TYPE_PATH: &'static str = "cu_sensor_payloads::CuDepthMap";
    const SHORT_TYPE_PATH: &'static str = "CuDepthMap";
    const TYPE_IDENT: &'static str = "CuDepthMap";

    fn decode(sample: Self::Sample) -> Option<Length> {
        Some(sample)
    }

    fn encode_sample(sample: Option<Length>) -> Self::Sample {
        sample.unwrap_or_else(|| Length::new::<meter>(f32::NAN))
    }
}

/// Builds a typed length for use in a compile-time depth resolution.
///
/// `Length` stores its value in meters internally, but its usual unit-aware
/// constructor is not currently a `const fn`.
pub const fn depth_resolution_meters(meters: f32) -> Length {
    Length { value: meters }
}

/// Compile-time physical distance represented by one integer increment.
pub trait CuDepthScale: Copy + Debug + Default + Send + Sync + 'static {
    const RESOLUTION: Length;
    const UNITS_PER_METER: f32 = 1.0 / Self::RESOLUTION.value;
}

/// Rational depth resolution expressed in meters.
///
/// For example, `CuDepthScaleRatio<1, 10>` represents 10 centimeters per
/// integer increment.
#[derive(Debug, Default, Clone, Copy)]
pub struct CuDepthScaleRatio<const NUMERATOR: u64, const DENOMINATOR: u64>;

impl<const NUMERATOR: u64, const DENOMINATOR: u64> CuDepthScale
    for CuDepthScaleRatio<NUMERATOR, DENOMINATOR>
{
    const RESOLUTION: Length = depth_resolution_meters(NUMERATOR as f32 / DENOMINATOR as f32);
    const UNITS_PER_METER: f32 = DENOMINATOR as f32 / NUMERATOR as f32;
}

/// One millimeter per integer increment.
pub type CuDepthMillimeter = CuDepthScaleRatio<1, 1_000>;

/// One decimeter (10 centimeters) per integer increment.
pub type CuDepthDecimeter = CuDepthScaleRatio<1, 10>;

/// Unsigned integer storage supported by [`CuDepthInteger`].
pub trait CuDepthIntegerSample: ElementType + 'static {
    const MAX: u64;

    fn to_u64(self) -> u64;
    fn from_u64(value: u64) -> Self;
}

macro_rules! impl_depth_integer_sample {
    ($($sample:ty),+ $(,)?) => {
        $(
            impl CuDepthIntegerSample for $sample {
                const MAX: u64 = <$sample>::MAX as u64;

                fn to_u64(self) -> u64 {
                    self as u64
                }

                fn from_u64(value: u64) -> Self {
                    value as Self
                }
            }
        )+
    };
}

impl_depth_integer_sample!(u8, u16, u32, u64);

/// Compact unsigned-integer depth encoding with a compile-time physical scale.
///
/// Zero represents an invalid measurement. The sample width and resolution are
/// independent: for example, `CuDepthInteger<u16, CuDepthDecimeter>` covers
/// depths in 10-centimeter increments.
///
/// A 200-meter camera with 10-centimeter increments can use:
///
/// ```
/// use cu_sensor_payloads::{
///     CuDepthInteger, CuDepthMap, CuDepthScaleRatio,
/// };
///
/// type LongRangeDepth =
///     CuDepthMap<Vec<u16>, CuDepthInteger<u16, CuDepthScaleRatio<1, 10>>>;
/// ```
#[derive(Debug, Default, Clone, Copy)]
pub struct CuDepthInteger<T, S>(PhantomData<(T, S)>);

impl<T, S> CuDepthEncoding for CuDepthInteger<T, S>
where
    T: CuDepthIntegerSample,
    S: CuDepthScale,
{
    type Sample = T;

    const DESCRIPTOR: CuDepthEncodingDescriptor = CuDepthEncodingDescriptor {
        units_per_meter: S::UNITS_PER_METER,
        invalid: Some(CuDepthInvalidValue::Unsigned(0)),
    };
    const SERIALIZED_MAP_SCHEMA: &'static str = INTEGER_DEPTH_MAP_SERIALIZED_SCHEMA;
    const TYPE_PATH: &'static str = "cu_sensor_payloads::CuDepthMapInteger";
    const SHORT_TYPE_PATH: &'static str = "CuDepthMapInteger";
    const TYPE_IDENT: &'static str = "CuDepthMapInteger";

    fn decode(sample: Self::Sample) -> Option<Length> {
        let raw = sample.to_u64();
        (raw != 0).then(|| depth_resolution_meters(raw as f32 / S::UNITS_PER_METER))
    }

    fn encode_sample(sample: Option<Length>) -> Self::Sample {
        let Some(meters) = sample.map(|sample| sample.get::<meter>()) else {
            return T::from_u64(0);
        };
        if !meters.is_finite() || meters <= 0.0 {
            return T::from_u64(0);
        }
        let raw = (meters * S::UNITS_PER_METER)
            .round()
            .clamp(1.0, T::MAX as f32) as u64;
        T::from_u64(raw)
    }
}

/// Standard depth-map payload backed by a Copper handle.
///
/// The default encoding stores [`Length`] directly. Alternate encodings can
/// retain a sensor's compact native representation without a producer-side
/// conversion or copy.
#[derive(Debug, Default, Clone, Reflect)]
#[reflect(
    from_reflect = false,
    no_field_bounds,
    type_path = false,
    SerializedPayloadSchema
)]
pub struct CuDepthMap<A, E = CuDepthLength>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample> + Send + Sync + 'static,
{
    pub format: CuDepthMapFormat,
    #[reflect(ignore)]
    pub buffer_handle: CuHandle<A>,
    #[reflect(ignore)]
    encoding: PhantomData<E>,
}

impl<A, E> TypePath for CuDepthMap<A, E>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample> + Send + Sync + 'static,
{
    fn type_path() -> &'static str {
        E::TYPE_PATH
    }

    fn short_type_path() -> &'static str {
        E::SHORT_TYPE_PATH
    }

    fn type_ident() -> Option<&'static str> {
        Some(E::TYPE_IDENT)
    }

    fn crate_name() -> Option<&'static str> {
        Some("cu_sensor_payloads")
    }

    fn module_path() -> Option<&'static str> {
        Some("cu_sensor_payloads")
    }
}

impl<A, E> Encode for CuDepthMap<A, E>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample> + Send + Sync + 'static,
    CuHandle<A>: Encode,
{
    fn encode<Enc: Encoder>(&self, encoder: &mut Enc) -> Result<(), EncodeError> {
        Encode::encode(&self.format, encoder)?;
        Encode::encode(&self.buffer_handle, encoder)
    }
}

impl<A, E> Decode<()> for CuDepthMap<A, E>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample> + Send + Sync + 'static,
    CuHandle<A>: Decode<()>,
{
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            format: Decode::decode(decoder)?,
            buffer_handle: Decode::decode(decoder)?,
            encoding: PhantomData,
        })
    }
}

impl<'de, A, E> Deserialize<'de> for CuDepthMap<A, E>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample> + Send + Sync + 'static,
    CuHandle<A>: Deserialize<'de>,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Wire<H> {
            format: CuDepthMapFormat,
            encoding: Option<CuDepthEncodingDescriptor>,
            handle: H,
        }

        let wire = Wire::<CuHandle<A>>::deserialize(deserializer)?;
        if let Some(encoding) = wire.encoding
            && encoding != E::DESCRIPTOR
        {
            return Err(serde::de::Error::custom(format!(
                "Depth encoding {:?} does not match {}",
                encoding,
                E::SHORT_TYPE_PATH
            )));
        }
        Ok(Self {
            format: wire.format,
            buffer_handle: wire.handle,
            encoding: PhantomData,
        })
    }
}

impl<A, E> Serialize for CuDepthMap<A, E>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample> + Send + Sync + 'static,
    CuHandle<A>: Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct(E::SHORT_TYPE_PATH, 3)?;
        state.serialize_field("format", &self.format)?;
        state.serialize_field("encoding", &E::DESCRIPTOR)?;
        state.serialize_field("handle", &self.buffer_handle)?;
        state.end()
    }
}

impl<A, E> CuDepthMap<A, E>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample> + Send + Sync + 'static,
{
    pub const fn encoding_descriptor() -> CuDepthEncodingDescriptor {
        E::DESCRIPTOR
    }

    pub fn new_encoded(format: CuDepthMapFormat, buffer_handle: CuHandle<A>) -> Self {
        assert!(
            E::DESCRIPTOR.units_per_meter.is_finite() && E::DESCRIPTOR.units_per_meter > 0.0,
            "Depth encoding must have a finite positive resolution."
        );
        assert!(
            format.is_valid(),
            "Depth-map stride must be at least its width."
        );
        assert!(
            format.required_elements() <= buffer_handle.with_inner(|inner| inner.len()),
            "Buffer size must at least match the depth-map format."
        );
        Self {
            format,
            buffer_handle,
            encoding: PhantomData,
        }
    }

    /// Returns the native encoded sample at `(x, y)`.
    pub fn get_raw(&self, x: u32, y: u32) -> Option<E::Sample> {
        if x >= self.format.width || y >= self.format.height {
            return None;
        }
        let index = y as usize * self.format.stride as usize + x as usize;
        self.buffer_handle
            .with_inner(|inner| inner.as_ref().get(index).copied())
    }

    /// Converts a native sample into a physical length.
    pub fn decode_sample(sample: E::Sample) -> Option<Length> {
        E::decode(sample)
    }

    /// Returns the physical depth sample at `(x, y)`.
    ///
    /// Returns `None` for out-of-bounds coordinates and encoded invalid values.
    pub fn get(&self, x: u32, y: u32) -> Option<Length> {
        Self::decode_sample(self.get_raw(x, y)?)
    }

    /// Returns the sample at `(x, y)` converted to meters.
    pub fn get_meters(&self, x: u32, y: u32) -> Option<f32> {
        self.get(x, y).map(|depth| depth.get::<meter>())
    }

    /// Converts this map into a caller-provided target map.
    ///
    /// Only logical pixels are written; target row padding is left untouched.
    /// This performs one source lock, one target lock, and no allocation.
    pub fn convert_into<B, F>(&self, target: &mut CuDepthMap<B, F>) -> CuResult<()>
    where
        F: CuDepthEncoding,
        B: ArrayLike<Element = F::Sample> + Send + Sync + 'static,
    {
        if self.format.width != target.format.width || self.format.height != target.format.height {
            return Err(CuError::from(format!(
                "Depth-map dimensions differ: source is {}x{}, target is {}x{}",
                self.format.width, self.format.height, target.format.width, target.format.height
            )));
        }
        if self.buffer_handle.storage_id() == target.buffer_handle.storage_id() {
            return Err(CuError::from(
                "Depth-map conversion requires distinct source and target storage",
            ));
        }

        let convert = |source: &[E::Sample],
                       source_format: CuDepthMapFormat,
                       destination: &mut [F::Sample],
                       destination_format: CuDepthMapFormat| {
            for y in 0..source_format.height as usize {
                let source_row = y * source_format.stride as usize;
                let destination_row = y * destination_format.stride as usize;
                for x in 0..source_format.width as usize {
                    destination[destination_row + x] =
                        F::encode_sample(E::decode(source[source_row + x]));
                }
            }
        };

        if self.buffer_handle.storage_id() < target.buffer_handle.storage_id() {
            self.with_samples(|source, source_format| {
                target.with_samples_mut(|destination, destination_format| {
                    convert(source, source_format, destination, destination_format);
                });
            });
        } else {
            target.with_samples_mut(|destination, destination_format| {
                self.with_samples(|source, source_format| {
                    convert(source, source_format, destination, destination_format);
                });
            });
        }
        Ok(())
    }

    /// Accesses all samples under a single buffer lock.
    ///
    /// The slice covers exactly [`CuDepthMapFormat::required_elements`], including
    /// any row padding described by [`CuDepthMapFormat::stride`].
    pub fn with_samples<R>(&self, f: impl FnOnce(&[E::Sample], CuDepthMapFormat) -> R) -> R {
        let format = self.format;
        self.buffer_handle
            .with_inner(|inner| f(&inner[..format.required_elements()], format))
    }

    /// Mutably accesses all samples under a single buffer lock.
    ///
    /// The slice covers exactly [`CuDepthMapFormat::required_elements`], including
    /// any row padding described by [`CuDepthMapFormat::stride`].
    pub fn with_samples_mut<R>(
        &mut self,
        f: impl FnOnce(&mut [E::Sample], CuDepthMapFormat) -> R,
    ) -> R {
        let format = self.format;
        self.buffer_handle
            .with_inner_mut(|inner| f(&mut inner[..format.required_elements()], format))
    }

    pub fn payload_should_log(&self) -> bool {
        self.buffer_handle.payload_should_log()
    }

    pub fn apply_handle_content_policy(&self, mode: cu29::pool::HandleContent) {
        self.buffer_handle.apply_handle_content_policy(mode);
    }

    pub fn mark_touched(&self) {
        self.buffer_handle.mark_touched();
    }
}

impl<A> CuDepthMap<A, CuDepthLength>
where
    A: ArrayLike<Element = Length> + Send + Sync + 'static,
{
    pub fn new(format: CuDepthMapFormat, buffer_handle: CuHandle<A>) -> Self {
        Self::new_encoded(format, buffer_handle)
    }
}

impl<A, T, S> CuDepthMap<A, CuDepthInteger<T, S>>
where
    T: CuDepthIntegerSample,
    S: CuDepthScale,
    A: ArrayLike<Element = T> + Send + Sync + 'static,
{
    pub fn from_integer(format: CuDepthMapFormat, buffer_handle: CuHandle<A>) -> Self {
        Self::new_encoded(format, buffer_handle)
    }
}

impl<A, E> cu29::pool::HandleContentAware for CuDepthMap<A, E>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample> + Send + Sync + 'static,
{
}

macro_rules! depth_map_serialized_schema {
    ($sample_schema:literal, $invalid_schema:literal) => {
        concat!(
            r#"{"$schema":"https://json-schema.org/draft-07/schema#","type":"object","properties":{
"format":{"type":"object","properties":{"width":{"type":"integer","minimum":0},"height":{"type":"integer","minimum":0},"stride":{"type":"integer","minimum":0}},"required":["width","height","stride"],"additionalProperties":false},
"encoding":{"type":"object","properties":{"units_per_meter":{"type":"number"},"invalid":"#,
            $invalid_schema,
            r#"},"required":["units_per_meter","invalid"],"additionalProperties":false},
"handle":{"type":"array","items":"#,
            $sample_schema,
            r#"}},"required":["format","encoding","handle"],"additionalProperties":false}"#
        )
    };
}

const LENGTH_DEPTH_MAP_SERIALIZED_SCHEMA: &str =
    depth_map_serialized_schema!(r#"{"type":"number"}"#, r#"{"type":"null"}"#);

const INTEGER_DEPTH_MAP_SERIALIZED_SCHEMA: &str = depth_map_serialized_schema!(
    r#"{"type":"integer","minimum":0}"#,
    r#"{"type":"object","properties":{"unsigned":{"type":"integer","const":0}},"required":["unsigned"],"additionalProperties":false}"#
);

const GENERIC_DEPTH_MAP_SERIALIZED_SCHEMA: &str = depth_map_serialized_schema!(r#"{}"#, r#"{}"#);

impl<A, E> SerializedPayloadSchema for CuDepthMap<A, E>
where
    E: CuDepthEncoding,
    A: ArrayLike<Element = E::Sample> + Send + Sync + 'static,
{
    fn serialized_payload_schema() -> &'static str {
        E::SERIALIZED_MAP_SCHEMA
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type U16MillimeterDepth = CuDepthMap<Vec<u16>, CuDepthInteger<u16, CuDepthMillimeter>>;

    const FORMAT: CuDepthMapFormat = CuDepthMapFormat {
        width: 3,
        height: 2,
        stride: 4,
    };

    #[test]
    fn padded_depth_map_indexes_distances_in_meters() {
        let depth = CuDepthMap::new(
            FORMAT,
            CuHandle::new_detached(
                [1.0, 1.5, f32::NAN, 99.0, 2.0, 2.5, 3.0, 99.0]
                    .map(Length::new::<meter>)
                    .to_vec(),
            ),
        );

        assert_eq!(depth.get(0, 0), Some(Length::new::<meter>(1.0)));
        assert_eq!(depth.get_meters(1, 1), Some(2.5));
        assert!(depth.get_meters(2, 0).expect("in bounds").is_nan());
        assert_eq!(depth.get(3, 0), None);
        assert_eq!(depth.get(0, 2), None);
    }

    #[test]
    fn sample_slice_covers_the_declared_layout_under_one_access() {
        let depth = CuDepthMap::new(
            FORMAT,
            CuHandle::new_detached(
                [1.0, 1.5, 2.0, 99.0, 2.5, 3.0, 3.5, 99.0, 100.0]
                    .map(Length::new::<meter>)
                    .to_vec(),
            ),
        );

        depth.with_samples(|samples, format| {
            assert_eq!(format, FORMAT);
            assert_eq!(samples.len(), FORMAT.required_elements());
            assert_eq!(samples[format.stride as usize], Length::new::<meter>(2.5));
            assert_eq!(samples[3], Length::new::<meter>(99.0));
        });
    }

    #[test]
    fn sample_slice_can_be_mutated_under_one_access() {
        let mut depth = CuDepthMap::new(
            FORMAT,
            CuHandle::new_detached(vec![Length::new::<meter>(0.0); FORMAT.required_elements()]),
        );

        depth.with_samples_mut(|samples, format| {
            let index = format.stride as usize + 1;
            samples[index] = Length::new::<meter>(4.25);
        });

        assert_eq!(depth.get_meters(1, 1), Some(4.25));
    }

    #[test]
    fn compact_millimeter_depth_decodes_without_changing_raw_storage() {
        let depth = U16MillimeterDepth::from_integer(
            FORMAT,
            CuHandle::new_detached(vec![1_000, 1_500, 0, 99, 2_000, 2_500, 3_000, 99]),
        );

        assert_eq!(depth.get_raw(1, 0), Some(1_500));
        assert_eq!(depth.get_meters(1, 0), Some(1.5));
        assert_eq!(depth.get_raw(2, 0), Some(0));
        assert_eq!(depth.get(2, 0), None);
        depth.with_samples(|samples, format| {
            assert_eq!(format, FORMAT);
            assert_eq!(samples, &[1_000, 1_500, 0, 99, 2_000, 2_500, 3_000, 99]);
        });
    }

    #[test]
    fn depth_maps_convert_between_encodings_without_touching_padding() {
        let source = CuDepthMap::new(
            FORMAT,
            CuHandle::new_detached(
                [1.234, f32::NAN, 2.0, -999.0, 3.5, 4.0, 5.0, -999.0]
                    .map(Length::new::<meter>)
                    .to_vec(),
            ),
        );
        let target_format = CuDepthMapFormat {
            width: FORMAT.width,
            height: FORMAT.height,
            stride: 5,
        };
        let mut compact = U16MillimeterDepth::from_integer(
            target_format,
            CuHandle::new_detached(vec![777u16; target_format.required_elements()]),
        );

        source.convert_into(&mut compact).expect("convert to u16");
        compact.with_samples(|samples, _| {
            assert_eq!(
                samples,
                &[1_234, 0, 2_000, 777, 777, 3_500, 4_000, 5_000, 777, 777]
            );
        });

        let mut typed = CuDepthMap::new(
            FORMAT,
            CuHandle::new_detached(vec![Length::new::<meter>(-1.0); FORMAT.required_elements()]),
        );
        compact.convert_into(&mut typed).expect("convert to Length");
        assert_eq!(typed.get_meters(0, 0), Some(1.234));
        assert!(
            typed
                .get_raw(1, 0)
                .expect("in bounds")
                .get::<meter>()
                .is_nan()
        );
    }

    #[test]
    fn depth_encodings_have_distinct_type_paths() {
        assert_eq!(
            <CuDepthMap<Vec<Length>> as TypePath>::type_path(),
            "cu_sensor_payloads::CuDepthMap"
        );
        assert_eq!(
            <U16MillimeterDepth as TypePath>::type_path(),
            "cu_sensor_payloads::CuDepthMapInteger"
        );
    }

    #[test]
    fn integer_depth_supports_all_widths_and_arbitrary_length_scales() {
        fn assert_integer_sample<T: CuDepthIntegerSample>() {}
        assert_integer_sample::<u8>();
        assert_integer_sample::<u16>();
        assert_integer_sample::<u32>();
        assert_integer_sample::<u64>();

        type U16DecimeterDepth = CuDepthMap<Vec<u16>, CuDepthInteger<u16, CuDepthDecimeter>>;
        let format = CuDepthMapFormat {
            width: 1,
            height: 1,
            stride: 1,
        };
        let depth = U16DecimeterDepth::from_integer(format, CuHandle::new_detached(vec![2_000]));

        assert_eq!(CuDepthDecimeter::RESOLUTION, Length::new::<meter>(0.1));
        assert_eq!(
            U16DecimeterDepth::encoding_descriptor().units_per_meter,
            10.0
        );
        assert_eq!(depth.get_meters(0, 0), Some(200.0));
    }

    #[test]
    fn format_rejects_short_stride() {
        assert!(
            !CuDepthMapFormat {
                width: 4,
                height: 2,
                stride: 3,
            }
            .is_valid()
        );
    }

    #[test]
    fn serde_wire_uses_handle_field() {
        let depth = CuDepthMap::new(
            FORMAT,
            CuHandle::new_detached(
                [1.0, 1.5, 2.0, 0.0, 2.5, 3.0, 3.5, 0.0]
                    .map(Length::new::<meter>)
                    .to_vec(),
            ),
        );
        let value = serde_json::to_value(depth).expect("serialize depth");
        assert_eq!(value["format"]["stride"], 4);
        assert_eq!(value["encoding"]["units_per_meter"], 1.0);
        assert!(value["encoding"]["invalid"].is_null());
        assert!(value.get("handle").is_some());
        assert!(value.get("buffer_handle").is_none());
        assert!(value.get("seq").is_none());
    }

    #[test]
    fn compact_serde_wire_describes_units_and_invalid_value() {
        let depth = U16MillimeterDepth::from_integer(
            FORMAT,
            CuHandle::new_detached(vec![0u16; FORMAT.required_elements()]),
        );
        let value = serde_json::to_value(depth).expect("serialize compact depth");

        assert_eq!(value["encoding"]["units_per_meter"], 1_000.0);
        assert_eq!(value["encoding"]["invalid"]["unsigned"], 0);
    }

    #[test]
    fn compact_serialized_schema_matches_depth_wire_fields() {
        let schema = serde_json::from_str::<serde_json::Value>(
            U16MillimeterDepth::serialized_payload_schema(),
        )
        .expect("parse compact depth schema");

        assert_eq!(schema["properties"]["handle"]["items"]["type"], "integer");
        assert_eq!(
            schema["properties"]["encoding"]["properties"]["invalid"]["properties"]["unsigned"]["const"],
            0
        );
        assert_eq!(
            schema["required"],
            serde_json::json!(["format", "encoding", "handle"])
        );
    }
}
