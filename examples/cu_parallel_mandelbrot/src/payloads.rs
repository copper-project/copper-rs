use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize, Serializer};

/// In-flight Mandelbrot state for one `(frame, stripe)` CopperList.
///
/// The large arrays live behind Copper handles so the benchmark stresses the
/// scheduler and the compute stages instead of paying for stripe copies
/// between tasks.
#[derive(Debug, Clone, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct MandelbrotStripe {
    /// Zoom frame index this stripe belongs to.
    pub frame_index: u32,
    /// Stripe index within the frame.
    pub stripe_index: u32,
    /// First image row covered by this stripe.
    pub start_row: u32,
    /// Number of valid rows in this stripe.
    pub row_count: u32,
    /// Configured stripe height used to compute frame-major order.
    pub stripe_rows: u32,
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Complex plane center X for the frame.
    pub center_x: f32,
    /// Complex plane center Y for the frame.
    pub center_y: f32,
    /// Horizontal span of the frame in complex-plane units.
    pub span_x: f32,
    /// Maximum Mandelbrot iterations for this zoom frame.
    pub max_iter: u16,
    /// Number of iterations already completed by upstream compute stages.
    pub completed_iters: u16,
    /// Mutable `z.re` state for every pixel in the stripe.
    pub z_re: CuHandle<Vec<f32>>,
    /// Mutable `z.im` state for every pixel in the stripe.
    pub z_im: CuHandle<Vec<f32>>,
    /// Escape iteration for every pixel, or `0` while the pixel is still active.
    pub escape_iter: CuHandle<Vec<u16>>,
    /// Final RGB bytes for the stripe; filled by the last compute stage.
    pub pixels_rgb: CuHandle<Vec<u8>>,
}

impl MandelbrotStripe {
    #[inline]
    pub fn stripes_per_frame(&self) -> u32 {
        self.height.div_ceil(self.stripe_rows.max(1))
    }

    #[inline]
    pub fn linear_index(&self) -> u64 {
        self.frame_index as u64 * self.stripes_per_frame() as u64 + self.stripe_index as u64
    }

    #[inline]
    pub fn span_y(&self) -> f32 {
        self.span_x * self.height as f32 / self.width as f32
    }

    #[inline]
    pub fn pixel_real(&self, x: usize) -> f32 {
        let width = self.width.max(2) as f32;
        let normalized = x as f32 / (width - 1.0);
        self.center_x + (normalized - 0.5) * self.span_x
    }

    #[inline]
    pub fn row_imag(&self, local_row: u32) -> f32 {
        let height = self.height.max(2) as f32;
        let row_index = self.start_row + local_row;
        let normalized = row_index as f32 / (height - 1.0);
        self.center_y + (normalized - 0.5) * self.span_y()
    }

    #[inline]
    pub fn stripe_rgb_len(&self) -> usize {
        self.width as usize * self.row_count as usize * 3
    }
}

impl Default for MandelbrotStripe {
    fn default() -> Self {
        Self {
            frame_index: 0,
            stripe_index: 0,
            start_row: 0,
            row_count: 0,
            stripe_rows: 0,
            width: 0,
            height: 0,
            center_x: 0.0,
            center_y: 0.0,
            span_x: 0.0,
            max_iter: 0,
            completed_iters: 0,
            z_re: CuHandle::new_detached(Vec::new()),
            z_im: CuHandle::new_detached(Vec::new()),
            escape_iter: CuHandle::new_detached(Vec::new()),
            pixels_rgb: CuHandle::new_detached(Vec::new()),
        }
    }
}

impl Encode for MandelbrotStripe {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.frame_index, encoder)?;
        Encode::encode(&self.stripe_index, encoder)?;
        Encode::encode(&self.start_row, encoder)?;
        Encode::encode(&self.row_count, encoder)?;
        Encode::encode(&self.stripe_rows, encoder)?;
        Encode::encode(&self.width, encoder)?;
        Encode::encode(&self.height, encoder)?;
        Encode::encode(&self.center_x, encoder)?;
        Encode::encode(&self.center_y, encoder)?;
        Encode::encode(&self.span_x, encoder)?;
        Encode::encode(&self.max_iter, encoder)?;
        Encode::encode(&self.completed_iters, encoder)?;
        Encode::encode(&self.z_re, encoder)?;
        Encode::encode(&self.z_im, encoder)?;
        Encode::encode(&self.escape_iter, encoder)?;
        Encode::encode(&self.pixels_rgb, encoder)?;
        Ok(())
    }
}

impl Decode<()> for MandelbrotStripe {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            frame_index: Decode::decode(decoder)?,
            stripe_index: Decode::decode(decoder)?,
            start_row: Decode::decode(decoder)?,
            row_count: Decode::decode(decoder)?,
            stripe_rows: Decode::decode(decoder)?,
            width: Decode::decode(decoder)?,
            height: Decode::decode(decoder)?,
            center_x: Decode::decode(decoder)?,
            center_y: Decode::decode(decoder)?,
            span_x: Decode::decode(decoder)?,
            max_iter: Decode::decode(decoder)?,
            completed_iters: Decode::decode(decoder)?,
            z_re: Decode::decode(decoder)?,
            z_im: Decode::decode(decoder)?,
            escape_iter: Decode::decode(decoder)?,
            pixels_rgb: Decode::decode(decoder)?,
        })
    }
}

impl Serialize for MandelbrotStripe {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;

        let mut state = serializer.serialize_struct("MandelbrotStripe", 16)?;
        state.serialize_field("frame_index", &self.frame_index)?;
        state.serialize_field("stripe_index", &self.stripe_index)?;
        state.serialize_field("start_row", &self.start_row)?;
        state.serialize_field("row_count", &self.row_count)?;
        state.serialize_field("stripe_rows", &self.stripe_rows)?;
        state.serialize_field("width", &self.width)?;
        state.serialize_field("height", &self.height)?;
        state.serialize_field("center_x", &self.center_x)?;
        state.serialize_field("center_y", &self.center_y)?;
        state.serialize_field("span_x", &self.span_x)?;
        state.serialize_field("max_iter", &self.max_iter)?;
        state.serialize_field("completed_iters", &self.completed_iters)?;
        state.serialize_field("z_re", &self.z_re.with_inner(|inner| inner.to_vec()))?;
        state.serialize_field("z_im", &self.z_im.with_inner(|inner| inner.to_vec()))?;
        state.serialize_field(
            "escape_iter",
            &self.escape_iter.with_inner(|inner| inner.to_vec()),
        )?;
        state.serialize_field(
            "pixels_rgb",
            &self.pixels_rgb.with_inner(|inner| inner.to_vec()),
        )?;
        state.end()
    }
}

impl<'de> Deserialize<'de> for MandelbrotStripe {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct MandelbrotStripeWire {
            frame_index: u32,
            stripe_index: u32,
            start_row: u32,
            row_count: u32,
            stripe_rows: u32,
            width: u32,
            height: u32,
            center_x: f32,
            center_y: f32,
            span_x: f32,
            max_iter: u16,
            completed_iters: u16,
            z_re: Vec<f32>,
            z_im: Vec<f32>,
            escape_iter: Vec<u16>,
            pixels_rgb: Vec<u8>,
        }

        let wire = MandelbrotStripeWire::deserialize(deserializer)?;
        Ok(Self {
            frame_index: wire.frame_index,
            stripe_index: wire.stripe_index,
            start_row: wire.start_row,
            row_count: wire.row_count,
            stripe_rows: wire.stripe_rows,
            width: wire.width,
            height: wire.height,
            center_x: wire.center_x,
            center_y: wire.center_y,
            span_x: wire.span_x,
            max_iter: wire.max_iter,
            completed_iters: wire.completed_iters,
            z_re: CuHandle::new_detached(wire.z_re),
            z_im: CuHandle::new_detached(wire.z_im),
            escape_iter: CuHandle::new_detached(wire.escape_iter),
            pixels_rgb: CuHandle::new_detached(wire.pixels_rgb),
        })
    }
}
