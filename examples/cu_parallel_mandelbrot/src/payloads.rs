use bincode::de::Decoder;
use bincode::enc::Encoder;
use bincode::error::{DecodeError, EncodeError};
use bincode::{Decode, Encode};
use cu29::prelude::*;
use serde::{Deserialize, Serialize, Serializer};

/// In-flight Mandelbrot state for one `(frame, row)` CopperList.
///
/// The large arrays live behind Copper handles so the benchmark stresses the
/// scheduler and the compute stages instead of paying for row copies between
/// tasks.
#[derive(Debug, Clone, Reflect)]
#[reflect(opaque, from_reflect = false)]
pub struct MandelbrotRow {
    /// Zoom frame index this row belongs to.
    pub frame_index: u32,
    /// Row index within the frame.
    pub row_index: u32,
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Complex plane center X for the frame.
    pub center_x: f64,
    /// Complex plane center Y for the frame.
    pub center_y: f64,
    /// Horizontal span of the frame in complex-plane units.
    pub span_x: f64,
    /// Maximum Mandelbrot iterations for this zoom frame.
    pub max_iter: u16,
    /// Number of iterations already completed by upstream compute stages.
    pub completed_iters: u16,
    /// Mutable `z.re` state for every pixel in the row.
    pub z_re: CuHandle<Vec<f64>>,
    /// Mutable `z.im` state for every pixel in the row.
    pub z_im: CuHandle<Vec<f64>>,
    /// Escape iteration for every pixel, or `0` while the pixel is still active.
    pub escape_iter: CuHandle<Vec<u16>>,
    /// Final RGB bytes for the row; filled by the last compute stage.
    pub pixels_rgb: CuHandle<Vec<u8>>,
}

impl MandelbrotRow {
    #[inline]
    pub fn linear_index(&self) -> u64 {
        self.frame_index as u64 * self.height as u64 + self.row_index as u64
    }

    #[inline]
    pub fn span_y(&self) -> f64 {
        self.span_x * self.height as f64 / self.width as f64
    }

    #[inline]
    pub fn pixel_real(&self, x: usize) -> f64 {
        let width = self.width.max(2) as f64;
        let normalized = x as f64 / (width - 1.0);
        self.center_x + (normalized - 0.5) * self.span_x
    }

    #[inline]
    pub fn row_imag(&self) -> f64 {
        let height = self.height.max(2) as f64;
        let normalized = self.row_index as f64 / (height - 1.0);
        self.center_y + (normalized - 0.5) * self.span_y()
    }

    #[inline]
    pub fn row_rgb_len(&self) -> usize {
        self.width as usize * 3
    }
}

impl Default for MandelbrotRow {
    fn default() -> Self {
        Self {
            frame_index: 0,
            row_index: 0,
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

impl Encode for MandelbrotRow {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.frame_index, encoder)?;
        Encode::encode(&self.row_index, encoder)?;
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

impl Decode<()> for MandelbrotRow {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            frame_index: Decode::decode(decoder)?,
            row_index: Decode::decode(decoder)?,
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

impl Serialize for MandelbrotRow {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        use serde::ser::SerializeStruct;

        let mut state = serializer.serialize_struct("MandelbrotRow", 13)?;
        state.serialize_field("frame_index", &self.frame_index)?;
        state.serialize_field("row_index", &self.row_index)?;
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

impl<'de> Deserialize<'de> for MandelbrotRow {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct MandelbrotRowWire {
            frame_index: u32,
            row_index: u32,
            width: u32,
            height: u32,
            center_x: f64,
            center_y: f64,
            span_x: f64,
            max_iter: u16,
            completed_iters: u16,
            z_re: Vec<f64>,
            z_im: Vec<f64>,
            escape_iter: Vec<u16>,
            pixels_rgb: Vec<u8>,
        }

        let wire = MandelbrotRowWire::deserialize(deserializer)?;
        Ok(Self {
            frame_index: wire.frame_index,
            row_index: wire.row_index,
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
