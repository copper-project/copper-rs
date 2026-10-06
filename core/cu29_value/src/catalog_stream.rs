//! Borrowed catalog descriptions serialized once at startup with fixed working memory.

use crate::catalog_header::{
    HEADER_LEN, MAGIC, VALUE_DECODE_CATALOG_MAX_BYTES, VERSION, ValueDecodeCatalogLayout,
};
use bincode::Encode;
use bincode::ValueDecodeSpec;
use bincode::config::standard;
use bincode::enc::Encoder;
use bincode::enc::EncoderImpl;
use bincode::enc::write::Writer;
use bincode::error::EncodeError;
use bincode::value_decode::{FieldSelector, ValueDecodeField, ValueDecodeRef};
use heatshrink::{Finish, Poll, SinkError};

// One function pointer per distinct native type; identities are never stored in the log.
const MAX_TYPES: usize = 256;
type Compressor = heatshrink::encoder::HeatshrinkEncoder<10, 5, 2048>;

/// A generated output position and its optional captured native wire description.
pub struct CatalogSlot {
    pub task_id: &'static str,
    pub msg_type: &'static str,
    pub payload: Option<ValueDecodeRef>,
}

/// Generated graph metadata; all strings and wire recipes are borrowed from the image.
pub struct CatalogDescription {
    pub mission: &'static str,
    pub config_ron: &'static str,
    pub layout: ValueDecodeCatalogLayout,
    pub slots: &'static [CatalogSlot],
}

struct Types {
    entries: [Option<ValueDecodeRef>; MAX_TYPES],
    len: usize,
}

impl Types {
    fn id(&self, reference: ValueDecodeRef) -> Option<usize> {
        let identity = (reference.describe)().type_id;
        self.entries[..self.len]
            .iter()
            .position(|entry| entry.is_some_and(|entry| (entry.describe)().type_id == identity))
    }

    fn add(&mut self, reference: ValueDecodeRef) -> Result<(), EncodeError> {
        if self.id(reference).is_none() {
            if self.len == MAX_TYPES {
                return Err(EncodeError::Other(
                    "Catalog exceeds 256 distinct payload types",
                ));
            }
            self.entries[self.len] = Some(reference);
            self.len += 1;
        }
        Ok(())
    }

    fn collect(catalog: &CatalogDescription) -> Result<Self, EncodeError> {
        let mut types = Self {
            entries: [None; MAX_TYPES],
            len: 0,
        };
        for slot in catalog.slots {
            if let Some(payload) = slot.payload {
                types.add(payload)?;
            }
        }
        if types.len == 0 {
            types.add(ValueDecodeRef::of::<()>())?;
        }
        let mut index = 0;
        while index < types.len {
            let reference =
                types.entries[index].ok_or(EncodeError::Other("Missing catalog type"))?;
            match (reference.describe)().spec {
                ValueDecodeSpec::Record { fields, .. } => {
                    for field in *fields {
                        types.add(field.value)?;
                    }
                }
                ValueDecodeSpec::Enum { variants, .. } => {
                    for variant in *variants {
                        for field in variant.fields {
                            types.add(field.value)?;
                        }
                    }
                }
                ValueDecodeSpec::Array { element, .. }
                | ValueDecodeSpec::Sequence { element, .. }
                | ValueDecodeSpec::Option(element)
                | ValueDecodeSpec::Delegate(element) => types.add(*element)?,
                ValueDecodeSpec::Map { key, value } => {
                    types.add(*key)?;
                    types.add(*value)?;
                }
                _ => {}
            }
            index += 1;
        }
        Ok(types)
    }

    fn reference<E: Encoder>(
        &self,
        reference: ValueDecodeRef,
        encoder: &mut E,
    ) -> Result<(), EncodeError> {
        self.id(reference)
            .ok_or(EncodeError::Other("Missing catalog child type"))?
            .encode(encoder)
    }

    fn operation_fields<E: Encoder>(
        &self,
        fields: &[ValueDecodeField],
        encoder: &mut E,
    ) -> Result<(), EncodeError> {
        fields.len().encode(encoder)?;
        for field in fields {
            self.reference(field.value, encoder)?;
        }
        Ok(())
    }

    fn schema_fields<E: Encoder>(
        &self,
        fields: &[ValueDecodeField],
        encoder: &mut E,
    ) -> Result<(), EncodeError> {
        fields.len().encode(encoder)?;
        for field in fields {
            let name = match field.selector {
                FieldSelector::Named(name) => Some(name),
                FieldSelector::Index { .. } => None,
            };
            name.encode(encoder)?;
            field.declaration_index.encode(encoder)?;
            self.reference(field.value, encoder)?;
        }
        Ok(())
    }

    fn operation<E: Encoder>(
        &self,
        spec: &ValueDecodeSpec,
        encoder: &mut E,
    ) -> Result<(), EncodeError> {
        match spec {
            ValueDecodeSpec::Unit => 0u32.encode(encoder),
            ValueDecodeSpec::Scalar(scalar) => {
                1u32.encode(encoder)?;
                scalar.encode(encoder)
            }
            ValueDecodeSpec::String => 2u32.encode(encoder),
            ValueDecodeSpec::Bytes => 3u32.encode(encoder),
            ValueDecodeSpec::Record { shape, fields } => {
                4u32.encode(encoder)?;
                shape.encode(encoder)?;
                self.operation_fields(fields, encoder)
            }
            ValueDecodeSpec::Array { element, len } => {
                5u32.encode(encoder)?;
                self.reference(*element, encoder)?;
                len.encode(encoder)
            }
            ValueDecodeSpec::Sequence {
                element,
                count,
                capacity,
            } => {
                6u32.encode(encoder)?;
                self.reference(*element, encoder)?;
                count.encode(encoder)?;
                capacity.encode(encoder)
            }
            ValueDecodeSpec::Map { key, value } => {
                7u32.encode(encoder)?;
                self.reference(*key, encoder)?;
                self.reference(*value, encoder)
            }
            ValueDecodeSpec::Option(element) => {
                8u32.encode(encoder)?;
                self.reference(*element, encoder)
            }
            ValueDecodeSpec::Delegate(element) => {
                9u32.encode(encoder)?;
                self.reference(*element, encoder)
            }
            ValueDecodeSpec::Enum { tag, variants } => {
                10u32.encode(encoder)?;
                tag.encode(encoder)?;
                variants.len().encode(encoder)?;
                for variant in *variants {
                    variant.tag.encode(encoder)?;
                    variant.shape.encode(encoder)?;
                    self.operation_fields(variant.fields, encoder)?;
                }
                Ok(())
            }
        }
    }

    fn encode<E: Encoder>(
        &self,
        catalog: &CatalogDescription,
        encoder: &mut E,
    ) -> Result<(), EncodeError> {
        // Preserve the owned ValueDecodeCatalog body layout. Allocation and wire-op
        // deduplication are unnecessary for producing its indexed tables.
        catalog.mission.encode(encoder)?;
        catalog.config_ron.encode(encoder)?;
        catalog.layout.encode(encoder)?;
        0usize.encode(encoder)?; // first root binding in ValueDecodeDescription
        self.len.encode(encoder)?;
        for index in 0..self.len {
            index.encode(encoder)?;
            index.encode(encoder)?;
        }
        self.len.encode(encoder)?;
        for reference in self.entries[..self.len].iter().flatten() {
            self.operation((reference.describe)().spec, encoder)?;
        }
        self.len.encode(encoder)?;
        for reference in self.entries[..self.len].iter().flatten() {
            let ty = (reference.describe)();
            ty.type_name.encode(encoder)?;
            ty.metadata.encode(encoder)?;
            match ty.spec {
                ValueDecodeSpec::Record { fields, .. } => {
                    self.schema_fields(fields, encoder)?;
                    0usize.encode(encoder)?;
                }
                ValueDecodeSpec::Enum { variants, .. } => {
                    0usize.encode(encoder)?;
                    variants.len().encode(encoder)?;
                    for variant in *variants {
                        variant.name.encode(encoder)?;
                        self.schema_fields(variant.fields, encoder)?;
                    }
                }
                _ => {
                    0usize.encode(encoder)?;
                    0usize.encode(encoder)?;
                }
            }
        }
        catalog.slots.len().encode(encoder)?;
        for slot in catalog.slots {
            slot.task_id.encode(encoder)?;
            slot.msg_type.encode(encoder)?;
            let binding = slot.payload.and_then(|payload| self.id(payload));
            binding.encode(encoder)?;
        }
        Ok(())
    }
}

/// Incremental IEEE CRC-32 used by the streaming catalog footer.
pub(crate) fn crc32(mut crc: u32, bytes: &[u8]) -> u32 {
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb88320 & 0u32.wrapping_sub(crc & 1));
        }
    }
    crc
}

pub(crate) struct CompressedWriter<W: Writer> {
    compressor: Compressor,
    output: W,
    crc: u32,
    raw_len: usize,
}
impl<W: Writer> CompressedWriter<W> {
    pub(crate) fn new(mut output: W) -> Result<Self, EncodeError> {
        let mut header = [0; HEADER_LEN];
        header[..8].copy_from_slice(MAGIC);
        header[8..].copy_from_slice(&VERSION.to_le_bytes());
        output.write(&header)?;
        Ok(Self {
            compressor: Compressor::new(),
            output,
            crc: u32::MAX,
            raw_len: 0,
        })
    }

    fn drain(&mut self) -> Result<(), EncodeError> {
        let mut buffer = [0; 128];
        loop {
            let result = self
                .compressor
                .poll(&mut buffer)
                .map_err(|_| EncodeError::Other("Catalog compressor failed"))?;
            self.output.write(&buffer[..result.bytes_written()])?;
            if matches!(result, Poll::Empty(_)) {
                return Ok(());
            }
        }
    }
    pub(crate) fn finish(mut self) -> Result<(), EncodeError> {
        while self.compressor.finish() != Finish::Done {
            self.drain()?;
        }
        self.output.write(&(self.raw_len as u32).to_le_bytes())?;
        self.output.write(&(!self.crc).to_le_bytes())
    }
}
impl<W: Writer> Writer for CompressedWriter<W> {
    fn write(&mut self, mut bytes: &[u8]) -> Result<(), EncodeError> {
        self.raw_len = self
            .raw_len
            .checked_add(bytes.len())
            .ok_or(EncodeError::Other("Catalog size overflow"))?;
        if self.raw_len > VALUE_DECODE_CATALOG_MAX_BYTES {
            return Err(EncodeError::Other("Catalog exceeds 16 MiB"));
        }
        self.crc = crc32(self.crc, bytes);
        while !bytes.is_empty() {
            match self.compressor.sink(bytes) {
                Ok(consumed) => bytes = &bytes[consumed..],
                Err(SinkError::Full) => {}
                Err(SinkError::Misuse) => {
                    return Err(EncodeError::Other("Catalog compressor sink failed"));
                }
            }
            self.drain()?;
        }
        Ok(())
    }
}

/// Validate reachable types, then stream the compressed catalog without heap allocation.
/// The working set consists of 256 type references and a 2 KiB compressor window.
pub fn write_catalog<W: Writer>(
    output: W,
    catalog: &CatalogDescription,
) -> Result<(), EncodeError> {
    let types = Types::collect(catalog)?;
    let writer = CompressedWriter::new(output)?;
    let mut encoder = EncoderImpl::new(writer, standard());
    types.encode(catalog, &mut encoder)?;
    encoder.into_writer().finish()
}
