//! Reader-only migration of the V1/V2 string metadata catalog layout.

use crate::catalog::ValueDecodeCatalog;
use crate::catalog::ValueDecodeCatalogSlot;
use crate::catalog_header::ValueDecodeCatalogLayout;
use crate::decode::{
    ValueDecodeBinding, ValueDecodeDescription, ValueDecodeOp, ValueDecodeSchema,
    ValueDecodeSchemaField, ValueDecodeSchemaVariant,
};
use crate::metadata::ValueDecodeMetadata;
use alloc::string::String;
use alloc::vec::Vec;
use bincode::Decode;
use bincode::Encode;
use bincode::error::DecodeError;

#[derive(Encode, Decode)]
pub(crate) struct LegacyCatalog {
    mission: String,
    config_ron: String,
    layout: ValueDecodeCatalogLayout,
    description: LegacyDescription,
    slots: Vec<ValueDecodeCatalogSlot>,
}
#[derive(Encode, Decode)]
struct LegacyDescription {
    root: usize,
    bindings: Vec<ValueDecodeBinding>,
    operations: Vec<ValueDecodeOp>,
    schemas: Vec<LegacySchema>,
}
#[derive(Encode, Decode)]
struct LegacySchema {
    type_path: String,
    quantity: Option<LegacyQuantity>,
    fields: Vec<ValueDecodeSchemaField>,
    variants: Vec<ValueDecodeSchemaVariant>,
}
#[derive(Encode, Decode)]
struct LegacyQuantity {
    quantity: String,
    storage_unit: String,
}

impl TryFrom<LegacyCatalog> for ValueDecodeCatalog {
    type Error = DecodeError;
    fn try_from(old: LegacyCatalog) -> Result<Self, Self::Error> {
        let schemas = old
            .description
            .schemas
            .into_iter()
            .map(|schema| {
                let metadata = schema
                    .quantity
                    .map(|quantity| {
                        ValueDecodeMetadata::from_legacy_quantity(
                            quantity.quantity,
                            quantity.storage_unit,
                        )
                    })
                    .transpose()?
                    .into_iter()
                    .collect();
                Ok(ValueDecodeSchema {
                    type_path: schema.type_path,
                    metadata,
                    fields: schema.fields,
                    variants: schema.variants,
                })
            })
            .collect::<Result<Vec<_>, DecodeError>>()?;
        Ok(Self {
            mission: old.mission,
            config_ron: old.config_ron,
            layout: old.layout,
            slots: old.slots,
            description: ValueDecodeDescription {
                root: old.description.root,
                bindings: old.description.bindings,
                operations: old.description.operations,
                schemas,
            },
        })
    }
}
