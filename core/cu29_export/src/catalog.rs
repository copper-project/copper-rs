//! Offline, version-pinned CopperList decoding from embedded V1 catalogs.
//!
//! The catalog version freezes both payload and envelope rules. These readers
//! deliberately decode the V1 wire fields explicitly, independently of the
//! application's generated tuple and the extractor's encoding features.

use crate::runs;
use bincode::Decode;
use bincode::Uleb128;
use cu29::prelude::{
    CuError, CuMsgMetadata, CuMsgOrigin, CuResult, CuTime, CuTimeRange, OptionCuTime,
    PartialCuTimeRange, Tov, UnifiedLogType, Value, ValueDecodeCatalog, ValueDecodeCatalogLayout,
    ValueDecodeLimits,
};
use cu29::value_decode::ValueDecodeBudget;
use std::path::Path;

const MAX_COPPERLIST_BYTES: usize = 16 * 1024 * 1024;
const MAX_SLOTS: usize = 65_536;

/// One recorded slot decoded without its native payload type. Experimental API.
#[derive(Debug, serde::Serialize)]
pub struct CuDecodedLogSlot {
    /// Task or bridge/channel identity in catalog order.
    pub task_id: String,
    /// Original presence, or unknown when Flat encoding suppressed the payload.
    pub original_payload_present: Option<bool>,
    /// Whether this record contains payload bytes.
    pub captured_payload_present: bool,
    /// Decoded payload; its catalog schema retains scalar widths and storage units.
    #[serde(serialize_with = "crate::value_export::serialize_payload")]
    pub payload: Option<Value>,
    /// Time of validity.
    pub tov: Tov,
    /// Process times, status and remote provenance.
    pub metadata: CuMsgMetadata,
}

/// A CopperList decoded to offline values. Experimental API.
#[derive(Debug, serde::Serialize)]
pub struct CuDecodedCopperList {
    /// Recorded cycle identifier.
    pub id: u64,
    /// Slots in native encoding order, including absent and uncaptured positions.
    pub msgs: Vec<CuDecodedLogSlot>,
}

/// Load and validate the selected run's catalog. Experimental API.
/// Multi-run logs require a zero-based `run` selection.
pub fn read_value_decode_catalog(path: &Path, run: Option<usize>) -> CuResult<ValueDecodeCatalog> {
    let recorded = runs::discover(path)?;
    let selected = runs::select(&recorded, run)?;
    load_catalog(selected, path)
}

fn configs_match(recorded: &str, described: &str) -> CuResult<bool> {
    if recorded == described {
        return Ok(true);
    }
    fn document(source: &str) -> CuResult<serde_json::Value> {
        let config = cu29::config::CuConfig::deserialize_ron(source)?;
        let mut document = serde_json::to_value(config).map_err(|error| {
            CuError::new_with_cause("Could not compare catalog configuration", error)
        })?;
        // Mission maps and their merged task lists can serialize in different
        // orders. Log slots are ordered explicitly by the catalog, while these
        // declarations identify the same configured nodes by id.
        let merged_missions = document
            .get("missions")
            .is_some_and(serde_json::Value::is_array);
        for field in ["tasks", "missions"] {
            if !merged_missions {
                continue;
            }
            if let Some(values) = document
                .get_mut(field)
                .and_then(serde_json::Value::as_array_mut)
            {
                values.sort_by(|left, right| left["id"].as_str().cmp(&right["id"].as_str()));
            }
        }
        for field in ["tasks", "cnx", "bridges", "resources", "monitors"] {
            if let Some(declarations) = document
                .get_mut(field)
                .and_then(serde_json::Value::as_array_mut)
            {
                for declaration in declarations {
                    if let Some(missions) = declaration
                        .get_mut("missions")
                        .and_then(serde_json::Value::as_array_mut)
                    {
                        missions.sort_by(|left, right| left.as_str().cmp(&right.as_str()));
                    }
                }
            }
        }
        Ok(document)
    }
    Ok(document(recorded)? == document(described)?)
}

pub(crate) fn load_catalog(run: &runs::RecordedRun, path: &Path) -> CuResult<ValueDecodeCatalog> {
    let mut reader = run.reader(path)?;
    let mut found = None;
    while let Some((position, bytes)) = reader
        .read_next_section_type_at(UnifiedLogType::ValueDecodeCatalog)
        .map_err(|error| CuError::from(format!("Run {} catalog discovery: {error}", run.index)))?
    {
        if found.is_some() {
            return Err(CuError::from(format!(
                "Run {} contains multiple ValueDecodeCatalog sections",
                run.index
            )));
        }
        let catalog = ValueDecodeCatalog::from_blob(&bytes).map_err(|error| {
            CuError::from(format!(
                "Run {} catalog at slab {} offset {}: {error}",
                run.index, position.slab_index, position.offset
            ))
        })?;
        if catalog.slots.len() > MAX_SLOTS {
            return Err("Catalog exceeds offline slot limit".into());
        }
        if let Some(config) = &run.config
            && !configs_match(config, &catalog.config_ron)?
        {
            return Err("Catalog configuration does not match the selected run".into());
        }
        if !run.missions.is_empty() && !run.missions.contains(&catalog.mission) {
            return Err("Catalog mission does not match the selected run".into());
        }
        found = Some(catalog);
    }
    found.ok_or_else(|| CuError::from(format!("Run {} has no ValueDecodeCatalog; catalog decoding and deep validation require a catalog", run.index)))
}

/// A fallible iterator that stops after the first malformed record. Experimental API.
pub struct CopperListValueReader {
    catalog: ValueDecodeCatalog,
    reader: runs::RunReader,
    run: usize,
    bytes: Vec<u8>,
    offset: usize,
    position: cu29::prelude::memmap::LogPosition,
    finished: bool,
}

/// Read the selected run's CopperLists using its embedded catalog. Experimental API.
/// No application decoder registration is needed; truncated records return errors.
pub fn copperlist_values_reader(
    path: &Path,
    run: Option<usize>,
) -> CuResult<CopperListValueReader> {
    let recorded = runs::discover(path)?;
    let selected = runs::select(&recorded, run)?;
    let catalog = load_catalog(selected, path)?;
    let reader = selected.reader(path)?;
    let position = reader.position();
    Ok(CopperListValueReader {
        catalog,
        reader,
        run: selected.index,
        bytes: Vec::new(),
        offset: 0,
        position,
        finished: false,
    })
}

impl CopperListValueReader {
    pub(crate) fn catalog(&self) -> &ValueDecodeCatalog {
        &self.catalog
    }
}

impl Iterator for CopperListValueReader {
    type Item = CuResult<CuDecodedCopperList>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }
        let result = (|| {
            while self.offset == self.bytes.len() {
                let Some((position, bytes)) = self
                    .reader
                    .read_next_section_type_at(UnifiedLogType::CopperList)?
                else {
                    self.finished = true;
                    return Ok(None);
                };
                self.position = position;
                self.bytes = bytes;
                self.offset = 0;
            }
            let (entry, used) = decode_copperlist(&self.catalog, &self.bytes[self.offset..])
                .map_err(|error| {
                    CuError::from(format!(
                        "Run {} slab {} section offset {} record byte {}: {error}",
                        self.run, self.position.slab_index, self.position.offset, self.offset
                    ))
                })?;
            self.offset += used;
            Ok(Some(entry))
        })();
        match result {
            Ok(entry) => entry.map(Ok),
            Err(error) => {
                self.finished = true;
                Some(Err(error))
            }
        }
    }
}

struct Cursor<'a> {
    bytes: &'a [u8],
    offset: usize,
}
impl Cursor<'_> {
    fn read<T: Decode<()>>(&mut self) -> CuResult<T> {
        let (value, used) = bincode::decode_from_slice(
            &self.bytes[self.offset..],
            bincode::config::standard().with_limit::<MAX_COPPERLIST_BYTES>(),
        )
        .map_err(|error| CuError::new_with_cause("Invalid V1 CopperList envelope", error))?;
        self.offset += used;
        Ok(value)
    }
    fn plane(&mut self, count: usize) -> CuResult<Vec<bool>> {
        match self.read::<u8>()? {
            0 => Ok(vec![false; count]),
            1 => Ok(vec![true; count]),
            2 => {
                let mut result = vec![false; count];
                for chunk in result.chunks_mut(8) {
                    let byte = self.read::<u8>()?;
                    for (bit, flag) in chunk.iter_mut().enumerate() {
                        *flag = byte & (1 << bit) != 0;
                    }
                    if chunk.len() < 8 && byte >> chunk.len() != 0 {
                        return Err("CopperList presence bitmap has nonzero padding".into());
                    }
                }
                Ok(result)
            }
            _ => Err("Invalid V1 CopperList presence-plane mode".into()),
        }
    }
    fn timestamp(&mut self, anchor: CuTime) -> CuResult<CuTime> {
        let delta = self.read::<Uleb128<i128>>()?.0;
        let nanos = i128::from(anchor.as_nanos())
            .checked_add(delta)
            .and_then(|value| u64::try_from(value).ok())
            .filter(|value| *value <= CuTime::MAX.as_nanos())
            .ok_or(CuError::from("CopperList timestamp outside CuTime range"))?;
        Ok(CuTime(nanos))
    }
    fn time(&mut self) -> CuResult<CuTime> {
        let nanos = self.read::<u64>()?;
        if nanos > CuTime::MAX.as_nanos() {
            return Err("CopperList timestamp outside CuTime range".into());
        }
        Ok(CuTime(nanos))
    }
    fn optional_time(&mut self) -> CuResult<OptionCuTime> {
        let nanos = self.read::<u64>()?;
        if nanos == u64::MAX {
            Ok(OptionCuTime::none())
        } else if nanos <= CuTime::MAX.as_nanos() {
            Ok(CuTime(nanos).into())
        } else {
            Err("CopperList timestamp outside CuTime range".into())
        }
    }
    fn status(&mut self) -> CuResult<cu29::prelude::CuCompactString> {
        let length = usize::try_from(self.read::<u64>()?)
            .map_err(|_| CuError::from("Status length overflow"))?;
        if length > ValueDecodeLimits::default().max_collection_len {
            return Err("CopperList status exceeds offline limit".into());
        }
        let end = self
            .offset
            .checked_add(length)
            .ok_or(CuError::from("Status length overflow"))?;
        let bytes = self
            .bytes
            .get(self.offset..end)
            .ok_or(CuError::from("Truncated CopperList status"))?;
        let status = std::str::from_utf8(bytes)
            .map_err(|error| CuError::new_with_cause("Invalid CopperList status UTF-8", error))?;
        self.offset = end;
        Ok(cu29::prelude::CuCompactString(status.into()))
    }
    fn origin(&mut self) -> CuResult<CuMsgOrigin> {
        Ok(CuMsgOrigin {
            subsystem_code: self.read()?,
            instance_id: self.read()?,
            cl_id: self.read()?,
        })
    }
}

pub(crate) fn decode_copperlist(
    catalog: &ValueDecodeCatalog,
    bytes: &[u8],
) -> CuResult<(CuDecodedCopperList, usize)> {
    decode_copperlist_with_payload_sizes(catalog, bytes, |_| {})
}

pub(crate) fn decode_copperlist_with_payload_sizes(
    catalog: &ValueDecodeCatalog,
    bytes: &[u8],
    mut payload_size: impl FnMut(usize),
) -> CuResult<(CuDecodedCopperList, usize)> {
    if catalog.slots.len() > MAX_SLOTS {
        return Err("Catalog exceeds offline slot limit".into());
    }
    let mut cursor = Cursor {
        bytes: &bytes[..bytes.len().min(MAX_COPPERLIST_BYTES)],
        offset: 0,
    };
    let id = cursor.read::<u64>()?;
    let mut budget = ValueDecodeBudget::new(ValueDecodeLimits::default());
    let mut msgs = catalog
        .slots
        .iter()
        .map(|slot| CuDecodedLogSlot {
            task_id: slot.task_id.clone(),
            original_payload_present: None,
            captured_payload_present: false,
            payload: None,
            tov: Tov::None,
            metadata: CuMsgMetadata::default(),
        })
        .collect::<Vec<_>>();
    if catalog.layout == ValueDecodeCatalogLayout::Compact {
        compact_metadata(&mut cursor, &mut msgs).map_err(|error| {
            CuError::from(format!(
                "CopperList #{id} metadata byte {}: {error}",
                cursor.offset
            ))
        })?;
    }
    for (index, (slot, msg)) in catalog.slots.iter().zip(&mut msgs).enumerate() {
        let result = (|| {
            if catalog.layout == ValueDecodeCatalogLayout::Flat {
                msg.captured_payload_present = match cursor.read::<u8>()? {
                    0 => false,
                    1 => true,
                    _ => return Err("Invalid Flat payload presence tag".into()),
                };
                msg.original_payload_present = msg.captured_payload_present.then_some(true);
            }
            if msg.captured_payload_present {
                let binding = slot
                    .binding
                    .ok_or(CuError::from("Captured slot has no payload description"))?;
                let (value, used) = catalog
                    .description
                    .decode_at_with_budget(
                        binding,
                        &cursor.bytes[cursor.offset..],
                        bincode::config::standard(),
                        ValueDecodeLimits::default(),
                        &mut budget,
                    )
                    .map_err(|error| CuError::new_with_cause("Invalid captured payload", error))?;
                cursor.offset += used;
                msg.payload = Some(value);
                payload_size(used);
            }
            if catalog.layout == ValueDecodeCatalogLayout::Flat {
                msg.tov = match cursor.read::<u32>()? {
                    0 => Tov::None,
                    1 => Tov::Time(cursor.time()?),
                    2 => Tov::Range(CuTimeRange {
                        start: cursor.time()?,
                        end: cursor.time()?,
                    }),
                    _ => return Err("Invalid V1 TOV tag".into()),
                };
                msg.metadata.process_time = PartialCuTimeRange {
                    start: cursor.optional_time()?,
                    end: cursor.optional_time()?,
                };
                msg.metadata.status_txt = cursor.status()?;
                msg.metadata.origin = match cursor.read::<u8>()? {
                    0 => None,
                    1 => Some(cursor.origin()?),
                    _ => return Err("Invalid V1 origin presence tag".into()),
                };
            }
            Ok(())
        })();
        result.map_err(|error: CuError| {
            CuError::from(format!(
                "CopperList #{id} slot {index} ({}) byte {}: {error}",
                slot.task_id, cursor.offset
            ))
        })?;
    }
    Ok((CuDecodedCopperList { id, msgs }, cursor.offset))
}

fn compact_metadata(cursor: &mut Cursor<'_>, msgs: &mut [CuDecodedLogSlot]) -> CuResult<()> {
    let count = msgs.len();
    let tov = cursor.plane(count)?;
    let range = cursor.plane(count)?;
    let start = cursor.plane(count)?;
    let end = cursor.plane(count)?;
    let status = cursor.plane(count)?;
    let origin = cursor.plane(count)?;
    let original = cursor.plane(count)?;
    let captured = cursor.plane(count)?;
    if range
        .iter()
        .zip(&tov)
        .any(|(range, present)| *range && !present)
    {
        return Err("TOV range marked without TOV presence".into());
    }
    if captured
        .iter()
        .zip(&original)
        .any(|(captured, present)| *captured && !present)
    {
        return Err("Captured payload absent from original view".into());
    }
    let has_time = tov.iter().chain(&start).chain(&end).any(|present| *present);
    let base = match cursor.read::<u8>()? {
        0 if !has_time => None,
        1 if has_time => {
            let mut raw = [0u8; 8];
            for byte in &mut raw {
                *byte = cursor.read()?;
            }
            let nanos = u64::from_le_bytes(raw);
            if nanos > CuTime::MAX.as_nanos() {
                return Err("CopperList timestamp base outside CuTime range".into());
            }
            Some(CuTime(nanos))
        }
        _ => return Err("CopperList timestamp-base tag disagrees with timestamp planes".into()),
    };
    if let Some(base) = base {
        for (index, msg) in msgs.iter_mut().enumerate() {
            if tov[index] {
                let time = cursor.timestamp(base)?;
                msg.tov = if range[index] {
                    Tov::Range(CuTimeRange {
                        start: time,
                        end: cursor.timestamp(time)?,
                    })
                } else {
                    Tov::Time(time)
                };
            }
        }
        for (index, msg) in msgs.iter_mut().enumerate() {
            let first = if start[index] {
                Some(cursor.timestamp(base)?)
            } else {
                None
            };
            let last = if end[index] {
                Some(cursor.timestamp(first.unwrap_or(base))?)
            } else {
                None
            };
            msg.metadata.process_time = PartialCuTimeRange {
                start: first.into(),
                end: last.into(),
            };
        }
    }
    let mut status_bytes = MAX_COPPERLIST_BYTES;
    for index in 0..count {
        if status[index] {
            let reference = usize::try_from(cursor.read::<Uleb128<u128>>()?.0)
                .map_err(|_| CuError::from("Status reference overflow"))?;
            msgs[index].metadata.status_txt = if reference == 0 {
                cursor.status()?
            } else if reference <= index && status[reference - 1] {
                {
                    let length = msgs[reference - 1].metadata.status_txt.0.len();
                    status_bytes = status_bytes
                        .checked_sub(length)
                        .ok_or(CuError::from("CopperList status allocation limit exceeded"))?;
                    msgs[reference - 1].metadata.status_txt.clone()
                }
            } else {
                return Err("Invalid status backreference".into());
            };
        }
    }
    for index in 0..count {
        if origin[index] {
            let reference = usize::try_from(cursor.read::<Uleb128<u128>>()?.0)
                .map_err(|_| CuError::from("Origin reference overflow"))?;
            msgs[index].metadata.origin = if reference == 0 {
                Some(cursor.origin()?)
            } else if reference <= index && origin[reference - 1] {
                msgs[reference - 1].metadata.origin.clone()
            } else {
                return Err("Invalid origin backreference".into());
            };
        }
        msgs[index].original_payload_present = Some(original[index]);
        msgs[index].captured_payload_present = captured[index];
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::Encode;
    use cu29::prelude::*;
    use cu29::value_decode::ValueDecodeOp;

    #[test]
    fn test_catalog_config_comparison_accepts_mission_map_order_and_rejects_changes() {
        let first = r#"(
            missions: [(id: "default"), (id: "flow")],
            tasks: [(id: "source", type: "Source", config: {"threshold": 1, "rate": 2, "missions": ["first", "second"]}), (id: "sink", type: "Sink")],
            cnx: [(src: "source", dst: "sink", msg: "u32")],
        )"#;
        let reordered = r#"(
            missions: [(id: "flow"), (id: "default")],
            tasks: [(id: "sink", type: "Sink"), (id: "source", type: "Source", config: {"rate": 2, "threshold": 1, "missions": ["first", "second"]})],
            cnx: [(src: "source", dst: "sink", msg: "u32")],
        )"#;
        assert!(configs_match(first, reordered).unwrap());
        assert!(!configs_match(first, &reordered.replace("u32", "u64")).unwrap());
        assert!(
            !configs_match(
                first,
                &reordered.replace("threshold\": 1", "threshold\": 3")
            )
            .unwrap()
        );
        assert!(
            !configs_match(
                first,
                &reordered.replace(r#"["first", "second"]"#, r#"["second", "first"]"#)
            )
            .unwrap()
        );
        assert!(configs_match(first, "invalid config").is_err());

        let scoped = first
            .replace(
                "type: \"Sink\"",
                "type: \"Sink\", missions: [\"default\", \"flow\"]",
            )
            .replace("msg: \"u32\"", "msg: \"u32\", missions: [\"default\"]");
        let scoped_reordered = scoped.replace(
            "missions: [\"default\", \"flow\"]",
            "missions: [\"flow\", \"default\"]",
        );
        assert!(configs_match(&scoped, &scoped_reordered).unwrap());
        let changed_membership = scoped.replace(
            "missions: [\"default\", \"flow\"]",
            "missions: [\"default\"]",
        );
        assert!(!configs_match(&scoped, &changed_membership).unwrap());

        // A plain graph's task declaration order remains significant.
        let plain_first = first.replace("missions: [(id: \"default\"), (id: \"flow\")],", "");
        let plain_reordered =
            reordered.replace("missions: [(id: \"flow\"), (id: \"default\")],", "");
        assert!(!configs_match(&plain_first, &plain_reordered).unwrap());
    }

    fn catalog(layout: ValueDecodeCatalogLayout) -> ValueDecodeCatalog {
        let mut builder = ValueDecodeCatalogBuilder::default();
        builder.add::<u32>("first", "u32");
        builder.add::<u32>("second", "u32");
        builder.add_uncaptured("hidden", "Opaque");
        builder.finish("()", "default", layout).unwrap()
    }

    fn messages() -> [CuMsg<u32>; 3] {
        let mut first = CuMsg::<u32>::default();
        first.set_payload(300);
        first.tov = Tov::Range(CuTimeRange {
            start: CuTime(20),
            end: CuTime(23),
        });
        first.metadata.process_time = PartialCuTimeRange {
            start: CuTime(10).into(),
            end: CuTime(15).into(),
        };
        first.metadata.set_status("ready");
        first.metadata.set_origin(CuMsgOrigin {
            subsystem_code: 7,
            instance_id: 13,
            cl_id: 41,
        });
        let mut second = first.clone();
        second.set_payload(42);
        let mut hidden = CuMsg::<u32>::default();
        hidden.tov = Tov::Time(CuTime(5));
        [first, second, hidden]
    }

    fn compact_bytes() -> Vec<u8> {
        struct Compact([CuMsg<u32>; 3]);
        impl Encode for Compact {
            fn encode<E: bincode::enc::Encoder>(
                &self,
                encoder: &mut E,
            ) -> Result<(), bincode::error::EncodeError> {
                Encode::encode(&19_u64, encoder)?;
                let refs = std::array::from_fn(|i| {
                    cu29::copperlist_codec::CommonMetadataRef::new(
                        &self.0[i].tov,
                        &self.0[i].metadata,
                    )
                });
                cu29::copperlist_codec::encode_common_metadata(
                    &refs,
                    &[true, true, true],
                    &[true, true, false],
                    encoder,
                )?;
                Encode::encode(&300_u32, encoder)?;
                Encode::encode(&42_u32, encoder)
            }
        }
        bincode::encode_to_vec(Compact(messages()), bincode::config::standard()).unwrap()
    }

    #[test]
    fn test_compact_native_agreement_and_consecutive_records() {
        let catalog = catalog(ValueDecodeCatalogLayout::Compact);
        let bytes = compact_bytes();
        let mut consecutive = bytes.clone();
        consecutive.extend_from_slice(&bytes);
        let mut payload_sizes = Vec::new();
        let (entry, used) = decode_copperlist_with_payload_sizes(&catalog, &consecutive, |size| {
            payload_sizes.push(size);
        })
        .unwrap();
        assert_eq!(payload_sizes, vec![3, 1]);
        assert_eq!(used, bytes.len());
        assert_eq!(entry.id, 19);
        assert_eq!(entry.msgs[0].payload, Some(Value::U32(300)));
        assert_eq!(entry.msgs[1].payload, Some(Value::U32(42)));
        assert_eq!(entry.msgs[2].original_payload_present, Some(true));
        assert!(!entry.msgs[2].captured_payload_present);
        for (actual, expected) in entry.msgs.iter().zip(messages()) {
            assert_eq!(actual.tov, expected.tov);
            assert_eq!(actual.metadata.status_txt, expected.metadata.status_txt);
            assert_eq!(actual.metadata.origin, expected.metadata.origin);
            assert_eq!(
                actual.metadata.process_time.start,
                expected.metadata.process_time.start
            );
            assert_eq!(
                actual.metadata.process_time.end,
                expected.metadata.process_time.end
            );
        }
        assert_eq!(
            decode_copperlist(&catalog, &consecutive[used..]).unwrap().1,
            used
        );
        for len in 0..bytes.len() {
            assert!(
                decode_copperlist(&catalog, &bytes[..len]).is_err(),
                "length {len}"
            );
        }
    }

    #[test]
    fn test_flat_native_agreement_and_suppressed_presence() {
        let catalog = catalog(ValueDecodeCatalogLayout::Flat);
        let mut bytes = bincode::encode_to_vec(19_u64, bincode::config::standard()).unwrap();
        for message in messages() {
            bytes.extend(bincode::encode_to_vec(message, bincode::config::standard()).unwrap());
        }
        let mut payload_sizes = Vec::new();
        let (entry, used) = decode_copperlist_with_payload_sizes(&catalog, &bytes, |size| {
            payload_sizes.push(size);
        })
        .unwrap();
        assert_eq!(payload_sizes, vec![3, 1]);
        assert_eq!(used, bytes.len());
        assert_eq!(entry.msgs[0].payload, Some(Value::U32(300)));
        assert_eq!(entry.msgs[2].original_payload_present, None);
        for (actual, expected) in entry.msgs.iter().zip(messages()) {
            assert_eq!(actual.tov, expected.tov);
            assert_eq!(actual.metadata.status_txt, expected.metadata.status_txt);
            assert_eq!(actual.metadata.origin, expected.metadata.origin);
        }
        for len in 0..bytes.len() {
            assert!(
                decode_copperlist(&catalog, &bytes[..len]).is_err(),
                "length {len}"
            );
        }
    }

    #[test]
    fn test_pinned_v1_compact_fixture() {
        let mut builder = ValueDecodeCatalogBuilder::default();
        builder.add::<u32>("only", "u32");
        let catalog = builder
            .finish("()", "default", ValueDecodeCatalogLayout::Compact)
            .unwrap();
        let bytes = [42, 0, 0, 0, 0, 0, 0, 1, 1, 0, 42];
        let (entry, used) = decode_copperlist(&catalog, &bytes).unwrap();
        assert_eq!(used, bytes.len());
        assert_eq!(entry.msgs[0].payload, Some(Value::U32(42)));
        for (offset, value) in [(1, 3), (2, 1), (7, 0), (9, 1)] {
            let mut invalid = bytes;
            invalid[offset] = value;
            assert!(decode_copperlist(&catalog, &invalid).is_err());
        }
    }

    #[test]
    fn test_graph_validation_includes_unselected_enum_branches() {
        #[derive(Encode, Reflect)]
        enum Payload {
            Empty,
            Sample { value: u32 },
        }
        let mut builder = ValueDecodeCatalogBuilder::default();
        builder.add::<Payload>("enum", "Payload");
        let mut catalog = builder
            .finish("()", "default", ValueDecodeCatalogLayout::Compact)
            .unwrap();
        catalog.description.validate().unwrap();
        let operation = catalog
            .description
            .operations
            .iter_mut()
            .find(|op| matches!(op, ValueDecodeOp::Enum { .. }))
            .unwrap();
        let ValueDecodeOp::Enum { branches, .. } = operation else {
            unreachable!()
        };
        branches[1].fields[0] = usize::MAX;
        assert!(catalog.description.validate().is_err());
    }
}
