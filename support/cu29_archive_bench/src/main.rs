use std::collections::BTreeMap;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};

use bincode::config::standard;
use bincode::decode_from_std_read;
use bincode::encode_to_vec;
use clap::Parser;
use cu29::prelude::*;
use cu29_log::CuLogEntry;
use cu29_unifiedlog::{UnifiedLogger, UnifiedLoggerBuilder, UnifiedLoggerIOReader};
use cu29_value::Value;
use serde::Serialize;

use cu_ads7883_new::ADSReadingPayload;
use cu_pid::PIDControlOutputPayload;
use cu_rp_encoder::EncoderPayload;
use cu_rp_sn754410_new::MotorPayload;

// This benchmark is currently bound to the balancebot schema.
gen_cumsgs!("../../examples/cu_rp_balancebot/copperconfig.ron");

#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Args {
    /// Path to a Copper log slab (balance_0.copper) or base name (balance.copper).
    #[arg(long)]
    log: PathBuf,

    /// Output directory for columnar streams and manifest.
    #[arg(long)]
    out_dir: PathBuf,

    /// Zstd levels to benchmark, comma-separated (ex: 3,19).
    #[arg(long, value_delimiter = ',', default_value = "3,19")]
    zstd_level: Vec<i32>,
}

#[derive(Debug, Serialize)]
struct SlabInfo {
    path: String,
    bytes: u64,
}

#[derive(Debug, Serialize)]
struct SectionInfo {
    entries: u64,
    raw_bytes: u64,
    zstd: BTreeMap<i32, u64>,
}

#[derive(Debug, Serialize)]
struct StreamInfo {
    name: String,
    path: String,
    raw_bytes: u64,
    zstd: BTreeMap<i32, u64>,
}

#[derive(Debug, Serialize)]
struct Totals {
    raw_bytes: u64,
    zstd: BTreeMap<i32, u64>,
}

#[derive(Debug, Serialize)]
struct Manifest {
    source_log: String,
    base_log: String,
    zstd_levels: Vec<i32>,
    slabs: Vec<SlabInfo>,
    totals: Totals,
    sections: BTreeMap<String, SectionInfo>,
    streams: Vec<StreamInfo>,
    columnar_total_raw_bytes: u64,
}

struct StreamSink {
    path: PathBuf,
    file: File,
    bytes: u64,
}

struct StreamRegistry {
    base_dir: PathBuf,
    streams: HashMap<String, StreamSink>,
}

impl StreamRegistry {
    fn new(base_dir: PathBuf) -> io::Result<Self> {
        fs::create_dir_all(&base_dir)?;
        Ok(Self {
            base_dir,
            streams: HashMap::new(),
        })
    }

    fn write_bytes(&mut self, name: &str, bytes: &[u8]) -> io::Result<()> {
        let sink = self.stream_mut(name)?;
        sink.file.write_all(bytes)?;
        sink.bytes += bytes.len() as u64;
        Ok(())
    }

    fn write_u8(&mut self, name: &str, value: u8) -> io::Result<()> {
        self.write_bytes(name, &[value])
    }

    fn write_u16(&mut self, name: &str, value: u16) -> io::Result<()> {
        self.write_bytes(name, &value.to_le_bytes())
    }

    fn write_u32(&mut self, name: &str, value: u32) -> io::Result<()> {
        self.write_bytes(name, &value.to_le_bytes())
    }

    fn write_u64(&mut self, name: &str, value: u64) -> io::Result<()> {
        self.write_bytes(name, &value.to_le_bytes())
    }

    fn write_i8(&mut self, name: &str, value: i8) -> io::Result<()> {
        self.write_bytes(name, &[value as u8])
    }

    fn write_i16(&mut self, name: &str, value: i16) -> io::Result<()> {
        self.write_bytes(name, &value.to_le_bytes())
    }

    fn write_i32(&mut self, name: &str, value: i32) -> io::Result<()> {
        self.write_bytes(name, &value.to_le_bytes())
    }

    fn write_i64(&mut self, name: &str, value: i64) -> io::Result<()> {
        self.write_bytes(name, &value.to_le_bytes())
    }

    fn write_f32(&mut self, name: &str, value: f32) -> io::Result<()> {
        self.write_bytes(name, &value.to_le_bytes())
    }

    fn write_f64(&mut self, name: &str, value: f64) -> io::Result<()> {
        self.write_bytes(name, &value.to_le_bytes())
    }

    fn finish(&mut self) -> io::Result<()> {
        for sink in self.streams.values_mut() {
            sink.file.flush()?;
        }
        Ok(())
    }

    fn stream_mut(&mut self, name: &str) -> io::Result<&mut StreamSink> {
        if !self.streams.contains_key(name) {
            let path = self.base_dir.join(format!("{name}.bin"));
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent)?;
            }
            let file = File::create(&path)?;
            self.streams.insert(
                name.to_string(),
                StreamSink {
                    path,
                    file,
                    bytes: 0,
                },
            );
        }
        Ok(self.streams.get_mut(name).expect("stream exists"))
    }
}

fn main() -> io::Result<()> {
    let args = Args::parse();
    let base_log = normalize_base_path(&args.log);
    let slabs = find_slab_files(&base_log)?;

    if slabs.is_empty() {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!("No slabs found for base log: {}", base_log.display()),
        ));
    }

    fs::create_dir_all(&args.out_dir)?;

    let slab_infos = collect_slab_info(&slabs)?;
    let raw_log = read_all_files(&slabs)?;
    let total_raw_bytes = raw_log.len() as u64;
    let total_zstd = zstd_sizes(&raw_log, &args.zstd_level)?;

    let structured_raw = read_section_bytes(&base_log, UnifiedLogType::StructuredLogLine)?;
    let structured_zstd = zstd_sizes(&structured_raw, &args.zstd_level)?;

    let copperlist_raw = read_section_bytes(&base_log, UnifiedLogType::CopperList)?;
    let copperlist_zstd = zstd_sizes(&copperlist_raw, &args.zstd_level)?;

    let mut structured_registry = StreamRegistry::new(args.out_dir.join("structured"))?;
    let structured_entries = extract_structured(&base_log, &mut structured_registry)?;
    structured_registry.finish()?;

    let mut copper_registry = StreamRegistry::new(args.out_dir.join("copperlist"))?;
    let copperlist_entries = extract_copperlists(&base_log, &mut copper_registry)?;
    copper_registry.finish()?;

    let mut stream_infos = Vec::new();
    let mut columnar_total_raw_bytes = 0u64;

    collect_stream_infos(
        &args.out_dir,
        &structured_registry,
        &args.zstd_level,
        &mut stream_infos,
        &mut columnar_total_raw_bytes,
    )?;
    collect_stream_infos(
        &args.out_dir,
        &copper_registry,
        &args.zstd_level,
        &mut stream_infos,
        &mut columnar_total_raw_bytes,
    )?;

    let mut sections = BTreeMap::new();
    sections.insert(
        "StructuredLogLine".to_string(),
        SectionInfo {
            entries: structured_entries,
            raw_bytes: structured_raw.len() as u64,
            zstd: structured_zstd,
        },
    );
    sections.insert(
        "CopperList".to_string(),
        SectionInfo {
            entries: copperlist_entries,
            raw_bytes: copperlist_raw.len() as u64,
            zstd: copperlist_zstd,
        },
    );

    let manifest = Manifest {
        source_log: args.log.display().to_string(),
        base_log: base_log.display().to_string(),
        zstd_levels: args.zstd_level.clone(),
        slabs: slab_infos,
        totals: Totals {
            raw_bytes: total_raw_bytes,
            zstd: total_zstd,
        },
        sections,
        streams: stream_infos,
        columnar_total_raw_bytes,
    };

    let manifest_path = args.out_dir.join("manifest.json");
    let manifest_json =
        serde_json::to_vec_pretty(&manifest).map_err(|e| io::Error::other(e.to_string()))?;
    fs::write(&manifest_path, manifest_json)?;

    println!("Wrote manifest: {}", manifest_path.display());
    Ok(())
}

fn normalize_base_path(path: &Path) -> PathBuf {
    let Some(stem_os) = path.file_stem() else {
        return path.to_path_buf();
    };
    let stem = stem_os.to_string_lossy();
    let base_stem = match stem.rsplit_once('_') {
        Some((base, suffix)) if suffix.chars().all(|c| c.is_ascii_digit()) => base,
        _ => stem.as_ref(),
    };

    let mut file_name = base_stem.to_string();
    if let Some(ext) = path.extension() {
        file_name.push('.');
        file_name.push_str(&ext.to_string_lossy());
    }

    path.with_file_name(file_name)
}

fn find_slab_files(base: &Path) -> io::Result<Vec<PathBuf>> {
    let dir = base.parent().unwrap_or_else(|| Path::new("."));
    let base_stem = base
        .file_stem()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "Missing file stem"))?
        .to_string_lossy()
        .to_string();
    let base_ext = base.extension().and_then(|ext| ext.to_str());

    let mut slabs = Vec::new();
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        let path_ext = path.extension().and_then(|ext| ext.to_str());
        if base_ext != path_ext {
            continue;
        }
        let Some(stem_os) = path.file_stem() else {
            continue;
        };
        let stem = stem_os.to_string_lossy();
        let Some((prefix, suffix)) = stem.rsplit_once('_') else {
            continue;
        };
        if prefix != base_stem {
            continue;
        }
        if !suffix.chars().all(|c| c.is_ascii_digit()) {
            continue;
        }
        slabs.push(path);
    }

    slabs.sort_by_key(|path| {
        path.file_stem()
            .and_then(|stem| {
                stem.to_string_lossy()
                    .rsplit_once('_')
                    .map(|(_, s)| s.to_string())
            })
            .and_then(|s| s.parse::<u64>().ok())
            .unwrap_or(0)
    });

    Ok(slabs)
}

fn collect_slab_info(slabs: &[PathBuf]) -> io::Result<Vec<SlabInfo>> {
    let mut infos = Vec::new();
    for slab in slabs {
        let meta = fs::metadata(slab)?;
        infos.push(SlabInfo {
            path: slab.display().to_string(),
            bytes: meta.len(),
        });
    }
    Ok(infos)
}

fn read_all_files(slabs: &[PathBuf]) -> io::Result<Vec<u8>> {
    let mut total = 0usize;
    for slab in slabs {
        total += fs::metadata(slab)?.len() as usize;
    }
    let mut buffer = Vec::with_capacity(total);
    for slab in slabs {
        let mut file = File::open(slab)?;
        file.read_to_end(&mut buffer)?;
    }
    Ok(buffer)
}

fn read_section_bytes(base: &Path, log_type: UnifiedLogType) -> io::Result<Vec<u8>> {
    let reader = open_reader(base, log_type)?;
    read_all_from_reader(reader)
}

fn read_all_from_reader(mut reader: impl Read) -> io::Result<Vec<u8>> {
    let mut buf = Vec::new();
    reader.read_to_end(&mut buf)?;
    Ok(buf)
}

fn zstd_sizes(data: &[u8], levels: &[i32]) -> io::Result<BTreeMap<i32, u64>> {
    let mut map = BTreeMap::new();
    for level in levels {
        let compressed =
            zstd::bulk::compress(data, *level).map_err(|e| io::Error::other(e.to_string()))?;
        map.insert(*level, compressed.len() as u64);
    }
    Ok(map)
}

fn open_reader(base: &Path, log_type: UnifiedLogType) -> io::Result<UnifiedLoggerIOReader> {
    let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
        .file_base_name(base)
        .build()
        .map_err(|e| io::Error::other(e.to_string()))?
    else {
        return Err(io::Error::other(
            "Failed to open unified logger for reading",
        ));
    };

    Ok(UnifiedLoggerIOReader::new(dl, log_type))
}

fn decode_next<T: bincode::Decode<()>>(reader: &mut impl Read) -> io::Result<Option<T>> {
    match decode_from_std_read(reader, standard()) {
        Ok(value) => Ok(Some(value)),
        Err(err) => match err {
            bincode::error::DecodeError::UnexpectedEnd { .. } => Ok(None),
            bincode::error::DecodeError::Io { inner, .. }
                if inner.kind() == io::ErrorKind::UnexpectedEof =>
            {
                Ok(None)
            }
            other => Err(io::Error::other(other.to_string())),
        },
    }
}

fn extract_structured(base: &Path, registry: &mut StreamRegistry) -> io::Result<u64> {
    let mut reader = open_reader(base, UnifiedLogType::StructuredLogLine)?;
    let mut entries = 0u64;

    while let Some(entry) = decode_next::<CuLogEntry>(&mut reader)? {
        entries += 1;
        write_structured_entry(registry, &entry)?;
    }

    Ok(entries)
}

fn write_structured_entry(registry: &mut StreamRegistry, entry: &CuLogEntry) -> io::Result<()> {
    registry.write_u64("time.u64", u64::from(entry.time))?;
    registry.write_u8("level.u8", entry.level as u8)?;
    registry.write_u32("msg_index.u32", entry.msg_index)?;

    let param_count = entry.params.len();
    registry.write_u32("param_count.u32", param_count as u32)?;

    if entry.paramname_indexes.len() != param_count {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "paramname_indexes length mismatch",
        ));
    }

    for index in &entry.paramname_indexes {
        registry.write_u32("paramname_index.u32", *index)?;
    }

    for value in &entry.params {
        write_structured_param(registry, value)?;
    }

    Ok(())
}

fn write_structured_param(registry: &mut StreamRegistry, value: &Value) -> io::Result<()> {
    let tag = structured_value_tag(value);
    registry.write_u8("param_tag.u8", tag)?;

    match value {
        Value::Bool(v) => registry.write_u8("param_bool.u8", *v as u8)?,
        Value::U8(v) => registry.write_u8("param_u8.u8", *v)?,
        Value::U16(v) => registry.write_u16("param_u16.u16", *v)?,
        Value::U32(v) => registry.write_u32("param_u32.u32", *v)?,
        Value::U64(v) => registry.write_u64("param_u64.u64", *v)?,
        Value::I8(v) => registry.write_i8("param_i8.i8", *v)?,
        Value::I16(v) => registry.write_i16("param_i16.i16", *v)?,
        Value::I32(v) => registry.write_i32("param_i32.i32", *v)?,
        Value::I64(v) => registry.write_i64("param_i64.i64", *v)?,
        Value::F32(v) => registry.write_f32("param_f32.f32", *v)?,
        Value::F64(v) => registry.write_f64("param_f64.f64", *v)?,
        Value::Char(v) => registry.write_u32("param_char.u32", *v as u32)?,
        Value::String(v) => {
            registry.write_u32("param_string.len.u32", v.len() as u32)?;
            registry.write_bytes("param_string.bytes", v.as_bytes())?;
        }
        Value::Bytes(v) => {
            registry.write_u32("param_bytes.len.u32", v.len() as u32)?;
            registry.write_bytes("param_bytes.bytes", v)?;
        }
        Value::CuTime(v) => registry.write_u64("param_cutime.u64", u64::from(*v))?,
        Value::Unit | Value::Option(_) | Value::Newtype(_) | Value::Seq(_) | Value::Map(_) => {
            let encoded =
                encode_to_vec(value, standard()).map_err(|e| io::Error::other(e.to_string()))?;
            registry.write_u32("param_complex.len.u32", encoded.len() as u32)?;
            registry.write_bytes("param_complex.bytes", &encoded)?;
        }
    }

    Ok(())
}

fn structured_value_tag(value: &Value) -> u8 {
    match value {
        Value::Bool(_) => 1,
        Value::U8(_) => 2,
        Value::U16(_) => 3,
        Value::U32(_) => 4,
        Value::U64(_) => 5,
        Value::I8(_) => 6,
        Value::I16(_) => 7,
        Value::I32(_) => 8,
        Value::I64(_) => 9,
        Value::F32(_) => 10,
        Value::F64(_) => 11,
        Value::Char(_) => 12,
        Value::String(_) => 13,
        Value::Bytes(_) => 14,
        Value::Unit => 15,
        Value::Option(_) => 16,
        Value::Newtype(_) => 17,
        Value::Seq(_) => 18,
        Value::Map(_) => 19,
        Value::CuTime(_) => 20,
    }
}

fn extract_copperlists(base: &Path, registry: &mut StreamRegistry) -> io::Result<u64> {
    let mut reader = open_reader(base, UnifiedLogType::CopperList)?;
    let mut entries = 0u64;

    while let Some(entry) = decode_next::<CopperList<CuStampedDataSet>>(&mut reader)? {
        entries += 1;
        write_copperlist_entry(registry, &entry)?;
    }

    Ok(entries)
}

fn write_copperlist_entry(
    registry: &mut StreamRegistry,
    entry: &CopperList<CuStampedDataSet>,
) -> io::Result<()> {
    registry.write_u32("culist.id.u32", entry.id)?;
    registry.write_u8("culist.state.u8", copperlist_state_tag(entry.get_state()))?;

    let balpos = entry.msgs.get_balpos_output();
    let railpos = entry.msgs.get_railpos_output();
    let balpos_pid = entry.msgs.get_balpos_pid_output();
    let railpos_pid = entry.msgs.get_railpos_pid_output();
    let merge_pids = entry.msgs.get_merge_pids_output();
    let motor = entry.msgs.get_motor_output();

    write_msg_common(registry, "balpos", balpos)?;
    write_msg_common(registry, "railpos", railpos)?;
    write_msg_common(registry, "balpos_pid", balpos_pid)?;
    write_msg_common(registry, "railpos_pid", railpos_pid)?;
    write_msg_common(registry, "merge_pids", merge_pids)?;
    write_msg_common(registry, "motor", motor)?;

    write_ads_payload(registry, "balpos", balpos.payload())?;
    write_encoder_payload(registry, "railpos", railpos.payload())?;
    write_pid_payload(registry, "balpos_pid", balpos_pid.payload())?;
    write_pid_payload(registry, "railpos_pid", railpos_pid.payload())?;
    write_motor_payload(registry, "merge_pids", merge_pids.payload())?;

    Ok(())
}

fn copperlist_state_tag(state: CopperListState) -> u8 {
    match state {
        CopperListState::Free => 0,
        CopperListState::Initialized => 1,
        CopperListState::Processing => 2,
        CopperListState::DoneProcessing => 3,
        CopperListState::BeingSerialized => 4,
    }
}

fn write_msg_common<T: CuMsgPayload>(
    registry: &mut StreamRegistry,
    prefix: &str,
    msg: &CuMsg<T>,
) -> io::Result<()> {
    registry.write_u8(
        &format!("{prefix}.present.u8"),
        msg.payload().is_some() as u8,
    )?;

    let (tov_tag, tov_time, tov_start, tov_end) = match msg.tov {
        Tov::None => (0u8, 0u64, 0u64, 0u64),
        Tov::Time(t) => (1u8, u64::from(t), 0u64, 0u64),
        Tov::Range(r) => (2u8, 0u64, u64::from(r.start), u64::from(r.end)),
    };

    registry.write_u8(&format!("{prefix}.tov_tag.u8"), tov_tag)?;
    registry.write_u64(&format!("{prefix}.tov_time.u64"), tov_time)?;
    registry.write_u64(&format!("{prefix}.tov_range_start.u64"), tov_start)?;
    registry.write_u64(&format!("{prefix}.tov_range_end.u64"), tov_end)?;

    let metadata = &msg.metadata;
    let start = option_cu_time_to_u64(metadata.process_time.start);
    let end = option_cu_time_to_u64(metadata.process_time.end);
    registry.write_u64(&format!("{prefix}.process_start.u64"), start)?;
    registry.write_u64(&format!("{prefix}.process_end.u64"), end)?;

    let status_bytes = metadata.status_txt.0.as_bytes();
    registry.write_u32(
        &format!("{prefix}.status_txt.len.u32"),
        status_bytes.len() as u32,
    )?;
    registry.write_bytes(&format!("{prefix}.status_txt.bytes"), status_bytes)?;

    Ok(())
}

fn option_cu_time_to_u64(value: OptionCuTime) -> u64 {
    let opt: Option<CuTime> = value.into();
    opt.map(u64::from).unwrap_or(u64::MAX)
}

fn write_ads_payload(
    registry: &mut StreamRegistry,
    prefix: &str,
    payload: Option<&ADSReadingPayload>,
) -> io::Result<()> {
    let value = payload.map(|p| p.analog_value).unwrap_or(0);
    registry.write_u16(&format!("{prefix}.payload.analog_value.u16"), value)
}

fn write_encoder_payload(
    registry: &mut StreamRegistry,
    prefix: &str,
    payload: Option<&EncoderPayload>,
) -> io::Result<()> {
    let value = payload.map(|p| p.ticks).unwrap_or(0);
    registry.write_i32(&format!("{prefix}.payload.ticks.i32"), value)
}

fn write_pid_payload(
    registry: &mut StreamRegistry,
    prefix: &str,
    payload: Option<&PIDControlOutputPayload>,
) -> io::Result<()> {
    let (p, i, d, output) = payload
        .map(|p| (p.p, p.i, p.d, p.output))
        .unwrap_or((0.0, 0.0, 0.0, 0.0));
    registry.write_f32(&format!("{prefix}.payload.p.f32"), p)?;
    registry.write_f32(&format!("{prefix}.payload.i.f32"), i)?;
    registry.write_f32(&format!("{prefix}.payload.d.f32"), d)?;
    registry.write_f32(&format!("{prefix}.payload.output.f32"), output)?;
    Ok(())
}

fn write_motor_payload(
    registry: &mut StreamRegistry,
    prefix: &str,
    payload: Option<&MotorPayload>,
) -> io::Result<()> {
    let value = payload.map(|p| p.power).unwrap_or(0.0);
    registry.write_f32(&format!("{prefix}.payload.power.f32"), value)
}

fn collect_stream_infos(
    out_dir: &Path,
    registry: &StreamRegistry,
    levels: &[i32],
    stream_infos: &mut Vec<StreamInfo>,
    total_raw: &mut u64,
) -> io::Result<()> {
    for (name, sink) in &registry.streams {
        let data = fs::read(&sink.path)?;
        let zstd = zstd_sizes(&data, levels)?;
        let rel_path = sink
            .path
            .strip_prefix(out_dir)
            .unwrap_or(&sink.path)
            .display()
            .to_string();
        *total_raw += sink.bytes;
        stream_infos.push(StreamInfo {
            name: name.to_string(),
            path: rel_path,
            raw_bytes: sink.bytes,
            zstd,
        });
    }
    Ok(())
}
