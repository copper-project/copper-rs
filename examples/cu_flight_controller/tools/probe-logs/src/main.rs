//! Recover committed runs from the MicoAir H743 over a CMSIS-DAP probe.
mod probe;

use anyhow::{Context, Result, bail, ensure};
use bincode::{config::standard, decode_from_slice, encode_into_slice, encode_to_vec};
use cu29_traits::{CuError, CuResult, UnifiedLogType};
use cu29_unifiedlog::byte_log::{
    ByteRegion, ByteStorage, encode_copied_section, read_header, read_main,
};
use cu29_unifiedlog::{ApplicationMetadata, MainHeader, SectionHeader};
use probe::{BLOCK, CHUNK, Probe};
use std::collections::{BTreeMap, HashSet};
use std::fs;
use std::io::Write;
use std::path::Path;
use std::sync::{Arc, Mutex};
use tempfile::NamedTempFile;

const HEADER: u64 = 1024;

#[derive(Clone)]
struct Card {
    probe: Arc<Mutex<Probe>>,
    lba: u64,
    length: u64,
}

impl ByteStorage for Card {
    type Region = Self;
    fn len(&self) -> u64 {
        self.length
    }
    fn grow(&mut self, _: u64) -> CuResult<()> {
        Err("read-only SD card".into())
    }
    fn region(&self, offset: u64, len: u64) -> CuResult<Self> {
        if !offset.is_multiple_of(BLOCK as u64)
            || offset.checked_add(len).is_none_or(|end| end > self.length)
        {
            return Err("invalid SD region".into());
        }
        Ok(Self {
            probe: self.probe.clone(),
            lba: self.lba + offset / BLOCK as u64,
            length: len,
        })
    }
}
impl ByteRegion for Card {
    fn len(&self) -> u64 {
        self.length
    }
    fn read(&self, offset: u64, bytes: &mut [u8]) -> CuResult<()> {
        let region = self.region(offset, bytes.len() as u64)?;
        self.probe
            .lock()
            .unwrap()
            .read(region.lba, bytes)
            .map_err(|e| CuError::from(e.to_string()))
    }
    fn write(&mut self, _: u64, _: &[u8]) -> CuResult<()> {
        Err("read-only SD card".into())
    }
    fn flush(&mut self) -> CuResult<()> {
        Ok(())
    }
}

#[derive(Clone)]
struct Section {
    offset: u64,
    header: SectionHeader,
}

struct Run {
    app: Section,
    catalog: Option<Section>,
    data: Vec<Section>,
}

fn list(card: &Card, main: &MainHeader) -> Result<BTreeMap<u64, Run>> {
    let mut runs = BTreeMap::new();
    let mut seen = HashSet::new();
    let mut app = None;
    let mut catalog = None;
    for (mut offset, lower, upper, static_metadata) in [
        (
            main.metadata_offset,
            BLOCK as u64,
            main.sections_begin,
            true,
        ),
        (
            main.head_section,
            main.sections_begin,
            main.sections_end,
            false,
        ),
    ] {
        while offset != 0 {
            ensure!(
                offset >= lower && offset < upper && seen.insert(offset),
                "invalid or cyclic section at {offset}"
            );
            let header = read_header(card, offset)?;
            ensure!(
                header.allocated <= upper - offset,
                "section extends past its region"
            );
            let next = header.next_section;
            if !static_metadata {
                ensure!(
                    (offset == main.tail_section) == (next == 0),
                    "invalid tail link"
                );
            }
            let section = Section { header, offset };
            match section.header.entry_type {
                UnifiedLogType::ApplicationMetadata => {
                    app = Some(section);
                    catalog = None;
                }
                UnifiedLogType::ValueDecodeCatalog => {
                    catalog = Some(section);
                }
                _ => {
                    ensure!(!static_metadata, "unexpected section in static metadata");
                    let app = app.as_ref().context("missing application metadata")?;
                    let run = runs
                        .entry(section.header.context.run_id)
                        .or_insert_with(|| Run {
                            app: app.clone(),
                            catalog: catalog.clone(),
                            data: Vec::new(),
                        });
                    ensure!(
                        run.app.offset == app.offset,
                        "run spans multiple build identities"
                    );
                    run.data.push(section);
                }
            }
            offset = next;
        }
    }
    Ok(runs)
}

fn download(card: &Card, mut main: MainHeader, run: Run, id: u64) -> Result<()> {
    let used = run.app.header.used as usize;
    ensure!(
        !run.app.header.is_open && used <= 16 * 1024 * 1024,
        "invalid application metadata size or commit"
    );
    let mut raw = vec![0; used];
    for (i, page) in raw.chunks_mut(CHUNK).enumerate() {
        card.read(run.app.offset + HEADER + (i * CHUNK) as u64, page)?;
    }
    let (mut app, n): (ApplicationMetadata, _) =
        decode_from_slice(&raw, standard().with_limit::<16777216>())?;
    ensure!(
        n == used && app.catalog_offset == run.catalog.as_ref().map_or(0, |c| c.offset),
        "catalog and application metadata disagree"
    );
    if let Some(c) = &run.catalog {
        ensure!(!c.header.is_open, "catalog is uncommitted");
    }
    let aligned = |n: u64| n.next_multiple_of(BLOCK as u64);
    // Relocating the catalog can change the varint length of its pointer.
    loop {
        raw = encode_to_vec(&app, standard())?;
        let pointer = if run.catalog.is_some() {
            BLOCK as u64 + aligned(HEADER + raw.len() as u64)
        } else {
            0
        };
        if app.catalog_offset == pointer {
            break;
        }
        app.catalog_offset = pointer;
    }
    let mut sections = vec![run.app];
    sections[0].header.used = raw.len().try_into()?;
    sections.extend(run.catalog);
    let first_record = sections.len();
    sections.extend(run.data);
    let mut positions = vec![BLOCK as u64];
    for s in &mut sections {
        s.header.allocated = aligned(HEADER + u64::from(s.header.used));
        positions.push(positions.last().unwrap() + s.header.allocated);
    }
    main.page_size = BLOCK as u16;
    main.metadata_offset = positions[0];
    main.sections_begin = positions[first_record];
    main.head_section = main.sections_begin;
    main.sections_end = *positions.last().unwrap();
    main.tail_section = positions[sections.len() - 1];
    main.clean_close = false;
    fs::create_dir_all("logs")?;
    let revision = app
        .git_commit
        .as_deref()
        .filter(|s| s.len() >= 12 && s.bytes().all(|c| c.is_ascii_hexdigit()))
        .map_or("unknown", |s| &s[..12]);
    let path = format!("logs/flight-{revision}-run{id}_0.copper");
    ensure!(!Path::new(&path).exists(), "{path} already exists");
    let mut out = NamedTempFile::new_in("logs")?;
    let mut page = [0; BLOCK];
    encode_into_slice(main, &mut page, standard())?;
    out.write_all(&page)?;
    let mut buffer = vec![0; CHUNK];
    for (i, section) in sections.iter_mut().enumerate() {
        section.header.next_section = if i + 1 == first_record || i + 1 == positions.len() - 1 {
            0
        } else {
            positions[i + 1]
        };
        out.write_all(&encode_copied_section(&section.header, positions[i])?)?;
        let used = section.header.used as usize;
        if used >= CHUNK {
            eprintln!("Copying {:?} ({used} bytes)", section.header.entry_type);
        }
        if i == 0 {
            out.write_all(&raw)?;
        } else {
            for j in (0..used).step_by(CHUNK) {
                let chunk = &mut buffer[..CHUNK.min(used - j)];
                card.read(section.offset + HEADER + j as u64, chunk)?;
                out.write_all(chunk)?;
            }
        }
        out.write_all(&[0; BLOCK][..used.next_multiple_of(BLOCK) - used])?;
    }
    out.as_file().sync_all()?;
    out.persist_noclobber(&path)?;
    println!("{path}");
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let (elf, selected) = match args.as_slice() {
        [elf, action, run] if action == "list" && run.is_empty() => (elf, None),
        [elf, action, run] if action == "download" => (
            elf,
            Some(run.parse::<u64>().context("expected a numeric SD run ID")?),
        ),
        _ => bail!("usage: just probe-logs list | just probe-logs download RUN"),
    };
    let mut probe = Probe::start(Path::new(elf))?;
    let (lba, length) = probe.partition()?;
    let card = Card {
        probe: Arc::new(Mutex::new(probe)),
        lba,
        length,
    };
    let header = read_main(&card)?;
    let mut runs = list(&card, &header)?;
    if let Some(id) = selected {
        download(
            &card,
            header,
            runs.remove(&id).context("run not found on SD card")?,
            id,
        )?;
    } else {
        println!("Run    Sections    Committed data bytes");
        for (id, run) in runs {
            println!(
                "{id:>3}    {:>8}    {:>20}",
                run.data.len(),
                run.data
                    .iter()
                    .map(|s| u64::from(s.header.used))
                    .sum::<u64>()
            );
        }
    }
    card.probe.lock().unwrap().finish()
}
