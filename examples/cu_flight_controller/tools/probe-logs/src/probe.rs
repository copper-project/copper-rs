use anyhow::{Context, Result, bail, ensure};
use std::collections::HashMap;
use std::fs;
use std::io::{BufRead, BufReader, Write};
#[cfg(unix)]
use std::os::unix::process::CommandExt;
use std::path::Path;
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::thread::sleep;
use std::time::{Duration, Instant};
use tempfile::TempDir;

pub const BLOCK: usize = 512;
pub const CHUNK: usize = 128 * BLOCK;
pub struct Probe {
    dir: TempDir,
    child: Child,
    input: Option<ChildStdin>,
    output: BufReader<ChildStdout>,
    symbols: HashMap<String, u32>,
    blocks: u64,
}

impl Probe {
    pub fn start(elf: &Path) -> Result<Self> {
        let dir = tempfile::tempdir()?;
        fs::copy(elf, dir.path().join("reader.elf"))?;
        fs::write(dir.path().join("reader.tcl"), include_str!("reader.tcl"))?;
        let nm = Command::new("nm")
            .args(["-g", "--defined-only"])
            .arg(elf)
            .output()?;
        ensure!(nm.status.success(), "nm failed to read the SRAM reader ELF");
        let mut symbols = HashMap::new();
        for line in std::str::from_utf8(&nm.stdout)?.lines() {
            let mut fields = line.split_whitespace();
            if let (Some(address), Some(name)) = (fields.next(), fields.nth(1)) {
                symbols.insert(name.to_owned(), u32::from_str_radix(address, 16)?);
            }
        }
        for name in [
            "READ_STATE",
            "READ_COUNT",
            "READ_DATA",
            "READ_LBA",
            "READ_CRC",
            "CARD_BLOCKS",
            "READER_READY",
        ] {
            ensure!(symbols.contains_key(name), "missing reader symbol {name}");
        }
        let mut openocd = Command::new("openocd");
        // An interrupted host closes stdin so the reader script can reset the board.
        #[cfg(unix)]
        openocd.process_group(0);
        let mut child = openocd
            .current_dir(dir.path())
            .args([
                "-f",
                "interface/cmsis-dap.cfg",
                "-c",
                "cmsis_dap_backend usb_bulk; adapter speed 8000",
                "-f",
                "target/stm32h7x.cfg",
                "-c",
            ])
            .arg(format!("set ready_address {}", symbols["READER_READY"]))
            .args(["-f", "reader.tcl"])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        let mut probe = Self {
            dir,
            input: child.stdin.take(),
            output: BufReader::new(child.stdout.take().unwrap()),
            child,
            symbols,
            blocks: 0,
        };
        ensure!(
            probe.line()? == "READY",
            "OpenOCD could not start the SRAM reader"
        );
        let deadline = Instant::now() + Duration::from_secs(10);
        while probe.word("READER_READY")? != 0x43555231 {
            ensure!(Instant::now() < deadline, "SD reader did not initialize");
            sleep(Duration::from_millis(10));
        }
        probe.blocks = u64::from(probe.word("CARD_BLOCKS")?);
        Ok(probe)
    }
    fn line(&mut self) -> Result<String> {
        let mut line = String::new();
        ensure!(
            self.output.read_line(&mut line)? > 0,
            "OpenOCD disconnected"
        );
        Ok(line.trim_end().to_owned())
    }
    fn command(&mut self, command: &str) -> Result<String> {
        writeln!(self.input.as_mut().context("closed probe")?, "{command}")?;
        let line = self.line()?;
        match line.strip_prefix("OK ") {
            Some(reply) => Ok(reply.to_owned()),
            None if line == "OK" => Ok(String::new()),
            _ => bail!("OpenOCD: {line}"),
        }
    }
    fn word(&mut self, name: &str) -> Result<u32> {
        let value = self.command(&format!("mrw {}", self.symbols[name]))?;
        Ok(u32::from_str_radix(value.trim_start_matches("0x"), 16)?)
    }
    pub fn read(&mut self, lba: u64, bytes: &mut [u8]) -> Result<()> {
        let count = bytes.len().div_ceil(BLOCK);
        ensure!(
            (1..=128).contains(&count)
                && lba
                    .checked_add(count as u64)
                    .is_some_and(|n| n <= self.blocks),
            "invalid card read at sector {lba}"
        );
        let s = &self.symbols;
        self.command(&format!(
            "mww {} {lba}; mww {} {count}; mww {} 1",
            s["READ_LBA"], s["READ_COUNT"], s["READ_STATE"]
        ))?;
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            match self.word("READ_STATE")? {
                2 => break,
                1 if Instant::now() < deadline => sleep(Duration::from_millis(1)),
                state => bail!("SD read failed at sector {lba}, state {state}"),
            }
        }
        let size = count * BLOCK;
        self.command(&format!(
            "dump_image sectors.bin {} {size}; set done 1",
            self.symbols["READ_DATA"]
        ))?;
        let data = fs::read(self.dir.path().join("sectors.bin"))?;
        ensure!(
            data.len() == size && crc32fast::hash(&data) == self.word("READ_CRC")?,
            "invalid SWD transfer at sector {lba}"
        );
        bytes.copy_from_slice(&data[..bytes.len()]);
        Ok(())
    }
    pub fn partition(&mut self) -> Result<(u64, u64)> {
        let mut h = [0; BLOCK];
        self.read(1, &mut h)?;
        let word = |b: &[u8]| u32::from_le_bytes(b.try_into().unwrap());
        let long = |b: &[u8]| u64::from_le_bytes(b.try_into().unwrap());
        let size = word(&h[12..16]) as usize;
        ensure!(
            &h[..8] == b"EFI PART" && (92..=BLOCK).contains(&size),
            "invalid GPT header"
        );
        let crc = word(&h[16..20]);
        h[16..20].fill(0);
        ensure!(
            crc32fast::hash(&h[..size]) == crc,
            "GPT header CRC mismatch"
        );
        let count = word(&h[80..84]) as usize;
        let stride = word(&h[84..88]) as usize;
        let length = count.checked_mul(stride).context("GPT size overflow")?;
        ensure!(
            stride >= 128 && stride.is_multiple_of(8) && (1..=CHUNK).contains(&length),
            "unsupported GPT entry table"
        );
        let mut table = vec![0; length];
        self.read(long(&h[72..80]), &mut table)?;
        ensure!(
            crc32fast::hash(&table) == word(&h[88..92]),
            "GPT entry CRC mismatch"
        );
        let mut matches = table.chunks_exact(stride).filter(|e| {
            e[..16]
                == [
                    0xc9, 0xe0, 0xa2, 0x29, 0, 0, 0x75, 0x4c, 0x92, 0x29, 0, 0, 0, 0, 0, 0x29,
                ]
        });
        let e = matches.next().context("Copper partition missing")?;
        ensure!(matches.next().is_none(), "multiple Copper partitions");
        let first = long(&e[32..40]);
        let last = long(&e[40..48]);
        ensure!(
            first > 0 && first <= last && last < self.blocks,
            "invalid Copper partition bounds"
        );
        Ok((first, (last - first + 1) * BLOCK as u64))
    }
    pub fn finish(&mut self) -> Result<()> {
        self.input.take();
        ensure!(
            self.child.wait()?.success(),
            "OpenOCD failed to restore flight firmware"
        );
        Ok(())
    }
}
impl Drop for Probe {
    fn drop(&mut self) {
        self.input.take();
        let _ = self.child.wait();
    }
}
