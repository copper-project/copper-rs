use std::io::{self, Write};
use std::time::Instant;

/// Progress bar types.
///
/// This can be set with
/// [`CacheBuilder::progress_bar()`](struct.CacheBuilder.html#method.progress_bar).
#[derive(Debug, Clone)]
pub enum ProgressBar {
    /// Gives pretty, verbose progress bars.
    Full,
    /// Gives progress bars with minimal output.
    ///
    /// This is a good option to use if your output is being captured to a file but you
    /// still want to see progress updates.
    Light,
}

impl Default for ProgressBar {
    fn default() -> Self {
        ProgressBar::Full
    }
}

impl ProgressBar {
    pub(crate) fn wrap_download<W: Write>(
        &self,
        resource: &str,
        content_length: Option<u64>,
        writer: W,
    ) -> DownloadWrapper<W> {
        let bar: Box<dyn DownloadBar> = match self {
            ProgressBar::Full => Box::new(FullDownloadBar::new(content_length)),
            ProgressBar::Light => Box::new(LightDownloadBar::new(resource, content_length)),
        };
        DownloadWrapper::new(bar, writer)
    }
}

pub(crate) struct DownloadWrapper<W: Write> {
    bar: Box<dyn DownloadBar>,
    writer: W,
}

impl<W> DownloadWrapper<W>
where
    W: Write,
{
    fn new(bar: Box<dyn DownloadBar>, writer: W) -> Self {
        Self { bar, writer }
    }

    pub(crate) fn finish(&self) {
        self.bar.finish();
    }
}

impl<W: Write> Write for DownloadWrapper<W> {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.writer.write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.writer.flush()
    }

    fn write_vectored(&mut self, bufs: &[io::IoSlice]) -> io::Result<usize> {
        self.writer.write_vectored(bufs)
    }

    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        self.writer.write_all(buf).map(|()| {
            self.bar.tick(buf.len());
        })
    }
}

trait DownloadBar {
    fn tick(&mut self, chunk_size: usize);

    fn finish(&self);
}

pub(crate) struct FullDownloadBar {
    bar: indicatif::ProgressBar,
}

impl FullDownloadBar {
    pub(crate) fn new(content_length: Option<u64>) -> Self {
        let bar = match content_length {
            Some(length) => {
                let bar = indicatif::ProgressBar::new(length);
                bar.set_style(
                    indicatif::ProgressStyle::default_bar()
                    .progress_chars("=>-")
                    .template(
                        "{msg:.bold.cyan/blue} [{bar:20.cyan/blue}][{percent}%] {bytes}/{total_bytes:.bold} |{bytes_per_sec}|",
                    )
                );
                bar
            }
            None => {
                let bar = indicatif::ProgressBar::new_spinner();
                bar.set_style(
                    indicatif::ProgressStyle::default_bar()
                        .tick_strings(&[
                            "⠁⠁⠉⠙⠚⠒⠂⠂⠒⠲⠴⠤⠄⠄⠤⠠⠠⠤⠦⠖",
                            "⠉⠙⠚⠒⠂⠂⠒⠲⠴⠤⠄⠄⠤⠠⠠⠤⠦⠖⠒⠐",
                            "⠚⠒⠂⠂⠒⠲⠴⠤⠄⠄⠤⠠⠠⠤⠦⠖⠒⠐⠐⠒",
                            "⠂⠂⠒⠲⠴⠤⠄⠄⠤⠠⠠⠤⠦⠖⠒⠐⠐⠒⠓⠋",
                            "⠒⠲⠴⠤⠄⠄⠤⠠⠠⠤⠦⠖⠒⠐⠐⠒⠓⠋⠉⠈",
                            "⠴⠤⠄⠄⠤⠠⠠⠤⠦⠖⠒⠐⠐⠒⠓⠋⠉⠈⠈⠉",
                            "⠄⠄⠤⠠⠠⠤⠦⠖⠒⠐⠐⠒⠓⠋⠉⠈⠈⠉⠙⠚",
                            "⠤⠠⠠⠤⠦⠖⠒⠐⠐⠒⠓⠋⠉⠈⠈⠉⠙⠚⠒⠂",
                            "⠠⠤⠦⠖⠒⠐⠐⠒⠓⠋⠉⠈⠈⠉⠙⠚⠒⠂⠂⠒",
                            "⠦⠖⠒⠐⠐⠒⠓⠋⠉⠈⠈⠉⠙⠚⠒⠂⠂⠒⠲⠴",
                            "⠒⠐⠐⠒⠓⠋⠉⠈⠈⠉⠙⠚⠒⠂⠂⠒⠲⠴⠤⠄",
                            "⠐⠒⠓⠋⠉⠈⠈⠉⠙⠚⠒⠂⠂⠒⠲⠴⠤⠄⠄⠤",
                            "⠓⠋⠉⠈⠈⠉⠙⠚⠒⠂⠂⠒⠲⠴⠤⠄⠄⠤⠠⠠",
                        ])
                        .template(
                            "{msg:.bold.cyan/blue} [{spinner:.cyan/blue}] {bytes:.bold} |{bytes_per_sec}|",
                        ),
                );
                bar
            }
        };
        bar.set_message("Downloading");
        // Update every 1 MBs.
        // NOTE: If we don't set this, the updates happen WAY too frequently and it makes downloads
        // take about twice as long.
        bar.set_draw_delta(1_000_000);
        Self { bar }
    }
}

impl DownloadBar for FullDownloadBar {
    fn tick(&mut self, chunk_size: usize) {
        self.bar.inc(chunk_size as u64);
    }

    fn finish(&self) {
        self.bar.set_message("Downloaded");
        self.bar.set_style(
            indicatif::ProgressStyle::default_bar()
                .template("{msg:.green.bold} {total_bytes:.bold} in {elapsed}"),
        );
        self.bar.finish_at_current_pos();
    }
}

pub(crate) struct LightDownloadBar {
    start_time: Instant,
    bytes: usize,
    bytes_since_last_update: usize,
}

impl LightDownloadBar {
    pub(crate) fn new(resource: &str, content_length: Option<u64>) -> Self {
        if let Some(size) = content_length {
            eprint!(
                "Downloading {} [{}]...",
                resource,
                indicatif::HumanBytes(size)
            );
        } else {
            eprint!("Downloading {}...", resource);
        }
        io::stderr().flush().ok();
        Self {
            start_time: Instant::now(),
            bytes: 0,
            bytes_since_last_update: 0,
        }
    }
}

impl DownloadBar for LightDownloadBar {
    fn tick(&mut self, chunk_size: usize) {
        self.bytes_since_last_update += chunk_size;
        // Update every 100 MBs.
        if self.bytes_since_last_update > 100_000_000 {
            eprint!(".");
            io::stderr().flush().ok();
            self.bytes_since_last_update = 0;
        }
        self.bytes += chunk_size;
    }

    fn finish(&self) {
        let duration = Instant::now().duration_since(self.start_time);
        eprintln!(
            " ✓ Done! Finished in {}",
            indicatif::HumanDuration(duration)
        );
        io::stderr().flush().ok();
    }
}
