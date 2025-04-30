use fs2::FileExt;
use glob::glob;
use log::{debug, error, info, warn};
use rand::distributions::{Distribution, Uniform};
use reqwest::blocking::{Client, ClientBuilder};
use reqwest::header::ETAG;
use std::default::Default;
use std::env;
use std::fs::{self, OpenOptions};
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{self, Duration};
use tempfile::NamedTempFile;

use crate::archives::{extract_archive, ArchiveFormat};
use crate::utils::hash_str;
use crate::{meta::Meta, Error, ProgressBar};

/// Builder to facilitate creating [`Cache`] objects.
#[derive(Debug)]
pub struct CacheBuilder {
    config: Config,
}

#[derive(Debug)]
struct Config {
    dir: Option<PathBuf>,
    client_builder: ClientBuilder,
    max_retries: u32,
    max_backoff: u32,
    freshness_lifetime: Option<u64>,
    offline: bool,
    progress_bar: Option<ProgressBar>,
}

impl CacheBuilder {
    /// Construct a new `CacheBuilder`.
    pub fn new() -> CacheBuilder {
        CacheBuilder {
            config: Config {
                dir: None,
                client_builder: ClientBuilder::new().timeout(None),
                max_retries: 3,
                max_backoff: 5000,
                freshness_lifetime: None,
                offline: false,
                progress_bar: Some(ProgressBar::default()),
            },
        }
    }

    /// Construct a new `CacheBuilder` with a `ClientBuilder`.
    pub fn with_client_builder(client_builder: ClientBuilder) -> CacheBuilder {
        CacheBuilder::new().client_builder(client_builder)
    }

    /// Set the cache location. This can be set through the environment
    /// variable `RUST_CACHED_PATH_ROOT`. Otherwise it will default to a subdirectory
    /// named 'cache' of the default system temp directory.
    pub fn dir(mut self, dir: PathBuf) -> CacheBuilder {
        self.config.dir = Some(dir);
        self
    }

    /// Set the `ClientBuilder`.
    pub fn client_builder(mut self, client_builder: ClientBuilder) -> CacheBuilder {
        self.config.client_builder = client_builder;
        self
    }

    /// Enable a request timeout.
    pub fn timeout(mut self, timeout: Duration) -> CacheBuilder {
        self.config.client_builder = self.config.client_builder.timeout(timeout);
        self
    }

    /// Enable a timeout for the connect phase of each HTTP request.
    pub fn connect_timeout(mut self, timeout: Duration) -> CacheBuilder {
        self.config.client_builder = self.config.client_builder.connect_timeout(timeout);
        self
    }

    /// Set maximum number of retries for HTTP requests.
    pub fn max_retries(mut self, max_retries: u32) -> CacheBuilder {
        self.config.max_retries = max_retries;
        self
    }

    /// Set the maximum backoff delay in milliseconds for retrying HTTP requests.
    pub fn max_backoff(mut self, max_backoff: u32) -> CacheBuilder {
        self.config.max_backoff = max_backoff;
        self
    }

    /// Set the default freshness lifetime, in seconds. The default is None, meaning
    /// the ETAG for an external resource will always be checked for a fresher value.
    pub fn freshness_lifetime(mut self, freshness_lifetime: u64) -> CacheBuilder {
        self.config.freshness_lifetime = Some(freshness_lifetime);
        self
    }

    /// Only use offline functionality.
    ///
    /// If set to `true`, when the cached path of an HTTP resource is requested,
    /// the latest cached version is returned without checking for freshness.
    /// But if no cached versions exist, a
    /// [`NoCachedVersions`](enum.Error.html#variant.NoCachedVersions) error is returned.
    pub fn offline(mut self, offline: bool) -> CacheBuilder {
        self.config.offline = offline;
        self
    }

    /// Set the type of progress bar to use.
    ///
    /// The default is `Some(ProgressBar::Full)`.
    pub fn progress_bar(mut self, progress_bar: Option<ProgressBar>) -> CacheBuilder {
        self.config.progress_bar = progress_bar;
        self
    }

    /// Build the `Cache` object.
    pub fn build(self) -> Result<Cache, Error> {
        let dir = self.config.dir.unwrap_or_else(|| {
            if let Some(dir_str) = env::var_os("RUST_CACHED_PATH_ROOT") {
                PathBuf::from(dir_str)
            } else {
                env::temp_dir().join("cache/")
            }
        });
        let http_client = self.config.client_builder.build()?;
        fs::create_dir_all(&dir)?;
        Ok(Cache {
            dir,
            http_client,
            max_retries: self.config.max_retries,
            max_backoff: self.config.max_backoff,
            freshness_lifetime: self.config.freshness_lifetime,
            offline: self.config.offline,
            progress_bar: self.config.progress_bar,
        })
    }
}

impl Default for CacheBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Options to use with [`Cache::cached_path_with_options`].
#[derive(Default)]
pub struct Options {
    /// An optional subdirectory (relative to the cache root) to cache the resource in.
    pub subdir: Option<String>,
    /// Automatically extract the resource, assuming the resource is an archive.
    pub extract: bool,
}

impl Options {
    pub fn new(subdir: Option<&str>, extract: bool) -> Self {
        Self {
            subdir: subdir.map(String::from),
            extract,
        }
    }

    /// The the cache subdirectory to use.
    pub fn subdir(mut self, subdir: &str) -> Self {
        self.subdir = Some(subdir.into());
        self
    }

    /// Treat the resource as an archive and try to extract it.
    pub fn extract(mut self) -> Self {
        self.extract = true;
        self
    }
}

/// Fetches and manages resources in a local cache directory.
#[derive(Debug, Clone)]
pub struct Cache {
    /// The root directory of the cache.
    pub dir: PathBuf,
    /// The maximum number of times to retry downloading a remote resource.
    max_retries: u32,
    /// The maximum amount of time (in milliseconds) to wait before retrying a download.
    max_backoff: u32,
    /// An optional freshness lifetime (in seconds).
    ///
    /// If set, resources that were cached within the past `freshness_lifetime` seconds
    /// will always be regarded as fresh, and so the ETag of the corresponding remote
    /// resource won't be checked.
    freshness_lifetime: Option<u64>,
    /// Offline mode.
    ///
    /// If set to `true`, no HTTP calls will be made.
    offline: bool,
    /// The verbosity level of the progress bar.
    progress_bar: Option<ProgressBar>,
    /// The HTTP client used to fetch remote resources.
    http_client: Client,
}

impl Cache {
    /// Create a new `Cache` instance.
    pub fn new() -> Result<Self, Error> {
        Cache::builder().build()
    }

    /// Create a `CacheBuilder`.
    pub fn builder() -> CacheBuilder {
        CacheBuilder::new()
    }

    /// Get the cached path to a resource.
    ///
    /// If the resource is local file, it's path is returned. If the resource is a static HTTP
    /// resource, it will cached locally and the path to the cache file will be returned.
    pub fn cached_path(&self, resource: &str) -> Result<PathBuf, Error> {
        self.cached_path_with_options(resource, &Options::default())
    }

    /// Get the cached path to a resource using the given options.
    ///
    /// # Examples
    ///
    /// Use a particular subdirectory of the cache root:
    ///
    /// ```rust,no_run
    /// # use cached_path::{Cache, Options};
    /// # let cache = Cache::new().unwrap();
    /// # let subdir = "target";
    /// # let resource = "README.md";
    /// let path = cache.cached_path_with_options(
    ///     resource,
    ///     &Options::default().subdir(subdir),
    /// ).unwrap();
    /// ```
    ///
    /// Treat the resource as an archive and extract it. The path returned is the
    /// path to the extraction directory:
    ///
    /// ```rust,no_run
    /// # use cached_path::{Cache, Options};
    /// # let cache = Cache::new().unwrap();
    /// # let subdir = "target";
    /// # let resource = "README.md";
    /// let path = cache.cached_path_with_options(
    ///     resource,
    ///     &Options::default().extract(),
    /// ).unwrap();
    /// assert!(path.is_dir());
    /// ```
    pub fn cached_path_with_options(
        &self,
        resource: &str,
        options: &Options,
    ) -> Result<PathBuf, Error> {
        let cached_path: PathBuf;
        let mut extraction_dir: Option<PathBuf> = None;

        if !resource.starts_with("http") {
            // If resource doesn't look like a URL, treat as local path, but return
            // an error if the path doesn't exist.
            info!("Treating {} as local file", resource);
            cached_path = PathBuf::from(resource);

            if !cached_path.is_file() {
                return Err(Error::ResourceNotFound(String::from(resource)));
            }

            if options.extract {
                // If we need to extract, we extract into a unique subdirectory of the cache directory
                // so as not to mess with the file system outside of the cache directory.
                // To make sure that we use a unique directory for each "version" of this local
                // resource, we treat the last modified time as an ETag.
                let resource_last_modified = fs::metadata(resource)?
                    .modified()
                    .ok()
                    .and_then(|sys_time| sys_time.elapsed().ok())
                    .map(|duration| format!("{}", duration.as_secs()));
                extraction_dir = Some(self.resource_to_filepath(
                    resource,
                    &resource_last_modified,
                    options.subdir.as_deref(),
                    Some("-extracted"),
                ));
            }
        } else {
            // This is a remote resource, so fetch it to the cache.
            let meta = self.fetch_remote_resource(resource, options.subdir.as_deref())?;

            // Check if we need to extract.
            if options.extract {
                extraction_dir = Some(meta.get_extraction_path());
            }

            cached_path = meta.resource_path;
        }

        if let Some(dirpath) = extraction_dir {
            // Extract archive.
            debug!("Treating {} as archive", resource);

            fs::create_dir_all(dirpath.parent().unwrap())?;

            // Need to acquire a lock here to make sure we don't try to extract
            // the same archive in parallel from multiple processes.
            debug!("Acquiring lock on extraction directory for {}", resource);
            let lock_path = format!("{}.lock", dirpath.to_str().unwrap());
            let filelock = OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .open(lock_path)?;
            filelock.lock_exclusive()?;
            debug!("Lock on extraction directory acquired for {}", resource);

            if !dirpath.is_dir() {
                info!("Extracting {} to {:?}", resource, dirpath);
                let format = ArchiveFormat::parse_from_extension(resource)?;
                extract_archive(&cached_path, &dirpath, &format)?;
            }

            filelock.unlock()?;
            debug!("Lock released on extraction directory for {}", resource);

            Ok(dirpath)
        } else {
            Ok(cached_path)
        }
    }

    /// A convenience method to get the cached path to a resource using the given
    /// cache subdirectory (relative to the cache root).
    ///
    /// This is equivalent to:
    ///
    /// ```rust,no_run
    /// # use cached_path::{Cache, Options};
    /// # let cache = Cache::new().unwrap();
    /// # let subdir = "target";
    /// # let resource = "README.md";
    /// let path = cache.cached_path_with_options(
    ///     resource,
    ///     &Options::default().subdir(subdir),
    /// ).unwrap();
    /// ```
    #[deprecated(
        since = "0.4.4",
        note = "Please use Cache::cached_path_with_options() instead"
    )]
    pub fn cached_path_in_subdir(
        &self,
        resource: &str,
        subdir: Option<&str>,
    ) -> Result<PathBuf, Error> {
        let options = Options::new(subdir, false);
        self.cached_path_with_options(resource, &options)
    }

    fn fetch_remote_resource(&self, resource: &str, subdir: Option<&str>) -> Result<Meta, Error> {
        // Otherwise we attempt to parse the URL.
        let url =
            reqwest::Url::parse(resource).map_err(|_| Error::InvalidUrl(String::from(resource)))?;

        // Ensure root directory exists in case it has changed or been removed.
        if let Some(subdir_path) = subdir {
            fs::create_dir_all(self.dir.join(subdir_path))?;
        } else {
            fs::create_dir_all(&self.dir)?;
        };

        // Find any existing cached versions of resource and check if they are still
        // fresh according to the `freshness_lifetime` setting.
        let versions = self.find_existing(resource, subdir); // already sorted, latest is first.
        if self.offline {
            if !versions.is_empty() {
                info!("Found existing cached version of {}", resource);
                return Ok(versions[0].clone());
            } else {
                error!("Offline mode is enabled but no cached versions of resource exist.");
                return Err(Error::NoCachedVersions(String::from(resource)));
            }
        } else if !versions.is_empty() && versions[0].is_fresh(self.freshness_lifetime) {
            // Oh hey, the latest version is still fresh!
            info!("Latest cached version of {} is still fresh", resource);
            return Ok(versions[0].clone());
        }

        // No existing version or the existing versions are older than their freshness
        // lifetimes, so we'll query for the ETAG of the resource and then compare
        // that with any existing versions.
        let etag = self.try_get_etag(resource, &url)?;
        let path = self.resource_to_filepath(resource, &etag, subdir, None);

        // Before going further we need to obtain a lock on the file to provide
        // parallel downloads of the same resource.
        debug!("Acquiring lock for cache of {}", resource);
        let lock_path = format!("{}.lock", path.to_str().unwrap());
        let filelock = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(lock_path)?;
        filelock.lock_exclusive()?;
        debug!("Lock acquired for {}", resource);

        if path.exists() {
            // Oh cool! The cache is up-to-date according to the ETAG.
            // We'll return the up-to-date version and clean up any other
            // dangling ones.
            info!("Cached version of {} is up-to-date", resource);
            filelock.unlock()?;
            return Meta::from_cache(&path);
        }

        // No up-to-date version cached, so we have to try downloading it.
        let meta = self.try_download_resource(resource, &url, &path, &etag)?;

        info!("New version of {} cached", resource);

        filelock.unlock()?;
        debug!("Lock released for {}", resource);

        Ok(meta)
    }

    /// Find existing versions of a cached resource, sorted by most recent first.
    fn find_existing(&self, resource: &str, subdir: Option<&str>) -> Vec<Meta> {
        let mut existing_meta: Vec<Meta> = vec![];
        let glob_string = format!(
            "{}*.meta",
            self.resource_to_filepath(resource, &None, subdir, None)
                .to_str()
                .unwrap(),
        );
        for meta_path in glob(&glob_string).unwrap().filter_map(Result::ok) {
            if let Ok(meta) = Meta::from_path(&meta_path) {
                existing_meta.push(meta);
            }
        }
        existing_meta
            .sort_unstable_by(|a, b| b.creation_time.partial_cmp(&a.creation_time).unwrap());
        existing_meta
    }

    fn get_retry_delay(&self, retries: u32) -> u32 {
        let between = Uniform::from(0..1000);
        let mut rng = rand::thread_rng();
        std::cmp::min(
            2u32.pow(retries - 1) * 1000 + between.sample(&mut rng),
            self.max_backoff,
        )
    }

    fn try_download_resource(
        &self,
        resource: &str,
        url: &reqwest::Url,
        path: &Path,
        etag: &Option<String>,
    ) -> Result<Meta, Error> {
        let mut retries: u32 = 0;
        loop {
            match self.download_resource(resource, url, path, etag) {
                Ok(meta) => {
                    return Ok(meta);
                }
                Err(err) => {
                    if retries >= self.max_retries {
                        error!("Max retries exceeded for {}", resource);
                        return Err(err);
                    }
                    if !err.is_retriable() {
                        error!("Download failed for {} with fatal error, {}", resource, err);
                        return Err(err);
                    }
                    retries += 1;
                    let retry_delay = self.get_retry_delay(retries);
                    warn!(
                        "Download failed for {}: {}\nRetrying in {} milliseconds...",
                        resource, err, retry_delay
                    );
                    thread::sleep(time::Duration::from_millis(u64::from(retry_delay)));
                }
            }
        }
    }

    fn download_resource(
        &self,
        resource: &str,
        url: &reqwest::Url,
        path: &Path,
        etag: &Option<String>,
    ) -> Result<Meta, Error> {
        debug!("Attempting connection to {}", url);

        let mut response = self
            .http_client
            .get(url.clone())
            .send()?
            .error_for_status()?;

        debug!("Opened connection to {}", url);

        // First we make a temporary file and download the contents of the resource into it.
        // Otherwise if we wrote directly to the cache file and the download got
        // interrupted we could be left with a corrupted cache file.
        let tempfile = NamedTempFile::new_in(path.parent().unwrap())?;
        let mut tempfile_write_handle = OpenOptions::new().write(true).open(tempfile.path())?;

        info!("Starting download of {}", url);

        let bytes = if let Some(progress_bar) = &self.progress_bar {
            let mut download_wrapper = progress_bar.wrap_download(
                resource,
                response.content_length(),
                tempfile_write_handle,
            );
            let bytes = response.copy_to(&mut download_wrapper)?;
            download_wrapper.finish();
            bytes
        } else {
            response.copy_to(&mut tempfile_write_handle)?
        };

        info!("Downloaded {} bytes", bytes);
        debug!("Writing meta file");

        let meta = Meta::new(
            String::from(resource),
            path.into(),
            etag.clone(),
            self.freshness_lifetime,
        );
        meta.to_file()?;

        debug!("Renaming temp file to cache location for {}", url);

        fs::rename(tempfile.path(), path)?;

        Ok(meta)
    }

    fn try_get_etag(&self, resource: &str, url: &reqwest::Url) -> Result<Option<String>, Error> {
        let mut retries: u32 = 0;
        loop {
            match self.get_etag(url) {
                Ok(etag) => return Ok(etag),
                Err(err) => {
                    if retries >= self.max_retries {
                        error!("Max retries exceeded for {}", resource);
                        return Err(err);
                    }
                    if !err.is_retriable() {
                        error!("ETAG fetch for {} failed with fatal error", resource);
                        return Err(err);
                    }
                    retries += 1;
                    let retry_delay = self.get_retry_delay(retries);
                    warn!(
                        "ETAG fetch failed for {}, retrying in {} milliseconds...",
                        resource, retry_delay
                    );
                    thread::sleep(time::Duration::from_millis(u64::from(retry_delay)));
                }
            }
        }
    }

    fn get_etag(&self, url: &reqwest::Url) -> Result<Option<String>, Error> {
        debug!("Fetching ETAG for {}", url);
        let response = self
            .http_client
            .head(url.clone())
            .send()?
            .error_for_status()?;
        if let Some(etag) = response.headers().get(ETAG) {
            if let Ok(s) = etag.to_str() {
                Ok(Some(s.into()))
            } else {
                debug!("No ETAG for {}", url);
                Ok(None)
            }
        } else {
            Ok(None)
        }
    }

    fn resource_to_filepath(
        &self,
        resource: &str,
        etag: &Option<String>,
        subdir: Option<&str>,
        suffix: Option<&str>,
    ) -> PathBuf {
        let resource_hash = hash_str(resource);
        let mut filename = if let Some(tag) = etag {
            let etag_hash = hash_str(&tag[..]);
            format!("{}.{}", resource_hash, etag_hash)
        } else {
            resource_hash
        };

        if let Some(suf) = suffix {
            filename.push_str(suf);
        }

        let filepath = PathBuf::from(filename);

        if let Some(subdir_path) = subdir {
            self.dir.join(subdir_path).join(filepath)
        } else {
            self.dir.join(filepath)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    #[test]
    fn test_url_to_filename_with_etag() {
        let cache_dir = tempdir().unwrap();
        let cache = Cache::builder()
            .dir(cache_dir.path().to_owned())
            .build()
            .unwrap();

        let resource = "http://localhost:5000/foo.txt";
        let etag = String::from("abcd");

        assert_eq!(
            cache
                .resource_to_filepath(resource, &Some(etag), None, None)
                .to_str()
                .unwrap(),
            format!(
                "{}{}{}.{}",
                cache_dir.path().to_str().unwrap(),
                std::path::MAIN_SEPARATOR,
                "b5696dbf866311125e26a62bef0125854dd40f010a70be9cfd23634c997c1874",
                "88d4266fd4e6338d13b845fcf289579d209c897823b9217da3e161936f031589"
            )
        );
    }

    #[test]
    fn test_url_to_filename_no_etag() {
        let cache_dir = tempdir().unwrap();
        let cache = Cache::builder()
            .dir(cache_dir.path().to_owned())
            .build()
            .unwrap();

        let resource = "http://localhost:5000/foo.txt";
        assert_eq!(
            cache
                .resource_to_filepath(resource, &None, None, None)
                .to_str()
                .unwrap(),
            format!(
                "{}{}{}",
                cache_dir.path().to_str().unwrap(),
                std::path::MAIN_SEPARATOR,
                "b5696dbf866311125e26a62bef0125854dd40f010a70be9cfd23634c997c1874",
            )
        );
    }

    #[test]
    fn test_url_to_filename_in_subdir() {
        let cache_dir = tempdir().unwrap();
        let cache = Cache::builder()
            .dir(cache_dir.path().to_owned())
            .build()
            .unwrap();

        let resource = "http://localhost:5000/foo.txt";
        assert_eq!(
            cache
                .resource_to_filepath(resource, &None, Some("target"), None)
                .to_str()
                .unwrap(),
            format!(
                "{}{}{}{}{}",
                cache_dir.path().to_str().unwrap(),
                std::path::MAIN_SEPARATOR,
                "target",
                std::path::MAIN_SEPARATOR,
                "b5696dbf866311125e26a62bef0125854dd40f010a70be9cfd23634c997c1874",
            )
        );
    }

    #[test]
    fn test_url_to_filename_with_suffix() {
        let cache_dir = tempdir().unwrap();
        let cache = Cache::builder()
            .dir(cache_dir.path().to_owned())
            .build()
            .unwrap();

        let resource = "http://localhost:5000/foo.txt";
        assert_eq!(
            cache
                .resource_to_filepath(resource, &None, Some("target"), Some("-extracted"))
                .to_str()
                .unwrap(),
            format!(
                "{}{}{}{}{}-extracted",
                cache_dir.path().to_str().unwrap(),
                std::path::MAIN_SEPARATOR,
                "target",
                std::path::MAIN_SEPARATOR,
                "b5696dbf866311125e26a62bef0125854dd40f010a70be9cfd23634c997c1874",
            )
        );
    }
}
