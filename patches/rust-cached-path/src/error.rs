use thiserror::Error;

/// Errors that can occur during caching.
#[derive(Error, Debug)]
pub enum Error {
    /// Arises when the resource looks like a local file but it doesn't exist.
    #[error("Treated resource as local file, but file does not exist ({0})")]
    ResourceNotFound(String),

    /// Arises when the resource looks like a URL, but is invalid.
    #[error("Unable to parse resource URL ({0})")]
    InvalidUrl(String),

    /// Arises when the cache is being used in offline mode, but it couldn't locate
    /// any cached versions of a remote resource.
    #[error("Offline mode is enabled but no cached versions of resouce exist ({0})")]
    NoCachedVersions(String),

    /// Arises when the cache is corrupted for some reason.
    ///
    /// If this error occurs, it is almost certainly the result of an external process
    /// "messing" with the cache directory, since `cached-path` takes great care
    /// to avoid accidental corruption on its own.
    #[error("Cache is corrupted ({0})")]
    CacheCorrupted(String),

    /// Arises when a resource is treated as archive, but the extraction process fails.
    #[error("Extracting archive failed ({0})")]
    ExtractionError(String),

    /// Any IO error that could arise while attempting to cache a remote resource.
    #[error("An IO error occurred")]
    IoError(#[from] std::io::Error),

    /// An HTTP error that could occur while attempting to fetch a remote resource.
    #[error(transparent)]
    HttpError(#[from] reqwest::Error),
}

impl Error {
    pub(crate) fn is_retriable(&self) -> bool {
        match self {
            Error::HttpError(source) => {
                if source.is_status() {
                    matches!(
                        source.status().map(|status| status.as_u16()),
                        Some(502) | Some(503) | Some(504)
                    )
                } else {
                    source.is_timeout()
                }
            }
            _ => false,
        }
    }

    pub fn status_code(&self) -> Option<u16> {
        if let Error::HttpError(inner) = self {
            Some(inner.status().unwrap().as_u16())
        } else {
            None
        }
    }
}
