//! The idea behind `cached-path` is to provide a unified, simple interface for
//! accessing both local and remote files. This can be used behind other APIs that need
//! to access files agnostic to where they are located.
//!
//! This is based on the Python library [`allenai/cached_path`](https://github.com/allenai/cached_path).
//!
//! ## Installation
//!
//! `cached-path` can be used as both a library and a command-line tool. To install `cached-path`
//! as a command-line tool, run
//!
//! ```bash
//! cargo install --features build-binary cached-path
//! ```
//!
//! ## Usage
//!
//! For remote resources, `cached-path` downloads and caches the resource, using the ETAG
//! to know when to update the cache. The path returned is the local path to the latest
//! cached version:
//!
//! ```rust
//! use cached_path::cached_path;
//!
//! let path = cached_path(
//!     "https://github.com/epwalsh/rust-cached-path/blob/main/README.md"
//! ).unwrap();
//! assert!(path.is_file());
//! ```
//!
//! ```bash
//! # From the command line:
//! $ cached-path https://github.com/epwalsh/rust-cached-path/blob/main/README.md
//! /tmp/cache/055968a99316f3a42e7bcff61d3f590227dd7b03d17e09c41282def7c622ba0f.efa33e7f611ef2d163fea874ce614bb6fa5ab2a9d39d5047425e39ebe59fe782
//! ```
//!
//! For local files, the path returned is just the original path supplied:
//!
//! ```rust
//! use cached_path::cached_path;
//!
//! let path = cached_path("README.md").unwrap();
//! assert_eq!(path.to_str().unwrap(), "README.md");
//! ```
//!
//! ```bash
//! # From the command line:
//! $ cached-path README.md
//! README.md
//! ```
//!
//! For resources that are archives, like `*.tar.gz` files, `cached-path` can also
//! automatically extract the files:
//!
//! ```rust
//! use cached_path::{cached_path_with_options, Options};
//!
//! let path = cached_path_with_options(
//!     "https://raw.githubusercontent.com/epwalsh/rust-cached-path/main/test_fixtures/utf-8_sample/archives/utf-8.tar.gz",
//!     &Options::default().extract(),
//! ).unwrap();
//! assert!(path.is_dir());
//! ```
//!
//! ```bash
//! # From the command line:
//! $ cached-path --extract https://raw.githubusercontent.com/epwalsh/rust-cached-path/main/test_fixtures/utf-8_sample/archives/utf-8.tar.gz
//! README.md
//! ```
//!
//! It's also easy to customize the cache location, the HTTP client, and other options
//! using a [`CacheBuilder`](crate::cache::CacheBuilder) to construct a custom
//! [`Cache`](crate::cache::Cache) object. This is the recommended thing
//! to do if your application makes multiple calls to `cached_path`, since it avoids the overhead
//! of creating a new HTTP client on each call:
//!
//! ```rust
//! use cached_path::Cache;
//!
//! let cache = Cache::builder()
//!     .dir(std::env::temp_dir().join("my-cache/"))
//!     .connect_timeout(std::time::Duration::from_secs(3))
//!     .build().unwrap();
//! let path = cache.cached_path("README.md").unwrap();
//! ```
//!
//! ```bash
//! # From the command line:
//! $ cached-path --dir /tmp/my-cache/ --connect-timeout 3 README.md
//! README.md
//! ```

use std::path::PathBuf;

pub(crate) mod archives;
mod cache;
mod error;
pub(crate) mod meta;
mod progress_bar;
pub(crate) mod utils;

pub use crate::cache::{Cache, CacheBuilder, Options};
pub use crate::error::Error;
pub use crate::progress_bar::ProgressBar;

/// Get the cached path to a resource.
///
/// This is equivalent to calling [`Cache::cached_path`](crate::cache::Cache#method.cached_path)
/// with a temporary [`Cache`](crate::cache::Cache) object.
/// Therefore if you're going to be calling this function multiple times,
/// it's more efficient to create and use a single `Cache` instead.
pub fn cached_path(resource: &str) -> Result<PathBuf, Error> {
    let cache = Cache::builder().build()?;
    cache.cached_path(resource)
}

/// Get the cached path to a resource using the given options.
///
/// This is equivalent to calling
/// [`Cache::cached_path_with_options`](crate::cache::Cache#method.cached_path_with_options)
/// with a temporary [`Cache`](crate::cache::Cache) object.
/// Therefore if you're going to be calling this function multiple times,
/// it's more efficient to create and use a single `Cache` instead.
pub fn cached_path_with_options(resource: &str, options: &Options) -> Result<PathBuf, Error> {
    let cache = Cache::builder().build()?;
    cache.cached_path_with_options(resource, options)
}

#[cfg(test)]
mod test;
