# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

## [v0.6.1](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.6.1) - 2023-02-24

### Fixed

- Fixed `RUSTSEC-2020-0071`

## [v0.6.0](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.6.0) - 2022-12-19

### Changed

- Consolidated HTTP error variants into a single variant, `HttpError`, which sources directly from the underlying `reqwest::Error` for better error messages.


## [v0.5.3](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.5.3) - 2022-03-07

### Added

- Added feature flag `rustls-tls` to make use of `rustls-tls` over `native-tls` in `reqwest`.

## [v0.5.2](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.5.2) - 2022-03-07

## [v0.5.1](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.4.5) - 2020-03-29

## [v0.5.0](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.4.5) - 2020-01-29

### Changed

- Switched to `color-eyre` for error handling in the CLI.
- Improved full download progress bar.
- `Some(ProgressBar::Full)` is now the default for the library.
- Upgraded `reqwest` dependency to `0.11`.

## [v0.4.5](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.4.5) - 2020-09-15

### Added

- Added a method `CacheBuilder::progress_bar` to set the progress bar type, or to disable the progess bar entirely. The options are `ProgressBar::Light` and `ProgressBar::Full`. The default when using `cached-path` as a library is `ProgressBar::Light`, while the default from the command-line is `ProgressBar::Full`. You can also disable the progress bar from the command-line by passing the  "-q" / "--quietly" flag.

## [v0.4.4](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.4.4) - 2020-09-13

### Added

- Added a method `Cache::cached_path_with_options` and a corresponding `Options` struct.
- Added ability to automatically extract archives through the `Cache::cached_path_with_options` method.
- Added integration tests.
- Added spinner progress bar to downloads.

### Changed

- `Meta` struct is no longer public.
- `Cache::cached_path_in_subdir` is now deprecated.
- `httpmock` updated and tests refactored.

### Removed

- Removed the `only_keep_latest` setting for the `Cache`.

## [v0.4.3](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.4.3) - 2020-09-11

### Changed

- Updated the download method to stream the response into the target file.

## [v0.4.2](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.4.2) - 2020-09-11

### Fixed

- `cached-path` now compiles on Windows.

## [v0.4.1](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.4.1) - 2020-09-10

### Added

- Added a method `Cache::cached_path_in_subdir` to use a specified subdirectory of the cache root.

### Fixed

- Ensure cache directory exists every time `Cache::cached_path` or `Cache::cached_path_in_subdir` is called.

## [v0.4.0](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.4.0) - 2020-09-10

### Fixed

- Fixed default timeout of `None`.
- `Meta` is now written to file before the tempfile of a downloaded resource is moved to its final cache location. This avoids a bug (albeit, an unlikely one) where the cache could be corrupted if writing the `Meta` to file fails.

## [v0.4.0-rc1](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.4.0-rc1) - 2020-09-09

### Changed

- Switched to using `thiserror` and `anyhow` for error handling.

## [v0.3.0](https://github.com/epwalsh/rust-cached-path/releases/tag/v0.3.0) - 2020-06-13

### Changed

- API is now syncronous
- `root` configuration option renamed to `dir`.

## v0.2.0

### Added

- Added a file lock mechanism to make guard against parallel downloads of the same file.
- Added an "offline" mode.

### Changed

- Minor improvements to internal logic that make caching more robust.
