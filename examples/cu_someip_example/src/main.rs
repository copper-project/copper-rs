//! SOME/IP Service Example
//!
//! Demonstrates a SOME/IP pipeline: SomeIpSource → SomeIpRouter → SomeIpSink.
//! Uses mock mode — no real UDP sockets required.

use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use std::fs;
use std::path::{Path, PathBuf};

#[copper_runtime(config = "copperconfig.ron")]
struct SomeIpExampleApplication {}

const SLAB_SIZE: Option<usize> = Some(64 * 1024 * 1024);

fn main() {
    let logger_path = "logs/someip_example.copper";
    if let Some(parent) = Path::new(logger_path).parent()
        && !parent.exists()
    {
        fs::create_dir_all(parent).expect("Failed to create logs directory");
    }

    let copper_ctx = basic_copper_setup(&PathBuf::from(logger_path), SLAB_SIZE, true, None)
        .expect("Failed to setup logger.");
    let mut application = SomeIpExampleApplicationBuilder::new()
        .with_context(&copper_ctx)
        .build()
        .expect("Failed to create SOME/IP example application.");

    println!("Starting SOME/IP example pipeline (SomeIpSource → SomeIpRouter → SomeIpSink)...");
    if let Err(error) = application.run() {
        println!("Application ended: {error}");
    }
}

