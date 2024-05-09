use cargo_metadata::{MetadataCommand, Package};
use std::fs;
use std::path::Path;

fn main() {
    let metadata = MetadataCommand::new()
        .exec()
        .expect("Failed to fetch metadata");

    for package in metadata.packages {
        // println!("cargo:warning=Found package {}", package.name);
        check_metadata(&package);
    }
}

fn check_metadata(package: &Package) {
    if let Some(metadata) = package.metadata.as_object() {
        if let Some(copper_plugin_type) = metadata.get("copper_plugin_type") {
            println!("cargo:warning=  --> Found copper-plugin-type in {}: {}", package.name, copper_plugin_type);
        }
    }
}

