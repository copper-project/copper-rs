extern crate proc_macro;
mod format;

use std::path::PathBuf;

use syn::Fields::{Named, Unnamed, Unit};
use syn::{parse_macro_input, parse_quote, ItemStruct, LitStr, Field};
use syn::meta::parser;
use proc_macro::TokenStream;

use quote::quote;
use walkdir::WalkDir;

use format::{rustfmt_generated_code, highlight_rust_code};


// Parses the CopperRuntime attribute like #[copper_runtime(config = "path")]
#[proc_macro_attribute]
pub fn copper_runtime(args: TokenStream, input: TokenStream) -> TokenStream {
    let mut item_struct = parse_macro_input!(input as ItemStruct);

    let mut config_file: Option<LitStr> = None;
     let attribute_config_parser = parser(|meta| {
        if meta.path.is_ident("config") {
            config_file = Some(meta.value()?.parse()?);
            Ok(())
        } else {
            Err(meta.error("unsupported tea property"))
        }
    });

    parse_macro_input!(args with attribute_config_parser);
    let config_file = config_file.expect("Expected config file attribute like #[CopperRuntime(config = \"path\")]").value();
    let mut config_full_path = caller_crate_root();
    config_full_path.push(config_file);
    let config_content = std::fs::read_to_string(&config_full_path).unwrap_or_else(|_| panic!("Failed to read configuration file: {:?}", &config_full_path));
    println!("Config content:\n {}", config_content);

    let name = &item_struct.ident;
    
    let new_field: Field = parse_quote! {
        node_instances: (u32, i32)
    };
    
    match &mut item_struct.fields {
        Named(fields_named) => {
            fields_named.named.push(new_field);
        },
        Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.push(new_field);
        },
        Unit => {
            // Handle unit structs if necessary
        }
    };

    // Convert the modified struct back into a TokenStream
    let result = quote! {
        #item_struct
        impl #name {
            pub fn hello(&self) {
                println!("Hello from CopperRuntime");
            }
        }
    };

    let tokens: TokenStream = result.into();

    // Print and format the generated code using rustfmt
    println!("Generated tokens: {}", tokens);
    let formatted_code = rustfmt_generated_code(tokens.to_string());
    println!("\n     ===    Gen. Runtime ===\n");
    println!("{}", highlight_rust_code(formatted_code));
    println!("\n     === === === === === ===\n");

    tokens
}

// Lifted this HORROR but it works.
fn caller_crate_root() -> PathBuf {
    let crate_name =
        std::env::var("CARGO_PKG_NAME").expect("failed to read ENV var `CARGO_PKG_NAME`!");
    let current_dir = std::env::current_dir().expect("failed to unwrap env::current_dir()!");
    let search_entry = format!("name=\"{crate_name}\"");
    for entry in WalkDir::new(&current_dir)
        .into_iter()
        .filter_entry(|e| !e.file_name().eq_ignore_ascii_case("target"))
    {
        let Ok(entry) = entry else { continue };
        if !entry.file_type().is_file() {
            continue;
        }
        let Some(file_name) = entry.path().file_name() else { continue };
        if !file_name.eq_ignore_ascii_case("Cargo.toml") {
            continue;
        }
        let Ok(cargo_toml) = std::fs::read_to_string(entry.path()) else {
            continue
        };
        if cargo_toml
            .chars()
            .filter(|&c| !c.is_whitespace())
            .collect::<String>()
            .contains(search_entry.as_str())
        {
            return entry.path().parent().unwrap().to_path_buf();
        }
    }
    current_dir
}

