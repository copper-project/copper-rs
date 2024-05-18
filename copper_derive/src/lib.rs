extern crate proc_macro;

use copper::config::CopperConfig;
use format::{highlight_rust_code, rustfmt_generated_code};
use proc_macro::TokenStream;
use quote::quote;
use syn::meta::parser;
use syn::Fields::{Named, Unit, Unnamed};
use syn::{parse_macro_input, parse_quote, Field, ItemStruct, LitStr};

mod format;
mod utils;

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
    let config_file = config_file
        .expect("Expected config file attribute like #[CopperRuntime(config = \"path\")]")
        .value();
    let mut config_full_path = utils::caller_crate_root();
    config_full_path.push(config_file);
    let config_content = std::fs::read_to_string(&config_full_path)
        .unwrap_or_else(|_| panic!("Failed to read configuration file: {:?}", &config_full_path));
    println!("Config content:\n {}", config_content);
    let deserialized = CopperConfig::deserialize(&config_content);

    let name = &item_struct.ident;

    let new_field: Field = parse_quote! {
        node_instances: (u32, i32)
    };

    match &mut item_struct.fields {
        Named(fields_named) => {
            fields_named.named.push(new_field);
        }
        Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.push(new_field);
        }
        Unit => {
            // Handle unit structs if necessary
        }
    };

    // Convert the modified struct back into a TokenStream
    let result = quote! {
        pub #item_struct
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
