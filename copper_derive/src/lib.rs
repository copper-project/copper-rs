extern crate proc_macro;
use std::path::PathBuf;
use std::process::Command;
use std::io::Write;

use syn::Attribute;
use proc_macro::TokenStream;

use syn::parse::{Parse, ParseStream, Parser, Result};
use syn::meta::ParseNestedMeta;
use syn::{parse, parse_macro_input, punctuated::Punctuated, Ident, ItemStruct, LitStr, Token};

use quote::quote;
use walkdir::WalkDir;

use syntect::easy::HighlightLines;
use syntect::highlighting::{ThemeSet, Style};
use syntect::parsing::SyntaxSet;
use syntect::util::{as_24_bit_terminal_escaped, LinesWithEndings};
use syntect::highlighting::Color;
use syntect::highlighting::Theme;

struct Args {
    pub vars: Vec<LitStr>,
}

impl Parse for Args {
    fn parse(input: ParseStream) -> Result<Self> {
        let vars = Punctuated::<syn::LitStr, Token![,]>::parse_terminated(input)?;
        Ok(Args {
            vars: vars.into_iter().collect::<Vec<LitStr>>(),
        })
    }
}

// Parses the CopperRuntime attribute like #[CopperRuntime(config = "path")]
#[proc_macro_attribute]
pub fn CopperRuntime(args: TokenStream, input: TokenStream) -> TokenStream {
    let mut item_struct = parse_macro_input!(input as ItemStruct);

    let mut config_file: Option<LitStr> = None;
     let attribute_config_parser = syn::meta::parser(|meta| {
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
    
    let new_field: syn::Field = syn::parse_quote! {
        node_instances: (u32, i32)
    };
    
    match &mut item_struct.fields {
        syn::Fields::Named(fields_named) => {
            fields_named.named.push(new_field);
        },
        syn::Fields::Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.push(new_field);
        },
        syn::Fields::Unit => {
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
    // println!("\n     ===    Gen. Runtime ===\n");
    println!("{}", highlight_rust_code(formatted_code));
    // println!("\n     === === === === === ===\n");

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

fn rustfmt_generated_code(code: String) -> String {
    let mut rustfmt = Command::new("rustfmt")
        .arg("--emit")
        .arg("stdout")
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .spawn()
        .expect("Failed to spawn rustfmt");

    {
        let stdin = rustfmt.stdin.as_mut().expect("Failed to open stdin");
        stdin.write_all(code.as_bytes()).expect("Failed to write to stdin");
    }

    let output = rustfmt.wait_with_output().expect("Failed to read stdout");
    String::from_utf8(output.stdout).expect("Output was not valid UTF-8")
}

fn create_black_theme() -> Theme {
    let mut theme = ThemeSet::load_defaults().themes["base16-ocean.dark"].clone();
    theme.settings.background = Some(Color { r: 0, g: 0, b: 0, a: 255 });
    theme
}

fn highlight_rust_code(code: String) -> String {
    let ps = SyntaxSet::load_defaults_newlines();
    let syntax = ps.find_syntax_by_extension("rs").unwrap();
    let theme = create_black_theme();
    let mut h = HighlightLines::new(syntax, &theme);

    let mut highlighted_code = String::new();

    for line in LinesWithEndings::from(&code) {
        let ranges: Vec<(Style, &str)> = h.highlight_line(line, &ps).unwrap();
        let escaped = as_24_bit_terminal_escaped(&ranges[..], true);
        highlighted_code.push_str(&escaped);
    }

    highlighted_code
}
