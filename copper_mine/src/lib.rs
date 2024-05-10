extern crate proc_macro;
use syn::Expr;
use syn::Lit;
use syn::MetaNameValue;
use syn::Meta;
use syn::DeriveInput;
use syn::parse_macro_input;
use proc_macro::TokenStream;
use quote::quote;
use std::path::PathBuf;
use walkdir::WalkDir;

#[proc_macro_derive(CopperRuntime, attributes(config))]
pub fn generate_runtime(input: TokenStream) -> TokenStream {
    println!("generate runtime called");
    let input: DeriveInput = parse_macro_input!(input as DeriveInput);


    let name = &input.ident;

    // Find the attribute and read the configuration file
    let mut config_file:Option<String> = None;
    for attr in &input.attrs {
        // Search of another attribute called "config" on the same structure
        if attr.path().is_ident("config") {
            // parse it as a key / value pair with an equal in between.
            if let Ok(Meta::NameValue(MetaNameValue { value: Expr::Lit(value), ..})) = attr.parse_args() {
                // the path should be a string litteral.
                if let Lit::Str(strlit) = value.lit {
                    config_file = Some(strlit.value());
                }
            }
            else {
                panic!("The config attribute for CopperRuntime is malformed. The syntax is #[config=\"filepath\"] after the #[CopperRuntime].")
            }
        }
        else {
            panic!("A CopperRuntime needs a #[config=\"filepath\"] attribute after it.");
        }
    }
    if config_file.is_none() {
        panic!("No CopperRuntime found in your project, you need to tag a struct with #[CopperRuntime].");
    }
    let config_file = PathBuf::from(config_file.unwrap());
    let mut config_full_path = caller_crate_root();
    config_full_path.push(config_file);
    println!("Called on: {:?}", config_full_path);

    let config_content = std::fs::read_to_string(&config_full_path).unwrap_or_else(|_| panic!("Failed to read configuration file: {:?}", &config_full_path));

    println!("Config content: {}", config_content);


    // Generate code based on the configuration
    // let field = "myfield";
    // let value = "myvalue";
    // let expanded = quote! {
    //     impl #name {
    //         pub fn new() -> Self {
    //             Self {
    //                 #field: #value.to_string(),
    //             }
    //         }
    //     }
    // };
    let expanded = quote! {};
    TokenStream::from(expanded)
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

