extern crate proc_macro;

use proc_macro::TokenStream;

use backtrace::Backtrace;
use std::panic;

use quote::{quote, ToTokens};
use syn::meta::parser;
use syn::Fields::{Named, Unit, Unnamed};
use syn::{parse_macro_input, parse_quote, parse_str, Field, ItemStruct, LitStr, Type, TypeTuple};

use copper::config::CuConfig;
use format::{highlight_rust_code, rustfmt_generated_code};

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
    config_full_path.push(&config_file);
    let config_content = std::fs::read_to_string(&config_full_path)
        .unwrap_or_else(|_| panic!("Failed to read configuration file: {:?}", &config_full_path));

    let copper_config = CuConfig::deserialize(&config_content);

    let (all_tasks_types_names, all_tasks_types) = extract_tasks_types(&copper_config);

    // Build the tuple of all those types
    // note the extraneous , at the end is to make the tuple work even if this is only one element
    let task_types_tuple: TypeTuple = parse_quote! {
        (#(#all_tasks_types),*,)
    };

    // add that to a new field
    let task_instances_field: Field = parse_quote! {
        task_instances: CuTasks
    };

    let (all_msgs_types_names, all_msgs_types) = extract_msgs_types(&copper_config);

    let msgs_types_tuple: TypeTuple = parse_quote! { (#(#all_msgs_types),*,)};

    let copper_lists_field: Field = parse_quote! {
        copper_lists: CuListsManager<CuList, 10>
    };

    let name = &item_struct.ident;

    match &mut item_struct.fields {
        Named(fields_named) => {
            fields_named.named.push(task_instances_field);
            fields_named.named.push(copper_lists_field);
        }
        Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.push(task_instances_field);
            // fields_unnamed.unnamed.push(copper_lists_field);
        }
        Unit => {
            // Handle unit structs if necessary
        }
    };

    // Generate the code to create instances of the nodes
    // It maps the types to their index
    let task_instances_init_code: Vec<_> = all_tasks_types
        .iter()
        .enumerate()
        .map(|(index, ty)| {
            let ty_name = &all_tasks_types_names[index];
            let error = format!(
                "Failed to get create instance for {}, instance index {}.",
                ty_name, index
            );
            quote! {
                #ty::new(all_instances_configs[#index]).expect(#error)
            }
        })
        .collect();

    // Generate the code to create instances of the messages
    let msg_instances_init_code: Vec<_> = all_msgs_types
        .iter()
        .map(|ty| {
            quote! {
                #ty::default()
            }
        })
        .collect();

    // Convert the modified struct back into a TokenStream
    let result = quote! {
        use std::fs::read_to_string;
        use copper::config::CuConfig;
        use copper::config::Node;
        use copper::config::NodeInstanceConfig;
        use copper::common::CuListsManager;
        use copper::cutask::CuSinkTask; // Needed for the instantiation of tasks
        use copper::cutask::CuSrcTask; // Needed for the instantiation of tasks
        use copper::cutask::CuTask; // Needed for the instantiation of tasks
        use copper::CuResult;

        pub type CuList = #msgs_types_tuple;
        pub type CuTasks = #task_types_tuple;

        pub #item_struct

        impl #name {

            pub fn new() -> CuResult<Self> {
                let config_filename = #config_file;

                let config_content = read_to_string(config_filename)
                    .unwrap_or_else(|_| panic!("Failed to read configuration file: {:?}", &config_filename));
                let copper_config = CuConfig::deserialize(&config_content);
                let all_instances_configs: Vec<Option<&NodeInstanceConfig>>  = copper_config.get_all_nodes().iter().map(|node_config| node_config.get_instance_config()).collect();

                let task_instances = (
                    #(#task_instances_init_code),*,
                );
                Ok(#name {
                    task_instances,
                    copper_lists: CuListsManager::new(),
                })
            }

            //fn create_new_copper_list() -> CopperList {
            //    (#(#msg_instances_init_code),*,)
            //}

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

/// Extract all the tasks types in their index order
fn extract_tasks_types(copper_config: &CuConfig) -> (Vec<String>, Vec<Type>) {
    let all_nodes = copper_config.get_all_nodes();

    // Collect all the type names used by our configs.
    let all_types_names: Vec<String> = all_nodes
        .iter()
        .map(|node_config| node_config.get_type_name().to_string())
        .collect();

    // Transform them as Rust types
    let all_types: Vec<Type> = all_types_names
        .iter()
        .map(|name| parse_str(name).unwrap())
        .collect();
    (all_types_names, all_types)
}

/// Extract all the messages types in their index order
fn extract_msgs_types(copper_config: &CuConfig) -> (Vec<String>, Vec<Type>) {
    let all_edges = copper_config.get_all_edges();

    let all_types_names: Vec<String> = all_edges
        .iter()
        .map(|edge| {
            let (_, _, type_name) = edge;
            type_name.clone()
        })
        .collect();

    // Transform them as Rust types
    let all_types: Vec<Type> = all_types_names
        .iter()
        .map(|name| parse_str(name).unwrap())
        .collect();
    (all_types_names, all_types)
}
