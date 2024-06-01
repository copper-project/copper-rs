extern crate proc_macro;

use proc_macro::TokenStream;

use quote::quote;
use syn::meta::parser;
use syn::Fields::{Named, Unnamed};
use syn::{parse_macro_input, parse_quote, parse_str, Field, ItemStruct, LitStr, Type, TypeTuple};

use copper::config::CuConfig;
use copper::curuntime::compute_runtime_plan;
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
            Err(meta.error("unsupported property"))
        }
    });

    parse_macro_input!(args with attribute_config_parser);
    let config_file = config_file
        .expect("Expected config file attribute like #[CopperRuntime(config = \"path\")]")
        .value();
    let mut config_full_path = utils::caller_crate_root();
    config_full_path.push(&config_file);
    let copper_config =
        copper::config::read_configuration(config_full_path.as_os_str().to_str().unwrap()).expect(
            &format!("Failed to read configuration file: {:?}", &config_full_path),
        );

    for (node_index, node) in compute_runtime_plan(&copper_config).unwrap() {
        let dst_edges = copper_config.get_dst_edges(node_index);
        for edge_index in dst_edges {
            let edge_type = copper_config.get_edge_weight(edge_index);
            if let Some(edge_type) = edge_type {
                println!("[edge:{}]   {} -> ", edge_index, edge_type);
            }
        }
        println!("   {}: {}", node_index, node.get_id());
        let src_edges = copper_config.get_src_edges(node_index);
        for edge_index in src_edges {
            let edge_type = copper_config.get_edge_weight(edge_index);
            if let Some(edge_type) = edge_type {
                println!("     -> [edge:{}]   {} ", edge_index, edge_type);
            }
        }
    }

    let (all_tasks_types_names, all_tasks_types) = extract_tasks_types(&copper_config);

    // Build the tuple of all those types
    // note the extraneous , at the end is to make the tuple work even if this is only one element
    let task_types_tuple: TypeTuple = parse_quote! {
        (#(#all_tasks_types),*,)
    };

    // add that to a new field
    let runtime_field: Field = parse_quote! {
        copper_runtime: CuRuntime<CuTasks, CuList>
    };

    let (_, all_msgs_types) = extract_msgs_types(&copper_config);

    let msgs_types_tuple: TypeTuple = parse_quote! { (#(#all_msgs_types),*,)};

    let name = &item_struct.ident;

    match &mut item_struct.fields {
        Named(fields_named) => {
            fields_named.named.push(runtime_field);
        }
        Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.push(runtime_field);
        }
        _ => (),
    };

    // Generate the code to create instances of the nodes
    // It maps the types to their index
    let task_instances_init_code: Vec<_> = all_tasks_types
        .iter()
        .enumerate()
        .map(|(index, ty)| {
            let ty_name = &all_tasks_types_names[index];
            let additional_error_info = format!(
                "Failed to get create instance for {}, instance index {}.",
                ty_name, index
            );
            quote! {
                #ty::new(all_instances_configs[#index]).map_err(|e| e.add_cause(#additional_error_info))?
            }
        })
        .collect();

    // Convert the modified struct back into a TokenStream
    let result = quote! {
        use copper::config::CuConfig;
        use copper::config::NodeInstanceConfig;
        use copper::config::read_configuration;
        use copper::curuntime::CuRuntime;
        use copper::cutask::CuTaskLifecycle; // Needed for the instantiation of tasks
        use copper::CuResult;
        use copper::CuError;

        pub type CuTasks = #task_types_tuple;
        pub type CuList = #msgs_types_tuple;


        fn tasks_instanciator(all_instances_configs: Vec<Option<&NodeInstanceConfig>>) -> CuResult<CuTasks> {
            Ok(( #(#task_instances_init_code),*, ))
        }

        pub #item_struct

        impl #name {

            pub fn new() -> CuResult<Self> {
                let config = read_configuration(#config_file)?;

                Ok(#name {
                    copper_runtime: CuRuntime::<CuTasks, CuList>::new(&config, tasks_instanciator)?
                })
            }
        }
    };
    let tokens: TokenStream = result.into();

    // Print and format the generated code using rustfmt
    // println!("Generated tokens: {}", tokens);
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
        .map(|node_config| node_config.get_type().to_string())
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
