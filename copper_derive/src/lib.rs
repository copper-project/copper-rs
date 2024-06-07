extern crate proc_macro;

use proc_macro::TokenStream;

use quote::quote;
use syn::meta::parser;
use syn::Fields::{Named, Unnamed};
use syn::{parse_macro_input, parse_quote, parse_str, Field, ItemStruct, LitStr, Type, TypeTuple};

use copper::config::{Node, NodeId, read_configuration};
use copper::config::CuConfig;
use copper::curuntime::compute_runtime_plan;
use format::{highlight_rust_code, rustfmt_generated_code};

mod format;
mod utils;

// TODO: this needs to be determined when the runtime is sizing itself.
const DEFAULT_CLNB: usize = 10;

// Parses the CopperRuntime attribute like #[copper_runtime(config = "path")]
#[proc_macro_attribute]
pub fn copper_runtime(args: TokenStream, input: TokenStream) -> TokenStream {
    println!("[entry]");
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

    println!("[parse]");
    parse_macro_input!(args with attribute_config_parser);
    let config_file = config_file
        .expect("Expected config file attribute like #[CopperRuntime(config = \"path\")]")
        .value();
    let mut config_full_path = utils::caller_crate_root();
    config_full_path.push(&config_file);
    let filename = config_full_path
        .as_os_str()
        .to_str()
        .expect("Could not interpret the config file name");
    let copper_config = read_configuration(filename).expect(&format!(
        "Failed to read configuration file: {:?}",
        &config_full_path
    ));

    let runtime_plan: Vec<(NodeId, &Node)> =
        compute_runtime_plan(&copper_config).expect("Could not compute runtime plan");
    println!("[runtime plan]");
    for (_, node) in runtime_plan {
        println!("-> {} ({})", node.get_id(), node.get_type());
    }
    println!("------------------------------");
    //for (node_index, node) in runtime_plan {
    //    let dst_edges = copper_config.get_dst_edges(node_index);
    //    for edge_index in dst_edges {
    //        let edge_type = copper_config.get_edge_weight(edge_index);
    //        if let Some(edge_type) = edge_type {
    //            println!("[edge:{}]   {} -> ", edge_index, edge_type);
    //        }
    //    }
    //    println!("   {}: {}", node_index, node.get_id());
    //    let src_edges = copper_config.get_src_edges(node_index);
    //    for edge_index in src_edges {
    //        let edge_type = copper_config.get_edge_weight(edge_index);
    //        if let Some(edge_type) = edge_type {
    //            println!("     -> [edge:{}]   {} ", edge_index, edge_type);
    //        }
    //    }
    //}

    println!("[extract tasks types]");
    let (all_tasks_types_names, all_tasks_types) = extract_tasks_types(&copper_config);

    println!("[build task tuples]");
    // Build the tuple of all those types
    // note the extraneous , at the end is to make the tuple work even if this is only one element
    let task_types_tuple: TypeTuple = parse_quote! {
        (#(#all_tasks_types),*,)
    };

    println!("[build runtime field]");
    // add that to a new field
    let runtime_field: Field = parse_quote! {
        copper_runtime: _CuRuntime<CuTasks, CuList, #DEFAULT_CLNB>
    };

    println!("[extract msg types]");
    let (_, all_msgs_types) = extract_msgs_types(&copper_config);

    println!("[build the copper list tuple]");
    let msgs_types_tuple: TypeTuple = if all_msgs_types.is_empty() {
        parse_quote! {()}
        // panic!("No messages types found. You need to at least define one message between tasks.");
    } else {
        parse_quote! { (#(#all_msgs_types),*,)}
    };

    // let msgs_types_tuple: TypeTuple = parse_quote! { (#(#all_msgs_types),*,)};

    let name = &item_struct.ident;

    println!("[match struct anonymity]");
    match &mut item_struct.fields {
        Named(fields_named) => {
            fields_named.named.push(runtime_field);
        }
        Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.push(runtime_field);
        }
        _ => (),
    };

    println!("[gen instances]");
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

    println!("[build result]");
    // Convert the modified struct back into a TokenStream
    let result = quote! {
        // import everything with an _ to avoid clashes with the user's code
        use copper::config::CuConfig as _CuConfig;
        use copper::config::NodeInstanceConfig as _NodeInstanceConfig;
        use copper::config::read_configuration as _read_configuration;
        use copper::curuntime::CuRuntime as _CuRuntime;
        use copper::cutask::CuTaskLifecycle as _CuTaskLifecycle; // Needed for the instantiation of tasks
        use copper::CuResult as _CuResult;
        use copper::CuError as _CuError;

        pub type CuTasks = #task_types_tuple;
        pub type CuList = #msgs_types_tuple;


        fn tasks_instanciator(all_instances_configs: Vec<Option<&_NodeInstanceConfig>>) -> _CuResult<CuTasks> {
            Ok(( #(#task_instances_init_code),*, ))
        }

        pub #item_struct

        impl #name {

            pub fn new() -> _CuResult<Self> {
                let config = _read_configuration(#config_file)?;

                Ok(#name {
                    copper_runtime: _CuRuntime::<CuTasks, CuList, #DEFAULT_CLNB>::new(&config, tasks_instanciator)?
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
        .map(|name| {
            parse_str(name)
                .expect(format!("Could not transform {} into a Task Rust type.", name).as_str())
        })
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
    println!("extract_msgs_types: all_types names: {:?}", all_types_names);

    // Transform them as Rust types
    let all_types: Vec<Type> = all_types_names
        .iter()
        .map(|name| {
            parse_str(name)
                .expect(format!("Could not transform {} into a Msg Rust type.", name).as_str())
        })
        .collect();

    println!("extract_msgs_types: {} types extracted", all_types.len());
    (all_types_names, all_types)
}
