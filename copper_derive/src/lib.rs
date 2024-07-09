extern crate proc_macro;

use proc_macro::TokenStream;

use quote::{quote, ToTokens};
use syn::meta::parser;
use syn::Fields::{Named, Unnamed};
use syn::{parse_macro_input, parse_quote, parse_str, Field, ItemStruct, LitStr, Type, TypeTuple};

use copper::config::read_configuration;
use copper::config::CuConfig;
use copper::curuntime::{compute_runtime_plan, CuExecutionStep, CuTaskType};
use format::{highlight_rust_code, rustfmt_generated_code};

mod format;
mod utils;

// TODO: this needs to be determined when the runtime is sizing itself.
const DEFAULT_CLNB: usize = 10;

#[inline]
fn int2index(i: u32) -> syn::Index {
    syn::Index::from(i as usize)
}

#[proc_macro]
pub fn gen_culist_payload(config_path_lit: TokenStream) -> TokenStream {
    let mut config = parse_macro_input!(config_path_lit as LitStr).value();
    println!("[gen culist payload with {:?}]", config);
    let cuconfig = read_config(&config);
    let runtime_plan: Vec<CuExecutionStep> =
        compute_runtime_plan(&cuconfig).expect("Could not compute runtime plan");
    let all_msgs_types_in_culist_order = extract_msg_types(&runtime_plan);
    build_culist_payload(&all_msgs_types_in_culist_order)
        .into_token_stream()
        .into()
}

/// Adds #[copper_runtime(config = "path")] to your application struct to generate the runtime.
/// This will add a "runtime" field to your struct and implement the "new" and "run" methods.
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
    let copper_config = read_config(&config_file);

    println!("[runtime plan]");
    let runtime_plan: Vec<CuExecutionStep> =
        compute_runtime_plan(&copper_config).expect("Could not compute runtime plan");

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
        copper_runtime: _CuRuntime<CuTasks, CuPayload, #DEFAULT_CLNB>
    };

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

    let runtime_plan_code: Vec<_> = runtime_plan
        .iter()
        .map(|step| {
            println!(
                "{} -> {} as {:?}. Input={:?}, Output={:?}",
                step.node.get_id(),
                step.node.get_type(),
                step.task_type,
                step.input_msg_type,
                step.output_msg_type
            );

            let node_index = int2index(step.node_id);
            let task_instance = quote! { self.copper_runtime.task_instances.#node_index };
            let comment_str = format!(
                "/// {} ({:?}) I:{:?} O:{:?}",
                step.node.get_id(),
                step.task_type,
                step.input_msg_type,
                step.output_msg_type
            );
            let comment_tokens: proc_macro2::TokenStream = parse_str(&comment_str).unwrap();

            let process_call = match step.task_type {
                CuTaskType::Source => {
                    let output_culist_index = int2index(
                        step.culist_output_index
                            .expect("Src task should have an output message index."),
                    );
                    quote! {
                        {
                            #comment_tokens
                            let cumsg_output = &mut payload.#output_culist_index;
                            cumsg_output.metadata.before_process = self.copper_runtime.clock.now().into();
                            #task_instance.process(&self.copper_runtime.clock, cumsg_output)?;
                            cumsg_output.metadata.after_process = self.copper_runtime.clock.now().into();
                        }
                    }
                }
                CuTaskType::Sink => {
                    let input_culist_index = int2index(
                        step.culist_input_index
                            .expect("Sink task should have an input message index."),
                    );
                    quote! {
                        {
                            #comment_tokens
                            let cumsg_input = &mut payload.#input_culist_index;
                            #task_instance.process(&self.copper_runtime.clock, cumsg_input)?;
                        }
                    }
                }
                CuTaskType::Regular => {
                    let input_culist_index = int2index(
                        step.culist_input_index
                            .expect("Sink task should have an input message index."),
                    );
                    let output_culist_index = int2index(
                        step.culist_output_index
                            .expect("Src task should have an output message index."),
                    );
                    quote! {
                    {
                        #comment_tokens
                        let cumsg_input = &mut payload.#input_culist_index;
                        let cumsg_output = &mut payload.#output_culist_index;
                        cumsg_output.metadata.before_process = self.copper_runtime.clock.now().into();
                        #task_instance.process(&self.copper_runtime.clock, cumsg_input, cumsg_output)?;
                        cumsg_output.metadata.after_process = self.copper_runtime.clock.now().into();
                    }
                    }
                }
            };

            process_call
        })
        .collect();

    println!("[extract msg types]");
    let all_msgs_types_in_culist_order: Vec<Type> = extract_msg_types(&runtime_plan);

    println!("[build the copper payload]");
    let msgs_types_tuple: TypeTuple = build_culist_payload(&all_msgs_types_in_culist_order);

    println!("[build the collect metadata function]");
    let culist_size = all_msgs_types_in_culist_order.len();
    let culist_indices = (0..(culist_size as u32)).map(int2index);
    let collect_metadata_function = quote! {
        pub fn collect_metadata<'a>(culist: &'a CuList) -> [&'a _CuMsgMetadata; #culist_size] {
            [#( &culist.payload.#culist_indices.metadata, )*]
        }
    };

    println!("[build the run method]");
    let run_method = quote! {

        #[inline]
        pub fn run_one_iteration(&mut self) -> _CuResult<()> {
            {
                let mut culist = &mut self.copper_runtime.copper_lists.create().expect("Ran out of space for copper lists"); // FIXME: error handling.
                let id = culist.id;
                culist.change_state(copper::copperlist::CopperListState::Processing);
                {
                    let payload = &mut culist.payload;
                    #(#runtime_plan_code)*
                } // drop(payload);

                {
                    let md = collect_metadata(&culist);
                    let e2e = md.last().unwrap().after_process.unwrap() - md.first().unwrap().before_process.unwrap();
                    let e2en: u64 = e2e.into();
                    debug!("End to end latency {}, mean latency per node: {}", e2e, e2en / (md.len() as u64));
                } // drop(md);

           }// drop(culist); avoids a double mutable borrow
           Ok(())
        }

        pub fn run(&mut self) -> _CuResult<()> {
            loop {
                self.run_one_iteration()?;
            }
        }
    };

    println!("[build result]");
    // Convert the modified struct back into a TokenStream
    let result = quote! {
        // import everything with an _ to avoid clashes with the user's code
        use copper::config::CuConfig as _CuConfig;
        use copper::config::NodeInstanceConfig as _NodeInstanceConfig;
        use copper::config::read_configuration as _read_configuration;
        use copper::curuntime::CuRuntime as _CuRuntime;
        use copper::CuResult as _CuResult;
        use copper::CuError as _CuError;
        use copper::cutask::CuTaskLifecycle as _CuTaskLifecycle; // Needed for the instantiation of tasks
        use copper::cutask::CuSrcTask as _CuSrcTask;
        use copper::cutask::CuSinkTask as _CuSinkTask;
        use copper::cutask::CuTask as _CuTask;
        use copper::cutask::CuMsg as _CuMsg;
        use copper::cutask::CuMsgMetadata as _CuMsgMetadata;
        use copper::copperlist::CopperList as _CopperList;
        use copper::clock::RobotClock as _RobotClock;
        use copper::clock::OptionCuTime as _OptionCuTime;
        use std::sync::Arc as _Arc;
        use std::sync::Mutex as _Mutex;
        use copper_unifiedlog::stream_write as _stream_write;
        use copper_unifiedlog::UnifiedLoggerWrite as _UnifiedLoggerWrite;
        use copper_traits::UnifiedLogType as _UnifiedLogType;
        use copper::clock::ClockProvider as _ClockProvider;

        // This is the heart of everything.
        // CuTasks is the list of all the tasks types.
        // CuList is a CopperList with the list of all the messages types as payload.
        pub type CuTasks = #task_types_tuple;
        pub type CuPayload = #msgs_types_tuple;
        pub type CuList = _CopperList<CuPayload>;

        // This generates a way to get the metadata of every single message of a culist at low cost
        #collect_metadata_function

        fn tasks_instanciator(all_instances_configs: Vec<Option<&_NodeInstanceConfig>>) -> _CuResult<CuTasks> {
            Ok(( #(#task_instances_init_code),*, ))
        }

        pub #item_struct

        impl #name {

            pub fn new(clock:_RobotClock, unified_logger: _Arc<_Mutex<_UnifiedLoggerWrite>>) -> _CuResult<Self> {
                let config = _read_configuration(#config_file)?;

                let copperlist_stream = _stream_write::<CuList>(
                    unified_logger.clone(),
                    _UnifiedLogType::CopperList,
                    60 * 1024, // FIXME: make this a config
                );

                Ok(#name {
                    copper_runtime: _CuRuntime::<CuTasks, CuPayload, #DEFAULT_CLNB>::new(clock, &config, tasks_instanciator, copperlist_stream)?
                })
            }

            #run_method
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

fn read_config(config_file: &String) -> CuConfig {
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
    copper_config
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

fn extract_msg_types(runtime_plan: &Vec<CuExecutionStep>) -> Vec<Type> {
    runtime_plan
        .iter()
        .filter_map(|step| {
            if step.output_msg_type.is_none() {
                None
            } else {
                Some(parse_str::<Type>(step.output_msg_type.clone().unwrap().as_str()).unwrap())
            }
        })
        .collect()
}

fn build_culist_payload(all_msgs_types_in_culist_order: &Vec<Type>) -> TypeTuple {
    if all_msgs_types_in_culist_order.is_empty() {
        parse_quote! {()}
    } else {
        parse_quote! { (#(_CuMsg<#all_msgs_types_in_culist_order>),*,)}
    }
}
