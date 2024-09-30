extern crate proc_macro;

use proc_macro::TokenStream;

use quote::quote;
use syn::meta::parser;
use syn::Fields::{Named, Unnamed};
use syn::{
    parse_macro_input, parse_quote, parse_str, Field, ItemImpl, ItemStruct, LitStr, Type, TypeTuple,
};

use cu29::config::read_configuration;
use cu29::config::CuConfig;
use cu29::curuntime::{compute_runtime_plan, CuExecutionLoop, CuExecutionUnit, CuTaskType};
use format::{highlight_rust_code, rustfmt_generated_code};

mod format;
mod utils;

// TODO: this needs to be determined when the runtime is sizing itself.
const DEFAULT_CLNB: usize = 10;

#[inline]
fn int2sliceindex(i: u32) -> syn::Index {
    syn::Index::from(i as usize)
}

/// Generates the CopperList content type from a config.
/// gen_cumsgs!("path/to/config.toml")
/// It will create a new type called CuMsgs you can pass to the log reader for decoding:
#[proc_macro]
pub fn gen_cumsgs(config_path_lit: TokenStream) -> TokenStream {
    let config = parse_macro_input!(config_path_lit as LitStr).value();
    eprintln!("[gen culist support with {:?}]", config);
    let cuconfig = read_config(&config);
    let runtime_plan: CuExecutionLoop =
        compute_runtime_plan(&cuconfig).expect("Could not compute runtime plan");
    let support = gen_culist_support(&runtime_plan);

    let with_uses = quote! {
        use bincode::Encode as _Encode;
        use bincode::enc::Encoder as _Encoder;
        use bincode::error::EncodeError as _EncodeError;
        use bincode::Decode as _Decode;
        use bincode::de::Decoder as _Decoder;
        use bincode::error::DecodeError as _DecodeError;
        use cu29::copperlist::CopperList as _CopperList;
        use cu29::cutask::CuMsgMetadata as _CuMsgMetadata;
        #support
    };
    with_uses.into()
}

/// Build the inner support of the copper list.
fn gen_culist_support(runtime_plan: &CuExecutionLoop) -> proc_macro2::TokenStream {
    eprintln!("[Extract msgs types]");
    let all_msgs_types_in_culist_order = extract_msg_types(runtime_plan);

    let culist_size = all_msgs_types_in_culist_order.len();
    let culist_indices = (0..(culist_size as u32)).map(int2sliceindex);

    eprintln!("[build the copperlist tuple]");
    let msgs_types_tuple: TypeTuple = build_culist_tuple(&all_msgs_types_in_culist_order);

    eprintln!("[build the copperlist tuple bincode support]");
    let msgs_types_tuple_encode = build_culist_tuple_encode(&all_msgs_types_in_culist_order);
    let msgs_types_tuple_decode = build_culist_tuple_decode(&all_msgs_types_in_culist_order);

    eprintln!("[build the copperlist tuple debug support]");
    let msgs_types_tuple_debug = build_culist_tuple_debug(&all_msgs_types_in_culist_order);

    let collect_metadata_function = quote! {
        pub fn collect_metadata<'a>(culist: &'a CuList) -> [&'a _CuMsgMetadata; #culist_size] {
            [#( &culist.msgs.0.#culist_indices.metadata, )*]
        }
    };

    // This generates a way to get the metadata of every single message of a culist at low cost
    quote! {
        #collect_metadata_function

        pub struct CuMsgs(#msgs_types_tuple);
        pub type CuList = _CopperList<CuMsgs>;

        // Adds the bincode support for the copper list tuple
        #msgs_types_tuple_encode
        #msgs_types_tuple_decode

        // Adds the debug support
        #msgs_types_tuple_debug
    }
}

/// Adds #[copper_runtime(config = "path")] to your application struct to generate the runtime.
/// This will add a "runtime" field to your struct and implement the "new" and "run" methods.
#[proc_macro_attribute]
pub fn copper_runtime(args: TokenStream, input: TokenStream) -> TokenStream {
    eprintln!("[entry]");
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

    eprintln!("[parse]");
    parse_macro_input!(args with attribute_config_parser);
    let config_file = config_file
        .expect("Expected config file attribute like #[CopperRuntime(config = \"path\")]")
        .value();
    let copper_config = read_config(&config_file);

    eprintln!("[runtime plan]");
    let runtime_plan: CuExecutionLoop =
        compute_runtime_plan(&copper_config).expect("Could not compute runtime plan");
    eprintln!("{:?}", runtime_plan);

    eprintln!("[extract tasks ids & types]");
    let (all_tasks_ids, all_tasks_types_names, all_tasks_types) =
        extract_tasks_types(&copper_config);
    eprintln!("tasks types: {:?}", all_tasks_types_names);

    eprintln!("[build task tuples]");
    // Build the tuple of all those types
    // note the extraneous , at the end is to make the tuple work even if this is only one element
    let task_types_tuple: TypeTuple = parse_quote! {
        (#(#all_tasks_types),*,)
    };

    eprintln!("[build monitor type]");
    let monitor_type = if let Some(monitor_config) = copper_config.get_monitor_config() {
        let monitor_type = parse_str::<Type>(monitor_config.get_type())
            .expect("Could not transform the monitor type name into a Rust type.");
        quote! { #monitor_type }
    } else {
        quote! { _NoMonitor }
    };

    eprintln!("[build runtime field]");
    // add that to a new field
    let runtime_field: Field = parse_quote! {
        copper_runtime: _CuRuntime<CuTasks, CuMsgs, #monitor_type, #DEFAULT_CLNB>
    };

    let name = &item_struct.ident;

    eprintln!("[match struct anonymity]");
    match &mut item_struct.fields {
        Named(fields_named) => {
            fields_named.named.push(runtime_field);
        }
        Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.push(runtime_field);
        }
        _ => (),
    };

    eprintln!("[gen instances]");
    // Generate the code to create instances of the nodes
    // It maps the types to their index
    let (task_instances_init_code,
        start_calls,
        stop_calls,
        preprocess_calls,
        postprocess_calls): (Vec<_>, Vec<_>, Vec<_>, Vec<_>, Vec<_>) = itertools::multiunzip(all_tasks_types
        .iter()
        .enumerate()
        .map(|(index, ty)| {
            let node_index = int2sliceindex(index as u32);
            let additional_error_info = format!(
                "Failed to get create instance for {}, instance index {}.",
                all_tasks_types_names[index], index
            );
            (
                quote! {
                    #ty::new(all_instances_configs[#index]).map_err(|e| e.add_cause(#additional_error_info))?
                },
                quote! {
                    {
                        let task = &mut self.copper_runtime.tasks.#node_index;
                        if let Err(error) = task.start(&self.copper_runtime.clock) {
                            let decision = self.copper_runtime.monitor.process_error(#index, _CuTaskState::Start, &error);
                            match decision {
                                _Decision::Abort => {
                                    debug!("Start: ABORT decision from monitoring. Task '{}' errored out \
                                    during start. Aborting all the other starts.", TASKS_IDS[#index]);
                                    return Ok(());

                                }
                                _Decision::Ignore => {
                                    debug!("Start: IGNORE decision from monitoring. Task '{}' errored out \
                                    during start. The runtime will continue.", TASKS_IDS[#index]);
                                }
                                _Decision::Shutdown => {
                                    debug!("Start: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                    during start. The runtime cannot continue.", TASKS_IDS[#index]);
                                    return Err(_CuError::new_with_cause("Task errored out during start.", error));
                                }
                            }
                        }
                    }
                },
                quote! {
                    {
                        let task = &mut self.copper_runtime.tasks.#node_index;
                        if let Err(error) = task.stop(&self.copper_runtime.clock) {
                            let decision = self.copper_runtime.monitor.process_error(#index, _CuTaskState::Stop, &error);
                            match decision {
                                _Decision::Abort => {
                                    debug!("Stop: ABORT decision from monitoring. Task '{}' errored out \
                                    during stop. Aborting all the other starts.", TASKS_IDS[#index]);
                                    return Ok(());

                                }
                                _Decision::Ignore => {
                                    debug!("Stop: IGNORE decision from monitoring. Task '{}' errored out \
                                    during stop. The runtime will continue.", TASKS_IDS[#index]);
                                }
                                _Decision::Shutdown => {
                                    debug!("Stop: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                    during stop. The runtime cannot continue.", TASKS_IDS[#index]);
                                    return Err(_CuError::new_with_cause("Task errored out during stop.", error));
                                }
                            }
                        }
                    }
                },
                quote! {
                    {
                        let task = &mut self.copper_runtime.tasks.#node_index;
                        if let Err(error) = task.preprocess(&self.copper_runtime.clock) {
                            let decision = self.copper_runtime.monitor.process_error(#index, _CuTaskState::Preprocess, &error);
                            match decision {
                                _Decision::Abort => {
                                    debug!("Preprocess: ABORT decision from monitoring. Task '{}' errored out \
                                    during preprocess. Aborting all the other starts.", TASKS_IDS[#index]);
                                    return Ok(());

                                }
                                _Decision::Ignore => {
                                    debug!("Preprocess: IGNORE decision from monitoring. Task '{}' errored out \
                                    during preprocess. The runtime will continue.", TASKS_IDS[#index]);
                                }
                                _Decision::Shutdown => {
                                    debug!("Preprocess: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                    during preprocess. The runtime cannot continue.", TASKS_IDS[#index]);
                                    return Err(_CuError::new_with_cause("Task errored out during preprocess.", error));
                                }
                            }
                        }
                    }
                },
                quote! {
                    {
                        let task = &mut self.copper_runtime.tasks.#node_index;
                        if let Err(error) = task.postprocess(&self.copper_runtime.clock) {
                            let decision = self.copper_runtime.monitor.process_error(#index, _CuTaskState::Postprocess, &error);
                            match decision {
                                _Decision::Abort => {
                                    debug!("Postprocess: ABORT decision from monitoring. Task '{}' errored out \
                                    during postprocess. Aborting all the other starts.", TASKS_IDS[#index]);
                                    return Ok(());

                                }
                                _Decision::Ignore => {
                                    debug!("Postprocess: IGNORE decision from monitoring. Task '{}' errored out \
                                    during postprocess. The runtime will continue.", TASKS_IDS[#index]);
                                }
                                _Decision::Shutdown => {
                                    debug!("Postprocess: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                    during postprocess. The runtime cannot continue.", TASKS_IDS[#index]);
                                    return Err(_CuError::new_with_cause("Task errored out during postprocess.", error));
                                }
                            }
                        }
                    }
                }
            )
        })
    );

    let runtime_plan_code: Vec<proc_macro2::TokenStream> = runtime_plan.steps
        .iter()
        .map(|unit| {
            match unit {
                CuExecutionUnit::Step(step) => {
                    eprintln!(
                        "{} -> {} as {:?}. Input={:?}, Output={:?}",
                        step.node.get_id(),
                        step.node.get_type(),
                        step.task_type,
                        step.input_msg_indices_types,
                        step.output_msg_index_type
                    );

                    let node_index = int2sliceindex(step.node_id);
                    let task_instance = quote! { self.copper_runtime.tasks.#node_index };
                    let comment_str = format!(
                        "/// {} ({:?}) I:{:?} O:{:?}",
                        step.node.get_id(),
                        step.task_type,
                        step.input_msg_indices_types,
                        step.output_msg_index_type
                    );
                    let comment_tokens: proc_macro2::TokenStream = parse_str(&comment_str).unwrap();


                    let process_call = match step.task_type {
                        CuTaskType::Source => {
                            if let Some((index, _)) = &step.output_msg_index_type {
                                let output_culist_index = int2sliceindex(*index);
                                let tid = output_culist_index.index as usize;
                                quote! {
                                    {
                                        #comment_tokens
                                        let cumsg_output = &mut msgs.#output_culist_index;
                                        cumsg_output.metadata.before_process = self.copper_runtime.clock.now().into();
                                        let maybe_error = #task_instance.process(&self.copper_runtime.clock, cumsg_output);
                                        cumsg_output.metadata.after_process = self.copper_runtime.clock.now().into();
                                        if let Err(error) = maybe_error {
                                            let decision = self.copper_runtime.monitor.process_error(#tid, _CuTaskState::Process, &error);
                                            match decision {
                                                _Decision::Abort => {
                                                    debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                                    during process. Skipping the processing of CL {}.", TASKS_IDS[#tid], id);
                                                    self.copper_runtime.monitor.process_copperlist(&collect_metadata(&culist))?;
                                                    self.copper_runtime.end_of_processing(id);
                                                    return Ok(()); // this returns early from the one iteration call.

                                                }
                                                _Decision::Ignore => {
                                                    debug!("Process: IGNORE decision from monitoring. Task '{}' errored out \
                                                    during process. The runtime will continue with a forced empty message.", TASKS_IDS[#tid]);
                                                    cumsg_output.clear_payload();
                                                }
                                                _Decision::Shutdown => {
                                                    debug!("Process: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                                    during process. The runtime cannot continue.", TASKS_IDS[#tid]);
                                                    return Err(_CuError::new_with_cause("Task errored out during process.", error));
                                                }
                                            }
                                        }
                                    }
                                }
                            } else {
                                panic!("Source task should have an output message index.");
                            }
                        }
                        CuTaskType::Sink => {
                            // collect the indices
                            let indices = step.input_msg_indices_types.iter().map(|(index, _)| int2sliceindex(*index));
                            if let Some((output_index, _)) = &step.output_msg_index_type {
                                let output_culist_index = int2sliceindex(*output_index);
                                let tid = output_culist_index.index as usize;
                                quote! {
                                    {
                                        #comment_tokens
                                        let cumsg_input = (#(&msgs.#indices),*);
                                        // This is the virtual output for the sink
                                        let cumsg_output = &mut msgs.#output_culist_index;
                                        cumsg_output.metadata.before_process = self.copper_runtime.clock.now().into();
                                        let maybe_error = #task_instance.process(&self.copper_runtime.clock, cumsg_input);
                                        cumsg_output.metadata.after_process = self.copper_runtime.clock.now().into();
                                        if let Err(error) = maybe_error {
                                            let decision = self.copper_runtime.monitor.process_error(#tid, _CuTaskState::Process, &error);
                                            match decision {
                                                _Decision::Abort => {
                                                    debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                                    during process. Skipping the processing of CL {}.", TASKS_IDS[#tid], id);
                                                    self.copper_runtime.monitor.process_copperlist(&collect_metadata(&culist))?;
                                                    self.copper_runtime.end_of_processing(id);
                                                    return Ok(()); // this returns early from the one iteration call.

                                                }
                                                _Decision::Ignore => {
                                                    debug!("Process: IGNORE decision from monitoring. Task '{}' errored out \
                                                    during process. The runtime will continue with a forced empty message.", TASKS_IDS[#tid]);
                                                    cumsg_output.clear_payload();
                                                }
                                                _Decision::Shutdown => {
                                                    debug!("Process: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                                    during process. The runtime cannot continue.", TASKS_IDS[#tid]);
                                                    return Err(_CuError::new_with_cause("Task errored out during process.", error));
                                                }
                                            }
                                        }
                                    }
                                }
                            } else {
                                panic!("Sink tasks should have a virtual output message index.");
                            }

                        }
                        CuTaskType::Regular => {
                            let indices = step.input_msg_indices_types.iter().map(|(index, _)| int2sliceindex(*index));
                            if let Some((output_index, _)) = &step.output_msg_index_type {
                                let output_culist_index = int2sliceindex(*output_index);
                                let tid = output_culist_index.index as usize;
                                quote! {
                                    {
                                        #comment_tokens
                                        let cumsg_input = (#(&msgs.#indices),*);
                                        let cumsg_output = &mut msgs.#output_culist_index;
                                        cumsg_output.metadata.before_process = self.copper_runtime.clock.now().into();
                                        let maybe_error = #task_instance.process(&self.copper_runtime.clock, cumsg_input, cumsg_output);
                                        cumsg_output.metadata.after_process = self.copper_runtime.clock.now().into();
                                        if let Err(error) = maybe_error {
                                            let decision = self.copper_runtime.monitor.process_error(#tid, _CuTaskState::Process, &error);
                                            match decision {
                                                _Decision::Abort => {
                                                    debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                                    during process. Skipping the processing of CL {}.", TASKS_IDS[#tid], id);
                                                    self.copper_runtime.monitor.process_copperlist(&collect_metadata(&culist))?;
                                                    self.copper_runtime.end_of_processing(id);
                                                    return Ok(()); // this returns early from the one iteration call.

                                                }
                                                _Decision::Ignore => {
                                                    debug!("Process: IGNORE decision from monitoring. Task '{}' errored out \
                                                    during process. The runtime will continue with a forced empty message.", TASKS_IDS[#tid]);
                                                    cumsg_output.clear_payload();
                                                }
                                                _Decision::Shutdown => {
                                                    debug!("Process: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                                    during process. The runtime cannot continue.", TASKS_IDS[#tid]);
                                                    return Err(_CuError::new_with_cause("Task errored out during process.", error));
                                                }
                                            }
                                        }
                                    }
                                }
                            } else {
                                panic!("Regular task should have an output message index.");
                            }
                        }
                    };

                    process_call
                }
                CuExecutionUnit::Loop(_) => todo!("Needs to be implemented"),
            }
        }).collect();

    eprintln!("[build the copperlist support]");
    let culist_support: proc_macro2::TokenStream = gen_culist_support(&runtime_plan);

    eprintln!("[build the run method]");
    let run_method = quote! {

        pub fn start_all_tasks(&mut self) -> _CuResult<()> {
            self.copper_runtime.monitor.start(&self.copper_runtime.clock)?;
            #(#start_calls)*
            Ok(())
        }

        #[inline]
        pub fn run_one_iteration(&mut self) -> _CuResult<()> {
            #(#preprocess_calls)*
            {
                let mut culist = &mut self.copper_runtime.copper_lists_manager.create().expect("Ran out of space for copper lists"); // FIXME: error handling.
                let id = culist.id;
                culist.change_state(cu29::copperlist::CopperListState::Processing);
                {
                    let msgs = &mut culist.msgs.0;
                    #(#runtime_plan_code)*
                } // drop(msgs);

                {
                    // End of CL monitoring
                    let md = collect_metadata(&culist);
                    let e2e = md.last().unwrap().after_process.unwrap() - md.first().unwrap().before_process.unwrap();
                    let e2en: u64 = e2e.into();
                } // drop(md);

                self.copper_runtime.monitor.process_copperlist(&collect_metadata(&culist))?;
                self.copper_runtime.end_of_processing(id);

           }// drop(culist); avoids a double mutable borrow
           #(#postprocess_calls)*
           Ok(())
        }

        pub fn stop_all_tasks(&mut self) -> _CuResult<()> {
            #(#stop_calls)*
            self.copper_runtime.monitor.stop(&self.copper_runtime.clock)?;
            Ok(())
        }

        pub fn run(&mut self) -> _CuResult<()> {
            self.start_all_tasks()?;
            let error = loop {
                let error = self.run_one_iteration();
                if error.is_err() {
                    break error;
                }
            };
            debug!("A task errored out: {}", &error);
            self.stop_all_tasks()?;
            error
        }
    };

    eprintln!("[build result]");
    // Convert the modified struct back into a TokenStream
    let result = quote! {
        // import everything with an _ to avoid clashes with the user's code
        use cu29::config::CuConfig as _CuConfig;
        use cu29::config::ComponentConfig as _ComponentConfig;
        use cu29::config::MonitorConfig as _MonitorConfig;
        use cu29::config::read_configuration as _read_configuration;
        use cu29::curuntime::CuRuntime as _CuRuntime;
        use cu29::CuResult as _CuResult;
        use cu29::CuError as _CuError;
        use cu29::cutask::CuTaskLifecycle as _CuTaskLifecycle; // Needed for the instantiation of tasks
        use cu29::cutask::CuSrcTask as _CuSrcTask;
        use cu29::cutask::CuSinkTask as _CuSinkTask;
        use cu29::cutask::CuTask as _CuTask;
        use cu29::cutask::CuMsg as _CuMsg;
        use cu29::cutask::CuMsgMetadata as _CuMsgMetadata;
        use cu29::copperlist::CopperList as _CopperList;
        use cu29::monitoring::CuMonitor as _CuMonitor; // Trait import.
        use cu29::monitoring::NoMonitor as _NoMonitor;
        use cu29::monitoring::CuTaskState as _CuTaskState;
        use cu29::monitoring::Decision as _Decision;
        use cu29::clock::RobotClock as _RobotClock;
        use cu29::clock::OptionCuTime as _OptionCuTime;
        use cu29::clock::ClockProvider as _ClockProvider;
        use std::sync::Arc as _Arc;
        use std::sync::Mutex as _Mutex;
        use bincode::Encode as _Encode;
        use bincode::enc::Encoder as _Encoder;
        use bincode::error::EncodeError as _EncodeError;
        use bincode::Decode as _Decode;
        use bincode::de::Decoder as _Decoder;
        use bincode::error::DecodeError as _DecodeError;
        use cu29_unifiedlog::stream_write as _stream_write;
        use cu29_unifiedlog::UnifiedLoggerWrite as _UnifiedLoggerWrite;
        use cu29_traits::UnifiedLogType as _UnifiedLogType;

        // This is the heart of everything.
        // CuTasks is the list of all the tasks types.
        // CuList is a CopperList with the list of all the messages types as msgs.
        pub type CuTasks = #task_types_tuple;

        const TASKS_IDS: &'static [&'static str] = &[#( #all_tasks_ids ),*];

        #culist_support


        fn tasks_instanciator(all_instances_configs: Vec<Option<&_ComponentConfig>>) -> _CuResult<CuTasks> {
            Ok(( #(#task_instances_init_code),*, ))
        }

        fn monitor_instanciator(monitor_config: Option<&_ComponentConfig>) -> #monitor_type {
            #monitor_type::new(monitor_config, TASKS_IDS).expect("Failed to create the given monitor.")
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
                    copper_runtime: _CuRuntime::<CuTasks, CuMsgs, #monitor_type, #DEFAULT_CLNB>::new(clock, &config, tasks_instanciator, monitor_instanciator, copperlist_stream)?
                })
            }

            #run_method
        }
    };
    let tokens: TokenStream = result.into();

    // Print and format the generated code using rustfmt
    // println!("Generated tokens: {}", tokens);
    let formatted_code = rustfmt_generated_code(tokens.to_string());
    eprintln!("\n     ===    Gen. Runtime ===\n");
    eprintln!("{}", highlight_rust_code(formatted_code));
    eprintln!("\n     === === === === === ===\n");

    tokens
}

fn read_config(config_file: &String) -> CuConfig {
    let mut config_full_path = utils::caller_crate_root();
    config_full_path.push(config_file);
    let filename = config_full_path
        .as_os_str()
        .to_str()
        .expect("Could not interpret the config file name");

    read_configuration(filename)
        .unwrap_or_else(|_| panic!("Failed to read configuration file: {:?}", &config_full_path))
}

/// Extract all the tasks types in their index order and their ids.
fn extract_tasks_types(copper_config: &CuConfig) -> (Vec<String>, Vec<String>, Vec<Type>) {
    let all_nodes = copper_config.get_all_nodes();

    // Get all the tasks Ids
    let all_tasks_ids: Vec<String> = all_nodes
        .iter()
        .map(|node_config| node_config.get_id().to_string())
        .collect();

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
                .unwrap_or_else(|_| panic!("Could not transform {} into a Task Rust type.", name))
        })
        .collect();
    (all_tasks_ids, all_types_names, all_types)
}

fn extract_msg_types(runtime_plan: &CuExecutionLoop) -> Vec<Type> {
    runtime_plan
        .steps
        .iter()
        .filter_map(|unit| match unit {
            CuExecutionUnit::Step(step) => {
                if let Some((_, output_msg_type)) = &step.output_msg_index_type {
                    Some(
                        parse_str::<Type>(output_msg_type.as_str()).unwrap_or_else(|_| {
                            panic!(
                                "Could not transform {} into a message Rust type.",
                                output_msg_type
                            )
                        }),
                    )
                } else {
                    None
                }
            }
            CuExecutionUnit::Loop(_) => todo!("Needs to be implemented"),
        })
        .collect()
}

/// Builds the tuple of the CuList as a tuple off all the messages types.
fn build_culist_tuple(all_msgs_types_in_culist_order: &Vec<Type>) -> TypeTuple {
    if all_msgs_types_in_culist_order.is_empty() {
        parse_quote! {()}
    } else {
        parse_quote! { (#(_CuMsg<#all_msgs_types_in_culist_order>),*,)}
    }
}

/// This is the bincode encoding part of the CuMsgs
fn build_culist_tuple_encode(all_msgs_types_in_culist_order: &Vec<Type>) -> ItemImpl {
    let indices: Vec<usize> = (0..all_msgs_types_in_culist_order.len()).collect();

    // Generate the `self.#i.encode(encoder)?` for each tuple index, including `()` types
    let encode_fields: Vec<_> = indices
        .iter()
        .map(|i| {
            let idx = syn::Index::from(*i);
            quote! { self.0.#idx.encode(encoder)?; }
        })
        .collect();

    parse_quote! {
        impl _Encode for CuMsgs {
            fn encode<E: _Encoder>(&self, encoder: &mut E) -> Result<(), _EncodeError> {
                #(#encode_fields)*
                Ok(())
            }
        }
    }
}

/// This is the bincode decoding part of the CuMsgs
fn build_culist_tuple_decode(all_msgs_types_in_culist_order: &Vec<Type>) -> ItemImpl {
    let indices: Vec<usize> = (0..all_msgs_types_in_culist_order.len()).collect();

    // Generate the `_CuMsg::<T>::decode(decoder)?` for each tuple index
    let decode_fields: Vec<_> = indices
        .iter()
        .map(|i| {
            let t = &all_msgs_types_in_culist_order[*i];
            quote! { _CuMsg::<#t>::decode(decoder)? }
        })
        .collect();

    parse_quote! {
        impl _Decode for CuMsgs {
            fn decode<D: _Decoder>(decoder: &mut D) -> Result<Self, _DecodeError> {
                Ok(CuMsgs ((
                    #(#decode_fields),*
                )))
            }
        }
    }
}

fn build_culist_tuple_debug(all_msgs_types_in_culist_order: &Vec<Type>) -> ItemImpl {
    let indices: Vec<usize> = (0..all_msgs_types_in_culist_order.len()).collect();

    let debug_fields: Vec<_> = indices
        .iter()
        .map(|i| {
            let idx = syn::Index::from(*i);
            quote! { .field(&self.0.#idx) }
        })
        .collect();

    parse_quote! {
        impl std::fmt::Debug for CuMsgs {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.debug_tuple("CuMsgs")
                    #(#debug_fields)*
                    .finish()
            }
        }
    }
}
