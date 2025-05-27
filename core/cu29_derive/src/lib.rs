extern crate proc_macro;

use proc_macro::TokenStream;
use quote::{format_ident, quote};
use std::fs::read_to_string;
use syn::meta::parser;
use syn::Fields::{Named, Unnamed};
use syn::{
    parse_macro_input, parse_quote, parse_str, Field, Fields, ItemImpl, ItemStruct, LitStr, Type,
    TypeTuple,
};

use crate::utils::config_id_to_enum;
use cu29_runtime::config::CuConfig;
use cu29_runtime::config::{read_configuration, CuGraph};
use cu29_runtime::curuntime::{
    compute_runtime_plan, find_task_type_for_id, CuExecutionLoop, CuExecutionUnit, CuTaskType,
};
use cu29_traits::CuResult;
use proc_macro2::{Ident, Span};

#[cfg(feature = "macro_debug")]
use crate::format::{highlight_rust_code, rustfmt_generated_code};

mod format;
mod utils;

// TODO: this needs to be determined when the runtime is sizing itself.
const DEFAULT_CLNB: usize = 10;

#[inline]
fn int2sliceindex(i: u32) -> syn::Index {
    syn::Index::from(i as usize)
}

#[inline(always)]
fn return_error(msg: String) -> TokenStream {
    syn::Error::new(Span::call_site(), msg)
        .to_compile_error()
        .into()
}

/// Generates the CopperList content type from a config.
/// gen_cumsgs!("path/to/config.toml")
/// It will create a new type called CuMsgs you can pass to the log reader for decoding:
#[proc_macro]
pub fn gen_cumsgs(config_path_lit: TokenStream) -> TokenStream {
    let config = parse_macro_input!(config_path_lit as LitStr).value();
    if !std::path::Path::new(&config_full_path(&config)).exists() {
        return return_error(format!(
            "The configuration file `{config}` does not exist. Please provide a valid path."
        ));
    }
    #[cfg(feature = "macro_debug")]
    eprintln!("[gen culist support with {config:?}]");
    let cuconfig = match read_config(&config) {
        Ok(cuconfig) => cuconfig,
        Err(e) => return return_error(e.to_string()),
    };
    let graph = cuconfig
        .get_graph(None) // FIXME(gbin): Multimission
        .expect("Could not find the specified mission for gen_cumsgs");
    let runtime_plan: CuExecutionLoop = match compute_runtime_plan(graph) {
        Ok(plan) => plan,
        Err(e) => return return_error(format!("Could not compute runtime plan: {e}")),
    };

    // Give a name compatible with a struct to match the task ids to their output in the CuMsgs tuple.
    let all_tasks_member_ids: Vec<String> = graph
        .get_all_nodes()
        .iter()
        .map(|(_, node)| utils::config_id_to_struct_member(node.get_id().as_str()))
        .collect();

    // All accesses are linear on the culist but the id of the tasks is random (determined by the Ron declaration order).
    // This records the task ids in call order.
    let taskid_order: Vec<usize> = runtime_plan
        .steps
        .iter()
        .filter_map(|unit| match unit {
            CuExecutionUnit::Step(step) => Some(step.node_id as usize),
            _ => None,
        })
        .collect();

    #[cfg(feature = "macro_debug")]
    eprintln!(
        "[The CuMsgs matching tasks ids are {:?}]",
        taskid_order
            .iter()
            .map(|i| all_tasks_member_ids[*i].clone())
            .collect::<Vec<_>>()
    );

    let support = gen_culist_support(&runtime_plan, &taskid_order, &all_tasks_member_ids);

    let with_uses = quote! {
        mod cumsgs {
            use cu29::bincode::Encode;
            use cu29::bincode::enc::Encoder;
            use cu29::bincode::error::EncodeError;
            use cu29::bincode::Decode;
            use cu29::bincode::de::Decoder;
            use cu29::bincode::error::DecodeError;
            use cu29::copperlist::CopperList;
            use cu29::cutask::CuMsgMetadata;
            use cu29::cutask::CuMsg;
            #support
        }
        use cumsgs::CuMsgs;
    };
    with_uses.into()
}

/// Build the inner support of the copper list.
fn gen_culist_support(
    runtime_plan: &CuExecutionLoop,
    taskid_call_order: &[usize],
    all_tasks_as_struct_member_name: &Vec<String>,
) -> proc_macro2::TokenStream {
    #[cfg(feature = "macro_debug")]
    eprintln!("[Extract msgs types]");
    let all_msgs_types_in_culist_order = extract_msg_types(runtime_plan);

    let culist_size = all_msgs_types_in_culist_order.len();
    let task_indices: Vec<_> = taskid_call_order
        .iter()
        .map(|i| syn::Index::from(*i))
        .collect();

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist struct]");
    let msgs_types_tuple: TypeTuple = build_culist_tuple(&all_msgs_types_in_culist_order);

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist tuple bincode support]");
    let msgs_types_tuple_encode = build_culist_tuple_encode(&all_msgs_types_in_culist_order);
    let msgs_types_tuple_decode = build_culist_tuple_decode(&all_msgs_types_in_culist_order);

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist tuple debug support]");
    let msgs_types_tuple_debug = build_culist_tuple_debug(&all_msgs_types_in_culist_order);

    let collect_metadata_function = quote! {
        pub fn collect_metadata<'a>(culist: &'a CuList) -> [&'a CuMsgMetadata; #culist_size] {
            [#( &culist.msgs.0.#task_indices.metadata, )*]
        }
    };

    let methods = itertools::multizip((all_tasks_as_struct_member_name, taskid_call_order)).map(
        |(name, output_position)| {
            let fn_name = format_ident!("get_{}_output", name);
            let payload_type = all_msgs_types_in_culist_order[*output_position].clone();
            let index = syn::Index::from(*output_position);
            quote! {
                pub fn #fn_name(&self) -> &CuMsg<#payload_type> {
                    &self.0.#index
                }
            }
        },
    );

    // This generates a way to get the metadata of every single message of a culist at low cost
    quote! {
        #collect_metadata_function

        pub struct CuMsgs(pub #msgs_types_tuple);

        pub type CuList = CopperList<CuMsgs>;

        impl CuMsgs {
            #(#methods)*

            fn get_tuple(&self) -> &#msgs_types_tuple {
                &self.0
            }

            fn get_tuple_mut(&mut self) -> &mut #msgs_types_tuple {
                &mut self.0
            }
        }

        // Adds the bincode support for the copper list tuple
        #msgs_types_tuple_encode
        #msgs_types_tuple_decode

        // Adds the debug support
        #msgs_types_tuple_debug
    }
}

fn gen_sim_support(runtime_plan: &CuExecutionLoop) -> proc_macro2::TokenStream {
    #[cfg(feature = "macro_debug")]
    eprintln!("[Sim: Build SimEnum]");
    let plan_enum: Vec<proc_macro2::TokenStream> = runtime_plan
        .steps
        .iter()
        .map(|unit| match unit {
            CuExecutionUnit::Step(step) => {
                let enum_entry_name = config_id_to_enum(step.node.get_id().as_str());
                let enum_ident = Ident::new(&enum_entry_name, proc_macro2::Span::call_site());
                let inputs: Vec<Type> = step
                    .input_msg_indices_types
                    .iter()
                    .map(|(_, t)| parse_str::<Type>(format!("CuMsg<{t}>").as_str()).unwrap())
                    .collect();
                let output: Option<Type> = step
                    .output_msg_index_type
                    .as_ref()
                    .map(|(_, t)| parse_str::<Type>(format!("CuMsg<{t}>").as_str()).unwrap());
                let no_output = parse_str::<Type>("CuMsg<()>").unwrap();
                let output = output.as_ref().unwrap_or(&no_output);
                quote! {
                    #enum_ident(cu29::simulation::CuTaskCallbackState<(#(&'a #inputs),*), &'a mut #output>)
                }
            }
            CuExecutionUnit::Loop(_) => {
                todo!("Needs to be implemented")
            }
        })
        .collect();
    quote! {
        pub enum SimStep<'a> {
            #(#plan_enum),*
        }
    }
}

/// Adds #[copper_runtime(config = "path", sim_mode = false/true)] to your application struct to generate the runtime.
/// if sim_mode is omitted, it is set to false.
/// This will add a "runtime" field to your struct and implement the "new" and "run" methods.
#[proc_macro_attribute]
pub fn copper_runtime(args: TokenStream, input: TokenStream) -> TokenStream {
    #[cfg(feature = "macro_debug")]
    eprintln!("[entry]");
    let mut application_struct = parse_macro_input!(input as ItemStruct);
    let application_name = &application_struct.ident;
    let builder_name = format_ident!("{}Builder", application_name);

    let mut config_file: Option<LitStr> = None;
    let mut sim_mode = false;

    // Custom parser for the attribute arguments
    let attribute_config_parser = parser(|meta| {
        if meta.path.is_ident("config") {
            config_file = Some(meta.value()?.parse()?);
            Ok(())
        } else if meta.path.is_ident("sim_mode") {
            // Check if `sim_mode` has an explicit value (true/false)
            if meta.input.peek(syn::Token![=]) {
                meta.input.parse::<syn::Token![=]>()?;
                let value: syn::LitBool = meta.input.parse()?;
                sim_mode = value.value();
                Ok(())
            } else {
                // If no value is provided, default to true
                sim_mode = true;
                Ok(())
            }
        } else {
            Err(meta.error("unsupported property"))
        }
    });

    #[cfg(feature = "macro_debug")]
    eprintln!("[parse]");
    // Parse the provided args with the custom parser
    parse_macro_input!(args with attribute_config_parser);

    // Check if the config file was provided
    let config_file = match config_file {
        Some(file) => file.value(),
        None => {
            return return_error(
                "Expected config file attribute like #[CopperRuntime(config = \"path\")]"
                    .to_string(),
            )
        }
    };

    if !std::path::Path::new(&config_full_path(&config_file)).exists() {
        return return_error(format!(
            "The configuration file `{config_file}` does not exist. Please provide a valid path."
        ));
    }

    let copper_config = match read_config(&config_file) {
        Ok(cuconfig) => cuconfig,
        Err(e) => return return_error(e.to_string()),
    };
    let copper_config_content = match read_to_string(config_full_path(config_file.as_str())) {
        Ok(ok) => ok,
        Err(e) => return return_error(format!("Could not read the config file (should not happen because we just succeeded just before). {e}"))
    };

    #[cfg(feature = "macro_debug")]
    eprintln!("[build monitor type]");
    let monitor_type = if let Some(monitor_config) = copper_config.get_monitor_config() {
        let monitor_type = parse_str::<Type>(monitor_config.get_type())
            .expect("Could not transform the monitor type name into a Rust type.");
        quote! { #monitor_type }
    } else {
        quote! { NoMonitor }
    };

    // This is common for all the mission as it will be inserted in the respective modules with their local CuTasks, CuMsgs etc...
    #[cfg(feature = "macro_debug")]
    eprintln!("[build runtime field]");
    // add that to a new field
    let runtime_field: Field = if sim_mode {
        parse_quote! {
            copper_runtime: cu29::curuntime::CuRuntime<CuSimTasks, CuMsgs, #monitor_type, #DEFAULT_CLNB>
        }
    } else {
        parse_quote! {
            copper_runtime: cu29::curuntime::CuRuntime<CuTasks, CuMsgs, #monitor_type, #DEFAULT_CLNB>
        }
    };

    #[cfg(feature = "macro_debug")]
    eprintln!("[match struct anonymity]");
    match &mut application_struct.fields {
        Named(fields_named) => {
            fields_named.named.push(runtime_field);
        }
        Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.push(runtime_field);
        }
        Fields::Unit => {
            panic!("This struct is a unit struct, it should have named or unnamed fields. use struct Something {{}} and not struct Something;")
        }
    };

    let all_missions = copper_config.graphs.get_all_missions_graphs();
    let mut all_missions_tokens = Vec::<proc_macro2::TokenStream>::new();
    for (mission, graph) in &all_missions {
        let mission_mod = parse_str::<Ident>(mission.as_str())
            .expect("Could not make an identifier of the mission name");

        #[cfg(feature = "macro_debug")]
        eprintln!("[runtime plan for mission {mission}]");
        let runtime_plan: CuExecutionLoop = match compute_runtime_plan(graph) {
            Ok(plan) => plan,
            Err(e) => return return_error(format!("Could not compute runtime plan: {e}")),
        };
        #[cfg(feature = "macro_debug")]
        eprintln!("{runtime_plan:?}");

        #[cfg(feature = "macro_debug")]
        eprintln!("[extract tasks ids & types]");
        let (all_tasks_ids, all_tasks_cutype, all_tasks_types_names, all_tasks_types) =
            extract_tasks_types(graph);

        let all_sim_tasks_types: Vec<Type> = all_tasks_ids
            .iter()
            .zip(&all_tasks_cutype)
            .zip(&all_tasks_types)
            .map(|((task_id, cutype), stype)| match cutype {
                CuTaskType::Source => {
                    let msg_type = graph
                        .get_node_output_msg_type(task_id.as_str())
                        .unwrap_or_else(|| panic!("CuSrcTask {task_id} should have an outgoing connection with a valid output msg type"));
                    let sim_task_name = format!("cu29::simulation::CuSimSrcTask<{msg_type}>");
                    parse_str(sim_task_name.as_str()).unwrap_or_else(|_| panic!("Could not build the placeholder for simulation: {sim_task_name}"))
                }
                CuTaskType::Regular => stype.clone(),
                CuTaskType::Sink => {
                    let msg_type = graph
                        .get_node_input_msg_type(task_id.as_str())
                        .unwrap_or_else(|| panic!("CuSinkTask {task_id} should have an incoming connection with a valid input msg type"));
                    let sim_task_name = format!("cu29::simulation::CuSimSinkTask<{msg_type}>");
                    parse_str(sim_task_name.as_str()).unwrap_or_else(|_| panic!("Could not build the placeholder for simulation: {sim_task_name}"))
                }
            })
            .collect();

        #[cfg(feature = "macro_debug")]
        eprintln!("[build task tuples]");
        // Build the tuple of all those types
        // note the extraneous, at the end is to make the tuple work even if this is only one element
        let task_types_tuple: TypeTuple = parse_quote! {
            (#(#all_tasks_types),*,)
        };

        let task_types_tuple_sim: TypeTuple = parse_quote! {
            (#(#all_sim_tasks_types),*,)
        };

        #[cfg(feature = "macro_debug")]
        eprintln!("[gen instances]");

        let task_sim_instances_init_code = all_sim_tasks_types.iter().enumerate().map(|(index, ty)| {
            let additional_error_info = format!(
                "Failed to get create instance for {}, instance index {}.",
                all_tasks_types_names[index], index
            );

            quote! {
            <#ty>::new(all_instances_configs[#index]).map_err(|e| e.add_cause(#additional_error_info))?
            }
        }).collect::<Vec<_>>();

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
                let task_index = int2sliceindex(index as u32);
                let task_enum_name = config_id_to_enum(&all_tasks_ids[index]);
                let enum_name = Ident::new(&task_enum_name, proc_macro2::Span::call_site());
                let additional_error_info = format!(
                    "Failed to get create instance for {}, instance index {}.",
                    all_tasks_types_names[index], index
                );
                (
                    quote! {
                        #ty::new(all_instances_configs[#index]).map_err(|e| e.add_cause(#additional_error_info))?
                    },
                    {
                        let monitoring_action = quote! {
                            let decision = self.copper_runtime.monitor.process_error(#index, CuTaskState::Start, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Start: ABORT decision from monitoring. Task '{}' errored out \
                                during start. Aborting all the other starts.", #mission_mod::TASKS_IDS[#index]);
                                    return Ok(());

                                }
                                Decision::Ignore => {
                                    debug!("Start: IGNORE decision from monitoring. Task '{}' errored out \
                                during start. The runtime will continue.", #mission_mod::TASKS_IDS[#index]);
                                }
                                Decision::Shutdown => {
                                    debug!("Start: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                during start. The runtime cannot continue.", #mission_mod::TASKS_IDS[#index]);
                                    return Err(CuError::new_with_cause("Task errored out during start.", error));
                                }
                            }
                        };

                        let call_sim_callback = if sim_mode {
                            quote! {
                                // Ask the sim if this task should be executed or overridden by the sim.
                                let ovr = sim_callback(SimStep::#enum_name(cu29::simulation::CuTaskCallbackState::Start));

                                let doit = if let cu29::simulation::SimOverride::Errored(reason) = ovr  {
                                    let error: CuError = reason.into();
                                    #monitoring_action
                                    false
                               }
                               else {
                                    ovr == cu29::simulation::SimOverride::ExecuteByRuntime
                               };
                            }
                        } else {
                            quote! {
                                let doit = true;  // in normal mode always execute the steps in the runtime.
                            }
                        };


                        quote! {
                            #call_sim_callback
                            if doit {
                                let task = &mut self.copper_runtime.tasks.#task_index;
                                if let Err(error) = task.start(&self.copper_runtime.clock) {
                                    #monitoring_action
                                }
                            }
                        }
                    },
                    {
                        let monitoring_action = quote! {
                                    let decision = self.copper_runtime.monitor.process_error(#index, CuTaskState::Stop, &error);
                                    match decision {
                                        Decision::Abort => {
                                            debug!("Stop: ABORT decision from monitoring. Task '{}' errored out \
                                    during stop. Aborting all the other starts.", #mission_mod::TASKS_IDS[#index]);
                                            return Ok(());

                                        }
                                        Decision::Ignore => {
                                            debug!("Stop: IGNORE decision from monitoring. Task '{}' errored out \
                                    during stop. The runtime will continue.", #mission_mod::TASKS_IDS[#index]);
                                        }
                                        Decision::Shutdown => {
                                            debug!("Stop: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                    during stop. The runtime cannot continue.", #mission_mod::TASKS_IDS[#index]);
                                            return Err(CuError::new_with_cause("Task errored out during stop.", error));
                                        }
                                    }
                            };
                        let call_sim_callback = if sim_mode {
                            quote! {
                                // Ask the sim if this task should be executed or overridden by the sim.
                                let ovr = sim_callback(SimStep::#enum_name(cu29::simulation::CuTaskCallbackState::Stop));

                                let doit = if let cu29::simulation::SimOverride::Errored(reason) = ovr  {
                                    let error: CuError = reason.into();
                                    #monitoring_action
                                    false
                               }
                               else {
                                    ovr == cu29::simulation::SimOverride::ExecuteByRuntime
                               };
                            }
                        } else {
                            quote! {
                                let doit = true;  // in normal mode always execute the steps in the runtime.
                            }
                        };
                        quote! {
                            #call_sim_callback
                            if doit {
                                let task = &mut self.copper_runtime.tasks.#task_index;
                                if let Err(error) = task.stop(&self.copper_runtime.clock) {
                                    #monitoring_action
                                }
                            }
                        }
                    },
                    {
                        let monitoring_action = quote! {
                            let decision = self.copper_runtime.monitor.process_error(#index, CuTaskState::Preprocess, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Preprocess: ABORT decision from monitoring. Task '{}' errored out \
                                during preprocess. Aborting all the other starts.", #mission_mod::TASKS_IDS[#index]);
                                    return Ok(());

                                }
                                Decision::Ignore => {
                                    debug!("Preprocess: IGNORE decision from monitoring. Task '{}' errored out \
                                during preprocess. The runtime will continue.", #mission_mod::TASKS_IDS[#index]);
                                }
                                Decision::Shutdown => {
                                    debug!("Preprocess: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                during preprocess. The runtime cannot continue.", #mission_mod::TASKS_IDS[#index]);
                                    return Err(CuError::new_with_cause("Task errored out during preprocess.", error));
                                }
                            }
                        };
                        let call_sim_callback = if sim_mode {
                            quote! {
                                // Ask the sim if this task should be executed or overridden by the sim.
                                let ovr = sim_callback(SimStep::#enum_name(cu29::simulation::CuTaskCallbackState::Preprocess));

                                let doit = if let cu29::simulation::SimOverride::Errored(reason) = ovr  {
                                    let error: CuError = reason.into();
                                    #monitoring_action
                                    false
                                } else {
                                    ovr == cu29::simulation::SimOverride::ExecuteByRuntime
                                };
                            }
                        } else {
                            quote! {
                                let doit = true;  // in normal mode always execute the steps in the runtime.
                            }
                        };
                        quote! {
                            #call_sim_callback
                            if doit {
                                let task = &mut self.copper_runtime.tasks.#task_index;
                                if let Err(error) = task.preprocess(&self.copper_runtime.clock) {
                                    #monitoring_action
                                }
                            }
                        }
                    },
                    {
                        let monitoring_action = quote! {
                            let decision = self.copper_runtime.monitor.process_error(#index, CuTaskState::Postprocess, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Postprocess: ABORT decision from monitoring. Task '{}' errored out \
                                during postprocess. Aborting all the other starts.", #mission_mod::TASKS_IDS[#index]);
                                    return Ok(());

                                }
                                Decision::Ignore => {
                                    debug!("Postprocess: IGNORE decision from monitoring. Task '{}' errored out \
                                during postprocess. The runtime will continue.", #mission_mod::TASKS_IDS[#index]);
                                }
                                Decision::Shutdown => {
                                    debug!("Postprocess: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                during postprocess. The runtime cannot continue.", #mission_mod::TASKS_IDS[#index]);
                                    return Err(CuError::new_with_cause("Task errored out during postprocess.", error));
                                }
                            }
                        };
                        let call_sim_callback = if sim_mode {
                            quote! {
                                // Ask the sim if this task should be executed or overridden by the sim.
                                let ovr = sim_callback(SimStep::#enum_name(cu29::simulation::CuTaskCallbackState::Postprocess));

                                let doit = if let cu29::simulation::SimOverride::Errored(reason) = ovr  {
                                    let error: CuError = reason.into();
                                    #monitoring_action
                                    false
                                } else {
                                    ovr == cu29::simulation::SimOverride::ExecuteByRuntime
                                };
                            }
                        } else {
                            quote! {
                                let doit = true;  // in normal mode always execute the steps in the runtime.
                            }
                        };
                        quote! {
                            #call_sim_callback
                            if doit {
                                let task = &mut self.copper_runtime.tasks.#task_index;
                                if let Err(error) = task.postprocess(&self.copper_runtime.clock) {
                                    #monitoring_action
                                }
                            }
                        }
                    }
                )
            })
        );

        // All accesses are linear on the culist but the id of the tasks is random (determined by the Ron declaration order).
        // This records the task ids in call order.
        let mut taskid_call_order: Vec<usize> = Vec::new();

        let runtime_plan_code: Vec<proc_macro2::TokenStream> = runtime_plan.steps
            .iter()
            .map(|unit| {
                match unit {
                    CuExecutionUnit::Step(step) => {
                        #[cfg(feature = "macro_debug")]
                        eprintln!(
                            "{} -> {} as {:?}. task_id: {} Input={:?}, Output={:?}",
                            step.node.get_id(),
                            step.node.get_type(),
                            step.task_type,
                            step.node_id,
                            step.input_msg_indices_types,
                            step.output_msg_index_type
                        );

                        let node_index = int2sliceindex(step.node_id);
                        let task_instance = quote! { self.copper_runtime.tasks.#node_index };
                        let comment_str = format!(
                            "/// {} ({:?}) Id:{} I:{:?} O:{:?}",
                            step.node.get_id(),
                            step.task_type,
                            step.node_id,
                            step.input_msg_indices_types,
                            step.output_msg_index_type
                        );
                        let comment_tokens: proc_macro2::TokenStream = parse_str(&comment_str).unwrap();
                        let tid = step.node_id as usize;
                        taskid_call_order.push(tid);

                        let task_enum_name = config_id_to_enum(&all_tasks_ids[tid]);
                        let enum_name = Ident::new(&task_enum_name, proc_macro2::Span::call_site());

                        let process_call = match step.task_type {
                            CuTaskType::Source => {
                                if let Some((index, _)) = &step.output_msg_index_type {
                                    let output_culist_index = int2sliceindex(*index);

                                    let monitoring_action = quote! {
                                        debug!("Task {}: Error during process: {}", #mission_mod::TASKS_IDS[#tid], &error);
                                        let decision = self.copper_runtime.monitor.process_error(#tid, CuTaskState::Process, &error);
                                        match decision {
                                            Decision::Abort => {
                                                debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                            during process. Skipping the processing of CL {}.", #mission_mod::TASKS_IDS[#tid], id);
                                                self.copper_runtime.monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                                                self.copper_runtime.end_of_processing(id);
                                                return Ok(()); // this returns early from the one iteration call.

                                            }
                                            Decision::Ignore => {
                                                debug!("Process: IGNORE decision from monitoring. Task '{}' errored out \
                                            during process. The runtime will continue with a forced empty message.", #mission_mod::TASKS_IDS[#tid]);
                                                let cumsg_output = &mut msgs.#output_culist_index;
                                                cumsg_output.clear_payload();
                                            }
                                            Decision::Shutdown => {
                                                debug!("Process: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                            during process. The runtime cannot continue.", #mission_mod::TASKS_IDS[#tid]);
                                                return Err(CuError::new_with_cause("Task errored out during process.", error));
                                            }
                                        }
                                    };
                                    let call_sim_callback = if sim_mode {
                                        quote! {
                                            let doit = {
                                                let cumsg_output = &mut msgs.#output_culist_index;
                                                let state = cu29::simulation::CuTaskCallbackState::Process((), cumsg_output);
                                                let ovr = sim_callback(SimStep::#enum_name(state));
                                                if let cu29::simulation::SimOverride::Errored(reason) = ovr  {
                                                    let error: CuError = reason.into();
                                                    #monitoring_action
                                                    false
                                                } else {
                                                    ovr == cu29::simulation::SimOverride::ExecuteByRuntime
                                                }
                                            };
                                         }
                                    } else {
                                        quote! {
                                            let  doit = true;  // in normal mode always execute the steps in the runtime.
                                       }
                                    };

                                    quote! {
                                        {
                                            #comment_tokens
                                            {
                                                #call_sim_callback
                                                let cumsg_output = &mut msgs.#output_culist_index;
                                                cumsg_output.metadata.process_time.start = self.copper_runtime.clock.now().into();
                                                let maybe_error = if doit {
                                                    #task_instance.process(&self.copper_runtime.clock, cumsg_output)
                                                } else {
                                                    Ok(())
                                                };
                                                cumsg_output.metadata.process_time.end = self.copper_runtime.clock.now().into();
                                                if let Err(error) = maybe_error {
                                                    #monitoring_action
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

                                    let monitoring_action = quote! {
                                        debug!("Task {}: Error during process: {}", #mission_mod::TASKS_IDS[#tid], &error);
                                        let decision = self.copper_runtime.monitor.process_error(#tid, CuTaskState::Process, &error);
                                        match decision {
                                            Decision::Abort => {
                                                debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                            during process. Skipping the processing of CL {}.", #mission_mod::TASKS_IDS[#tid], id);
                                                self.copper_runtime.monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                                                self.copper_runtime.end_of_processing(id);
                                                return Ok(()); // this returns early from the one iteration call.

                                            }
                                            Decision::Ignore => {
                                                debug!("Process: IGNORE decision from monitoring. Task '{}' errored out \
                                            during process. The runtime will continue with a forced empty message.", #mission_mod::TASKS_IDS[#tid]);
                                                let cumsg_output = &mut msgs.#output_culist_index;
                                                cumsg_output.clear_payload();
                                            }
                                            Decision::Shutdown => {
                                                debug!("Process: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                            during process. The runtime cannot continue.", #mission_mod::TASKS_IDS[#tid]);
                                                return Err(CuError::new_with_cause("Task errored out during process.", error));
                                            }
                                        }
                                    };

                                    let call_sim_callback = if sim_mode {
                                        quote! {
                                            let doit = {
                                                let cumsg_input = (#(&msgs.#indices),*);
                                                // This is the virtual output for the sink
                                                let cumsg_output = &mut msgs.#output_culist_index;
                                                let state = cu29::simulation::CuTaskCallbackState::Process(cumsg_input, cumsg_output);
                                                let ovr = sim_callback(SimStep::#enum_name(state));

                                                if let cu29::simulation::SimOverride::Errored(reason) = ovr  {
                                                    let error: CuError = reason.into();
                                                    #monitoring_action
                                                    false
                                                } else {
                                                    ovr == cu29::simulation::SimOverride::ExecuteByRuntime
                                                }
                                            };
                                         }
                                    } else {
                                        quote! {
                                            let doit = true;  // in normal mode always execute the steps in the runtime.
                                       }
                                    };

                                    let indices = step.input_msg_indices_types.iter().map(|(index, _)| int2sliceindex(*index));
                                    quote! {
                                        {
                                            #comment_tokens
                                            #call_sim_callback
                                            let cumsg_input = (#(&msgs.#indices),*);
                                            // This is the virtual output for the sink
                                            let cumsg_output = &mut msgs.#output_culist_index;
                                            cumsg_output.metadata.process_time.start = self.copper_runtime.clock.now().into();
                                            let maybe_error = if doit {#task_instance.process(&self.copper_runtime.clock, cumsg_input)} else {Ok(())};
                                            cumsg_output.metadata.process_time.end = self.copper_runtime.clock.now().into();
                                            if let Err(error) = maybe_error {
                                                #monitoring_action
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

                                    let monitoring_action = quote! {
                                        debug!("Task {}: Error during process: {}", #mission_mod::TASKS_IDS[#tid], &error);
                                        let decision = self.copper_runtime.monitor.process_error(#tid, CuTaskState::Process, &error);
                                        match decision {
                                            Decision::Abort => {
                                                debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                            during process. Skipping the processing of CL {}.", #mission_mod::TASKS_IDS[#tid], id);
                                                self.copper_runtime.monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                                                self.copper_runtime.end_of_processing(id);
                                                return Ok(()); // this returns early from the one iteration call.

                                            }
                                            Decision::Ignore => {
                                                debug!("Process: IGNORE decision from monitoring. Task '{}' errored out \
                                            during process. The runtime will continue with a forced empty message.", #mission_mod::TASKS_IDS[#tid]);
                                                let cumsg_output = &mut msgs.#output_culist_index;
                                                cumsg_output.clear_payload();
                                            }
                                            Decision::Shutdown => {
                                                debug!("Process: SHUTDOWN decision from monitoring. Task '{}' errored out \
                                            during process. The runtime cannot continue.", #mission_mod::TASKS_IDS[#tid]);
                                                return Err(CuError::new_with_cause("Task errored out during process.", error));
                                            }
                                        }
                                    };

                                    let call_sim_callback = if sim_mode {
                                        quote! {
                                            let doit = {
                                                let cumsg_input = (#(&msgs.#indices),*);
                                                let cumsg_output = &mut msgs.#output_culist_index;
                                                let state = cu29::simulation::CuTaskCallbackState::Process(cumsg_input, cumsg_output);
                                                let ovr = sim_callback(SimStep::#enum_name(state));

                                                if let cu29::simulation::SimOverride::Errored(reason) = ovr  {
                                                    let error: CuError = reason.into();
                                                    #monitoring_action
                                                    false
                                                }
                                                else {
                                                    ovr == cu29::simulation::SimOverride::ExecuteByRuntime
                                                }
                                            };
                                         }
                                    } else {
                                        quote! {
                                            let doit = true;  // in normal mode always execute the steps in the runtime.
                                       }
                                    };

                                    let indices = step.input_msg_indices_types.iter().map(|(index, _)| int2sliceindex(*index));
                                    quote! {
                                        {
                                            #comment_tokens
                                            #call_sim_callback
                                            let cumsg_input = (#(&msgs.#indices),*);
                                            let cumsg_output = &mut msgs.#output_culist_index;
                                            cumsg_output.metadata.process_time.start = self.copper_runtime.clock.now().into();
                                            let maybe_error = if doit {#task_instance.process(&self.copper_runtime.clock, cumsg_input, cumsg_output)} else {Ok(())};
                                            cumsg_output.metadata.process_time.end = self.copper_runtime.clock.now().into();
                                            if let Err(error) = maybe_error {
                                                #monitoring_action
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
        #[cfg(feature = "macro_debug")]
        eprintln!("[Culist access order:  {taskid_call_order:?}]");

        // Give a name compatible with a struct to match the task ids to their output in the CuMsgs tuple.
        let all_tasks_member_ids: Vec<String> = all_tasks_ids
            .iter()
            .map(|name| utils::config_id_to_struct_member(name.as_str()))
            .collect();

        #[cfg(feature = "macro_debug")]
        eprintln!("[build the copperlist support]");
        let culist_support: proc_macro2::TokenStream =
            gen_culist_support(&runtime_plan, &taskid_call_order, &all_tasks_member_ids);

        #[cfg(feature = "macro_debug")]
        eprintln!("[build the sim support]");
        let sim_support: proc_macro2::TokenStream = gen_sim_support(&runtime_plan);

        let (new, run_one_iteration, start_all_tasks, stop_all_tasks, run) = if sim_mode {
            (
                quote! {
                    fn new(clock:RobotClock, unified_logger: Arc<Mutex<UnifiedLoggerWrite>>, config_override: Option<CuConfig>, sim_callback: &mut impl FnMut(SimStep) -> cu29::simulation::SimOverride) -> CuResult<Self>
                },
                quote! {
                    fn run_one_iteration(&mut self, sim_callback: &mut impl FnMut(SimStep) -> cu29::simulation::SimOverride) -> CuResult<()>
                },
                quote! {
                    fn start_all_tasks(&mut self, sim_callback: &mut impl FnMut(SimStep) -> cu29::simulation::SimOverride) -> CuResult<()>
                },
                quote! {
                    fn stop_all_tasks(&mut self, sim_callback: &mut impl FnMut(SimStep) -> cu29::simulation::SimOverride) -> CuResult<()>
                },
                quote! {
                    fn run(&mut self, sim_callback: &mut impl FnMut(SimStep) -> cu29::simulation::SimOverride) -> CuResult<()>
                },
            )
        } else {
            (
                quote! {
                    fn new(clock:RobotClock, unified_logger: Arc<Mutex<UnifiedLoggerWrite>>, config_override: Option<CuConfig>) -> CuResult<Self>
                },
                quote! {
                    fn run_one_iteration(&mut self) -> CuResult<()>
                },
                quote! {
                    fn start_all_tasks(&mut self) -> CuResult<()>
                },
                quote! {
                    fn stop_all_tasks(&mut self) -> CuResult<()>
                },
                quote! {
                    fn run(&mut self) -> CuResult<()>
                },
            )
        };

        let sim_callback_arg = if sim_mode {
            Some(quote!(sim_callback))
        } else {
            None
        };

        let sim_callback_on_new_calls = all_tasks_ids.iter().enumerate().map(|(i, id)| {
            let enum_name = config_id_to_enum(id);
            let enum_ident = Ident::new(&enum_name, Span::call_site());
            quote! {
                // the answer is ignored, we have to instantiate the tasks anyway.
                sim_callback(SimStep::#enum_ident(cu29::simulation::CuTaskCallbackState::New(all_instances_configs[#i].cloned())));
            }
        });

        let sim_callback_on_new = if sim_mode {
            Some(quote! {
                let graph = config.get_graph(Some(#mission)).expect("Could not find the mission #mission");
                let all_instances_configs: Vec<Option<&ComponentConfig>> = graph
                    .get_all_nodes()
                    .iter()
                    .map(|(_, node)| node.get_instance_config())
                    .collect();
                #(#sim_callback_on_new_calls)*
            })
        } else {
            None
        };

        #[cfg(feature = "macro_debug")]
        eprintln!("[build the run methods]");
        let run_methods = quote! {

            #run_one_iteration {
                #(#preprocess_calls)*
                {
                    let mut culist: &mut _ = &mut self.copper_runtime.copper_lists_manager.create().expect("Ran out of space for copper lists"); // FIXME: error handling.
                    let id = culist.id;
                    culist.change_state(cu29::copperlist::CopperListState::Processing);
                    {
                        let msgs = &mut culist.msgs.0;
                        #(#runtime_plan_code)*
                    } // drop(msgs);

                    {
                        // End of CL monitoring
                        let md = #mission_mod::collect_metadata(&culist);
                        let e2e = md.last().unwrap().process_time.end.unwrap() - md.first().unwrap().process_time.start.unwrap();
                        let e2en: u64 = e2e.into();
                    } // drop(md);

                    self.copper_runtime.monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                    self.copper_runtime.end_of_processing(id);

               }// drop(culist); avoids a double mutable borrow
               #(#postprocess_calls)*
               Ok(())
            }

            #start_all_tasks {
                #(#start_calls)*
                self.copper_runtime.monitor.start(&self.copper_runtime.clock)?;
                Ok(())
            }

            #stop_all_tasks {
                #(#stop_calls)*
                self.copper_runtime.monitor.stop(&self.copper_runtime.clock)?;
                Ok(())
            }

            #run {
                self.start_all_tasks(#sim_callback_arg)?;
                let error = loop {
                    let error = self.run_one_iteration(#sim_callback_arg);
                    if error.is_err() {
                        break error;
                    }
                };
                debug!("A task errored out: {}", &error);
                self.stop_all_tasks(#sim_callback_arg)?;
                error
            }
        };

        let tasks_type = if sim_mode {
            quote!(CuSimTasks)
        } else {
            quote!(CuTasks)
        };

        let tasks_instanciator = if sim_mode {
            quote!(tasks_instanciator_sim)
        } else {
            quote!(tasks_instanciator)
        };

        let app_impl_decl = if sim_mode {
            quote!(impl CuSimApplication for #application_name)
        } else {
            quote!(impl CuApplication for #application_name)
        };
        let simstep_type_decl = if sim_mode {
            quote!(
                type Step<'z> = SimStep<'z>;
            )
        } else {
            quote!()
        };

        #[cfg(feature = "macro_debug")]
        eprintln!("[build result]");

        let application_impl = quote! {
            #app_impl_decl {
                #simstep_type_decl

                #new {
                    let config_filename = #config_file;
                    let config = if config_override.is_some() {
                        let overridden_config = config_override.unwrap();
                        debug!("CuConfig: Overridden programmatically: {}", &overridden_config.serialize_ron());
                        overridden_config
                    } else if std::path::Path::new(config_filename).exists() {
                        debug!("CuConfig: Reading configuration from file: {}", config_filename);
                        cu29::config::read_configuration(config_filename)?
                    } else {
                        let original_config = Self::get_original_config();
                        debug!("CuConfig: Using the original configuration the project was compiled with: {}", &original_config);
                        cu29::config::read_configuration_str(original_config, None)?
                    };

                    // For simple cases we can say the section is just a bunch of Copper Lists.
                    // But we can now have allocations outside of it so we can override it from the config.
                    let mut default_section_size = std::mem::size_of::<super::#mission_mod::CuList>() * 64;
                    // Check if there is a logging configuration with section_size_mib
                    if let Some(section_size_mib) = config.logging.as_ref().and_then(|l| l.section_size_mib) {
                        // Convert MiB to bytes
                        default_section_size = section_size_mib as usize * 1024usize * 1024usize;
                    }
                    let copperlist_stream = stream_write::<#mission_mod::CuList>(
                        unified_logger.clone(),
                        UnifiedLogType::CopperList,
                        default_section_size,
                        // the 2 sizes are not directly related as we encode the CuList but we can
                        // assume the encoded size is close or lower than the non encoded one
                        // This is to be sure we have the size of at least a Culist and some.
                    );

                    let application = Ok(#application_name {
                        copper_runtime: CuRuntime::<#mission_mod::#tasks_type, #mission_mod::CuMsgs, #monitor_type, #DEFAULT_CLNB>::new(
                            clock,
                            &config,
                            Some(#mission),
                            #mission_mod::#tasks_instanciator,
                            #mission_mod::monitor_instanciator,
                            copperlist_stream)?,
                    });

                    #sim_callback_on_new

                    application
                }

                fn get_original_config() -> String {
                    #copper_config_content.to_string()
                }

                #run_methods
            }
        };

        let (
            builder_struct,
            builder_new,
            builder_impl,
            builder_sim_callback_method,
            builder_build_sim_callback_arg,
        ) = if sim_mode {
            (
                quote! {
                    pub struct #builder_name <'a, F> {
                        clock: Option<RobotClock>,
                        unified_logger: Option<Arc<Mutex<UnifiedLoggerWrite>>>,
                        config_override: Option<CuConfig>,
                        sim_callback: Option<&'a mut F>
                    }
                },
                quote! {
                    pub fn new() -> Self {
                        Self {
                            clock: None,
                            unified_logger: None,
                            config_override: None,
                            sim_callback: None,
                        }
                    }
                },
                quote! {
                    impl<'a, F> #builder_name <'a, F>
                    where
                        F: FnMut(SimStep) -> cu29::simulation::SimOverride,
                },
                Some(quote! {
                    pub fn with_sim_callback(mut self, sim_callback: &'a mut F) -> Self
                    {
                        self.sim_callback = Some(sim_callback);
                        self
                    }
                }),
                Some(quote! {
                    self.sim_callback
                        .ok_or(CuError::from("Sim callback missing from builder"))?,
                }),
            )
        } else {
            (
                quote! {
                    pub struct #builder_name {
                        clock: Option<RobotClock>,
                        unified_logger: Option<Arc<Mutex<UnifiedLoggerWrite>>>,
                        config_override: Option<CuConfig>,
                    }
                },
                quote! {
                    pub fn new() -> Self {
                        Self {
                            clock: None,
                            unified_logger: None,
                            config_override: None,
                        }
                    }
                },
                quote! {
                    impl #builder_name
                },
                None,
                None,
            )
        };

        let application_builder = quote! {
            #builder_struct

            #builder_impl
            {
                #builder_new

                pub fn with_clock(mut self, clock: RobotClock) -> Self {
                    self.clock = Some(clock);
                    self
                }

                pub fn with_unified_logger(mut self, unified_logger: Arc<Mutex<UnifiedLoggerWrite>>) -> Self {
                    self.unified_logger = Some(unified_logger);
                    self
                }

                pub fn with_context(mut self, copper_ctx: &CopperContext) -> Self {
                    self.clock = Some(copper_ctx.clock.clone());
                    self.unified_logger = Some(copper_ctx.unified_logger.clone());
                    self
                }

                pub fn with_config(mut self, config_override: CuConfig) -> Self {
                        self.config_override = Some(config_override);
                        self
                }

                #builder_sim_callback_method

                pub fn build(self) -> CuResult<#application_name> {
                    #application_name::new(
                        self.clock
                            .ok_or(CuError::from("Clock missing from builder"))?,
                        self.unified_logger
                            .ok_or(CuError::from("Unified logger missing from builder"))?,
                        self.config_override,
                        #builder_build_sim_callback_arg
                    )
                }
            }
        };

        // Convert the modified struct back into a TokenStream
        let mission_mod_tokens = quote! {
            mod #mission_mod {
                use super::*;  // import the modules the main app did.

                use cu29::bincode::Encode;
                use cu29::bincode::enc::Encoder;
                use cu29::bincode::error::EncodeError;
                use cu29::bincode::Decode;
                use cu29::bincode::de::Decoder;
                use cu29::bincode::error::DecodeError;
                use cu29::clock::RobotClock;
                use cu29::clock::OptionCuTime;
                use cu29::clock::ClockProvider;
                use cu29::config::CuConfig;
                use cu29::config::ComponentConfig;
                use cu29::config::MonitorConfig;
                use cu29::config::read_configuration;
                use cu29::config::read_configuration_str;
                use cu29::curuntime::CuRuntime;
                use cu29::curuntime::CopperContext;
                use cu29::CuResult;
                use cu29::CuError;
                use cu29::cutask::CuSrcTask;
                use cu29::cutask::CuSinkTask;
                use cu29::cutask::CuTask;
                use cu29::cutask::CuMsg;
                use cu29::cutask::CuMsgMetadata;
                use cu29::copperlist::CopperList;
                use cu29::monitoring::CuMonitor; // Trait import.
                use cu29::monitoring::NoMonitor;
                use cu29::monitoring::CuTaskState;
                use cu29::monitoring::Decision;
                use cu29::prelude::app::CuApplication;
                use cu29::prelude::app::CuSimApplication;
                use cu29::prelude::debug;
                use cu29::prelude::stream_write;
                use cu29::prelude::UnifiedLoggerWrite;
                use cu29::prelude::UnifiedLogType;
                use std::sync::Arc;
                use std::sync::Mutex;


                // This is the heart of everything.
                // CuTasks is the list of all the tasks types.
                // CuList is a CopperList with the list of all the messages types as msgs.
                pub type CuTasks = #task_types_tuple;

                // This is the variation with stubs for the sources and sinks in simulation mode.
                pub type CuSimTasks = #task_types_tuple_sim;

                pub const TASKS_IDS: &'static [&'static str] = &[#( #all_tasks_ids ),*];

                #culist_support

                #sim_support

                pub fn tasks_instanciator(all_instances_configs: Vec<Option<&ComponentConfig>>) -> CuResult<CuTasks> {
                    Ok(( #(#task_instances_init_code),*, ))
                }

                pub fn tasks_instanciator_sim(all_instances_configs: Vec<Option<&ComponentConfig>>) -> CuResult<CuSimTasks> {
                    Ok(( #(#task_sim_instances_init_code),*, ))
                }

                pub fn monitor_instanciator(config: &CuConfig) -> #monitor_type {
                    #monitor_type::new(config, #mission_mod::TASKS_IDS).expect("Failed to create the given monitor.")
                }

                // The application for this mission
                pub #application_struct

                #application_impl

                #application_builder
            }

        };
        all_missions_tokens.push(mission_mod_tokens);
    }

    let default_application_tokens = if all_missions.contains_key("default") {
        quote! {
        use default::#builder_name;
        use default::#application_name;
        }
    } else {
        quote! {}
    };

    let result: proc_macro2::TokenStream = quote! {
        #(#all_missions_tokens)*
        #default_application_tokens
    };

    // Print and format the generated code using rustfmt
    #[cfg(feature = "macro_debug")]
    {
        let formatted_code = rustfmt_generated_code(result.to_string());
        eprintln!("\n     ===    Gen. Runtime ===\n");
        eprintln!("{}", highlight_rust_code(formatted_code));
        eprintln!("\n     === === === === === ===\n");
    }
    result.into()
}

fn read_config(config_file: &str) -> CuResult<CuConfig> {
    let filename = config_full_path(config_file);

    read_configuration(filename.as_str())
}

fn config_full_path(config_file: &str) -> String {
    let mut config_full_path = utils::caller_crate_root();
    config_full_path.push(config_file);
    let filename = config_full_path
        .as_os_str()
        .to_str()
        .expect("Could not interpret the config file name");
    filename.to_string()
}

/// Extract all the tasks types in their index order and their ids.
fn extract_tasks_types(graph: &CuGraph) -> (Vec<String>, Vec<CuTaskType>, Vec<String>, Vec<Type>) {
    let all_id_nodes = graph.get_all_nodes();

    // Get all the tasks Ids
    let all_tasks_ids: Vec<String> = all_id_nodes
        .iter()
        .map(|(_, node)| node.get_id().to_string())
        .collect();

    let all_task_cutype: Vec<CuTaskType> = all_id_nodes
        .iter()
        .map(|(id, _)| find_task_type_for_id(graph, *id))
        .collect();

    // Collect all the type names used by our configs.
    let all_types_names: Vec<String> = all_id_nodes
        .iter()
        .map(|(_, node)| node.get_type().to_string())
        .collect();

    // Transform them as Rust types
    let all_types: Vec<Type> = all_types_names
        .iter()
        .map(|name| {
            parse_str(name)
                .unwrap_or_else(|_| panic!("Could not transform {name} into a Task Rust type."))
        })
        .collect();
    (all_tasks_ids, all_task_cutype, all_types_names, all_types)
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
                                "Could not transform {output_msg_type} into a message Rust type."
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
fn build_culist_tuple(all_msgs_types_in_culist_order: &[Type]) -> TypeTuple {
    if all_msgs_types_in_culist_order.is_empty() {
        parse_quote! { () }
    } else {
        parse_quote! {
            ( #( CuMsg<#all_msgs_types_in_culist_order> ),* )
        }
    }
}

/// This is the bincode encoding part of the CuMsgs
fn build_culist_tuple_encode(all_msgs_types_in_culist_order: &[Type]) -> ItemImpl {
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
        impl Encode for CuMsgs {
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                #(#encode_fields)*
                Ok(())
            }
        }
    }
}

/// This is the bincode decoding part of the CuMsgs
fn build_culist_tuple_decode(all_msgs_types_in_culist_order: &[Type]) -> ItemImpl {
    let indices: Vec<usize> = (0..all_msgs_types_in_culist_order.len()).collect();

    // Generate the `CuMsg::<T>::decode(decoder)?` for each tuple index
    let decode_fields: Vec<_> = indices
        .iter()
        .map(|i| {
            let t = &all_msgs_types_in_culist_order[*i];
            quote! { CuMsg::<#t>::decode(decoder)? }
        })
        .collect();

    parse_quote! {
        impl Decode<()> for CuMsgs {
            fn decode<D: Decoder<Context=()>>(decoder: &mut D) -> Result<Self, DecodeError> {
                Ok(CuMsgs ((
                    #(#decode_fields),*
                )))
            }
        }
    }
}

fn build_culist_tuple_debug(all_msgs_types_in_culist_order: &[Type]) -> ItemImpl {
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

#[cfg(test)]
mod tests {
    // See tests/compile_file directory for more information
    #[test]
    fn test_compile_fail() {
        let t = trybuild::TestCases::new();
        t.compile_fail("tests/compile_fail/*/*.rs");
    }
}
