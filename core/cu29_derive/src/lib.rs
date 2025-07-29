use proc_macro::TokenStream;
use quote::{format_ident, quote};
use std::fs::read_to_string;
use syn::meta::parser;
use syn::Fields::{Named, Unnamed};
use syn::{
    parse_macro_input, parse_quote, parse_str, Field, Fields, ItemImpl, ItemStruct, LitStr, Type,
    TypeTuple,
};

#[cfg(feature = "macro_debug")]
use crate::format::rustfmt_generated_code;
use crate::utils::config_id_to_enum;
use cu29_runtime::config::CuConfig;
use cu29_runtime::config::{read_configuration, CuGraph};
use cu29_runtime::curuntime::{
    compute_runtime_plan, find_task_type_for_id, CuExecutionLoop, CuExecutionUnit, CuTaskType,
};
use cu29_traits::CuResult;
use proc_macro2::{Ident, Span};

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
/// It will create a new type called CuStampedDataSet you can pass to the log reader for decoding:
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

    // Give a name compatible with a struct to match the task ids to their output in the CuStampedDataSet tuple.
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
        "[The CuStampedDataSet matching tasks ids are {:?}]",
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
            use cu29::prelude::ErasedCuStampedData;
            use cu29::prelude::ErasedCuStampedDataSet;
            use cu29::prelude::MatchingTasks;
            use cu29::prelude::Serialize;
            use cu29::prelude::CuMsg;
            use cu29::prelude::CuMsgMetadata;
            #support
        }
        use cumsgs::CuStampedDataSet;
        type CuMsgs=CuStampedDataSet;
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

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist tuple serialize support]");
    let msgs_types_tuple_serialize = build_culist_tuple_serialize(&all_msgs_types_in_culist_order);

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the default tuple support]");
    let msgs_types_tuple_default = build_culist_tuple_default(&all_msgs_types_in_culist_order);

    #[cfg(feature = "macro_debug")]
    eprintln!("[build erasedcumsgs]");

    let erasedmsg_trait_impl = build_culist_erasedcumsgs(&all_msgs_types_in_culist_order);

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
                #[allow(dead_code)]
                pub fn #fn_name(&self) -> &CuMsg<#payload_type> {
                    &self.0.#index
                }
            }
        },
    );

    // This generates a way to get the metadata of every single message of a culist at low cost
    quote! {
        #collect_metadata_function

        pub struct CuStampedDataSet(pub #msgs_types_tuple);

        pub type CuList = CopperList<CuStampedDataSet>;

        impl CuStampedDataSet {
            #(#methods)*

            #[allow(dead_code)]
            fn get_tuple(&self) -> &#msgs_types_tuple {
                &self.0
            }

            #[allow(dead_code)]
            fn get_tuple_mut(&mut self) -> &mut #msgs_types_tuple {
                &mut self.0
            }
        }

        impl MatchingTasks for CuStampedDataSet {
            #[allow(dead_code)]
            fn get_all_task_ids() -> &'static [&'static str] {
                &[#(#all_tasks_as_struct_member_name),*]
            }
        }

        // Adds the bincode support for the copper list tuple
        #msgs_types_tuple_encode
        #msgs_types_tuple_decode

        // Adds the debug support
        #msgs_types_tuple_debug

        // Adds the serialization support
        #msgs_types_tuple_serialize

        // Adds the default support
        #msgs_types_tuple_default

        // Adds the type erased CuStampedDataSet support (to help generic serialized conversions)
        #erasedmsg_trait_impl
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
                let enum_ident = Ident::new(&enum_entry_name, Span::call_site());
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

                let inputs_type = if inputs.is_empty() {
                    quote! { () }
                } else if inputs.len() == 1 {
                    let input = inputs.first().unwrap();
                    quote! { &'a #input }
                } else {
                    quote! { &'a (#(&'a #inputs),*) }
                };

                quote! {
                    #enum_ident(cu29::simulation::CuTaskCallbackState<#inputs_type, &'a mut #output>)
                }
            }
            CuExecutionUnit::Loop(_) => {
                todo!("Needs to be implemented")
            }
        })
        .collect();
    quote! {
        // not used if sim is not generated but this is ok.
        #[allow(dead_code)]
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

    // This is common for all the mission as it will be inserted in the respective modules with their local CuTasks, CuStampedDataSet etc...
    #[cfg(feature = "macro_debug")]
    eprintln!("[build runtime field]");
    // add that to a new field
    let runtime_field: Field = if sim_mode {
        parse_quote! {
            copper_runtime: cu29::curuntime::CuRuntime<CuSimTasks, CuStampedDataSet, #monitor_type, #DEFAULT_CLNB>
        }
    } else {
        parse_quote! {
            copper_runtime: cu29::curuntime::CuRuntime<CuTasks, CuStampedDataSet, #monitor_type, #DEFAULT_CLNB>
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
        eprintln!("[extract tasks ids & types]");
        let task_specs = CuTaskSpecSet::from_graph(graph);

        #[cfg(feature = "macro_debug")]
        eprintln!("[runtime plan for mission {mission}]");
        let runtime_plan: CuExecutionLoop = match compute_runtime_plan(graph) {
            Ok(plan) => plan,
            Err(e) => return return_error(format!("Could not compute runtime plan: {e}")),
        };

        #[cfg(feature = "macro_debug")]
        eprintln!("{runtime_plan:?}");

        let all_sim_tasks_types: Vec<Type> = task_specs.ids
            .iter()
            .zip(&task_specs.cutypes)
            .zip(&task_specs.sim_task_types)
            .zip(&task_specs.background_flags)
            .map(|(((task_id, cutype), stype), background)| {
                match cutype {
                    CuTaskType::Source => {
                        if *background {
                            panic!("CuSrcTask {task_id} cannot be a background task, it should be a regular task.");
                        }
                        let msg_type = graph
                            .get_node_output_msg_type(task_id.as_str())
                            .unwrap_or_else(|| panic!("CuSrcTask {task_id} should have an outgoing connection with a valid output msg type"));
                        let sim_task_name = format!("cu29::simulation::CuSimSrcTask<{msg_type}>");
                        parse_str(sim_task_name.as_str()).unwrap_or_else(|_| panic!("Could not build the placeholder for simulation: {sim_task_name}"))
                    }
                    CuTaskType::Regular => {
                        // TODO: wrap that correctly in a background task if background is true.
                        stype.clone()
                    },
                    CuTaskType::Sink => {
                        if *background {
                            panic!("CuSinkTask {task_id} cannot be a background task, it should be a regular task.");
                        }
                        let msg_type = graph
                            .get_node_input_msg_type(task_id.as_str())
                            .unwrap_or_else(|| panic!("CuSinkTask {task_id} should have an incoming connection with a valid input msg type"));
                        let sim_task_name = format!("cu29::simulation::CuSimSinkTask<{msg_type}>");
                        parse_str(sim_task_name.as_str()).unwrap_or_else(|_| panic!("Could not build the placeholder for simulation: {sim_task_name}"))
                    }
                }
    })
    .collect();

        #[cfg(feature = "macro_debug")]
        eprintln!("[build task tuples]");

        let task_types = &task_specs.task_types;
        // Build the tuple of all those types
        // note the extraneous, at the end is to make the tuple work even if this is only one element
        let task_types_tuple: TypeTuple = parse_quote! {
            (#(#task_types),*,)
        };

        let task_types_tuple_sim: TypeTuple = parse_quote! {
            (#(#all_sim_tasks_types),*,)
        };

        #[cfg(feature = "macro_debug")]
        eprintln!("[gen instances]");
        // FIXME: implement here the threadpool emulation.
        let task_sim_instances_init_code = all_sim_tasks_types.iter().enumerate().map(|(index, ty)| {
            let additional_error_info = format!(
                "Failed to get create instance for {}, instance index {}.",
                task_specs.type_names[index], index
            );

            quote! {
                <#ty>::new(all_instances_configs[#index]).map_err(|e| e.add_cause(#additional_error_info))?
            }
        }).collect::<Vec<_>>();

        let task_instances_init_code = task_specs.instantiation_types.iter().zip(&task_specs.background_flags).enumerate().map(|(index, (task_type, background))| {
            let additional_error_info = format!(
                "Failed to get create instance for {}, instance index {}.",
                task_specs.type_names[index], index
            );
            if *background {
                quote! {
                    #task_type::new(all_instances_configs[#index], threadpool).map_err(|e| e.add_cause(#additional_error_info))?
                }
            } else {
                quote! {
                    #task_type::new(all_instances_configs[#index]).map_err(|e| e.add_cause(#additional_error_info))?
                }
            }
        }).collect::<Vec<_>>();

        // Generate the code to create instances of the nodes
        // It maps the types to their index
        let (
            task_restore_code,
            start_calls,
            stop_calls,
            preprocess_calls,
            postprocess_calls,
            ): (Vec<_>, Vec<_>, Vec<_>, Vec<_>, Vec<_>) = itertools::multiunzip(
            (0..task_specs.task_types.len())
            .map(|index| {
                let task_index = int2sliceindex(index as u32);
                let task_tuple_index = syn::Index::from(index);
                let task_enum_name = config_id_to_enum(&task_specs.ids[index]);
                let enum_name = Ident::new(&task_enum_name, Span::call_site());
                (
                    // Tasks keyframe restore code
                    quote! {
                        tasks.#task_tuple_index.thaw(&mut decoder).map_err(|e| CuError::new_with_cause("Failed to thaw", e))?
                    },
                    {  // Start calls
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
                    {  // Stop calls
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
                    {  // Preprocess calls
                        let monitoring_action = quote! {
                            let decision = monitor.process_error(#index, CuTaskState::Preprocess, &error);
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
                                if let Err(error) = tasks.#task_index.preprocess(clock) {
                                    #monitoring_action
                                }
                            }
                        }
                    },
                    {  // Postprocess calls
                        let monitoring_action = quote! {
                            let decision = monitor.process_error(#index, CuTaskState::Postprocess, &error);
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
                                if let Err(error) = tasks.#task_index.postprocess(clock) {
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

        let runtime_plan_code_and_logging: Vec<(proc_macro2::TokenStream, proc_macro2::TokenStream)> = runtime_plan.steps
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
                        let task_instance = quote! { tasks.#node_index };
                        let comment_str = format!(
                            "DEBUG ->> {} ({:?}) Id:{} I:{:?} O:{:?}",
                            step.node.get_id(),
                            step.task_type,
                            step.node_id,
                            step.input_msg_indices_types,
                            step.output_msg_index_type
                        );
                        let comment_tokens = quote! {
                            {
                                let _ = stringify!(#comment_str);
                            }
                        };
                        // let comment_tokens: proc_macro2::TokenStream = parse_str(&comment_str).unwrap();
                        let tid = step.node_id as usize;
                        taskid_call_order.push(tid);

                        let task_enum_name = config_id_to_enum(&task_specs.ids[tid]);
                        let enum_name = Ident::new(&task_enum_name, proc_macro2::Span::call_site());

                        let (process_call, preprocess_logging) = match step.task_type {
                            CuTaskType::Source => {
                                if let Some((output_index, _)) = &step.output_msg_index_type {
                                    let output_culist_index = int2sliceindex(*output_index);

                                    let monitoring_action = quote! {
                                        debug!("Task {}: Error during process: {}", #mission_mod::TASKS_IDS[#tid], &error);
                                        let decision = monitor.process_error(#tid, CuTaskState::Process, &error);
                                        match decision {
                                            Decision::Abort => {
                                                debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                            during process. Skipping the processing of CL {}.", #mission_mod::TASKS_IDS[#tid], clid);
                                                monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                                                cl_manager.end_of_processing(clid)?;
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

                                    (quote! {  // process call
                                        {
                                            #comment_tokens
                                            {
                                                // Maybe freeze the task if this is a "key frame"
                                                kf_manager.freeze_task(clid, &#task_instance)?;
                                                #call_sim_callback
                                                let cumsg_output = &mut msgs.#output_culist_index;
                                                cumsg_output.metadata.process_time.start = clock.now().into();
                                                let maybe_error = if doit {
                                                    #task_instance.process(clock, cumsg_output)
                                                } else {
                                                    Ok(())
                                                };
                                                cumsg_output.metadata.process_time.end = clock.now().into();
                                                if let Err(error) = maybe_error {
                                                    #monitoring_action
                                                }
                                            }
                                        }
                                    }, {  // logging preprocess
                                        if !task_specs.logging_enabled[*output_index as usize] {

                                            #[cfg(feature = "macro_debug")]
                                            eprintln!(
                                                "{} -> Logging Disabled",
                                                step.node.get_id(),
                                            );


                                            let output_culist_index = int2sliceindex(*output_index);
                                            quote! {
                                                let mut cumsg_output = &mut culist.msgs.0.#output_culist_index;
                                                cumsg_output.clear_payload();
                                            }
                                        } else {
                                            #[cfg(feature = "macro_debug")]
                                            eprintln!(
                                                "{} -> Logging Enabled",
                                                step.node.get_id(),
                                            );
                                            quote!() // do nothing
                                        }
                                    }
                                    )
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
                                        let decision = monitor.process_error(#tid, CuTaskState::Process, &error);
                                        match decision {
                                            Decision::Abort => {
                                                debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                            during process. Skipping the processing of CL {}.", #mission_mod::TASKS_IDS[#tid], clid);
                                                monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                                                cl_manager.end_of_processing(clid)?;
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
                                        let inputs_type = if indices.len() == 1 {
                                            // Not a tuple for a single input
                                            quote! { #(msgs.#indices)* }
                                        } else {
                                            // A tuple for multiple inputs
                                            quote! { (#(&msgs.#indices),*) }
                                        };

                                        quote! {
                                            let doit = {
                                                let cumsg_input = &#inputs_type;
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

                                    let inputs_type = if indices.len() == 1 {
                                        // Not a tuple for a single input
                                        quote! { #(msgs.#indices)* }
                                    } else {
                                        // A tuple for multiple inputs
                                        quote! { (#(&msgs.#indices),*) }
                                    };

                                    (quote! {
                                        {
                                            #comment_tokens
                                            // Maybe freeze the task if this is a "key frame"
                                            kf_manager.freeze_task(clid, &#task_instance)?;
                                            #call_sim_callback
                                            let cumsg_input = &#inputs_type;
                                            // This is the virtual output for the sink
                                            let cumsg_output = &mut msgs.#output_culist_index;
                                            cumsg_output.metadata.process_time.start = clock.now().into();
                                            let maybe_error = if doit {#task_instance.process(clock, cumsg_input)} else {Ok(())};
                                            cumsg_output.metadata.process_time.end = clock.now().into();
                                            if let Err(error) = maybe_error {
                                                #monitoring_action
                                            }
                                        }
                                    }, {
                                        quote!() // do nothing for logging
                                    })
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
                                        let decision = monitor.process_error(#tid, CuTaskState::Process, &error);
                                        match decision {
                                            Decision::Abort => {
                                                debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                            during process. Skipping the processing of CL {}.", #mission_mod::TASKS_IDS[#tid], clid);
                                                monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                                                cl_manager.end_of_processing(clid)?;
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
                                        let inputs_type = if indices.len() == 1 {
                                            // Not a tuple for a single input
                                            quote! { #(msgs.#indices)* }
                                        } else {
                                            // A tuple for multiple inputs
                                            quote! { (#(&msgs.#indices),*) }
                                        };

                                        quote! {
                                            let doit = {
                                                let cumsg_input = &#inputs_type;
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
                                    let inputs_type = if indices.len() == 1 {
                                        // Not a tuple for a single input
                                        quote! { #(msgs.#indices)* }
                                    } else {
                                        // A tuple for multiple inputs
                                        quote! { (#(&msgs.#indices),*) }
                                    };

                                    (quote! {
                                        {
                                            #comment_tokens
                                            // Maybe freeze the task if this is a "key frame"
                                            kf_manager.freeze_task(clid, &#task_instance)?;
                                            #call_sim_callback
                                            let cumsg_input = &#inputs_type;
                                            let cumsg_output = &mut msgs.#output_culist_index;
                                            cumsg_output.metadata.process_time.start = clock.now().into();
                                            let maybe_error = if doit {#task_instance.process(clock, cumsg_input, cumsg_output)} else {Ok(())};
                                            cumsg_output.metadata.process_time.end = clock.now().into();
                                            if let Err(error) = maybe_error {
                                                #monitoring_action
                                            }
                                        }
                                    }, {

                                    if !task_specs.logging_enabled[*output_index as usize] {
                                        #[cfg(feature = "macro_debug")]
                                        eprintln!(
                                            "{} -> Logging Disabled",
                                            step.node.get_id(),
                                        );
                                        let output_culist_index = int2sliceindex(*output_index);
                                        quote! {
                                                let mut cumsg_output = &mut culist.msgs.0.#output_culist_index;
                                                cumsg_output.clear_payload();
                                            }
                                   } else {
                                        #[cfg(feature = "macro_debug")]
                                        eprintln!(
                                            "{} -> Logging Enabled",
                                            step.node.get_id(),
                                        );
                                         quote!() // do nothing logging is enabled
                                   }
                                })
                                } else {
                                    panic!("Regular task should have an output message index.");
                                }
                            }
                        };

                        (process_call, preprocess_logging)
                    }
                    CuExecutionUnit::Loop(_) => todo!("Needs to be implemented"),
                }
            }).collect();
        #[cfg(feature = "macro_debug")]
        eprintln!("[Culist access order:  {taskid_call_order:?}]");

        // Give a name compatible with a struct to match the task ids to their output in the CuStampedDataSet tuple.
        let all_tasks_member_ids: Vec<String> = task_specs
            .ids
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

        let sim_callback_on_new_calls = task_specs.ids.iter().enumerate().map(|(i, id)| {
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

        let (runtime_plan_code, preprocess_logging_calls): (Vec<_>, Vec<_>) =
            itertools::multiunzip(runtime_plan_code_and_logging);

        #[cfg(feature = "macro_debug")]
        eprintln!("[build the run methods]");
        let run_methods = quote! {

            #run_one_iteration {

                // Pre-explode the runtime to avoid complexity with partial borrowing in the generated code.
                let runtime = &mut self.copper_runtime;
                let clock = &runtime.clock;
                let monitor = &mut runtime.monitor;
                let tasks = &mut runtime.tasks;
                let cl_manager = &mut runtime.copperlists_manager;
                let kf_manager = &mut runtime.keyframes_manager;

                // Preprocess calls can happen at any time, just packed them up front.
                #(#preprocess_calls)*

                let culist = cl_manager.inner.create().expect("Ran out of space for copper lists"); // FIXME: error handling
                let clid = culist.id;
                kf_manager.reset(clid, clock); // beginning of processing, we empty the serialized frozen states of the tasks.
                culist.change_state(cu29::copperlist::CopperListState::Processing);
                {
                    let msgs = &mut culist.msgs.0;
                    #(#runtime_plan_code)*
                } // drop(msgs);
                monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;

                // here drop the payloads if we don't want them to be logged.
                #(#preprocess_logging_calls)*

                cl_manager.end_of_processing(clid)?;
                kf_manager.end_of_processing(clid)?;

                // Postprocess calls can happen at any time, just packed them up at the end.
                #(#postprocess_calls)*
                Ok(())
            }

            fn restore_keyframe(&mut self, keyframe: &KeyFrame) -> CuResult<()> {
                let runtime = &mut self.copper_runtime;
                let clock = &runtime.clock;
                let tasks = &mut runtime.tasks;
                let config = cu29::bincode::config::standard();
                let reader = cu29::bincode::de::read::SliceReader::new(&keyframe.serialized_tasks);
                let mut decoder = DecoderImpl::new(reader, config, ());
                #(#task_restore_code);*;
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
                static STOP_FLAG: AtomicBool = AtomicBool::new(false);
                ctrlc::set_handler(move || {
                    STOP_FLAG.store(true, Ordering::SeqCst);
                }).expect("Error setting Ctrl-C handler");

                self.start_all_tasks(#sim_callback_arg)?;
                let result = loop  {
                    let result = self.run_one_iteration(#sim_callback_arg);

                    if STOP_FLAG.load(Ordering::SeqCst) || result.is_err() {
                        break result;
                    }
                };
                if result.is_err() {
                    error!("A task errored out: {}", &result);
                }
                self.stop_all_tasks(#sim_callback_arg)?;
                result
            }
        };

        let tasks_type = if sim_mode {
            quote!(CuSimTasks)
        } else {
            quote!(CuTasks)
        };

        let tasks_instanciator_fn = if sim_mode {
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

                    let keyframes_stream = stream_write::<KeyFrame>(
                        unified_logger.clone(),
                        UnifiedLogType::FrozenTasks,
                        1024 * 1024 * 10, // 10 MiB
                    );


                    let application = Ok(#application_name {
                        copper_runtime: CuRuntime::<#mission_mod::#tasks_type, #mission_mod::CuStampedDataSet, #monitor_type, #DEFAULT_CLNB>::new(
                            clock,
                            &config,
                            Some(#mission),
                            #mission_mod::#tasks_instanciator_fn,
                            #mission_mod::monitor_instanciator,
                            copperlist_stream,
                            keyframes_stream)?, // FIXME: gbin
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
                    #[allow(dead_code)]
                    pub struct #builder_name <'a, F> {
                        clock: Option<RobotClock>,
                        unified_logger: Option<Arc<Mutex<UnifiedLoggerWrite>>>,
                        config_override: Option<CuConfig>,
                        sim_callback: Option<&'a mut F>
                    }
                },
                quote! {
                    #[allow(dead_code)]
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
                    #[allow(dead_code)]
                    pub struct #builder_name {
                        clock: Option<RobotClock>,
                        unified_logger: Option<Arc<Mutex<UnifiedLoggerWrite>>>,
                        config_override: Option<CuConfig>,
                    }
                },
                quote! {
                    #[allow(dead_code)]
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

                #[allow(dead_code)]
                pub fn with_clock(mut self, clock: RobotClock) -> Self {
                    self.clock = Some(clock);
                    self
                }

                #[allow(dead_code)]
                pub fn with_unified_logger(mut self, unified_logger: Arc<Mutex<UnifiedLoggerWrite>>) -> Self {
                    self.unified_logger = Some(unified_logger);
                    self
                }

                #[allow(dead_code)]
                pub fn with_context(mut self, copper_ctx: &CopperContext) -> Self {
                    self.clock = Some(copper_ctx.clock.clone());
                    self.unified_logger = Some(copper_ctx.unified_logger.clone());
                    self
                }

                #[allow(dead_code)]
                pub fn with_config(mut self, config_override: CuConfig) -> Self {
                        self.config_override = Some(config_override);
                        self
                }

                #builder_sim_callback_method

                #[allow(dead_code)]
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

        let ids = task_specs.ids;
        // Convert the modified struct back into a TokenStream
        let mission_mod_tokens = quote! {
            mod #mission_mod {
                use super::*;  // import the modules the main app did.

                use cu29::bincode::Encode;
                use cu29::bincode::enc::Encoder;
                use cu29::bincode::error::EncodeError;
                use cu29::bincode::Decode;
                use cu29::bincode::de::Decoder;
                use cu29::bincode::de::DecoderImpl;
                use cu29::bincode::error::DecodeError;
                use cu29::rayon::ThreadPool;
                use cu29::clock::RobotClock;
                use cu29::config::CuConfig;
                use cu29::config::ComponentConfig;
                use cu29::cuasynctask::CuAsyncTask;
                use cu29::curuntime::CuRuntime;
                use cu29::curuntime::KeyFrame;
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
                use cu29::monitoring::CuTaskState;
                use cu29::monitoring::Decision;
                use cu29::prelude::app::CuApplication;
                use cu29::prelude::debug;
                use cu29::prelude::stream_write;
                use cu29::prelude::UnifiedLoggerWrite;
                use cu29::prelude::UnifiedLogType;
                use std::sync::Arc;
                use std::sync::Mutex;
                use std::sync::atomic::{AtomicBool, Ordering};

                // Not used if the used doesn't generate Sim.
                #[allow(unused_imports)]
                use cu29::prelude::app::CuSimApplication;

                // Not used if a monitor is present
                #[allow(unused_imports)]
                use cu29::monitoring::NoMonitor;

                // This is the heart of everything.
                // CuTasks is the list of all the tasks types.
                // CuList is a CopperList with the list of all the messages types as msgs.
                pub type CuTasks = #task_types_tuple;

                // This is the variation with stubs for the sources and sinks in simulation mode.
                // Not used if the used doesn't generate Sim.
                #[allow(dead_code)]
                pub type CuSimTasks = #task_types_tuple_sim;

                pub const TASKS_IDS: &'static [&'static str] = &[#( #ids ),*];

                #culist_support

                #sim_support

                pub fn tasks_instanciator<'c>(all_instances_configs: Vec<Option<&'c ComponentConfig>>, threadpool: Arc<ThreadPool>) -> CuResult<CuTasks> {
                    Ok(( #(#task_instances_init_code),*, ))
                }

                #[allow(dead_code)]
                pub fn tasks_instanciator_sim(all_instances_configs: Vec<Option<&ComponentConfig>>, _threadpool: Arc<ThreadPool>) -> CuResult<CuSimTasks> {
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
        // you can bypass the builder and not use it
        #[allow(unused_imports)]
        use default::#builder_name;

        #[allow(unused_imports)]
        use default::#application_name;
        }
    } else {
        quote!() // do nothing
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
        eprintln!("{formatted_code}");
        // if you need colors back: eprintln!("{}", highlight_rust_code(formatted_code)); was disabled for cubuild.
        // or simply use cargo expand
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

fn extract_tasks_output_types(graph: &CuGraph) -> Vec<Option<Type>> {
    let result = graph
        .get_all_nodes()
        .iter()
        .map(|(_, node)| {
            let id = node.get_id();
            let type_str = graph.get_node_output_msg_type(id.as_str());
            let result = type_str.map(|type_str| {
                let result = parse_str::<Type>(type_str.as_str())
                    .expect("Could not parse output message type.");
                result
            });
            result
        })
        .collect();
    result
}

struct CuTaskSpecSet {
    pub ids: Vec<String>,
    pub cutypes: Vec<CuTaskType>,
    pub background_flags: Vec<bool>,
    pub logging_enabled: Vec<bool>,
    pub type_names: Vec<String>,
    pub task_types: Vec<Type>,
    pub instantiation_types: Vec<Type>,
    pub sim_task_types: Vec<Type>,
    #[allow(dead_code)]
    pub output_types: Vec<Option<Type>>,
}

impl CuTaskSpecSet {
    pub fn from_graph(graph: &CuGraph) -> Self {
        let all_id_nodes = graph.get_all_nodes();

        let ids = all_id_nodes
            .iter()
            .map(|(_, node)| node.get_id().to_string())
            .collect();

        let cutypes = all_id_nodes
            .iter()
            .map(|(id, _)| find_task_type_for_id(graph, *id))
            .collect();

        let background_flags: Vec<bool> = all_id_nodes
            .iter()
            .map(|(_, node)| node.is_background())
            .collect();

        let logging_enabled: Vec<bool> = all_id_nodes
            .iter()
            .map(|(_, node)| node.is_logging_enabled())
            .collect();

        let type_names: Vec<String> = all_id_nodes
            .iter()
            .map(|(_, node)| node.get_type().to_string())
            .collect();

        let output_types = extract_tasks_output_types(graph);

        let task_types = type_names
            .iter()
            .zip(background_flags.iter())
            .zip(output_types.iter())
            .map(|((name, &background), output_type)| {
                let name_type = parse_str::<Type>(name).unwrap_or_else(|error| {
                    panic!("Could not transform {name} into a Task Rust type: {error}");
                });
                if background {
                    if let Some(output_type) = output_type {
                        parse_quote!(cu29::cuasynctask::CuAsyncTask<#name_type, #output_type>)
                    } else {
                        panic!("{name}: If a task is background, it has to have an output");
                    }
                } else {
                    name_type
                }
            })
            .collect();

        let instantiation_types = type_names
            .iter()
            .zip(background_flags.iter())
            .zip(output_types.iter())
            .map(|((name, &background), output_type)| {
                let name_type = parse_str::<Type>(name).unwrap_or_else(|error| {
                    panic!("Could not transform {name} into a Task Rust type: {error}");
                });
                if background {
                    if let Some(output_type) = output_type {
                        parse_quote!(cu29::cuasynctask::CuAsyncTask::<#name_type, #output_type>)
                    } else {
                        panic!("{name}: If a task is background, it has to have an output");
                    }
                } else {
                    name_type
                }
            })
            .collect();

        let sim_task_types = type_names
            .iter()
            .map(|name| {
                parse_str::<Type>(name).unwrap_or_else(|err| {
                    eprintln!("Could not transform {name} into a Task Rust type.");
                    panic!("{err}")
                })
            })
            .collect();

        Self {
            ids,
            cutypes,
            background_flags,
            logging_enabled,
            type_names,
            task_types,
            instantiation_types,
            sim_task_types,
            output_types,
        }
    }
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

/// This is the bincode encoding part of the CuStampedDataSet
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
        impl Encode for CuStampedDataSet {
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                #(#encode_fields)*
                Ok(())
            }
        }
    }
}

/// This is the bincode decoding part of the CuStampedDataSet
fn build_culist_tuple_decode(all_msgs_types_in_culist_order: &[Type]) -> ItemImpl {
    let indices: Vec<usize> = (0..all_msgs_types_in_culist_order.len()).collect();

    // Generate the `CuStampedData::<T>::decode(decoder)?` for each tuple index
    let decode_fields: Vec<_> = indices
        .iter()
        .map(|i| {
            let t = &all_msgs_types_in_culist_order[*i];
            quote! { CuMsg::<#t>::decode(decoder)? }
        })
        .collect();

    parse_quote! {
        impl Decode<()> for CuStampedDataSet {
            fn decode<D: Decoder<Context=()>>(decoder: &mut D) -> Result<Self, DecodeError> {
                Ok(CuStampedDataSet ((
                    #(#decode_fields),*
                )))
            }
        }
    }
}

fn build_culist_erasedcumsgs(all_msgs_types_in_culist_order: &[Type]) -> ItemImpl {
    let indices: Vec<usize> = (0..all_msgs_types_in_culist_order.len()).collect();
    let casted_fields: Vec<_> = indices
        .iter()
        .map(|i| {
            let idx = syn::Index::from(*i);
            quote! { &self.0.#idx as &dyn ErasedCuStampedData }
        })
        .collect();
    parse_quote! {
        impl ErasedCuStampedDataSet for CuStampedDataSet {
            fn cumsgs(&self) -> Vec<&dyn ErasedCuStampedData> {
                vec![
                    #(#casted_fields),*
                ]
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
        impl std::fmt::Debug for CuStampedDataSet {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.debug_tuple("CuStampedDataSet")
                    #(#debug_fields)*
                    .finish()
            }
        }
    }
}

/// This is the serde serialization part of the CuStampedDataSet
fn build_culist_tuple_serialize(all_msgs_types_in_culist_order: &[Type]) -> ItemImpl {
    let indices: Vec<usize> = (0..all_msgs_types_in_culist_order.len()).collect();
    let tuple_len = all_msgs_types_in_culist_order.len();

    // Generate the serialization for each tuple field
    let serialize_fields: Vec<_> = indices
        .iter()
        .map(|i| {
            let idx = syn::Index::from(*i);
            quote! { &self.0.#idx }
        })
        .collect();

    parse_quote! {
        impl Serialize for CuStampedDataSet {
            fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
            where
                S: serde::Serializer,
            {
                use serde::ser::SerializeTuple;
                let mut tuple = serializer.serialize_tuple(#tuple_len)?;
                #(tuple.serialize_element(#serialize_fields)?;)*
                tuple.end()
            }
        }
    }
}

/// This is the default implementation for CuStampedDataSet
fn build_culist_tuple_default(all_msgs_types_in_culist_order: &[Type]) -> ItemImpl {
    // Generate the serialization for each tuple field
    let default_fields: Vec<_> = all_msgs_types_in_culist_order
        .iter()
        .map(|msg_type| quote! { CuStampedData::<#msg_type, CuMsgMetadata>::default() })
        .collect();

    parse_quote! {
        impl Default for CuStampedDataSet {
            fn default() -> CuStampedDataSet
            {
                CuStampedDataSet((
                    #(#default_fields),*
                ))
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
