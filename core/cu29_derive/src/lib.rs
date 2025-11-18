use proc_macro::TokenStream;
use quote::{format_ident, quote};
use std::collections::HashMap;
use std::fs::read_to_string;
use syn::meta::parser;
use syn::Fields::{Named, Unnamed};
use syn::{
    parse_macro_input, parse_quote, parse_str, Field, Fields, ItemImpl, ItemStruct, LitStr, Type,
    TypeTuple,
};

#[cfg(feature = "macro_debug")]
use crate::format::rustfmt_generated_code;
use crate::utils::{config_id_to_bridge_const, config_id_to_enum, config_id_to_struct_member};
use cu29_runtime::config::CuConfig;
use cu29_runtime::config::{
    read_configuration, BridgeChannelConfigRepresentation, CuGraph, Flavor, Node, NodeId,
};
use cu29_runtime::curuntime::{
    compute_runtime_plan, find_task_type_for_id, CuExecutionLoop, CuExecutionStep, CuExecutionUnit,
    CuTaskType,
};
use cu29_traits::{CuError, CuResult};
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
    #[cfg(feature = "std")]
    let std = true;

    #[cfg(not(feature = "std"))]
    let std = false;

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
    let task_specs = CuTaskSpecSet::from_graph(graph);
    let channel_usage = collect_bridge_channel_usage(graph);
    let mut bridge_specs = build_bridge_specs(&cuconfig, graph, &channel_usage);
    let (culist_plan, exec_entities, plan_to_original) =
        match build_execution_plan(graph, &task_specs, &mut bridge_specs) {
            Ok(plan) => plan,
            Err(e) => return return_error(format!("Could not compute copperlist plan: {e}")),
        };
    let task_member_names = collect_task_member_names(graph);
    let (culist_order, node_output_positions) = collect_culist_metadata(
        &culist_plan,
        &exec_entities,
        &mut bridge_specs,
        &plan_to_original,
    );

    #[cfg(feature = "macro_debug")]
    eprintln!(
        "[The CuStampedDataSet matching tasks ids are {:?}]",
        culist_order
    );

    let support = gen_culist_support(
        &culist_plan,
        &culist_order,
        &node_output_positions,
        &task_member_names,
    );

    let extra_imports = if !std {
        quote! {
            use core::fmt::Debug;
            use core::fmt::Formatter;
            use core::fmt::Result as FmtResult;
            use alloc::vec;
        }
    } else {
        quote! {
            use std::fmt::Debug;
            use std::fmt::Formatter;
            use std::fmt::Result as FmtResult;
        }
    };

    let with_uses = quote! {
        mod cumsgs {
            use cu29::bincode::Encode;
            use cu29::bincode::enc::Encoder;
            use cu29::bincode::error::EncodeError;
            use cu29::bincode::Decode;
            use cu29::bincode::de::Decoder;
            use cu29::bincode::error::DecodeError;
            use cu29::copperlist::CopperList;
            use cu29::prelude::CuStampedData;
            use cu29::prelude::ErasedCuStampedData;
            use cu29::prelude::ErasedCuStampedDataSet;
            use cu29::prelude::MatchingTasks;
            use cu29::prelude::Serialize;
            use cu29::prelude::CuMsg;
            use cu29::prelude::CuMsgMetadata;
            use cu29::prelude::CuListZeroedInit;
            use cu29::prelude::CuCompactString;
            #extra_imports
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
    culist_indices_in_plan_order: &[usize],
    node_output_positions: &HashMap<NodeId, usize>,
    task_member_names: &[(NodeId, String)],
) -> proc_macro2::TokenStream {
    #[cfg(feature = "macro_debug")]
    eprintln!("[Extract msgs types]");
    let all_msgs_types_in_culist_order = extract_msg_types(runtime_plan);

    let culist_size = all_msgs_types_in_culist_order.len();
    let task_indices: Vec<_> = culist_indices_in_plan_order
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

    let task_name_literals: Vec<String> = task_member_names
        .iter()
        .map(|(_, name)| name.clone())
        .collect();

    let methods = task_member_names.iter().map(|(node_id, name)| {
        let output_position = node_output_positions
            .get(node_id)
            .unwrap_or_else(|| panic!("Task {name} (id: {node_id}) not found in execution order"));

        let fn_name = format_ident!("get_{}_output", name);
        let payload_type = all_msgs_types_in_culist_order[*output_position].clone();
        let index = syn::Index::from(*output_position);
        quote! {
            #[allow(dead_code)]
            pub fn #fn_name(&self) -> &CuMsg<#payload_type> {
                &self.0.#index
            }
        }
    });

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
                &[#(#task_name_literals),*]
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

        impl CuListZeroedInit for CuStampedDataSet {
            fn init_zeroed(&mut self) {
                #(self.0.#task_indices.metadata.status_txt = CuCompactString::default();)*
            }
        }
    }
}

fn gen_sim_support(
    runtime_plan: &CuExecutionLoop,
    exec_entities: &[ExecutionEntity],
) -> proc_macro2::TokenStream {
    #[cfg(feature = "macro_debug")]
    eprintln!("[Sim: Build SimEnum]");
    let plan_enum: Vec<proc_macro2::TokenStream> = runtime_plan
        .steps
        .iter()
        .filter_map(|unit| match unit {
            CuExecutionUnit::Step(step) => {
                if !matches!(
                    exec_entities[step.node_id as usize].kind,
                    ExecutionEntityKind::Task { .. }
                ) {
                    return None;
                }
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

                Some(quote! {
                    #enum_ident(CuTaskCallbackState<#inputs_type, &'a mut #output>)
                })
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

    #[cfg(feature = "std")]
    let std = true;

    #[cfg(not(feature = "std"))]
    let std = false;

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

    // Adds the generic parameter for the UnifiedLogger if this is a real application (not sim)
    // This allows to adapt either to the no-std (custom impl) and std (default file based one)
    // if !sim_mode {
    //     application_struct
    //         .generics
    //         .params
    //         .push(syn::parse_quote!(L: UnifiedLogWrite + 'static));
    // }

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
            copper_runtime: cu29::curuntime::CuRuntime<CuSimTasks, CuBridges, CuStampedDataSet, #monitor_type, #DEFAULT_CLNB>
        }
    } else {
        parse_quote! {
            copper_runtime: cu29::curuntime::CuRuntime<CuTasks, CuBridges, CuStampedDataSet, #monitor_type, #DEFAULT_CLNB>
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

        let culist_channel_usage = collect_bridge_channel_usage(graph);
        let mut culist_bridge_specs =
            build_bridge_specs(&copper_config, graph, &culist_channel_usage);
        let (culist_plan, culist_exec_entities, culist_plan_to_original) =
            match build_execution_plan(graph, &task_specs, &mut culist_bridge_specs) {
                Ok(plan) => plan,
                Err(e) => return return_error(format!("Could not compute copperlist plan: {e}")),
            };
        let task_member_names = collect_task_member_names(graph);
        let (culist_call_order, node_output_positions) = collect_culist_metadata(
            &culist_plan,
            &culist_exec_entities,
            &mut culist_bridge_specs,
            &culist_plan_to_original,
        );

        #[cfg(feature = "macro_debug")]
        {
            eprintln!("[runtime plan for mission {mission}]");
            eprintln!("{culist_plan:?}");
        }

        let culist_support: proc_macro2::TokenStream = gen_culist_support(
            &culist_plan,
            &culist_call_order,
            &node_output_positions,
            &task_member_names,
        );

        let ids = build_monitored_ids(&task_specs.ids, &mut culist_bridge_specs);

        let bridge_types: Vec<Type> = culist_bridge_specs
            .iter()
            .map(|spec| spec.type_path.clone())
            .collect();
        let bridges_type_tokens: proc_macro2::TokenStream = if bridge_types.is_empty() {
            quote! { () }
        } else {
            let tuple: TypeTuple = parse_quote! { (#(#bridge_types),*,) };
            quote! { #tuple }
        };

        let bridge_binding_idents: Vec<Ident> = culist_bridge_specs
            .iter()
            .enumerate()
            .map(|(idx, _)| format_ident!("bridge_{idx}"))
            .collect();

        let bridge_init_statements: Vec<proc_macro2::TokenStream> = culist_bridge_specs
            .iter()
            .enumerate()
            .map(|(idx, spec)| {
                let binding_ident = &bridge_binding_idents[idx];
                let bridge_type = &spec.type_path;
                let bridge_name = spec.id.clone();
                let config_index = syn::Index::from(spec.config_index);
                let tx_configs: Vec<proc_macro2::TokenStream> = spec
                    .tx_channels
                    .iter()
                    .map(|channel| {
                        let const_ident = &channel.const_ident;
                        let channel_name = channel.id.clone();
                        let channel_config_index = syn::Index::from(channel.config_index);
                        quote! {
                            {
                                let (channel_route, channel_config) = match &bridge_cfg.channels[#channel_config_index] {
                                    cu29::config::BridgeChannelConfigRepresentation::Tx { route, config, .. } => {
                                        (route.clone(), config.clone())
                                    }
                                    _ => panic!(
                                        "Bridge '{}' channel '{}' expected to be Tx",
                                        #bridge_name,
                                        #channel_name
                                    ),
                                };
                                cu29::cubridge::BridgeChannelConfig::from_static(
                                    &<#bridge_type as cu29::cubridge::CuBridge>::Tx::#const_ident,
                                    channel_route,
                                    channel_config,
                                )
                            }
                        }
                    })
                    .collect();
                let rx_configs: Vec<proc_macro2::TokenStream> = spec
                    .rx_channels
                    .iter()
                    .map(|channel| {
                        let const_ident = &channel.const_ident;
                        let channel_name = channel.id.clone();
                        let channel_config_index = syn::Index::from(channel.config_index);
                        quote! {
                            {
                                let (channel_route, channel_config) = match &bridge_cfg.channels[#channel_config_index] {
                                    cu29::config::BridgeChannelConfigRepresentation::Rx { route, config, .. } => {
                                        (route.clone(), config.clone())
                                    }
                                    _ => panic!(
                                        "Bridge '{}' channel '{}' expected to be Rx",
                                        #bridge_name,
                                        #channel_name
                                    ),
                                };
                                cu29::cubridge::BridgeChannelConfig::from_static(
                                    &<#bridge_type as cu29::cubridge::CuBridge>::Rx::#const_ident,
                                    channel_route,
                                    channel_config,
                                )
                            }
                        }
                    })
                    .collect();
                quote! {
                    let #binding_ident = {
                        let bridge_cfg = config
                            .bridges
                            .get(#config_index)
                            .unwrap_or_else(|| panic!("Bridge '{}' missing from configuration", #bridge_name));
                        let tx_channels: &[cu29::cubridge::BridgeChannelConfig<
                            <<#bridge_type as cu29::cubridge::CuBridge>::Tx as cu29::cubridge::BridgeChannelSet>::Id,
                        >] = &[#(#tx_configs),*];
                        let rx_channels: &[cu29::cubridge::BridgeChannelConfig<
                            <<#bridge_type as cu29::cubridge::CuBridge>::Rx as cu29::cubridge::BridgeChannelSet>::Id,
                        >] = &[#(#rx_configs),*];
                        <#bridge_type as cu29::cubridge::CuBridge>::new(
                            bridge_cfg.config.as_ref(),
                            tx_channels,
                            rx_channels,
                        )?
                    };
                }
            })
            .collect();

        let bridges_instanciator = if culist_bridge_specs.is_empty() {
            quote! {
                pub fn bridges_instanciator(_config: &CuConfig) -> CuResult<CuBridges> {
                    Ok(())
                }
            }
        } else {
            let bridge_bindings = bridge_binding_idents.clone();
            quote! {
                pub fn bridges_instanciator(config: &CuConfig) -> CuResult<CuBridges> {
                    #(#bridge_init_statements)*
                    Ok((#(#bridge_bindings),*,))
                }
            }
        };

        let all_sim_tasks_types: Vec<Type> = task_specs.ids
            .iter()
            .zip(&task_specs.cutypes)
            .zip(&task_specs.sim_task_types)
            .zip(&task_specs.background_flags)
            .zip(&task_specs.run_in_sim_flags)
            .map(|((((task_id, task_type), sim_type), background), run_in_sim)| {
                match task_type {
                    CuTaskType::Source => {
                        if *background {
                            panic!("CuSrcTask {task_id} cannot be a background task, it should be a regular task.");
                        }
                        if *run_in_sim {
                            sim_type.clone()
                        } else {
                            let msg_type = graph
                                .get_node_output_msg_type(task_id.as_str())
                                .unwrap_or_else(|| panic!("CuSrcTask {task_id} should have an outgoing connection with a valid output msg type"));
                            let sim_task_name = format!("CuSimSrcTask<{msg_type}>");
                            parse_str(sim_task_name.as_str()).unwrap_or_else(|_| panic!("Could not build the placeholder for simulation: {sim_task_name}"))
                        }
                    }
                    CuTaskType::Regular => {
                        // TODO: wrap that correctly in a background task if background is true.
                        // run_in_sim has no effect for normal tasks, they are always run in sim as is.
                        sim_type.clone()
                    },
                    CuTaskType::Sink => {
                        if *background {
                            panic!("CuSinkTask {task_id} cannot be a background task, it should be a regular task.");
                        }

                        if *run_in_sim {
                            // Use the real task in sim if asked to.
                            sim_type.clone()
                        }
                        else {
                            // Use the placeholder sim task.
                            let msg_type = graph
                                .get_node_input_msg_type(task_id.as_str())
                                .unwrap_or_else(|| panic!("CuSinkTask {task_id} should have an incoming connection with a valid input msg type"));
                            let sim_task_name = format!("CuSimSinkTask<{msg_type}>");
                            parse_str(sim_task_name.as_str()).unwrap_or_else(|_| panic!("Could not build the placeholder for simulation: {sim_task_name}"))
                        }
                    }
                }
    })
    .collect();

        #[cfg(feature = "macro_debug")]
        eprintln!("[build task tuples]");

        let task_types = &task_specs.task_types;
        // Build the tuple of all those types
        // note the extraneous, at the end is to make the tuple work even if this is only one element
        let task_types_tuple: TypeTuple = if task_types.is_empty() {
            parse_quote! { () }
        } else {
            parse_quote! { (#(#task_types),*,) }
        };

        let task_types_tuple_sim: TypeTuple = if all_sim_tasks_types.is_empty() {
            parse_quote! { () }
        } else {
            parse_quote! { (#(#all_sim_tasks_types),*,) }
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
                    #task_type::new(all_instances_configs[#index], threadpool.clone()).map_err(|e| e.add_cause(#additional_error_info))?
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
            task_start_calls,
            task_stop_calls,
            task_preprocess_calls,
            task_postprocess_calls,
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
                                let ovr = sim_callback(SimStep::#enum_name(CuTaskCallbackState::Start));

                                let doit = if let SimOverride::Errored(reason) = ovr  {
                                    let error: CuError = reason.into();
                                    #monitoring_action
                                    false
                               }
                               else {
                                    ovr == SimOverride::ExecuteByRuntime
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
                                let ovr = sim_callback(SimStep::#enum_name(CuTaskCallbackState::Stop));

                                let doit = if let SimOverride::Errored(reason) = ovr  {
                                    let error: CuError = reason.into();
                                    #monitoring_action
                                    false
                               }
                               else {
                                    ovr == SimOverride::ExecuteByRuntime
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
                                let ovr = sim_callback(SimStep::#enum_name(CuTaskCallbackState::Preprocess));

                                let doit = if let SimOverride::Errored(reason) = ovr  {
                                    let error: CuError = reason.into();
                                    #monitoring_action
                                    false
                                } else {
                                    ovr == SimOverride::ExecuteByRuntime
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
                                let ovr = sim_callback(SimStep::#enum_name(CuTaskCallbackState::Postprocess));

                                let doit = if let SimOverride::Errored(reason) = ovr  {
                                    let error: CuError = reason.into();
                                    #monitoring_action
                                    false
                                } else {
                                    ovr == SimOverride::ExecuteByRuntime
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

        let bridge_start_calls: Vec<proc_macro2::TokenStream> = culist_bridge_specs
            .iter()
            .map(|spec| {
                let bridge_index = int2sliceindex(spec.tuple_index as u32);
                let monitor_index = syn::Index::from(
                    spec.monitor_index
                        .expect("Bridge missing monitor index for start"),
                );
                quote! {
                    {
                        let bridge = &mut self.copper_runtime.bridges.#bridge_index;
                        if let Err(error) = bridge.start(&self.copper_runtime.clock) {
                            let decision = self.copper_runtime.monitor.process_error(#monitor_index, CuTaskState::Start, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Start: ABORT decision from monitoring. Task '{}' errored out during start. Aborting all the other starts.", #mission_mod::TASKS_IDS[#monitor_index]);
                                    return Ok(());
                                }
                                Decision::Ignore => {
                                    debug!("Start: IGNORE decision from monitoring. Task '{}' errored out during start. The runtime will continue.", #mission_mod::TASKS_IDS[#monitor_index]);
                                }
                                Decision::Shutdown => {
                                    debug!("Start: SHUTDOWN decision from monitoring. Task '{}' errored out during start. The runtime cannot continue.", #mission_mod::TASKS_IDS[#monitor_index]);
                                    return Err(CuError::new_with_cause("Task errored out during start.", error));
                                }
                            }
                        }
                    }
                }
            })
            .collect();

        let bridge_stop_calls: Vec<proc_macro2::TokenStream> = culist_bridge_specs
            .iter()
            .map(|spec| {
                let bridge_index = int2sliceindex(spec.tuple_index as u32);
                let monitor_index = syn::Index::from(
                    spec.monitor_index
                        .expect("Bridge missing monitor index for stop"),
                );
                quote! {
                    {
                        let bridge = &mut self.copper_runtime.bridges.#bridge_index;
                        if let Err(error) = bridge.stop(&self.copper_runtime.clock) {
                            let decision = self.copper_runtime.monitor.process_error(#monitor_index, CuTaskState::Stop, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Stop: ABORT decision from monitoring. Task '{}' errored out during stop. Aborting all the other stops.", #mission_mod::TASKS_IDS[#monitor_index]);
                                    return Ok(());
                                }
                                Decision::Ignore => {
                                    debug!("Stop: IGNORE decision from monitoring. Task '{}' errored out during stop. The runtime will continue.", #mission_mod::TASKS_IDS[#monitor_index]);
                                }
                                Decision::Shutdown => {
                                    debug!("Stop: SHUTDOWN decision from monitoring. Task '{}' errored out during stop. The runtime cannot continue.", #mission_mod::TASKS_IDS[#monitor_index]);
                                    return Err(CuError::new_with_cause("Task errored out during stop.", error));
                                }
                            }
                        }
                    }
                }
            })
            .collect();

        let bridge_preprocess_calls: Vec<proc_macro2::TokenStream> = culist_bridge_specs
            .iter()
            .map(|spec| {
                let bridge_index = int2sliceindex(spec.tuple_index as u32);
                let monitor_index = syn::Index::from(
                    spec.monitor_index
                        .expect("Bridge missing monitor index for preprocess"),
                );
                quote! {
                    {
                        let bridge = &mut bridges.#bridge_index;
                        if let Err(error) = bridge.preprocess(clock) {
                            let decision = monitor.process_error(#monitor_index, CuTaskState::Preprocess, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Preprocess: ABORT decision from monitoring. Task '{}' errored out during preprocess. Aborting all the other starts.", #mission_mod::TASKS_IDS[#monitor_index]);
                                    return Ok(());
                                }
                                Decision::Ignore => {
                                    debug!("Preprocess: IGNORE decision from monitoring. Task '{}' errored out during preprocess. The runtime will continue.", #mission_mod::TASKS_IDS[#monitor_index]);
                                }
                                Decision::Shutdown => {
                                    debug!("Preprocess: SHUTDOWN decision from monitoring. Task '{}' errored out during preprocess. The runtime cannot continue.", #mission_mod::TASKS_IDS[#monitor_index]);
                                    return Err(CuError::new_with_cause("Task errored out during preprocess.", error));
                                }
                            }
                        }
                    }
                }
            })
            .collect();

        let bridge_postprocess_calls: Vec<proc_macro2::TokenStream> = culist_bridge_specs
            .iter()
            .map(|spec| {
                let bridge_index = int2sliceindex(spec.tuple_index as u32);
                let monitor_index = syn::Index::from(
                    spec.monitor_index
                        .expect("Bridge missing monitor index for postprocess"),
                );
                quote! {
                    {
                        let bridge = &mut bridges.#bridge_index;
                        if let Err(error) = bridge.postprocess(clock) {
                            let decision = monitor.process_error(#monitor_index, CuTaskState::Postprocess, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Postprocess: ABORT decision from monitoring. Task '{}' errored out during postprocess. Aborting all the other starts.", #mission_mod::TASKS_IDS[#monitor_index]);
                                    return Ok(());
                                }
                                Decision::Ignore => {
                                    debug!("Postprocess: IGNORE decision from monitoring. Task '{}' errored out during postprocess. The runtime will continue.", #mission_mod::TASKS_IDS[#monitor_index]);
                                }
                                Decision::Shutdown => {
                                    debug!("Postprocess: SHUTDOWN decision from monitoring. Task '{}' errored out during postprocess. The runtime cannot continue.", #mission_mod::TASKS_IDS[#monitor_index]);
                                    return Err(CuError::new_with_cause("Task errored out during postprocess.", error));
                                }
                            }
                        }
                    }
                }
            })
            .collect();

        let mut start_calls = bridge_start_calls;
        start_calls.extend(task_start_calls);
        let mut stop_calls = task_stop_calls;
        stop_calls.extend(bridge_stop_calls);
        let mut preprocess_calls = bridge_preprocess_calls;
        preprocess_calls.extend(task_preprocess_calls);
        let mut postprocess_calls = task_postprocess_calls;
        postprocess_calls.extend(bridge_postprocess_calls);

        let runtime_plan_code_and_logging: Vec<(
            proc_macro2::TokenStream,
            proc_macro2::TokenStream,
        )> = culist_plan
            .steps
            .iter()
            .map(|unit| match unit {
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

                    match &culist_exec_entities[step.node_id as usize].kind {
                        ExecutionEntityKind::Task { task_index } => generate_task_execution_tokens(
                            step,
                            *task_index,
                            &task_specs,
                            sim_mode,
                            &mission_mod,
                        ),
                        ExecutionEntityKind::BridgeRx {
                            bridge_index,
                            channel_index,
                        } => {
                            let spec = &culist_bridge_specs[*bridge_index];
                            generate_bridge_rx_execution_tokens(spec, *channel_index, &mission_mod)
                        }
                        ExecutionEntityKind::BridgeTx {
                            bridge_index,
                            channel_index,
                        } => {
                            let spec = &culist_bridge_specs[*bridge_index];
                            generate_bridge_tx_execution_tokens(
                                step,
                                spec,
                                *channel_index,
                                &mission_mod,
                            )
                        }
                    }
                }
                CuExecutionUnit::Loop(_) => {
                    panic!("Execution loops are not supported in runtime generation");
                }
            })
            .collect();

        let sim_support = if sim_mode {
            Some(gen_sim_support(&culist_plan, &culist_exec_entities))
        } else {
            None
        };

        let (new, run_one_iteration, start_all_tasks, stop_all_tasks, run) = if sim_mode {
            (
                quote! {
                    fn new(clock:RobotClock, unified_logger: Arc<Mutex<L>>, config_override: Option<CuConfig>, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<Self>
                },
                quote! {
                    fn run_one_iteration(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()>
                },
                quote! {
                    fn start_all_tasks(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()>
                },
                quote! {
                    fn stop_all_tasks(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()>
                },
                quote! {
                    fn run(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()>
                },
            )
        } else {
            (
                if std {
                    quote! {
                        fn new(clock:RobotClock, unified_logger: Arc<Mutex<L>>, config_override: Option<CuConfig>) -> CuResult<Self>
                    }
                } else {
                    quote! {
                        // no config override is possible in no-std
                        fn new(clock:RobotClock, unified_logger: Arc<Mutex<L>>) -> CuResult<Self>
                    }
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

        let app_trait = if sim_mode {
            quote!(CuSimApplication)
        } else {
            quote!(CuApplication)
        };

        let sim_callback_on_new_calls = task_specs.ids.iter().enumerate().map(|(i, id)| {
            let enum_name = config_id_to_enum(id);
            let enum_ident = Ident::new(&enum_name, Span::call_site());
            quote! {
                // the answer is ignored, we have to instantiate the tasks anyway.
                sim_callback(SimStep::#enum_ident(CuTaskCallbackState::New(all_instances_configs[#i].cloned())));
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

        let config_load_stmt = if std {
            quote! {
                let config = if let Some(overridden_config) = config_override {
                    debug!("CuConfig: Overridden programmatically: {}", overridden_config.serialize_ron());
                    overridden_config
                } else if ::std::path::Path::new(config_filename).exists() {
                    debug!("CuConfig: Reading configuration from file: {}", config_filename);
                    cu29::config::read_configuration(config_filename)?
                } else {
                    let original_config = <Self as #app_trait<S, L>>::get_original_config();
                    debug!("CuConfig: Using the original configuration the project was compiled with: {}", &original_config);
                    cu29::config::read_configuration_str(original_config, None)?
                };
            }
        } else {
            quote! {
                // Only the original config is available in no-std
                let original_config = <Self as #app_trait<S, L>>::get_original_config();
                debug!("CuConfig: Using the original configuration the project was compiled with: {}", &original_config);
                let config = cu29::config::read_configuration_str(original_config, None)?;
            }
        };

        let kill_handler = if std {
            Some(quote! {
                ctrlc::set_handler(move || {
                    STOP_FLAG.store(true, Ordering::SeqCst);
                }).expect("Error setting Ctrl-C handler");
            })
        } else {
            None
        };

        let run_loop = if std {
            quote! {
                loop  {
                    let iter_start = self.copper_runtime.clock.now();
                    let result = <Self as #app_trait<S, L>>::run_one_iteration(self, #sim_callback_arg);

                    if let Some(rate) = self.copper_runtime.runtime_config.rate_target_hz {
                        let period: CuDuration = (1_000_000_000u64 / rate).into();
                        let elapsed = self.copper_runtime.clock.now() - iter_start;
                        if elapsed < period {
                            std::thread::sleep(std::time::Duration::from_nanos(period.as_nanos() - elapsed.as_nanos()));
                        }
                    }

                    if STOP_FLAG.load(Ordering::SeqCst) || result.is_err() {
                        break result;
                    }
                }
            }
        } else {
            quote! {
                loop  {
                    let iter_start = self.copper_runtime.clock.now();
                    let result = <Self as #app_trait<S, L>>::run_one_iteration(self, #sim_callback_arg);
                    if let Some(rate) = self.copper_runtime.runtime_config.rate_target_hz {
                        let period: CuDuration = (1_000_000_000u64 / rate).into();
                        let elapsed = self.copper_runtime.clock.now() - iter_start;
                        if elapsed < period {
                            busy_wait_for(period - elapsed);
                        }
                    }

                    if STOP_FLAG.load(Ordering::SeqCst) || result.is_err() {
                        break result;
                    }
                }
            }
        };

        #[cfg(feature = "macro_debug")]
        eprintln!("[build the run methods]");
        let run_methods = quote! {

            #run_one_iteration {

                // Pre-explode the runtime to avoid complexity with partial borrowing in the generated code.
                let runtime = &mut self.copper_runtime;
                let clock = &runtime.clock;
                let monitor = &mut runtime.monitor;
                let tasks = &mut runtime.tasks;
                let bridges = &mut runtime.bridges;
                let cl_manager = &mut runtime.copperlists_manager;
                let kf_manager = &mut runtime.keyframes_manager;

                // Preprocess calls can happen at any time, just packed them up front.
                #(#preprocess_calls)*

                let culist = cl_manager.inner.create().expect("Ran out of space for copper lists"); // FIXME: error handling
                let clid = culist.id;
                kf_manager.reset(clid, clock); // beginning of processing, we empty the serialized frozen states of the tasks.
                culist.change_state(cu29::copperlist::CopperListState::Processing);
                culist.msgs.init_zeroed();
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

                #kill_handler

                <Self as #app_trait<S, L>>::start_all_tasks(self, #sim_callback_arg)?;
                let result = #run_loop;

                if result.is_err() {
                    error!("A task errored out: {}", &result);
                }
                <Self as #app_trait<S, L>>::stop_all_tasks(self, #sim_callback_arg)?;
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
            quote!(impl<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static> CuSimApplication<S, L> for #application_name)
        } else {
            quote!(impl<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static> CuApplication<S, L> for #application_name)
        };

        let simstep_type_decl = if sim_mode {
            quote!(
                type Step<'z> = SimStep<'z>;
            )
        } else {
            quote!()
        };

        #[cfg(feature = "std")]
        #[cfg(feature = "macro_debug")]
        eprintln!("[build result]");
        let application_impl = quote! {
            #app_impl_decl {
                #simstep_type_decl

                #new {
                    let config_filename = #config_file;

                    #config_load_stmt

                    // For simple cases we can say the section is just a bunch of Copper Lists.
                    // But we can now have allocations outside of it so we can override it from the config.
                    let mut default_section_size = size_of::<super::#mission_mod::CuList>() * 64;
                    // Check if there is a logging configuration with section_size_mib
                    if let Some(section_size_mib) = config.logging.as_ref().and_then(|l| l.section_size_mib) {
                        // Convert MiB to bytes
                        default_section_size = section_size_mib as usize * 1024usize * 1024usize;
                    }
                    let copperlist_stream = stream_write::<#mission_mod::CuList, S>(
                        unified_logger.clone(),
                        UnifiedLogType::CopperList,
                        default_section_size,
                        // the 2 sizes are not directly related as we encode the CuList but we can
                        // assume the encoded size is close or lower than the non encoded one
                        // This is to be sure we have the size of at least a Culist and some.
                    )?;

                    let keyframes_stream = stream_write::<KeyFrame, S>(
                        unified_logger.clone(),
                        UnifiedLogType::FrozenTasks,
                        1024 * 1024 * 10, // 10 MiB
                    )?;


                    let application = Ok(#application_name {
                        copper_runtime: CuRuntime::<#mission_mod::#tasks_type, #mission_mod::CuBridges, #mission_mod::CuStampedDataSet, #monitor_type, #DEFAULT_CLNB>::new(
                            clock,
                            &config,
                            Some(#mission),
                            #mission_mod::#tasks_instanciator_fn,
                            #mission_mod::monitor_instanciator,
                            #mission_mod::bridges_instanciator,
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
                        F: FnMut(SimStep) -> SimOverride,
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

        // backward compat on std non-parameterized impl.
        let std_application_impl = if sim_mode {
            // sim mode
            Some(quote! {
                        impl #application_name {
                            pub fn start_all_tasks(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::start_all_tasks(self, sim_callback)
                            }
                            pub fn run_one_iteration(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::run_one_iteration(self, sim_callback)
                            }
                            pub fn run(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::run(self, sim_callback)
                            }
                            pub fn stop_all_tasks(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::stop_all_tasks(self, sim_callback)
                            }
                        }
            })
        } else if std {
            // std and normal mode, we use the memory mapped starage for those
            Some(quote! {
                        impl #application_name {
                            pub fn start_all_tasks(&mut self) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::start_all_tasks(self)
                            }
                            pub fn run_one_iteration(&mut self) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::run_one_iteration(self)
                            }
                            pub fn run(&mut self) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::run(self)
                            }
                            pub fn stop_all_tasks(&mut self) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::stop_all_tasks(self)
                            }
                        }
            })
        } else {
            None // if no-std, let the user figure our the correct logger type they need to provide anyway.
        };

        let application_builder = if std {
            Some(quote! {
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
            })
        } else {
            // in no-std the user has to construct that manually anyway so don't make any helper here.
            None
        };

        let sim_imports = if sim_mode {
            Some(quote! {
                use cu29::simulation::SimOverride;
                use cu29::simulation::CuTaskCallbackState;
                use cu29::simulation::CuSimSrcTask;
                use cu29::simulation::CuSimSinkTask;
                use cu29::prelude::app::CuSimApplication;
            })
        } else {
            None
        };

        let sim_tasks = if sim_mode {
            Some(quote! {
                // This is the variation with stubs for the sources and sinks in simulation mode.
                // Not used if the used doesn't generate Sim.
                pub type CuSimTasks = #task_types_tuple_sim;
            })
        } else {
            None
        };

        let sim_inst_body = if task_sim_instances_init_code.is_empty() {
            quote! { Ok(()) }
        } else {
            quote! { Ok(( #(#task_sim_instances_init_code),*, )) }
        };

        let sim_tasks_instanciator = if sim_mode {
            Some(quote! {
                pub fn tasks_instanciator_sim(all_instances_configs: Vec<Option<&ComponentConfig>>, _threadpool: Arc<ThreadPool>) -> CuResult<CuSimTasks> {
                    #sim_inst_body
            }})
        } else {
            None
        };

        let tasks_inst_body_std = if task_instances_init_code.is_empty() {
            quote! {
                let _ = threadpool;
                Ok(())
            }
        } else {
            quote! { Ok(( #(#task_instances_init_code),*, )) }
        };

        let tasks_inst_body_nostd = if task_instances_init_code.is_empty() {
            quote! { Ok(()) }
        } else {
            quote! { Ok(( #(#task_instances_init_code),*, )) }
        };

        let tasks_instanciator = if std {
            quote! {
                pub fn tasks_instanciator<'c>(all_instances_configs: Vec<Option<&'c ComponentConfig>>, threadpool: Arc<ThreadPool>) -> CuResult<CuTasks> {
                    #tasks_inst_body_std
                }
            }
        } else {
            // no thread pool in the no-std impl
            quote! {
                pub fn tasks_instanciator<'c>(all_instances_configs: Vec<Option<&'c ComponentConfig>>) -> CuResult<CuTasks> {
                    #tasks_inst_body_nostd
                }
            }
        };

        let imports = if std {
            quote! {
                use cu29::rayon::ThreadPool;
                use cu29::cuasynctask::CuAsyncTask;
                use cu29::curuntime::CopperContext;
                use cu29::prelude::UnifiedLoggerWrite;
                use cu29::prelude::memmap::MmapSectionStorage;
                use std::fmt::{Debug, Formatter};
                use std::fmt::Result as FmtResult;
                use std::mem::size_of;
                use std::sync::Arc;
                use std::sync::atomic::{AtomicBool, Ordering};
                use std::sync::Mutex;
            }
        } else {
            quote! {
                use alloc::sync::Arc;
                use alloc::string::String;
                use alloc::string::ToString;
                use core::sync::atomic::{AtomicBool, Ordering};
                use core::fmt::{Debug, Formatter};
                use core::fmt::Result as FmtResult;
                use core::mem::size_of;
                use spin::Mutex;
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
                use cu29::bincode::de::DecoderImpl;
                use cu29::bincode::error::DecodeError;
                use cu29::clock::RobotClock;
                use cu29::config::CuConfig;
                use cu29::config::ComponentConfig;
                use cu29::curuntime::CuRuntime;
                use cu29::curuntime::KeyFrame;
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
                use cu29::prelude::UnifiedLogType;
                use cu29::prelude::UnifiedLogWrite;

                #imports

                #sim_imports

                // Not used if a monitor is present
                #[allow(unused_imports)]
                use cu29::monitoring::NoMonitor;

                // This is the heart of everything.
                // CuTasks is the list of all the tasks types.
                // CuList is a CopperList with the list of all the messages types as msgs.
                pub type CuTasks = #task_types_tuple;
                pub type CuBridges = #bridges_type_tokens;

                #sim_tasks
                #sim_support
                #sim_tasks_instanciator

                pub const TASKS_IDS: &'static [&'static str] = &[#( #ids ),*];

                #culist_support
                #tasks_instanciator
                #bridges_instanciator

                pub fn monitor_instanciator(config: &CuConfig) -> #monitor_type {
                    #monitor_type::new(config, #mission_mod::TASKS_IDS).expect("Failed to create the given monitor.")
                }

                // The application for this mission
                pub #application_struct

                #application_impl

                #std_application_impl

                #application_builder
            }

        };
        all_missions_tokens.push(mission_mod_tokens);
    }

    let default_application_tokens = if all_missions.contains_key("default") {
        let default_builder = if std {
            Some(quote! {
                // you can bypass the builder and not use it
                #[allow(unused_imports)]
                use default::#builder_name;
            })
        } else {
            None
        };
        quote! {
            #default_builder

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
    pub run_in_sim_flags: Vec<bool>,
    #[allow(dead_code)]
    pub output_types: Vec<Option<Type>>,
    pub node_id_to_task_index: Vec<Option<usize>>,
}

impl CuTaskSpecSet {
    pub fn from_graph(graph: &CuGraph) -> Self {
        let all_id_nodes: Vec<(NodeId, &Node)> = graph
            .get_all_nodes()
            .into_iter()
            .filter(|(_, node)| node.get_flavor() == Flavor::Task)
            .collect();

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
                        parse_quote!(CuAsyncTask<#name_type, #output_type>)
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
                        parse_quote!(CuAsyncTask::<#name_type, #output_type>)
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

        let run_in_sim_flags = all_id_nodes
            .iter()
            .map(|(_, node)| node.is_run_in_sim())
            .collect();

        let mut node_id_to_task_index = vec![None; graph.node_count()];
        for (index, (node_id, _)) in all_id_nodes.iter().enumerate() {
            node_id_to_task_index[*node_id as usize] = Some(index);
        }

        Self {
            ids,
            cutypes,
            background_flags,
            logging_enabled,
            type_names,
            task_types,
            instantiation_types,
            sim_task_types,
            run_in_sim_flags,
            output_types,
            node_id_to_task_index,
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
        impl Debug for CuStampedDataSet {
            fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
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

fn collect_bridge_channel_usage(graph: &CuGraph) -> HashMap<BridgeChannelKey, String> {
    let mut usage = HashMap::new();
    for edge_idx in graph.0.edge_indices() {
        let cnx = graph
            .0
            .edge_weight(edge_idx)
            .expect("Edge should exist while collecting bridge usage")
            .clone();
        if let Some(channel) = &cnx.src_channel {
            let key = BridgeChannelKey {
                bridge_id: cnx.src.clone(),
                channel_id: channel.clone(),
                direction: BridgeChannelDirection::Rx,
            };
            usage
                .entry(key)
                .and_modify(|msg| {
                    if msg != &cnx.msg {
                        panic!(
                            "Bridge '{}' channel '{}' is used with incompatible message types: {} vs {}",
                            cnx.src, channel, msg, cnx.msg
                        );
                    }
                })
                .or_insert(cnx.msg.clone());
        }
        if let Some(channel) = &cnx.dst_channel {
            let key = BridgeChannelKey {
                bridge_id: cnx.dst.clone(),
                channel_id: channel.clone(),
                direction: BridgeChannelDirection::Tx,
            };
            usage
                .entry(key)
                .and_modify(|msg| {
                    if msg != &cnx.msg {
                        panic!(
                            "Bridge '{}' channel '{}' is used with incompatible message types: {} vs {}",
                            cnx.dst, channel, msg, cnx.msg
                        );
                    }
                })
                .or_insert(cnx.msg.clone());
        }
    }
    usage
}

fn build_bridge_specs(
    config: &CuConfig,
    graph: &CuGraph,
    channel_usage: &HashMap<BridgeChannelKey, String>,
) -> Vec<BridgeSpec> {
    let mut specs = Vec::new();
    for (bridge_index, bridge_cfg) in config.bridges.iter().enumerate() {
        if graph.get_node_id_by_name(bridge_cfg.id.as_str()).is_none() {
            continue;
        }

        let type_path = parse_str::<Type>(bridge_cfg.type_.as_str()).unwrap_or_else(|err| {
            panic!(
                "Could not parse bridge type '{}' for '{}': {err}",
                bridge_cfg.type_, bridge_cfg.id
            )
        });

        let mut rx_channels = Vec::new();
        let mut tx_channels = Vec::new();

        for (channel_index, channel) in bridge_cfg.channels.iter().enumerate() {
            match channel {
                BridgeChannelConfigRepresentation::Rx { id, .. } => {
                    let key = BridgeChannelKey {
                        bridge_id: bridge_cfg.id.clone(),
                        channel_id: id.clone(),
                        direction: BridgeChannelDirection::Rx,
                    };
                    if let Some(msg_type) = channel_usage.get(&key) {
                        let msg_type = parse_str::<Type>(msg_type).unwrap_or_else(|err| {
                            panic!(
                                "Could not parse message type '{msg_type}' for bridge '{}' channel '{}': {err}",
                                bridge_cfg.id, id
                            )
                        });
                        let const_ident =
                            Ident::new(&config_id_to_bridge_const(id.as_str()), Span::call_site());
                        rx_channels.push(BridgeChannelSpec {
                            id: id.clone(),
                            const_ident,
                            msg_type,
                            config_index: channel_index,
                            plan_node_id: None,
                            culist_index: None,
                            monitor_index: None,
                        });
                    }
                }
                BridgeChannelConfigRepresentation::Tx { id, .. } => {
                    let key = BridgeChannelKey {
                        bridge_id: bridge_cfg.id.clone(),
                        channel_id: id.clone(),
                        direction: BridgeChannelDirection::Tx,
                    };
                    if let Some(msg_type) = channel_usage.get(&key) {
                        let msg_type = parse_str::<Type>(msg_type).unwrap_or_else(|err| {
                            panic!(
                                "Could not parse message type '{msg_type}' for bridge '{}' channel '{}': {err}",
                                bridge_cfg.id, id
                            )
                        });
                        let const_ident =
                            Ident::new(&config_id_to_bridge_const(id.as_str()), Span::call_site());
                        tx_channels.push(BridgeChannelSpec {
                            id: id.clone(),
                            const_ident,
                            msg_type,
                            config_index: channel_index,
                            plan_node_id: None,
                            culist_index: None,
                            monitor_index: None,
                        });
                    }
                }
            }
        }

        if rx_channels.is_empty() && tx_channels.is_empty() {
            continue;
        }

        specs.push(BridgeSpec {
            id: bridge_cfg.id.clone(),
            type_path,
            config_index: bridge_index,
            tuple_index: 0,
            monitor_index: None,
            rx_channels,
            tx_channels,
        });
    }

    for (tuple_index, spec) in specs.iter_mut().enumerate() {
        spec.tuple_index = tuple_index;
    }

    specs
}

fn collect_task_member_names(graph: &CuGraph) -> Vec<(NodeId, String)> {
    graph
        .get_all_nodes()
        .iter()
        .filter(|(_, node)| node.get_flavor() == Flavor::Task)
        .map(|(node_id, node)| (*node_id, config_id_to_struct_member(node.get_id().as_str())))
        .collect()
}

fn build_execution_plan(
    graph: &CuGraph,
    task_specs: &CuTaskSpecSet,
    bridge_specs: &mut [BridgeSpec],
) -> CuResult<(
    CuExecutionLoop,
    Vec<ExecutionEntity>,
    HashMap<NodeId, NodeId>,
)> {
    let mut plan_graph = CuGraph::default();
    let mut exec_entities = Vec::new();
    let mut original_to_plan = HashMap::new();
    let mut plan_to_original = HashMap::new();
    let mut name_to_original = HashMap::new();
    let mut channel_nodes = HashMap::new();

    for (node_id, node) in graph.get_all_nodes() {
        name_to_original.insert(node.get_id(), node_id);
        if node.get_flavor() != Flavor::Task {
            continue;
        }
        let plan_node_id = plan_graph.add_node(node.clone())?;
        let task_index = task_specs.node_id_to_task_index[node_id as usize]
            .expect("Task missing from specifications");
        plan_to_original.insert(plan_node_id, node_id);
        original_to_plan.insert(node_id, plan_node_id);
        if plan_node_id as usize != exec_entities.len() {
            panic!("Unexpected node ordering while mirroring tasks in plan graph");
        }
        exec_entities.push(ExecutionEntity {
            kind: ExecutionEntityKind::Task { task_index },
        });
    }

    for (bridge_index, spec) in bridge_specs.iter_mut().enumerate() {
        for (channel_index, channel_spec) in spec.rx_channels.iter_mut().enumerate() {
            let mut node = Node::new(
                format!("{}::rx::{}", spec.id, channel_spec.id).as_str(),
                "__CuBridgeRxChannel",
            );
            node.set_flavor(Flavor::Bridge);
            let plan_node_id = plan_graph.add_node(node)?;
            if plan_node_id as usize != exec_entities.len() {
                panic!("Unexpected node ordering while inserting bridge rx channel");
            }
            channel_spec.plan_node_id = Some(plan_node_id);
            exec_entities.push(ExecutionEntity {
                kind: ExecutionEntityKind::BridgeRx {
                    bridge_index,
                    channel_index,
                },
            });
            channel_nodes.insert(
                BridgeChannelKey {
                    bridge_id: spec.id.clone(),
                    channel_id: channel_spec.id.clone(),
                    direction: BridgeChannelDirection::Rx,
                },
                plan_node_id,
            );
        }

        for (channel_index, channel_spec) in spec.tx_channels.iter_mut().enumerate() {
            let mut node = Node::new(
                format!("{}::tx::{}", spec.id, channel_spec.id).as_str(),
                "__CuBridgeTxChannel",
            );
            node.set_flavor(Flavor::Bridge);
            let plan_node_id = plan_graph.add_node(node)?;
            if plan_node_id as usize != exec_entities.len() {
                panic!("Unexpected node ordering while inserting bridge tx channel");
            }
            channel_spec.plan_node_id = Some(plan_node_id);
            exec_entities.push(ExecutionEntity {
                kind: ExecutionEntityKind::BridgeTx {
                    bridge_index,
                    channel_index,
                },
            });
            channel_nodes.insert(
                BridgeChannelKey {
                    bridge_id: spec.id.clone(),
                    channel_id: channel_spec.id.clone(),
                    direction: BridgeChannelDirection::Tx,
                },
                plan_node_id,
            );
        }
    }

    for edge_idx in graph.0.edge_indices() {
        let cnx = graph
            .0
            .edge_weight(edge_idx)
            .expect("Edge should exist while building plan")
            .clone();

        let src_plan = if let Some(channel) = &cnx.src_channel {
            let key = BridgeChannelKey {
                bridge_id: cnx.src.clone(),
                channel_id: channel.clone(),
                direction: BridgeChannelDirection::Rx,
            };
            *channel_nodes
                .get(&key)
                .unwrap_or_else(|| panic!("Bridge source {:?} missing from plan graph", key))
        } else {
            let node_id = name_to_original
                .get(&cnx.src)
                .copied()
                .unwrap_or_else(|| panic!("Unknown source node '{}'", cnx.src));
            *original_to_plan
                .get(&node_id)
                .unwrap_or_else(|| panic!("Source node '{}' missing from plan", cnx.src))
        };

        let dst_plan = if let Some(channel) = &cnx.dst_channel {
            let key = BridgeChannelKey {
                bridge_id: cnx.dst.clone(),
                channel_id: channel.clone(),
                direction: BridgeChannelDirection::Tx,
            };
            *channel_nodes
                .get(&key)
                .unwrap_or_else(|| panic!("Bridge destination {:?} missing from plan graph", key))
        } else {
            let node_id = name_to_original
                .get(&cnx.dst)
                .copied()
                .unwrap_or_else(|| panic!("Unknown destination node '{}'", cnx.dst));
            *original_to_plan
                .get(&node_id)
                .unwrap_or_else(|| panic!("Destination node '{}' missing from plan", cnx.dst))
        };

        plan_graph
            .connect_ext(
                src_plan,
                dst_plan,
                &cnx.msg,
                cnx.missions.clone(),
                None,
                None,
            )
            .map_err(|e| CuError::from(e.to_string()))?;
    }

    let runtime_plan = compute_runtime_plan(&plan_graph)?;
    Ok((runtime_plan, exec_entities, plan_to_original))
}

fn collect_culist_metadata(
    runtime_plan: &CuExecutionLoop,
    exec_entities: &[ExecutionEntity],
    bridge_specs: &mut [BridgeSpec],
    plan_to_original: &HashMap<NodeId, NodeId>,
) -> (Vec<usize>, HashMap<NodeId, usize>) {
    let mut culist_order = Vec::new();
    let mut node_output_positions = HashMap::new();

    for unit in &runtime_plan.steps {
        if let CuExecutionUnit::Step(step) = unit {
            if let Some((output_idx, _)) = &step.output_msg_index_type {
                culist_order.push(*output_idx as usize);
                match &exec_entities[step.node_id as usize].kind {
                    ExecutionEntityKind::Task { .. } => {
                        if let Some(original_node_id) = plan_to_original.get(&step.node_id) {
                            node_output_positions.insert(*original_node_id, *output_idx as usize);
                        }
                    }
                    ExecutionEntityKind::BridgeRx {
                        bridge_index,
                        channel_index,
                    } => {
                        bridge_specs[*bridge_index].rx_channels[*channel_index].culist_index =
                            Some(*output_idx as usize);
                    }
                    ExecutionEntityKind::BridgeTx { .. } => {}
                }
            }
        }
    }

    (culist_order, node_output_positions)
}

#[allow(dead_code)]
fn build_monitored_ids(task_ids: &[String], bridge_specs: &mut [BridgeSpec]) -> Vec<String> {
    let mut names = task_ids.to_vec();
    for spec in bridge_specs.iter_mut() {
        spec.monitor_index = Some(names.len());
        names.push(format!("bridge::{}", spec.id));
        for channel in spec.rx_channels.iter_mut() {
            channel.monitor_index = Some(names.len());
            names.push(format!("bridge::{}::rx::{}", spec.id, channel.id));
        }
        for channel in spec.tx_channels.iter_mut() {
            channel.monitor_index = Some(names.len());
            names.push(format!("bridge::{}::tx::{}", spec.id, channel.id));
        }
    }
    names
}

fn generate_task_execution_tokens(
    step: &CuExecutionStep,
    task_index: usize,
    task_specs: &CuTaskSpecSet,
    sim_mode: bool,
    mission_mod: &Ident,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let node_index = int2sliceindex(task_index as u32);
    let task_instance = quote! { tasks.#node_index };
    let comment_str = format!(
        "DEBUG ->> {} ({:?}) Id:{} I:{:?} O:{:?}",
        step.node.get_id(),
        step.task_type,
        step.node_id,
        step.input_msg_indices_types,
        step.output_msg_index_type
    );
    let comment_tokens = quote! {{
        let _ = stringify!(#comment_str);
    }};
    let tid = task_index;
    let task_enum_name = config_id_to_enum(&task_specs.ids[tid]);
    let enum_name = Ident::new(&task_enum_name, Span::call_site());

    match step.task_type {
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
                            return Ok(());
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
                            let state = CuTaskCallbackState::Process((), cumsg_output);
                            let ovr = sim_callback(SimStep::#enum_name(state));

                            if let SimOverride::Errored(reason) = ovr  {
                                let error: CuError = reason.into();
                                #monitoring_action
                                false
                            } else {
                                ovr == SimOverride::ExecuteByRuntime
                            }
                        };
                    }
                } else {
                    quote! { let doit = true; }
                };

                let logging_tokens = if !task_specs.logging_enabled[tid] {
                    let output_culist_index = int2sliceindex(*output_index);
                    quote! {
                        let mut cumsg_output = &mut culist.msgs.0.#output_culist_index;
                        cumsg_output.clear_payload();
                    }
                } else {
                    quote!()
                };

                (
                    quote! {
                        {
                            #comment_tokens
                            kf_manager.freeze_task(clid, &#task_instance)?;
                            #call_sim_callback
                            let cumsg_output = &mut msgs.#output_culist_index;
                            cumsg_output.metadata.process_time.start = clock.now().into();
                            let maybe_error = if doit { #task_instance.process(clock, cumsg_output) } else { Ok(()) };
                            cumsg_output.metadata.process_time.end = clock.now().into();
                            if let Err(error) = maybe_error {
                                #monitoring_action
                            }
                        }
                    },
                    logging_tokens,
                )
            } else {
                panic!("Source task should have an output message index.");
            }
        }
        CuTaskType::Sink => {
            if let Some((output_index, _)) = &step.output_msg_index_type {
                let output_culist_index = int2sliceindex(*output_index);
                let indices = step
                    .input_msg_indices_types
                    .iter()
                    .map(|(index, _)| int2sliceindex(*index));

                let monitoring_action = quote! {
                    debug!("Task {}: Error during process: {}", #mission_mod::TASKS_IDS[#tid], &error);
                    let decision = monitor.process_error(#tid, CuTaskState::Process, &error);
                    match decision {
                        Decision::Abort => {
                            debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                    during process. Skipping the processing of CL {}.", #mission_mod::TASKS_IDS[#tid], clid);
                            monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                            cl_manager.end_of_processing(clid)?;
                            return Ok(());
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

                let inputs_type = if indices.len() == 1 {
                    quote! { #(msgs.#indices)* }
                } else {
                    quote! { (#(&msgs.#indices),*) }
                };

                let call_sim_callback = if sim_mode {
                    quote! {
                        let doit = {
                            let cumsg_input = &#inputs_type;
                            let cumsg_output = &mut msgs.#output_culist_index;
                            let state = CuTaskCallbackState::Process(cumsg_input, cumsg_output);
                            let ovr = sim_callback(SimStep::#enum_name(state));

                            if let SimOverride::Errored(reason) = ovr  {
                                let error: CuError = reason.into();
                                #monitoring_action
                                false
                            } else {
                                ovr == SimOverride::ExecuteByRuntime
                            }
                        };
                    }
                } else {
                    quote! { let doit = true; }
                };

                (
                    quote! {
                        {
                            #comment_tokens
                            kf_manager.freeze_task(clid, &#task_instance)?;
                            #call_sim_callback
                            let cumsg_input = &#inputs_type;
                            let cumsg_output = &mut msgs.#output_culist_index;
                            cumsg_output.metadata.process_time.start = clock.now().into();
                            let maybe_error = if doit { #task_instance.process(clock, cumsg_input) } else { Ok(()) };
                            cumsg_output.metadata.process_time.end = clock.now().into();
                            if let Err(error) = maybe_error {
                                #monitoring_action
                            }
                        }
                    },
                    quote! {},
                )
            } else {
                panic!("Sink tasks should have a virtual output message index.");
            }
        }
        CuTaskType::Regular => {
            if let Some((output_index, _)) = &step.output_msg_index_type {
                let output_culist_index = int2sliceindex(*output_index);
                let indices = step
                    .input_msg_indices_types
                    .iter()
                    .map(|(index, _)| int2sliceindex(*index));

                let monitoring_action = quote! {
                    debug!("Task {}: Error during process: {}", #mission_mod::TASKS_IDS[#tid], &error);
                    let decision = monitor.process_error(#tid, CuTaskState::Process, &error);
                    match decision {
                        Decision::Abort => {
                            debug!("Process: ABORT decision from monitoring. Task '{}' errored out \
                                    during process. Skipping the processing of CL {}.", #mission_mod::TASKS_IDS[#tid], clid);
                            monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                            cl_manager.end_of_processing(clid)?;
                            return Ok(());
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

                let inputs_type = if indices.len() == 1 {
                    quote! { #(msgs.#indices)* }
                } else {
                    quote! { (#(&msgs.#indices),*) }
                };

                let call_sim_callback = if sim_mode {
                    quote! {
                        let doit = {
                            let cumsg_input = &#inputs_type;
                            let cumsg_output = &mut msgs.#output_culist_index;
                            let state = CuTaskCallbackState::Process(cumsg_input, cumsg_output);
                            let ovr = sim_callback(SimStep::#enum_name(state));

                            if let SimOverride::Errored(reason) = ovr  {
                                let error: CuError = reason.into();
                                #monitoring_action
                                false
                            }
                            else {
                                ovr == SimOverride::ExecuteByRuntime
                            }
                        };
                    }
                } else {
                    quote! { let doit = true; }
                };

                let logging_tokens = if !task_specs.logging_enabled[tid] {
                    let output_culist_index = int2sliceindex(*output_index);
                    quote! {
                        let mut cumsg_output = &mut culist.msgs.0.#output_culist_index;
                        cumsg_output.clear_payload();
                    }
                } else {
                    quote!()
                };

                (
                    quote! {
                        {
                            #comment_tokens
                            kf_manager.freeze_task(clid, &#task_instance)?;
                            #call_sim_callback
                            let cumsg_input = &#inputs_type;
                            let cumsg_output = &mut msgs.#output_culist_index;
                            cumsg_output.metadata.process_time.start = clock.now().into();
                            let maybe_error = if doit { #task_instance.process(clock, cumsg_input, cumsg_output) } else { Ok(()) };
                            cumsg_output.metadata.process_time.end = clock.now().into();
                            if let Err(error) = maybe_error {
                                #monitoring_action
                            }
                        }
                    },
                    logging_tokens,
                )
            } else {
                panic!("Regular task should have an output message index.");
            }
        }
    }
}

fn generate_bridge_rx_execution_tokens(
    bridge_spec: &BridgeSpec,
    channel_index: usize,
    mission_mod: &Ident,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let bridge_tuple_index = int2sliceindex(bridge_spec.tuple_index as u32);
    let channel = &bridge_spec.rx_channels[channel_index];
    let culist_index = channel
        .culist_index
        .unwrap_or_else(|| panic!("Bridge Rx channel missing output index"));
    let culist_index_ts = int2sliceindex(culist_index as u32);
    let monitor_index = syn::Index::from(
        channel
            .monitor_index
            .expect("Bridge Rx channel missing monitor index"),
    );
    let bridge_type = &bridge_spec.type_path;
    let const_ident = &channel.const_ident;
    (
        quote! {
            {
                let bridge = &mut bridges.#bridge_tuple_index;
                let cumsg_output = &mut msgs.#culist_index_ts;
                cumsg_output.metadata.process_time.start = clock.now().into();
                let maybe_error = bridge.receive(
                    clock,
                    &<#bridge_type as cu29::cubridge::CuBridge>::Rx::#const_ident,
                    cumsg_output,
                );
                cumsg_output.metadata.process_time.end = clock.now().into();
                if let Err(error) = maybe_error {
                    let decision = monitor.process_error(#monitor_index, CuTaskState::Process, &error);
                    match decision {
                        Decision::Abort => {
                            debug!("Process: ABORT decision from monitoring. Task '{}' errored out during process. Skipping the processing of CL {}.", #mission_mod::TASKS_IDS[#monitor_index], clid);
                            monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                            cl_manager.end_of_processing(clid)?;
                            return Ok(());
                        }
                        Decision::Ignore => {
                            debug!("Process: IGNORE decision from monitoring. Task '{}' errored out during process. The runtime will continue with a forced empty message.", #mission_mod::TASKS_IDS[#monitor_index]);
                            let cumsg_output = &mut msgs.#culist_index_ts;
                            cumsg_output.clear_payload();
                        }
                        Decision::Shutdown => {
                            debug!("Process: SHUTDOWN decision from monitoring. Task '{}' errored out during process. The runtime cannot continue.", #mission_mod::TASKS_IDS[#monitor_index]);
                            return Err(CuError::new_with_cause("Task errored out during process.", error));
                        }
                    }
                }
            }
        },
        quote! {},
    )
}

fn generate_bridge_tx_execution_tokens(
    step: &CuExecutionStep,
    bridge_spec: &BridgeSpec,
    channel_index: usize,
    mission_mod: &Ident,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let channel = &bridge_spec.tx_channels[channel_index];
    let monitor_index = syn::Index::from(
        channel
            .monitor_index
            .expect("Bridge Tx channel missing monitor index"),
    );
    let input_index = step
        .input_msg_indices_types
        .first()
        .map(|(idx, _)| int2sliceindex(*idx))
        .expect("Bridge Tx channel should have exactly one input");
    let bridge_tuple_index = int2sliceindex(bridge_spec.tuple_index as u32);
    let bridge_type = &bridge_spec.type_path;
    let const_ident = &channel.const_ident;
    (
        quote! {
            {
                let bridge = &mut bridges.#bridge_tuple_index;
                let cumsg_input = &mut msgs.#input_index;
                // Stamp timing so monitors see consistent ranges for bridge Tx as well.
                cumsg_input.metadata.process_time.start = clock.now().into();
                if let Err(error) = bridge.send(
                    clock,
                    &<#bridge_type as cu29::cubridge::CuBridge>::Tx::#const_ident,
                    &*cumsg_input,
                ) {
                    let decision = monitor.process_error(#monitor_index, CuTaskState::Process, &error);
                    match decision {
                        Decision::Abort => {
                            debug!("Process: ABORT decision from monitoring. Task '{}' errored out during process. Skipping the processing of CL {}.", #mission_mod::TASKS_IDS[#monitor_index], clid);
                            monitor.process_copperlist(&#mission_mod::collect_metadata(&culist))?;
                            cl_manager.end_of_processing(clid)?;
                            return Ok(());
                        }
                        Decision::Ignore => {
                            debug!("Process: IGNORE decision from monitoring. Task '{}' errored out during process. The runtime will continue with a forced empty message.", #mission_mod::TASKS_IDS[#monitor_index]);
                        }
                        Decision::Shutdown => {
                            debug!("Process: SHUTDOWN decision from monitoring. Task '{}' errored out during process. The runtime cannot continue.", #mission_mod::TASKS_IDS[#monitor_index]);
                            return Err(CuError::new_with_cause("Task errored out during process.", error));
                        }
                    }
                }
                cumsg_input.metadata.process_time.end = clock.now().into();
            }
        },
        quote! {},
    )
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum BridgeChannelDirection {
    Rx,
    Tx,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct BridgeChannelKey {
    bridge_id: String,
    channel_id: String,
    direction: BridgeChannelDirection,
}

#[derive(Clone)]
struct BridgeChannelSpec {
    id: String,
    const_ident: Ident,
    #[allow(dead_code)]
    msg_type: Type,
    config_index: usize,
    plan_node_id: Option<NodeId>,
    culist_index: Option<usize>,
    monitor_index: Option<usize>,
}

#[derive(Clone)]
struct BridgeSpec {
    id: String,
    type_path: Type,
    config_index: usize,
    tuple_index: usize,
    monitor_index: Option<usize>,
    rx_channels: Vec<BridgeChannelSpec>,
    tx_channels: Vec<BridgeChannelSpec>,
}

#[derive(Clone)]
struct ExecutionEntity {
    kind: ExecutionEntityKind,
}

#[derive(Clone)]
enum ExecutionEntityKind {
    Task {
        task_index: usize,
    },
    BridgeRx {
        bridge_index: usize,
        channel_index: usize,
    },
    BridgeTx {
        bridge_index: usize,
        channel_index: usize,
    },
}
