use proc_macro::TokenStream;
use quote::{ToTokens, format_ident, quote};
use std::collections::{BTreeMap, HashMap};
use std::fs::read_to_string;
use std::path::Path;
use std::process::Command;
use syn::Fields::{Named, Unnamed};
use syn::meta::parser;
use syn::parse::Parser;
use syn::{
    Field, Fields, ItemImpl, ItemStruct, LitStr, Type, TypeTuple, parse_macro_input, parse_quote,
    parse_str,
};

use crate::utils::{config_id_to_bridge_const, config_id_to_enum, config_id_to_struct_member};
use cu29_runtime::config::CuConfig;
use cu29_runtime::config::{
    BridgeChannelConfigRepresentation, ConfigGraphs, CuGraph, Flavor, Node, NodeId,
    ResourceBundleConfig, read_configuration,
};
use cu29_runtime::curuntime::{
    CuExecutionLoop, CuExecutionStep, CuExecutionUnit, CuTaskType, compute_runtime_plan,
    find_task_type_for_id,
};
use cu29_traits::{CuError, CuResult};
use proc_macro2::{Ident, Span};

mod bundle_resources;
mod resources;
mod utils;

const DEFAULT_CLNB: usize = 2; // We can double buffer for now until we add the parallel copperlist execution support.

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

fn rtsan_guard_tokens() -> proc_macro2::TokenStream {
    if cfg!(feature = "rtsan") {
        quote! {
            let _rt_guard = ::cu29::rtsan::ScopedSanitizeRealtime::default();
        }
    } else {
        quote! {}
    }
}

fn git_output_trimmed(repo_root: &Path, args: &[&str]) -> Option<String> {
    let output = Command::new("git")
        .arg("-C")
        .arg(repo_root)
        .args(args)
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let stdout = String::from_utf8(output.stdout).ok()?;
    Some(stdout.trim().to_string())
}

fn detect_git_info(repo_root: &Path) -> (Option<String>, Option<bool>) {
    let in_repo = git_output_trimmed(repo_root, &["rev-parse", "--is-inside-work-tree"])
        .is_some_and(|value| value == "true");
    if !in_repo {
        return (None, None);
    }

    let commit = git_output_trimmed(repo_root, &["rev-parse", "HEAD"]).filter(|s| !s.is_empty());
    // Porcelain output is empty when tree is clean.
    let dirty = git_output_trimmed(repo_root, &["status", "--porcelain"]).map(|s| !s.is_empty());
    (commit, dirty)
}

#[derive(Debug, Clone)]
struct CopperRuntimeArgs {
    config_path: String,
    subsystem_id: Option<String>,
    sim_mode: bool,
    ignore_resources: bool,
}

impl CopperRuntimeArgs {
    fn parse_tokens(args: proc_macro2::TokenStream) -> Result<Self, syn::Error> {
        let mut config_file: Option<LitStr> = None;
        let mut subsystem_id: Option<LitStr> = None;
        let mut sim_mode = false;
        let mut ignore_resources = false;

        let parser = parser(|meta| {
            if meta.path.is_ident("config") {
                config_file = Some(meta.value()?.parse()?);
                Ok(())
            } else if meta.path.is_ident("subsystem") {
                subsystem_id = Some(meta.value()?.parse()?);
                Ok(())
            } else if meta.path.is_ident("sim_mode") {
                if meta.input.peek(syn::Token![=]) {
                    meta.input.parse::<syn::Token![=]>()?;
                    let value: syn::LitBool = meta.input.parse()?;
                    sim_mode = value.value();
                } else {
                    sim_mode = true;
                }
                Ok(())
            } else if meta.path.is_ident("ignore_resources") {
                if meta.input.peek(syn::Token![=]) {
                    meta.input.parse::<syn::Token![=]>()?;
                    let value: syn::LitBool = meta.input.parse()?;
                    ignore_resources = value.value();
                } else {
                    ignore_resources = true;
                }
                Ok(())
            } else {
                Err(meta.error("unsupported property"))
            }
        });

        parser.parse2(args)?;

        let config_path = config_file
            .ok_or_else(|| {
                syn::Error::new(
                    Span::call_site(),
                    "Expected config file attribute like #[copper_runtime(config = \"path\")]",
                )
            })?
            .value();

        Ok(Self {
            config_path,
            subsystem_id: subsystem_id.map(|value| value.value()),
            sim_mode,
            ignore_resources,
        })
    }
}

#[derive(Debug)]
struct ResolvedRuntimeConfig {
    local_config: CuConfig,
    bundled_local_config_content: String,
    subsystem_id: Option<String>,
    subsystem_code: u16,
}

#[proc_macro]
pub fn resources(input: TokenStream) -> TokenStream {
    resources::resources(input)
}

#[proc_macro]
pub fn bundle_resources(input: TokenStream) -> TokenStream {
    bundle_resources::bundle_resources(input)
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

    let extra_imports = if !std {
        quote! {
            use core::fmt::Debug;
            use core::fmt::Formatter;
            use core::fmt::Result as FmtResult;
            use alloc::vec;
            use alloc::vec::Vec;
        }
    } else {
        quote! {
            use std::fmt::Debug;
            use std::fmt::Formatter;
            use std::fmt::Result as FmtResult;
        }
    };

    let common_imports = quote! {
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
        use cu29::prelude::CuListZeroedInit;
        use cu29::prelude::CuCompactString;
        #extra_imports
    };

    let with_uses = match &cuconfig.graphs {
        ConfigGraphs::Simple(graph) => {
            let support = match build_gen_cumsgs_support(&cuconfig, graph, None) {
                Ok(support) => support,
                Err(e) => return return_error(e.to_string()),
            };

            quote! {
                mod cumsgs {
                    #common_imports
                    #support
                }
                use cumsgs::CuStampedDataSet;
                type CuMsgs=CuStampedDataSet;
            }
        }
        ConfigGraphs::Missions(graphs) => {
            let mut missions: Vec<_> = graphs.iter().collect();
            missions.sort_by(|a, b| a.0.cmp(b.0));

            let mut mission_modules = Vec::<proc_macro2::TokenStream>::new();
            for (mission, graph) in missions {
                let mission_mod = match parse_str::<Ident>(mission.as_str()) {
                    Ok(id) => id,
                    Err(_) => {
                        return return_error(format!(
                            "Mission '{mission}' is not a valid Rust identifier for gen_cumsgs output."
                        ));
                    }
                };

                let support = match build_gen_cumsgs_support(&cuconfig, graph, Some(mission)) {
                    Ok(support) => support,
                    Err(e) => return return_error(e.to_string()),
                };

                mission_modules.push(quote! {
                    pub mod #mission_mod {
                        #common_imports
                        #support
                    }
                });
            }

            let default_exports = if graphs.contains_key("default") {
                quote! {
                    use cumsgs::default::CuStampedDataSet;
                    type CuMsgs=CuStampedDataSet;
                }
            } else {
                quote! {}
            };

            quote! {
                mod cumsgs {
                    #(#mission_modules)*
                }
                #default_exports
            }
        }
    };
    with_uses.into()
}

fn build_gen_cumsgs_support(
    cuconfig: &CuConfig,
    graph: &CuGraph,
    mission_label: Option<&str>,
) -> CuResult<proc_macro2::TokenStream> {
    let task_specs = CuTaskSpecSet::from_graph(graph);
    let channel_usage = collect_bridge_channel_usage(graph);
    let mut bridge_specs = build_bridge_specs(cuconfig, graph, &channel_usage);
    let (culist_plan, exec_entities, plan_to_original) =
        build_execution_plan(graph, &task_specs, &mut bridge_specs).map_err(|e| {
            if let Some(mission) = mission_label {
                CuError::from(format!(
                    "Could not compute copperlist plan for mission '{mission}': {e}"
                ))
            } else {
                CuError::from(format!("Could not compute copperlist plan: {e}"))
            }
        })?;
    let task_names = collect_task_names(graph);
    let (culist_order, node_output_positions) = collect_culist_metadata(
        &culist_plan,
        &exec_entities,
        &mut bridge_specs,
        &plan_to_original,
    );

    #[cfg(feature = "macro_debug")]
    if let Some(mission) = mission_label {
        eprintln!(
            "[The CuStampedDataSet matching tasks ids for mission '{mission}' are {:?}]",
            culist_order
        );
    } else {
        eprintln!(
            "[The CuStampedDataSet matching tasks ids are {:?}]",
            culist_order
        );
    }

    Ok(gen_culist_support(
        cuconfig,
        mission_label,
        &culist_plan,
        &culist_order,
        &node_output_positions,
        &task_names,
        &bridge_specs,
    ))
}

/// Build the inner support of the copper list.
fn gen_culist_support(
    cuconfig: &CuConfig,
    mission_label: Option<&str>,
    runtime_plan: &CuExecutionLoop,
    culist_indices_in_plan_order: &[usize],
    node_output_positions: &HashMap<NodeId, usize>,
    task_names: &[(NodeId, String, String)],
    bridge_specs: &[BridgeSpec],
) -> proc_macro2::TokenStream {
    #[cfg(feature = "macro_debug")]
    eprintln!("[Extract msgs types]");
    let output_packs = extract_output_packs(runtime_plan);
    let slot_types: Vec<Type> = output_packs.iter().map(|pack| pack.slot_type()).collect();

    let culist_size = output_packs.len();

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist struct]");
    let msgs_types_tuple: TypeTuple = build_culist_tuple(&slot_types);
    let cumsg_count: usize = output_packs.iter().map(|pack| pack.msg_types.len()).sum();
    let flat_codec_bindings = build_flat_slot_codec_bindings(
        cuconfig,
        mission_label,
        &output_packs,
        node_output_positions,
        task_names,
    )
    .unwrap_or_else(|err| panic!("Could not resolve log codec bindings: {err}"));
    let default_config_ron_ident = format_ident!("__CU_LOGCODEC_DEFAULT_CONFIG_RON");
    let default_config_ron = cuconfig
        .serialize_ron()
        .unwrap_or_else(|_| "<failed to serialize config>".to_string());
    let default_config_ron_lit = LitStr::new(&default_config_ron, Span::call_site());
    let (codec_helper_fns, encode_helper_names, decode_helper_names) = build_culist_codec_helpers(
        &flat_codec_bindings,
        &default_config_ron_ident,
        mission_label,
    );
    let default_config_ron_const = if flat_codec_bindings.iter().any(Option::is_some) {
        quote! {
            const #default_config_ron_ident: &str = #default_config_ron_lit;
        }
    } else {
        quote! {}
    };

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist tuple bincode support]");
    let msgs_types_tuple_encode = build_culist_tuple_encode(&output_packs, &encode_helper_names);
    let msgs_types_tuple_decode = build_culist_tuple_decode(
        &output_packs,
        &slot_types,
        cumsg_count,
        &decode_helper_names,
    );

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist tuple debug support]");
    let msgs_types_tuple_debug = build_culist_tuple_debug(&slot_types);

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist tuple serialize support]");
    let msgs_types_tuple_serialize = build_culist_tuple_serialize(&slot_types);

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the default tuple support]");
    let msgs_types_tuple_default = build_culist_tuple_default(&slot_types, cumsg_count);

    #[cfg(feature = "macro_debug")]
    eprintln!("[build erasedcumsgs]");

    let erasedmsg_trait_impl = build_culist_erasedcumsgs(&output_packs);

    let metadata_accessors: Vec<proc_macro2::TokenStream> = culist_indices_in_plan_order
        .iter()
        .map(|idx| {
            let slot_index = syn::Index::from(*idx);
            let pack = output_packs
                .get(*idx)
                .unwrap_or_else(|| panic!("Missing output pack for index {idx}"));
            if pack.is_multi() {
                quote! { &culist.msgs.0.#slot_index.0.metadata }
            } else {
                quote! { &culist.msgs.0.#slot_index.metadata }
            }
        })
        .collect();
    let mut zeroed_init_tokens: Vec<proc_macro2::TokenStream> = Vec::new();
    for idx in culist_indices_in_plan_order {
        let slot_index = syn::Index::from(*idx);
        let pack = output_packs
            .get(*idx)
            .unwrap_or_else(|| panic!("Missing output pack for index {idx}"));
        if pack.is_multi() {
            for port_idx in 0..pack.msg_types.len() {
                let port_index = syn::Index::from(port_idx);
                zeroed_init_tokens.push(quote! {
                    self.0.#slot_index.#port_index.metadata.status_txt = CuCompactString::default();
                    self.0.#slot_index.#port_index.metadata.process_time.start =
                        cu29::clock::OptionCuTime::none();
                    self.0.#slot_index.#port_index.metadata.process_time.end =
                        cu29::clock::OptionCuTime::none();
                    self.0.#slot_index.#port_index.metadata.origin = None;
                });
            }
        } else {
            zeroed_init_tokens.push(quote! {
                self.0.#slot_index.metadata.status_txt = CuCompactString::default();
                self.0.#slot_index.metadata.process_time.start = cu29::clock::OptionCuTime::none();
                self.0.#slot_index.metadata.process_time.end = cu29::clock::OptionCuTime::none();
                self.0.#slot_index.metadata.origin = None;
            });
        }
    }
    let collect_metadata_function = quote! {
        pub fn collect_metadata<'a>(culist: &'a CuList) -> [&'a CuMsgMetadata; #culist_size] {
            [#( #metadata_accessors, )*]
        }
    };

    let payload_bytes_accumulators: Vec<proc_macro2::TokenStream> = culist_indices_in_plan_order
        .iter()
        .scan(0usize, |flat_idx, idx| {
            let slot_index = syn::Index::from(*idx);
            let pack = output_packs
                .get(*idx)
                .unwrap_or_else(|| panic!("Missing output pack for index {idx}"));
            if pack.is_multi() {
                let iter = (0..pack.msg_types.len()).map(|port_idx| {
                    let port_index = syn::Index::from(port_idx);
                    let cache_index = syn::Index::from(*flat_idx);
                    *flat_idx += 1;
                    quote! {
                        if let Some(payload) = culist.msgs.0.#slot_index.#port_index.payload() {
                            let cached = culist.msgs.1.get(#cache_index);
                            let io = if cached.present {
                                cu29::monitoring::PayloadIoStats {
                                    resident_bytes: cached.resident_bytes as usize,
                                    encoded_bytes: cached.encoded_bytes as usize,
                                    handle_bytes: cached.handle_bytes as usize,
                                }
                            } else {
                                cu29::monitoring::payload_io_stats(payload)?
                            };
                            raw += io.resident_bytes;
                            handles += io.handle_bytes;
                        }
                    }
                });
                Some(quote! { #(#iter)* })
            } else {
                let cache_index = syn::Index::from(*flat_idx);
                *flat_idx += 1;
                Some(quote! {
                    if let Some(payload) = culist.msgs.0.#slot_index.payload() {
                        let cached = culist.msgs.1.get(#cache_index);
                        let io = if cached.present {
                            cu29::monitoring::PayloadIoStats {
                                resident_bytes: cached.resident_bytes as usize,
                                encoded_bytes: cached.encoded_bytes as usize,
                                handle_bytes: cached.handle_bytes as usize,
                            }
                        } else {
                            cu29::monitoring::payload_io_stats(payload)?
                        };
                        raw += io.resident_bytes;
                        handles += io.handle_bytes;
                    }
                })
            }
        })
        .collect();

    let payload_raw_bytes_accumulators: Vec<proc_macro2::TokenStream> = output_packs
        .iter()
        .enumerate()
        .scan(0usize, |flat_idx, (slot_idx, pack)| {
            let slot_index = syn::Index::from(slot_idx);
            if pack.is_multi() {
                let iter = (0..pack.msg_types.len()).map(|port_idx| {
                    let port_index = syn::Index::from(port_idx);
                    let cache_index = syn::Index::from(*flat_idx);
                    *flat_idx += 1;
                    quote! {
                        if let Some(payload) = self.0.#slot_index.#port_index.payload() {
                            let cached = self.1.get(#cache_index);
                            bytes.push(if cached.present {
                                Some(cached.resident_bytes)
                            } else {
                                cu29::monitoring::payload_io_stats(payload)
                                    .ok()
                                    .map(|io| io.resident_bytes as u64)
                            });
                        } else {
                            bytes.push(None);
                        }
                    }
                });
                Some(quote! { #(#iter)* })
            } else {
                let cache_index = syn::Index::from(*flat_idx);
                *flat_idx += 1;
                Some(quote! {
                    if let Some(payload) = self.0.#slot_index.payload() {
                        let cached = self.1.get(#cache_index);
                        bytes.push(if cached.present {
                            Some(cached.resident_bytes)
                        } else {
                            cu29::monitoring::payload_io_stats(payload)
                                .ok()
                                .map(|io| io.resident_bytes as u64)
                        });
                    } else {
                        bytes.push(None);
                    }
                })
            }
        })
        .collect();

    let compute_payload_bytes_fn = quote! {
        pub fn compute_payload_bytes(culist: &CuList) -> cu29::prelude::CuResult<(u64, u64)> {
            let mut raw: usize = 0;
            let mut handles: usize = 0;
            #(#payload_bytes_accumulators)*
            Ok((raw as u64, handles as u64))
        }
    };

    let payload_raw_bytes_impl = quote! {
        impl ::cu29::CuPayloadRawBytes for CuStampedDataSet {
            fn payload_raw_bytes(&self) -> Vec<Option<u64>> {
                let mut bytes: Vec<Option<u64>> = Vec::with_capacity(#cumsg_count);
                #(#payload_raw_bytes_accumulators)*
                bytes
            }
        }
    };

    let mut slot_origin_ids: Vec<Option<String>> = vec![None; output_packs.len()];
    let mut slot_task_names: Vec<Option<String>> = vec![None; output_packs.len()];

    let mut methods = Vec::new();
    for (node_id, task_id, member_name) in task_names {
        let output_position = node_output_positions.get(node_id).unwrap_or_else(|| {
            panic!("Task {task_id} (node id: {node_id}) not found in execution order")
        });
        let pack = output_packs
            .get(*output_position)
            .unwrap_or_else(|| panic!("Missing output pack for task {task_id}"));
        let slot_index = syn::Index::from(*output_position);
        slot_origin_ids[*output_position] = Some(task_id.clone());
        slot_task_names[*output_position] = Some(member_name.clone());

        if pack.msg_types.len() == 1 {
            let fn_name = format_ident!("get_{}_output", member_name);
            let payload_type = pack.msg_types.first().unwrap();
            methods.push(quote! {
                #[allow(dead_code)]
                pub fn #fn_name(&self) -> &CuMsg<#payload_type> {
                    &self.0.#slot_index
                }
            });
        } else {
            let outputs_fn = format_ident!("get_{}_outputs", member_name);
            let slot_type = pack.slot_type();
            for (port_idx, payload_type) in pack.msg_types.iter().enumerate() {
                let fn_name = format_ident!("get_{}_output_{}", member_name, port_idx);
                let port_index = syn::Index::from(port_idx);
                methods.push(quote! {
                    #[allow(dead_code)]
                    pub fn #fn_name(&self) -> &CuMsg<#payload_type> {
                        &self.0.#slot_index.#port_index
                    }
                });
            }
            methods.push(quote! {
                #[allow(dead_code)]
                pub fn #outputs_fn(&self) -> &#slot_type {
                    &self.0.#slot_index
                }
            });
        }
    }

    for spec in bridge_specs {
        for channel in &spec.rx_channels {
            if let Some(culist_index) = channel.culist_index {
                let origin_id = format!("bridge::{}::rx::{}", spec.id, channel.id);
                let Some(existing_slot) = slot_origin_ids.get_mut(culist_index) else {
                    panic!(
                        "Bridge origin '{origin_id}' points to out-of-range copperlist slot {culist_index}"
                    );
                };
                if let Some(existing) = existing_slot.as_ref() {
                    panic!(
                        "Duplicate slot origin assignment for slot {culist_index}: '{existing}' and '{origin_id}'"
                    );
                }
                *existing_slot = Some(origin_id.clone());
                let Some(slot_name) = slot_task_names.get_mut(culist_index) else {
                    panic!(
                        "Bridge origin '{origin_id}' points to out-of-range name slot {culist_index}"
                    );
                };
                *slot_name = Some(origin_id);
            }
        }
        for channel in &spec.tx_channels {
            if let Some(culist_index) = channel.culist_index {
                let origin_id = format!("bridge::{}::tx::{}", spec.id, channel.id);
                let Some(existing_slot) = slot_origin_ids.get_mut(culist_index) else {
                    panic!(
                        "Bridge origin '{origin_id}' points to out-of-range copperlist slot {culist_index}"
                    );
                };
                if let Some(existing) = existing_slot.as_ref() {
                    panic!(
                        "Duplicate slot origin assignment for slot {culist_index}: '{existing}' and '{origin_id}'"
                    );
                }
                *existing_slot = Some(origin_id.clone());
                let Some(slot_name) = slot_task_names.get_mut(culist_index) else {
                    panic!(
                        "Bridge origin '{origin_id}' points to out-of-range name slot {culist_index}"
                    );
                };
                *slot_name = Some(origin_id);
            }
        }
    }

    let task_name_literals = flatten_slot_origin_ids(&output_packs, &slot_origin_ids);
    let task_output_specs = flatten_task_output_specs(&output_packs, &slot_origin_ids);
    let task_output_spec_literals: Vec<proc_macro2::TokenStream> = task_output_specs
        .iter()
        .map(|(task_id, msg_type, payload_type_path)| {
            let task_id = LitStr::new(task_id, Span::call_site());
            let msg_type = LitStr::new(msg_type, Span::call_site());
            let payload_type_path = LitStr::new(payload_type_path, Span::call_site());
            quote! {
                cu29::TaskOutputSpec {
                    task_id: #task_id,
                    msg_type: #msg_type,
                    payload_type_path: #payload_type_path,
                }
            }
        })
        .collect();

    let mut logviz_blocks = Vec::new();
    for (slot_idx, pack) in output_packs.iter().enumerate() {
        if pack.msg_types.is_empty() {
            continue;
        }
        let slot_index = syn::Index::from(slot_idx);
        let slot_name = slot_task_names.get(slot_idx).and_then(|name| name.as_ref());

        if pack.is_multi() {
            for (port_idx, _) in pack.msg_types.iter().enumerate() {
                let port_index = syn::Index::from(port_idx);
                let path_expr = if let Some(name) = slot_name {
                    let lit = LitStr::new(name, Span::call_site());
                    quote! { format!("{}/{}", #lit, #port_idx) }
                } else {
                    quote! { format!("slot_{}/{}", #slot_idx, #port_idx) }
                };
                logviz_blocks.push(quote! {
                    {
                        let msg = &self.0.#slot_index.#port_index;
                        if let Some(payload) = msg.payload() {
                            ::cu29_logviz::apply_tov(rec, &msg.tov);
                            let path = #path_expr;
                            ::cu29_logviz::log_payload_auto(rec, &path, payload)?;
                        }
                    }
                });
            }
        } else {
            let path_expr = if let Some(name) = slot_name {
                let lit = LitStr::new(name, Span::call_site());
                quote! { #lit.to_string() }
            } else {
                quote! { format!("slot_{}", #slot_idx) }
            };
            logviz_blocks.push(quote! {
                {
                    let msg = &self.0.#slot_index;
                    if let Some(payload) = msg.payload() {
                        ::cu29_logviz::apply_tov(rec, &msg.tov);
                        let path = #path_expr;
                        ::cu29_logviz::log_payload_auto(rec, &path, payload)?;
                    }
                }
            });
        }
    }

    let logviz_impl = if cfg!(feature = "logviz") {
        quote! {
            impl ::cu29_logviz::LogvizDataSet for CuStampedDataSet {
                fn logviz_emit(
                    &self,
                    rec: &::cu29_logviz::RecordingStream,
                ) -> ::cu29::prelude::CuResult<()> {
                    #(#logviz_blocks)*
                    Ok(())
                }
            }
        }
    } else {
        quote! {}
    };
    // Generate bridge channel getter methods
    for spec in bridge_specs {
        for channel in &spec.rx_channels {
            if let Some(culist_index) = channel.culist_index {
                let slot_index = syn::Index::from(culist_index);
                let bridge_name = config_id_to_struct_member(spec.id.as_str());
                let channel_name = config_id_to_struct_member(channel.id.as_str());
                let fn_name = format_ident!("get_{}_rx_{}", bridge_name, channel_name);
                let msg_type = &channel.msg_type;

                methods.push(quote! {
                    #[allow(dead_code)]
                    pub fn #fn_name(&self) -> &CuMsg<#msg_type> {
                        &self.0.#slot_index
                    }
                });
            }
        }
    }

    // This generates a way to get the metadata of every single message of a culist at low cost
    quote! {
        #collect_metadata_function
        #compute_payload_bytes_fn
        #default_config_ron_const
        #(#codec_helper_fns)*

        pub struct CuStampedDataSet(pub #msgs_types_tuple, cu29::monitoring::CuMsgIoCache<#cumsg_count>);

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

        #payload_raw_bytes_impl
        #logviz_impl

        impl MatchingTasks for CuStampedDataSet {
            #[allow(dead_code)]
            fn get_all_task_ids() -> &'static [&'static str] {
                &[#(#task_name_literals),*]
            }

            #[allow(dead_code)]
            fn get_output_specs() -> &'static [cu29::TaskOutputSpec] {
                &[#(#task_output_spec_literals),*]
            }
        }

        // Note: PayloadSchemas is NOT implemented here.
        // Users who want MCAP export with schemas should implement it manually
        // using cu29_export::trace_type_to_jsonschema.

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
                self.1.clear();
                #(#zeroed_init_tokens)*
            }
        }
    }
}

fn gen_sim_support(
    runtime_plan: &CuExecutionLoop,
    exec_entities: &[ExecutionEntity],
    bridge_specs: &[BridgeSpec],
) -> proc_macro2::TokenStream {
    #[cfg(feature = "macro_debug")]
    eprintln!("[Sim: Build SimEnum]");
    let plan_enum: Vec<proc_macro2::TokenStream> = runtime_plan
        .steps
        .iter()
        .map(|unit| match unit {
            CuExecutionUnit::Step(step) => match &exec_entities[step.node_id as usize].kind {
                ExecutionEntityKind::Task { .. } => {
                    let enum_entry_name = config_id_to_enum(step.node.get_id().as_str());
                    let enum_ident = Ident::new(&enum_entry_name, Span::call_site());
                    let inputs: Vec<Type> = step
                        .input_msg_indices_types
                        .iter()
                        .map(|input| {
                            parse_str::<Type>(format!("CuMsg<{}>", input.msg_type).as_str()).unwrap()
                        })
                        .collect();
                    let output: Option<Type> = step.output_msg_pack.as_ref().map(|pack| {
                        let msg_types: Vec<Type> = pack
                            .msg_types
                            .iter()
                            .map(|msg_type| {
                                parse_str::<Type>(msg_type.as_str()).unwrap_or_else(|_| {
                                    panic!("Could not transform {msg_type} into a message Rust type.")
                                })
                            })
                            .collect();
                        build_output_slot_type(&msg_types)
                    });
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
                        #enum_ident(CuTaskCallbackState<#inputs_type, &'a mut #output>)
                    }
                }
                ExecutionEntityKind::BridgeRx { bridge_index, channel_index } => {
                    let bridge_spec = &bridge_specs[*bridge_index];
                    let channel = &bridge_spec.rx_channels[*channel_index];
                    let enum_entry_name = config_id_to_enum(&format!("{}_rx_{}", bridge_spec.id, channel.id));
                    let enum_ident = Ident::new(&enum_entry_name, Span::call_site());
                    let channel_type: Type = parse_str::<Type>(channel.msg_type_name.as_str()).unwrap();
                    let bridge_type = runtime_bridge_type_for_spec(bridge_spec, true);
                    let _const_ident = &channel.const_ident;
                    quote! {
                        #enum_ident {
                            channel: &'static cu29::cubridge::BridgeChannel<< <#bridge_type as cu29::cubridge::CuBridge>::Rx as cu29::cubridge::BridgeChannelSet >::Id, #channel_type>,
                            msg: &'a mut CuMsg<#channel_type>,
                        }
                    }
                }
                ExecutionEntityKind::BridgeTx { bridge_index, channel_index } => {
                    let bridge_spec = &bridge_specs[*bridge_index];
                    let channel = &bridge_spec.tx_channels[*channel_index];
                    let enum_entry_name = config_id_to_enum(&format!("{}_tx_{}", bridge_spec.id, channel.id));
                    let enum_ident = Ident::new(&enum_entry_name, Span::call_site());
                    let channel_type: Type = parse_str::<Type>(channel.msg_type_name.as_str()).unwrap();
                    let output_pack = step
                        .output_msg_pack
                        .as_ref()
                        .expect("Bridge Tx channel missing output pack for sim support");
                    let output_types: Vec<Type> = output_pack
                        .msg_types
                        .iter()
                        .map(|msg_type| {
                            parse_str::<Type>(msg_type.as_str()).unwrap_or_else(|_| {
                                panic!("Could not transform {msg_type} into a message Rust type.")
                            })
                        })
                        .collect();
                    let output_type = build_output_slot_type(&output_types);
                    let bridge_type = runtime_bridge_type_for_spec(bridge_spec, true);
                    let _const_ident = &channel.const_ident;
                    quote! {
                        #enum_ident {
                            channel: &'static cu29::cubridge::BridgeChannel<< <#bridge_type as cu29::cubridge::CuBridge>::Tx as cu29::cubridge::BridgeChannelSet >::Id, #channel_type>,
                            msg: &'a CuMsg<#channel_type>,
                            output: &'a mut #output_type,
                        }
                    }
                }
            },
            CuExecutionUnit::Loop(_) => {
                todo!("Needs to be implemented")
            }
        })
        .collect();

    // bridge lifecycle variants (one per bridge)
    let mut variants = plan_enum;

    // add bridge lifecycle variants
    for bridge_spec in bridge_specs {
        let enum_entry_name = config_id_to_enum(&format!("{}_bridge", bridge_spec.id));
        let enum_ident = Ident::new(&enum_entry_name, Span::call_site());
        variants.push(quote! {
            #enum_ident(cu29::simulation::CuBridgeLifecycleState)
        });
    }

    variants.push(quote! { __Phantom(core::marker::PhantomData<&'a ()>) });
    quote! {
        // not used if sim is not generated but this is ok.
        #[allow(dead_code, unused_lifetimes)]
        pub enum SimStep<'a> {
            #(#variants),*
        }
    }
}

fn gen_recorded_replay_support(
    runtime_plan: &CuExecutionLoop,
    exec_entities: &[ExecutionEntity],
    bridge_specs: &[BridgeSpec],
) -> proc_macro2::TokenStream {
    let replay_arms: Vec<proc_macro2::TokenStream> = runtime_plan
        .steps
        .iter()
        .filter_map(|unit| match unit {
            CuExecutionUnit::Step(step) => match &exec_entities[step.node_id as usize].kind {
                ExecutionEntityKind::Task { .. } => {
                    let enum_entry_name = config_id_to_enum(step.node.get_id().as_str());
                    let enum_ident = Ident::new(&enum_entry_name, Span::call_site());
                    let output_pack = step
                        .output_msg_pack
                        .as_ref()
                        .expect("Task step missing output pack for recorded replay");
                    let culist_index = int2sliceindex(output_pack.culist_index);
                    Some(quote! {
                        SimStep::#enum_ident(CuTaskCallbackState::Process(_, output)) => {
                            *output = recorded.msgs.0.#culist_index.clone();
                            SimOverride::ExecutedBySim
                        }
                    })
                }
                ExecutionEntityKind::BridgeRx {
                    bridge_index,
                    channel_index,
                } => {
                    let bridge_spec = &bridge_specs[*bridge_index];
                    let channel = &bridge_spec.rx_channels[*channel_index];
                    let enum_entry_name =
                        config_id_to_enum(&format!("{}_rx_{}", bridge_spec.id, channel.id));
                    let enum_ident = Ident::new(&enum_entry_name, Span::call_site());
                    let output_pack = step
                        .output_msg_pack
                        .as_ref()
                        .expect("Bridge Rx channel missing output pack for recorded replay");
                    let port_index = output_pack
                        .msg_types
                        .iter()
                        .position(|msg| msg == &channel.msg_type_name)
                        .unwrap_or_else(|| {
                            panic!(
                                "Bridge Rx channel '{}' missing output port for '{}'",
                                channel.id, channel.msg_type_name
                            )
                        });
                    let culist_index = int2sliceindex(output_pack.culist_index);
                    let recorded_slot = if output_pack.msg_types.len() == 1 {
                        quote! { recorded.msgs.0.#culist_index.clone() }
                    } else {
                        let port_index = syn::Index::from(port_index);
                        quote! { recorded.msgs.0.#culist_index.#port_index.clone() }
                    };
                    Some(quote! {
                        SimStep::#enum_ident { msg, .. } => {
                            *msg = #recorded_slot;
                            SimOverride::ExecutedBySim
                        }
                    })
                }
                ExecutionEntityKind::BridgeTx {
                    bridge_index,
                    channel_index,
                } => {
                    let bridge_spec = &bridge_specs[*bridge_index];
                    let channel = &bridge_spec.tx_channels[*channel_index];
                    let enum_entry_name =
                        config_id_to_enum(&format!("{}_tx_{}", bridge_spec.id, channel.id));
                    let enum_ident = Ident::new(&enum_entry_name, Span::call_site());
                    let output_pack = step
                        .output_msg_pack
                        .as_ref()
                        .expect("Bridge Tx channel missing output pack for recorded replay");
                    let culist_index = int2sliceindex(output_pack.culist_index);
                    Some(quote! {
                        SimStep::#enum_ident { output, .. } => {
                            *output = recorded.msgs.0.#culist_index.clone();
                            SimOverride::ExecutedBySim
                        }
                    })
                }
            },
            CuExecutionUnit::Loop(_) => None,
        })
        .collect();

    quote! {
        #[allow(dead_code)]
        pub fn recorded_replay_step<'a>(
            step: SimStep<'a>,
            recorded: &CopperList<CuStampedDataSet>,
        ) -> SimOverride {
            match step {
                #(#replay_arms),*,
                _ => SimOverride::ExecuteByRuntime,
            }
        }
    }
}

/// Adds `#[copper_runtime(config = "path", subsystem = "id", sim_mode = false/true, ignore_resources = false/true)]`
/// to your application struct to generate the runtime.
/// if sim_mode is omitted, it is set to false.
/// if ignore_resources is omitted, it is set to false.
/// if `subsystem` is provided, `config` must point to a strict multi-Copper config and the
/// selected subsystem local config will be embedded into the generated runtime.
/// This will add a "runtime" field to your struct and implement the "new" and "run" methods.
#[proc_macro_attribute]
pub fn copper_runtime(args: TokenStream, input: TokenStream) -> TokenStream {
    #[cfg(feature = "macro_debug")]
    eprintln!("[entry]");
    let mut application_struct = parse_macro_input!(input as ItemStruct);

    let application_name = &application_struct.ident;
    let builder_name = format_ident!("{}Builder", application_name);
    let runtime_args = match CopperRuntimeArgs::parse_tokens(args.into()) {
        Ok(runtime_args) => runtime_args,
        Err(err) => return err.to_compile_error().into(),
    };
    let config_file = runtime_args.config_path.clone();
    let sim_mode = runtime_args.sim_mode;
    let ignore_resources = runtime_args.ignore_resources;

    #[cfg(feature = "std")]
    let std = true;

    #[cfg(not(feature = "std"))]
    let std = false;
    let signal_handler = cfg!(feature = "signal-handler");
    let parallel_rt_enabled = cfg!(feature = "parallel-rt");
    let rt_guard = rtsan_guard_tokens();

    if ignore_resources && !sim_mode {
        return return_error(
            "`ignore_resources` is only supported when `sim_mode` is enabled".to_string(),
        );
    }

    // Adds the generic parameter for the UnifiedLogger if this is a real application (not sim)
    // This allows to adapt either to the no-std (custom impl) and std (default file based one)
    // if !sim_mode {
    //     application_struct
    //         .generics
    //         .params
    //         .push(syn::parse_quote!(L: UnifiedLogWrite + 'static));
    // }

    let resolved_runtime_config = match resolve_runtime_config(&runtime_args) {
        Ok(resolved_runtime_config) => resolved_runtime_config,
        Err(e) => return return_error(e.to_string()),
    };
    let subsystem_code = resolved_runtime_config.subsystem_code;
    let subsystem_id = resolved_runtime_config.subsystem_id.clone();
    let copper_config_content = resolved_runtime_config.bundled_local_config_content.clone();
    let copper_config = resolved_runtime_config.local_config;
    let copperlist_count = copper_config
        .logging
        .as_ref()
        .and_then(|logging| logging.copperlist_count)
        .unwrap_or(DEFAULT_CLNB);
    let copperlist_count_tokens = proc_macro2::Literal::usize_unsuffixed(copperlist_count);
    let caller_root = utils::caller_crate_root();
    let (git_commit, git_dirty) = detect_git_info(&caller_root);
    let git_commit_tokens = if let Some(commit) = git_commit {
        quote! { Some(#commit.to_string()) }
    } else {
        quote! { None }
    };
    let git_dirty_tokens = if let Some(dirty) = git_dirty {
        quote! { Some(#dirty) }
    } else {
        quote! { None }
    };
    let subsystem_code_literal = proc_macro2::Literal::u16_unsuffixed(subsystem_code);
    let subsystem_id_tokens = if let Some(subsystem_id) = subsystem_id.as_deref() {
        quote! { Some(#subsystem_id) }
    } else {
        quote! { None }
    };

    #[cfg(feature = "macro_debug")]
    eprintln!("[build monitor type]");
    let monitor_configs = copper_config.get_monitor_configs();
    let (monitor_type, monitor_instanciator_body) = if monitor_configs.is_empty() {
        (
            quote! { NoMonitor },
            quote! {
                let monitor_metadata = metadata.with_subsystem_id(#subsystem_id_tokens);
                let monitor = NoMonitor::new(monitor_metadata, runtime)
                    .expect("Failed to create NoMonitor.");
                monitor
            },
        )
    } else if monitor_configs.len() == 1 {
        let only_monitor_type = parse_str::<Type>(monitor_configs[0].get_type())
            .expect("Could not transform the monitor type name into a Rust type.");
        (
            quote! { #only_monitor_type },
            quote! {
                let monitor_metadata = metadata.with_monitor_config(
                    config
                        .get_monitor_configs()
                        .first()
                        .and_then(|entry| entry.get_config().cloned())
                )
                .with_subsystem_id(#subsystem_id_tokens);
                let monitor = #only_monitor_type::new(monitor_metadata, runtime)
                    .expect("Failed to create the given monitor.");
                monitor
            },
        )
    } else {
        let monitor_types: Vec<Type> = monitor_configs
            .iter()
            .map(|monitor_config| {
                parse_str::<Type>(monitor_config.get_type())
                    .expect("Could not transform the monitor type name into a Rust type.")
            })
            .collect();
        let monitor_bindings: Vec<Ident> = (0..monitor_types.len())
            .map(|idx| format_ident!("__cu_monitor_{idx}"))
            .collect();
        let monitor_indices: Vec<syn::Index> =
            (0..monitor_types.len()).map(syn::Index::from).collect();

        let monitor_builders: Vec<proc_macro2::TokenStream> = monitor_types
            .iter()
            .zip(monitor_bindings.iter())
            .zip(monitor_indices.iter())
            .map(|((monitor_ty, monitor_binding), monitor_idx)| {
                quote! {
                    let __cu_monitor_cfg_entry = config
                        .get_monitor_configs()
                        .get(#monitor_idx)
                        .and_then(|entry| entry.get_config().cloned());
                    let __cu_monitor_metadata = metadata
                        .clone()
                        .with_monitor_config(__cu_monitor_cfg_entry)
                        .with_subsystem_id(#subsystem_id_tokens);
                    let #monitor_binding = #monitor_ty::new(__cu_monitor_metadata, runtime.clone())
                    .expect("Failed to create one of the configured monitors.");
                }
            })
            .collect();
        let tuple_type: TypeTuple = parse_quote! { (#(#monitor_types),*,) };
        (
            quote! { #tuple_type },
            quote! {
                #(#monitor_builders)*
                let monitor: #tuple_type = (#(#monitor_bindings),*,);
                monitor
            },
        )
    };

    // This is common for all the mission as it will be inserted in the respective modules with their local CuTasks, CuStampedDataSet etc...
    #[cfg(feature = "macro_debug")]
    eprintln!("[build runtime field]");
    // add that to a new field
    let runtime_field: Field = if sim_mode {
        parse_quote! {
            copper_runtime: cu29::curuntime::CuRuntime<CuSimTasks, CuBridges, CuStampedDataSet, #monitor_type, #copperlist_count_tokens>
        }
    } else {
        parse_quote! {
            copper_runtime: cu29::curuntime::CuRuntime<CuTasks, CuBridges, CuStampedDataSet, #monitor_type, #copperlist_count_tokens>
        }
    };
    let lifecycle_stream_field: Field = parse_quote! {
        runtime_lifecycle_stream: Option<Box<dyn WriteStream<RuntimeLifecycleRecord>>>
    };
    let logger_runtime_field: Field = parse_quote! {
        logger_runtime: cu29::prelude::LoggerRuntime
    };

    #[cfg(feature = "macro_debug")]
    eprintln!("[match struct anonymity]");
    match &mut application_struct.fields {
        Named(fields_named) => {
            fields_named.named.push(runtime_field);
            fields_named.named.push(lifecycle_stream_field);
            fields_named.named.push(logger_runtime_field);
        }
        Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.push(runtime_field);
            fields_unnamed.unnamed.push(lifecycle_stream_field);
            fields_unnamed.unnamed.push(logger_runtime_field);
        }
        Fields::Unit => {
            panic!(
                "This struct is a unit struct, it should have named or unnamed fields. use struct Something {{}} and not struct Something;"
            )
        }
    };

    let all_missions = copper_config.graphs.get_all_missions_graphs();
    let mut all_missions_tokens = Vec::<proc_macro2::TokenStream>::new();
    for (mission, graph) in &all_missions {
        let git_commit_tokens = git_commit_tokens.clone();
        let git_dirty_tokens = git_dirty_tokens.clone();
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
        let task_names = collect_task_names(graph);
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
            &copper_config,
            Some(mission.as_str()),
            &culist_plan,
            &culist_call_order,
            &node_output_positions,
            &task_names,
            &culist_bridge_specs,
        );

        let (
            threadpool_bundle_index,
            resources_module,
            resources_instanciator_fn,
            task_resource_mappings,
            bridge_resource_mappings,
        ) = if ignore_resources {
            if task_specs.background_flags.iter().any(|&flag| flag) {
                return return_error(
                    "`ignore_resources` cannot be used with background tasks because they require the threadpool resource bundle"
                        .to_string(),
                );
            }

            let bundle_specs: Vec<BundleSpec> = Vec::new();
            let resource_specs: Vec<ResourceKeySpec> = Vec::new();
            let (resources_module, resources_instanciator_fn) =
                match build_resources_module(&bundle_specs) {
                    Ok(tokens) => tokens,
                    Err(e) => return return_error(e.to_string()),
                };
            let task_resource_mappings =
                match build_task_resource_mappings(&resource_specs, &task_specs) {
                    Ok(tokens) => tokens,
                    Err(e) => return return_error(e.to_string()),
                };
            let bridge_resource_mappings =
                build_bridge_resource_mappings(&resource_specs, &culist_bridge_specs, sim_mode);
            (
                None,
                resources_module,
                resources_instanciator_fn,
                task_resource_mappings,
                bridge_resource_mappings,
            )
        } else {
            let bundle_specs = match build_bundle_specs(&copper_config, mission.as_str()) {
                Ok(specs) => specs,
                Err(e) => return return_error(e.to_string()),
            };
            let threadpool_bundle_index = if task_specs.background_flags.iter().any(|&flag| flag) {
                match bundle_specs
                    .iter()
                    .position(|bundle| bundle.id == "threadpool")
                {
                    Some(index) => Some(index),
                    None => {
                        return return_error(
                            "Background tasks require the threadpool bundle to be configured"
                                .to_string(),
                        );
                    }
                }
            } else {
                None
            };

            let resource_specs = match collect_resource_specs(
                graph,
                &task_specs,
                &culist_bridge_specs,
                &bundle_specs,
            ) {
                Ok(specs) => specs,
                Err(e) => return return_error(e.to_string()),
            };

            let (resources_module, resources_instanciator_fn) =
                match build_resources_module(&bundle_specs) {
                    Ok(tokens) => tokens,
                    Err(e) => return return_error(e.to_string()),
                };
            let task_resource_mappings =
                match build_task_resource_mappings(&resource_specs, &task_specs) {
                    Ok(tokens) => tokens,
                    Err(e) => return return_error(e.to_string()),
                };
            let bridge_resource_mappings =
                build_bridge_resource_mappings(&resource_specs, &culist_bridge_specs, sim_mode);
            (
                threadpool_bundle_index,
                resources_module,
                resources_instanciator_fn,
                task_resource_mappings,
                bridge_resource_mappings,
            )
        };

        let task_ids = task_specs.ids.clone();
        let ids = build_monitored_ids(&task_ids, &mut culist_bridge_specs);
        let parallel_rt_stage_entries = match build_parallel_rt_stage_entries(
            &culist_plan,
            &culist_exec_entities,
            &task_specs,
            &culist_bridge_specs,
        ) {
            Ok(entries) => entries,
            Err(e) => return return_error(e.to_string()),
        };
        let parallel_rt_metadata_defs = if std && parallel_rt_enabled {
            Some(quote! {
                pub const PARALLEL_RT_STAGES: &'static [cu29::parallel_rt::ParallelRtStageMetadata] =
                    &[#( #parallel_rt_stage_entries ),*];
                pub const PARALLEL_RT_METADATA: cu29::parallel_rt::ParallelRtMetadata =
                    cu29::parallel_rt::ParallelRtMetadata::new(PARALLEL_RT_STAGES);
            })
        } else {
            None
        };
        let monitored_component_entries: Vec<proc_macro2::TokenStream> = ids
            .iter()
            .enumerate()
            .map(|(idx, id)| {
                let id_lit = LitStr::new(id, Span::call_site());
                if idx < task_specs.task_types.len() {
                    let task_ty = &task_specs.task_types[idx];
                    let component_type = match task_specs.cutypes[idx] {
                        CuTaskType::Source => quote! { cu29::monitoring::ComponentType::Source },
                        CuTaskType::Regular => quote! { cu29::monitoring::ComponentType::Task },
                        CuTaskType::Sink => quote! { cu29::monitoring::ComponentType::Sink },
                    };
                    quote! {
                        cu29::monitoring::MonitorComponentMetadata::new(
                            #id_lit,
                            #component_type,
                            Some(stringify!(#task_ty)),
                        )
                    }
                } else {
                    quote! {
                        cu29::monitoring::MonitorComponentMetadata::new(
                            #id_lit,
                            cu29::monitoring::ComponentType::Bridge,
                            None,
                        )
                    }
                }
            })
            .collect();
        let culist_component_mapping = match build_monitor_culist_component_mapping(
            &culist_plan,
            &culist_exec_entities,
            &culist_bridge_specs,
        ) {
            Ok(mapping) => mapping,
            Err(e) => return return_error(e),
        };

        let task_reflect_read_arms: Vec<proc_macro2::TokenStream> = task_specs
            .ids
            .iter()
            .enumerate()
            .map(|(index, task_id)| {
                let task_index = syn::Index::from(index);
                let task_id_lit = LitStr::new(task_id, Span::call_site());
                quote! {
                    #task_id_lit => Some(&self.copper_runtime.tasks.#task_index as &dyn cu29::reflect::Reflect),
                }
            })
            .collect();

        let task_reflect_write_arms: Vec<proc_macro2::TokenStream> = task_specs
            .ids
            .iter()
            .enumerate()
            .map(|(index, task_id)| {
                let task_index = syn::Index::from(index);
                let task_id_lit = LitStr::new(task_id, Span::call_site());
                quote! {
                    #task_id_lit => Some(&mut self.copper_runtime.tasks.#task_index as &mut dyn cu29::reflect::Reflect),
                }
            })
            .collect();

        let mut reflect_registry_types: BTreeMap<String, Type> = BTreeMap::new();
        let mut add_reflect_type = |ty: Type| {
            let key = quote! { #ty }.to_string();
            reflect_registry_types.entry(key).or_insert(ty);
        };

        for task_type in &task_specs.task_types {
            add_reflect_type(task_type.clone());
        }

        let mut sim_bridge_channel_decls = Vec::<proc_macro2::TokenStream>::new();
        let bridge_runtime_types: Vec<Type> = culist_bridge_specs
            .iter()
            .map(|spec| {
                if sim_mode && !spec.run_in_sim {
                    let (tx_set_ident, tx_id_ident, rx_set_ident, rx_id_ident) =
                        sim_bridge_channel_set_idents(spec.tuple_index);

                    if !spec.tx_channels.is_empty() {
                        let tx_entries = spec.tx_channels.iter().map(|channel| {
                            let entry_ident = Ident::new(
                                &channel.const_ident.to_string().to_lowercase(),
                                Span::call_site(),
                            );
                            let msg_type = &channel.msg_type;
                            quote! { #entry_ident => #msg_type, }
                        });
                        sim_bridge_channel_decls.push(quote! {
                            cu29::tx_channels! {
                                pub struct #tx_set_ident : #tx_id_ident {
                                    #(#tx_entries)*
                                }
                            }
                        });
                    }

                    if !spec.rx_channels.is_empty() {
                        let rx_entries = spec.rx_channels.iter().map(|channel| {
                            let entry_ident = Ident::new(
                                &channel.const_ident.to_string().to_lowercase(),
                                Span::call_site(),
                            );
                            let msg_type = &channel.msg_type;
                            quote! { #entry_ident => #msg_type, }
                        });
                        sim_bridge_channel_decls.push(quote! {
                            cu29::rx_channels! {
                                pub struct #rx_set_ident : #rx_id_ident {
                                    #(#rx_entries)*
                                }
                            }
                        });
                    }
                }
                runtime_bridge_type_for_spec(spec, sim_mode)
            })
            .collect();
        let sim_bridge_channel_defs = quote! { #(#sim_bridge_channel_decls)* };

        for (bridge_index, bridge_spec) in culist_bridge_specs.iter().enumerate() {
            add_reflect_type(bridge_runtime_types[bridge_index].clone());
            for channel in bridge_spec
                .rx_channels
                .iter()
                .chain(bridge_spec.tx_channels.iter())
            {
                add_reflect_type(channel.msg_type.clone());
            }
        }

        for output_pack in extract_output_packs(&culist_plan) {
            for msg_type in output_pack.msg_types {
                add_reflect_type(msg_type);
            }
        }

        let reflect_type_registration_calls: Vec<proc_macro2::TokenStream> = reflect_registry_types
            .values()
            .map(|ty| {
                quote! {
                    registry.register::<#ty>();
                }
            })
            .collect();

        let bridges_type_tokens: proc_macro2::TokenStream = if bridge_runtime_types.is_empty() {
            quote! { () }
        } else {
            let bridge_types_for_tuple = bridge_runtime_types.clone();
            let tuple: TypeTuple = parse_quote! { (#(#bridge_types_for_tuple),*,) };
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
                let bridge_mapping_ref = bridge_resource_mappings.refs[idx].clone();
                let bridge_type = &bridge_runtime_types[idx];
                let bridge_name = spec.id.clone();
                let config_index = syn::Index::from(spec.config_index);
                let binding_error = LitStr::new(
                    &format!("Failed to bind resources for bridge '{}'", bridge_name),
                    Span::call_site(),
                );
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
                        let bridge_mapping = #bridge_mapping_ref;
                        let bridge_resources = <<#bridge_type as cu29::cubridge::CuBridge>::Resources<'_> as ResourceBindings>::from_bindings(
                            resources,
                            bridge_mapping,
                        )
                        .map_err(|e| cu29::CuError::new_with_cause(#binding_error, e))?;
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
                            bridge_resources,
                        )?
                    };
                }
            })
            .collect();

        let bridges_instanciator = if culist_bridge_specs.is_empty() {
            quote! {
                pub fn bridges_instanciator(_config: &CuConfig, resources: &mut ResourceManager) -> CuResult<CuBridges> {
                    let _ = resources;
                    Ok(())
                }
            }
        } else {
            let bridge_bindings = bridge_binding_idents.clone();
            quote! {
                pub fn bridges_instanciator(config: &CuConfig, resources: &mut ResourceManager) -> CuResult<CuBridges> {
                    #(#bridge_init_statements)*
                    Ok((#(#bridge_bindings),*,))
                }
            }
        };

        let all_sim_tasks_types: Vec<Type> = task_specs
            .ids
            .iter()
            .zip(&task_specs.cutypes)
            .zip(&task_specs.sim_task_types)
            .zip(&task_specs.background_flags)
            .zip(&task_specs.run_in_sim_flags)
            .zip(task_specs.output_types.iter())
            .map(|(((((task_id, task_type), sim_type), background), run_in_sim), output_type)| {
                match task_type {
                    CuTaskType::Source => {
                        if *background {
                            if let Some(out_ty) = output_type {
                                parse_quote!(CuAsyncSrcTask<#sim_type, #out_ty>)
                            } else {
                                panic!("{task_id}: If a source is background, it has to have an output");
                            }
                        } else if *run_in_sim {
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
                        if *background {
                            if let Some(out_ty) = output_type {
                                parse_quote!(CuAsyncTask<#sim_type, #out_ty>)
                            } else {
                                panic!("{task_id}: If a task is background, it has to have an output");
                            }
                        } else {
                            // run_in_sim has no effect for normal tasks, they are always run in sim as is.
                            sim_type.clone()
                        }
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
                            let msg_types = graph
                                .get_node_input_msg_types(task_id.as_str())
                                .unwrap_or_else(|| panic!("CuSinkTask {task_id} should have an incoming connection with a valid input msg type"));
                            let msg_type = if msg_types.len() == 1 {
                                format!("({},)", msg_types[0])
                            } else {
                                format!("({})", msg_types.join(", "))
                            };
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
        let task_sim_instances_init_code = all_sim_tasks_types
            .iter()
            .enumerate()
            .map(|(index, ty)| {
                let additional_error_info = format!(
                    "Failed to get create instance for {}, instance index {}.",
                    task_specs.type_names[index], index
                );
                let mapping_ref = task_resource_mappings.refs[index].clone();
                let background = task_specs.background_flags[index];
                let inner_task_type = &task_specs.sim_task_types[index];
                match task_specs.cutypes[index] {
                    CuTaskType::Source => {
                        if background {
                            let threadpool_bundle_index = threadpool_bundle_index
                                .expect("threadpool bundle missing for background tasks");
                            quote! {
                                {
                                    let inner_resources = <<#inner_task_type as CuSrcTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    let threadpool_key = cu29::resource::ResourceKey::new(
                                        cu29::resource::BundleIndex::new(#threadpool_bundle_index),
                                        <cu29::resource::ThreadPoolBundle as cu29::resource::ResourceBundleDecl>::Id::BgThreads as usize,
                                    );
                                    let threadpool = resources.borrow_shared_arc(threadpool_key)?;
                                    let resources = cu29::cuasynctask::CuAsyncSrcTaskResources {
                                        inner: inner_resources,
                                        threadpool,
                                    };
                                    <#ty as CuSrcTask>::new(all_instances_configs[#index], resources)
                                        .map_err(|e| e.add_cause(#additional_error_info))?
                                }
                            }
                        } else {
                            quote! {
                                {
                                    let resources = <<#ty as CuSrcTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    <#ty as CuSrcTask>::new(all_instances_configs[#index], resources)
                                        .map_err(|e| e.add_cause(#additional_error_info))?
                                }
                            }
                        }
                    }
                    CuTaskType::Regular => {
                        if background {
                            let threadpool_bundle_index = threadpool_bundle_index
                                .expect("threadpool bundle missing for background tasks");
                            quote! {
                                {
                                    let inner_resources = <<#inner_task_type as CuTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    let threadpool_key = cu29::resource::ResourceKey::new(
                                        cu29::resource::BundleIndex::new(#threadpool_bundle_index),
                                        <cu29::resource::ThreadPoolBundle as cu29::resource::ResourceBundleDecl>::Id::BgThreads as usize,
                                    );
                                    let threadpool = resources.borrow_shared_arc(threadpool_key)?;
                                    let resources = cu29::cuasynctask::CuAsyncTaskResources {
                                        inner: inner_resources,
                                        threadpool,
                                    };
                                    <#ty as CuTask>::new(all_instances_configs[#index], resources)
                                        .map_err(|e| e.add_cause(#additional_error_info))?
                                }
                            }
                        } else {
                            quote! {
                                {
                                    let resources = <<#ty as CuTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    <#ty as CuTask>::new(all_instances_configs[#index], resources)
                                        .map_err(|e| e.add_cause(#additional_error_info))?
                                }
                            }
                        }
                    }
                    CuTaskType::Sink => quote! {
                        {
                            let resources = <<#ty as CuSinkTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                resources,
                                #mapping_ref,
                            ).map_err(|e| e.add_cause(#additional_error_info))?;
                            <#ty as CuSinkTask>::new(all_instances_configs[#index], resources)
                                .map_err(|e| e.add_cause(#additional_error_info))?
                        }
                    },
                }
            })
            .collect::<Vec<_>>();

        let task_instances_init_code = task_specs
            .instantiation_types
            .iter()
            .zip(&task_specs.background_flags)
            .enumerate()
            .map(|(index, (task_type, background))| {
                let additional_error_info = format!(
                    "Failed to get create instance for {}, instance index {}.",
                    task_specs.type_names[index], index
                );
                let mapping_ref = task_resource_mappings.refs[index].clone();
                let inner_task_type = &task_specs.sim_task_types[index];
                match task_specs.cutypes[index] {
                    CuTaskType::Source => {
                        if *background {
                            let threadpool_bundle_index = threadpool_bundle_index
                                .expect("threadpool bundle missing for background tasks");
                            quote! {
                                {
                                    let inner_resources = <<#inner_task_type as CuSrcTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    let threadpool_key = cu29::resource::ResourceKey::new(
                                        cu29::resource::BundleIndex::new(#threadpool_bundle_index),
                                        <cu29::resource::ThreadPoolBundle as cu29::resource::ResourceBundleDecl>::Id::BgThreads as usize,
                                    );
                                    let threadpool = resources.borrow_shared_arc(threadpool_key)?;
                                    let resources = cu29::cuasynctask::CuAsyncSrcTaskResources {
                                        inner: inner_resources,
                                        threadpool,
                                    };
                                    <#task_type as CuSrcTask>::new(all_instances_configs[#index], resources)
                                        .map_err(|e| e.add_cause(#additional_error_info))?
                                }
                            }
                        } else {
                            quote! {
                                {
                                    let resources = <<#task_type as CuSrcTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    <#task_type as CuSrcTask>::new(all_instances_configs[#index], resources)
                                        .map_err(|e| e.add_cause(#additional_error_info))?
                                }
                            }
                        }
                    }
                    CuTaskType::Regular => {
                        if *background {
                            let threadpool_bundle_index = threadpool_bundle_index
                                .expect("threadpool bundle missing for background tasks");
                            quote! {
                                {
                                    let inner_resources = <<#inner_task_type as CuTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    let threadpool_key = cu29::resource::ResourceKey::new(
                                        cu29::resource::BundleIndex::new(#threadpool_bundle_index),
                                        <cu29::resource::ThreadPoolBundle as cu29::resource::ResourceBundleDecl>::Id::BgThreads as usize,
                                    );
                                    let threadpool = resources.borrow_shared_arc(threadpool_key)?;
                                    let resources = cu29::cuasynctask::CuAsyncTaskResources {
                                        inner: inner_resources,
                                        threadpool,
                                    };
                                    <#task_type as CuTask>::new(all_instances_configs[#index], resources)
                                        .map_err(|e| e.add_cause(#additional_error_info))?
                                }
                            }
                        } else {
                            quote! {
                                {
                                    let resources = <<#task_type as CuTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    <#task_type as CuTask>::new(all_instances_configs[#index], resources)
                                        .map_err(|e| e.add_cause(#additional_error_info))?
                                }
                            }
                        }
                    }
                    CuTaskType::Sink => quote! {
                        {
                            let resources = <<#task_type as CuSinkTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                resources,
                                #mapping_ref,
                            ).map_err(|e| e.add_cause(#additional_error_info))?;
                            <#task_type as CuSinkTask>::new(all_instances_configs[#index], resources)
                                .map_err(|e| e.add_cause(#additional_error_info))?
                        }
                    },
                }
            })
            .collect::<Vec<_>>();

        let mut keyframe_task_restore_order = Vec::new();
        for unit in &culist_plan.steps {
            let CuExecutionUnit::Step(step) = unit else {
                panic!("Execution loops are not supported in runtime generation");
            };
            let ExecutionEntityKind::Task { task_index } =
                &culist_exec_entities[step.node_id as usize].kind
            else {
                continue;
            };
            if !keyframe_task_restore_order.contains(task_index) {
                keyframe_task_restore_order.push(*task_index);
            }
        }
        if keyframe_task_restore_order.len() != task_specs.task_types.len() {
            return return_error(format!(
                "Keyframe restore order covers {} task steps but mission declares {} tasks",
                keyframe_task_restore_order.len(),
                task_specs.task_types.len()
            ));
        }
        let task_restore_code: Vec<proc_macro2::TokenStream> = keyframe_task_restore_order
            .iter()
            .map(|index| {
                let task_tuple_index = syn::Index::from(*index);
                quote! {
                    tasks.#task_tuple_index.thaw(&mut decoder).map_err(|e| CuError::from("Failed to thaw").add_cause(&e.to_string()))?
                }
            })
            .collect();

        // Generate the code to create instances of the nodes
        // It maps the types to their index
        let (
            task_start_calls,
            task_stop_calls,
            task_preprocess_calls,
            task_postprocess_calls,
        ): (Vec<_>, Vec<_>, Vec<_>, Vec<_>) = itertools::multiunzip(
            (0..task_specs.task_types.len())
            .map(|index| {
                let task_index = int2sliceindex(index as u32);
                let task_enum_name = config_id_to_enum(&task_specs.ids[index]);
                let enum_name = Ident::new(&task_enum_name, Span::call_site());
                (
                    {  // Start calls
                        let monitoring_action = quote! {
                            let decision = self.copper_runtime.monitor.process_error(cu29::monitoring::ComponentId::new(#index), CuComponentState::Start, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Start: ABORT decision from monitoring. Component '{}' errored out \
                                during start. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                    return Ok(());

                                }
                                Decision::Ignore => {
                                    debug!("Start: IGNORE decision from monitoring. Component '{}' errored out \
                                during start. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                }
                                Decision::Shutdown => {
                                    debug!("Start: SHUTDOWN decision from monitoring. Component '{}' errored out \
                                during start. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                    return Err(CuError::new_with_cause("Component errored out during start.", error));
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
                                self.copper_runtime.record_execution_marker(
                                    cu29::monitoring::ExecutionMarker {
                                        component_id: cu29::monitoring::ComponentId::new(#index),
                                        step: CuComponentState::Start,
                                        culistid: None,
                                    }
                                );
                                let task = &mut self.copper_runtime.tasks.#task_index;
                                ctx.set_current_task(#index);
                                if let Err(error) = task.start(&ctx) {
                                    #monitoring_action
                                }
                            }
                        }
                    },
                    {  // Stop calls
                        let monitoring_action = quote! {
                                    let decision = self.copper_runtime.monitor.process_error(cu29::monitoring::ComponentId::new(#index), CuComponentState::Stop, &error);
                                    match decision {
                                        Decision::Abort => {
                                            debug!("Stop: ABORT decision from monitoring. Component '{}' errored out \
                                    during stop. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                            return Ok(());

                                        }
                                        Decision::Ignore => {
                                            debug!("Stop: IGNORE decision from monitoring. Component '{}' errored out \
                                    during stop. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                        }
                                        Decision::Shutdown => {
                                            debug!("Stop: SHUTDOWN decision from monitoring. Component '{}' errored out \
                                    during stop. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                            return Err(CuError::new_with_cause("Component errored out during stop.", error));
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
                                self.copper_runtime.record_execution_marker(
                                    cu29::monitoring::ExecutionMarker {
                                        component_id: cu29::monitoring::ComponentId::new(#index),
                                        step: CuComponentState::Stop,
                                        culistid: None,
                                    }
                                );
                                let task = &mut self.copper_runtime.tasks.#task_index;
                                ctx.set_current_task(#index);
                                if let Err(error) = task.stop(&ctx) {
                                    #monitoring_action
                                }
                            }
                        }
                    },
                    {  // Preprocess calls
                        let monitoring_action = quote! {
                            let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#index), CuComponentState::Preprocess, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Preprocess: ABORT decision from monitoring. Component '{}' errored out \
                                during preprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                    return Ok(());

                                }
                                Decision::Ignore => {
                                    debug!("Preprocess: IGNORE decision from monitoring. Component '{}' errored out \
                                during preprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                }
                                Decision::Shutdown => {
                                    debug!("Preprocess: SHUTDOWN decision from monitoring. Component '{}' errored out \
                                during preprocess. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                    return Err(CuError::new_with_cause("Component errored out during preprocess.", error));
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
                                execution_probe.record(cu29::monitoring::ExecutionMarker {
                                    component_id: cu29::monitoring::ComponentId::new(#index),
                                    step: CuComponentState::Preprocess,
                                    culistid: None,
                                });
                                ctx.set_current_task(#index);
                                let maybe_error = {
                                    #rt_guard
                                    tasks.#task_index.preprocess(&ctx)
                                };
                                if let Err(error) = maybe_error {
                                    #monitoring_action
                                }
                            }
                        }
                    },
                    {  // Postprocess calls
                        let monitoring_action = quote! {
                            let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#index), CuComponentState::Postprocess, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Postprocess: ABORT decision from monitoring. Component '{}' errored out \
                                during postprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                    return Ok(());

                                }
                                Decision::Ignore => {
                                    debug!("Postprocess: IGNORE decision from monitoring. Component '{}' errored out \
                                during postprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                }
                                Decision::Shutdown => {
                                    debug!("Postprocess: SHUTDOWN decision from monitoring. Component '{}' errored out \
                                during postprocess. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                    return Err(CuError::new_with_cause("Component errored out during postprocess.", error));
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
                                execution_probe.record(cu29::monitoring::ExecutionMarker {
                                    component_id: cu29::monitoring::ComponentId::new(#index),
                                    step: CuComponentState::Postprocess,
                                    culistid: None,
                                });
                                ctx.set_current_task(#index);
                                let maybe_error = {
                                    #rt_guard
                                    tasks.#task_index.postprocess(&ctx)
                                };
                                if let Err(error) = maybe_error {
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
                let enum_ident = Ident::new(
                    &config_id_to_enum(&format!("{}_bridge", spec.id)),
                    Span::call_site(),
                );
                let call_sim = if sim_mode {
                    quote! {
                        let doit = {
                            let state = SimStep::#enum_ident(cu29::simulation::CuBridgeLifecycleState::Start);
                            let ovr = sim_callback(state);
                            if let SimOverride::Errored(reason) = ovr {
                                let error: CuError = reason.into();
                                let decision = self.copper_runtime.monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Start, &error);
                                match decision {
                                    Decision::Abort => { debug!("Start: ABORT decision from monitoring. Component '{}' errored out during start. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Ok(()); }
                                    Decision::Ignore => { debug!("Start: IGNORE decision from monitoring. Component '{}' errored out during start. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); false }
                                    Decision::Shutdown => { debug!("Start: SHUTDOWN decision from monitoring. Component '{}' errored out during start. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Err(CuError::new_with_cause("Component errored out during start.", error)); }
                                }
                            } else {
                                ovr == SimOverride::ExecuteByRuntime
                            }
                        };
                    }
                } else {
                    quote! { let doit = true; }
                };
                quote! {
                    {
                        #call_sim
                        if !doit { return Ok(()); }
                        self.copper_runtime.record_execution_marker(
                            cu29::monitoring::ExecutionMarker {
                                component_id: cu29::monitoring::ComponentId::new(#monitor_index),
                                step: CuComponentState::Start,
                                culistid: None,
                            }
                        );
                        ctx.clear_current_task();
                        let bridge = &mut self.copper_runtime.bridges.#bridge_index;
                        if let Err(error) = bridge.start(&ctx) {
                            let decision = self.copper_runtime.monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Start, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Start: ABORT decision from monitoring. Component '{}' errored out during start. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                    return Ok(());
                                }
                                Decision::Ignore => {
                                    debug!("Start: IGNORE decision from monitoring. Component '{}' errored out during start. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                }
                                Decision::Shutdown => {
                                    debug!("Start: SHUTDOWN decision from monitoring. Component '{}' errored out during start. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                    return Err(CuError::new_with_cause("Component errored out during start.", error));
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
                let enum_ident = Ident::new(
                    &config_id_to_enum(&format!("{}_bridge", spec.id)),
                    Span::call_site(),
                );
                let call_sim = if sim_mode {
                    quote! {
                        let doit = {
                            let state = SimStep::#enum_ident(cu29::simulation::CuBridgeLifecycleState::Stop);
                            let ovr = sim_callback(state);
                            if let SimOverride::Errored(reason) = ovr {
                                let error: CuError = reason.into();
                                let decision = self.copper_runtime.monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Stop, &error);
                                match decision {
                                    Decision::Abort => { debug!("Stop: ABORT decision from monitoring. Component '{}' errored out during stop. Aborting all the other stops.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Ok(()); }
                                    Decision::Ignore => { debug!("Stop: IGNORE decision from monitoring. Component '{}' errored out during stop. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); false }
                                    Decision::Shutdown => { debug!("Stop: SHUTDOWN decision from monitoring. Component '{}' errored out during stop. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Err(CuError::new_with_cause("Component errored out during stop.", error)); }
                                }
                            } else {
                                ovr == SimOverride::ExecuteByRuntime
                            }
                        };
                    }
                } else {
                    quote! { let doit = true; }
                };
                quote! {
                    {
                        #call_sim
                        if !doit { return Ok(()); }
                        self.copper_runtime.record_execution_marker(
                            cu29::monitoring::ExecutionMarker {
                                component_id: cu29::monitoring::ComponentId::new(#monitor_index),
                                step: CuComponentState::Stop,
                                culistid: None,
                            }
                        );
                        ctx.clear_current_task();
                        let bridge = &mut self.copper_runtime.bridges.#bridge_index;
                        if let Err(error) = bridge.stop(&ctx) {
                            let decision = self.copper_runtime.monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Stop, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!("Stop: ABORT decision from monitoring. Component '{}' errored out during stop. Aborting all the other stops.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                    return Ok(());
                                }
                                Decision::Ignore => {
                                    debug!("Stop: IGNORE decision from monitoring. Component '{}' errored out during stop. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                }
                                Decision::Shutdown => {
                                    debug!("Stop: SHUTDOWN decision from monitoring. Component '{}' errored out during stop. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                    return Err(CuError::new_with_cause("Component errored out during stop.", error));
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
                let enum_ident = Ident::new(
                    &config_id_to_enum(&format!("{}_bridge", spec.id)),
                    Span::call_site(),
                );
                let call_sim = if sim_mode {
                    quote! {
                        let doit = {
                            let state = SimStep::#enum_ident(cu29::simulation::CuBridgeLifecycleState::Preprocess);
                            let ovr = sim_callback(state);
                            if let SimOverride::Errored(reason) = ovr {
                                let error: CuError = reason.into();
                                let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Preprocess, &error);
                                match decision {
                                    Decision::Abort => { debug!("Preprocess: ABORT decision from monitoring. Component '{}' errored out during preprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Ok(()); }
                                    Decision::Ignore => { debug!("Preprocess: IGNORE decision from monitoring. Component '{}' errored out during preprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); false }
                                    Decision::Shutdown => { debug!("Preprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during preprocess. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Err(CuError::new_with_cause("Component errored out during preprocess.", error)); }
                                }
                            } else {
                                ovr == SimOverride::ExecuteByRuntime
                            }
                        };
                    }
                } else {
                    quote! { let doit = true; }
                };
                quote! {
                    {
                        #call_sim
                        if doit {
                            ctx.clear_current_task();
                            let bridge = &mut __cu_bridges.#bridge_index;
                            execution_probe.record(cu29::monitoring::ExecutionMarker {
                                component_id: cu29::monitoring::ComponentId::new(#monitor_index),
                                step: CuComponentState::Preprocess,
                                culistid: None,
                            });
                            let maybe_error = {
                                #rt_guard
                                bridge.preprocess(&ctx)
                            };
                            if let Err(error) = maybe_error {
                                let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Preprocess, &error);
                                match decision {
                                    Decision::Abort => {
                                        debug!("Preprocess: ABORT decision from monitoring. Component '{}' errored out during preprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                        return Ok(());
                                    }
                                    Decision::Ignore => {
                                        debug!("Preprocess: IGNORE decision from monitoring. Component '{}' errored out during preprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                    }
                                    Decision::Shutdown => {
                                        debug!("Preprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during preprocess. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                        return Err(CuError::new_with_cause("Component errored out during preprocess.", error));
                                    }
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
                let enum_ident = Ident::new(
                    &config_id_to_enum(&format!("{}_bridge", spec.id)),
                    Span::call_site(),
                );
                let call_sim = if sim_mode {
                    quote! {
                        let doit = {
                            let state = SimStep::#enum_ident(cu29::simulation::CuBridgeLifecycleState::Postprocess);
                            let ovr = sim_callback(state);
                            if let SimOverride::Errored(reason) = ovr {
                                let error: CuError = reason.into();
                                let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Postprocess, &error);
                                match decision {
                                    Decision::Abort => { debug!("Postprocess: ABORT decision from monitoring. Component '{}' errored out during postprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Ok(()); }
                                    Decision::Ignore => { debug!("Postprocess: IGNORE decision from monitoring. Component '{}' errored out during postprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); false }
                                    Decision::Shutdown => { debug!("Postprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during postprocess. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Err(CuError::new_with_cause("Component errored out during postprocess.", error)); }
                                }
                            } else {
                                ovr == SimOverride::ExecuteByRuntime
                            }
                        };
                    }
                } else {
                    quote! { let doit = true; }
                };
                quote! {
                    {
                        #call_sim
                        if doit {
                            ctx.clear_current_task();
                            let bridge = &mut __cu_bridges.#bridge_index;
                            kf_manager.freeze_any(clid, bridge)?;
                            execution_probe.record(cu29::monitoring::ExecutionMarker {
                                component_id: cu29::monitoring::ComponentId::new(#monitor_index),
                                step: CuComponentState::Postprocess,
                                culistid: Some(clid),
                            });
                            let maybe_error = {
                                #rt_guard
                                bridge.postprocess(&ctx)
                            };
                            if let Err(error) = maybe_error {
                                let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Postprocess, &error);
                                match decision {
                                    Decision::Abort => {
                                        debug!("Postprocess: ABORT decision from monitoring. Component '{}' errored out during postprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                        return Ok(());
                                    }
                                    Decision::Ignore => {
                                        debug!("Postprocess: IGNORE decision from monitoring. Component '{}' errored out during postprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                    }
                                    Decision::Shutdown => {
                                        debug!("Postprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during postprocess. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                        return Err(CuError::new_with_cause("Component errored out during postprocess.", error));
                                    }
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
        let parallel_rt_run_supported = std && parallel_rt_enabled && !sim_mode;

        // Bridges are frozen alongside tasks; restore them in the same order.
        let bridge_restore_code: Vec<proc_macro2::TokenStream> = culist_bridge_specs
            .iter()
            .enumerate()
            .map(|(index, _)| {
                let bridge_tuple_index = syn::Index::from(index);
                quote! {
                    __cu_bridges.#bridge_tuple_index
                        .thaw(&mut decoder)
                        .map_err(|e| CuError::from("Failed to thaw bridge").add_cause(&e.to_string()))?
                }
            })
            .collect();

        let output_pack_sizes = collect_output_pack_sizes(&culist_plan);
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
                        step.output_msg_pack
                    );

                    match &culist_exec_entities[step.node_id as usize].kind {
                        ExecutionEntityKind::Task { task_index } => generate_task_execution_tokens(
                            step,
                            *task_index,
                            &task_specs,
                            StepGenerationContext::new(
                                &output_pack_sizes,
                                sim_mode,
                                &mission_mod,
                                ParallelLifecyclePlacement::default(),
                                false,
                            ),
                            TaskExecutionTokens::new(quote! {}, {
                                let node_index = int2sliceindex(*task_index as u32);
                                quote! { tasks.#node_index }
                            }),
                        ),
                        ExecutionEntityKind::BridgeRx {
                            bridge_index,
                            channel_index,
                        } => {
                            let spec = &culist_bridge_specs[*bridge_index];
                            generate_bridge_rx_execution_tokens(
                                step,
                                spec,
                                *channel_index,
                                StepGenerationContext::new(
                                    &output_pack_sizes,
                                    sim_mode,
                                    &mission_mod,
                                    ParallelLifecyclePlacement::default(),
                                    false,
                                ),
                                {
                                    let bridge_tuple_index =
                                        int2sliceindex(spec.tuple_index as u32);
                                    quote! { let bridge = &mut __cu_bridges.#bridge_tuple_index; }
                                },
                            )
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
                                StepGenerationContext::new(
                                    &output_pack_sizes,
                                    sim_mode,
                                    &mission_mod,
                                    ParallelLifecyclePlacement::default(),
                                    false,
                                ),
                                {
                                    let bridge_tuple_index =
                                        int2sliceindex(spec.tuple_index as u32);
                                    quote! { let bridge = &mut __cu_bridges.#bridge_tuple_index; }
                                },
                            )
                        }
                    }
                }
                CuExecutionUnit::Loop(_) => {
                    panic!("Execution loops are not supported in runtime generation");
                }
            })
            .collect();
        let parallel_lifecycle_placements = if parallel_rt_run_supported {
            Some(build_parallel_lifecycle_placements(
                &culist_plan,
                &culist_exec_entities,
            ))
        } else {
            None
        };
        let runtime_plan_parallel_code_and_logging: Option<
            Vec<(proc_macro2::TokenStream, proc_macro2::TokenStream)>,
        > = if parallel_rt_run_supported {
            Some(
                culist_plan
                    .steps
                    .iter()
                    .enumerate()
                    .map(|(step_index, unit)| match unit {
                        CuExecutionUnit::Step(step) => match &culist_exec_entities
                            [step.node_id as usize]
                            .kind
                        {
                            ExecutionEntityKind::Task { task_index } => {
                                let task_index_ts = int2sliceindex(*task_index as u32);
                                generate_task_execution_tokens(
                                    step,
                                    *task_index,
                                    &task_specs,
                                    StepGenerationContext::new(
                                        &output_pack_sizes,
                                        false,
                                        &mission_mod,
                                        parallel_lifecycle_placements
                                            .as_ref()
                                            .expect("parallel lifecycle placements missing")[step_index],
                                        true,
                                    ),
                                    TaskExecutionTokens::new(quote! {
                                        let _task_lock = step_rt.task_locks.#task_index_ts.lock().expect("parallel task lock poisoned");
                                        let task = unsafe { step_rt.task_ptrs.#task_index_ts.as_mut() };
                                    }, quote! { (*task) }),
                                )
                            }
                            ExecutionEntityKind::BridgeRx {
                                bridge_index,
                                channel_index,
                            } => {
                                let spec = &culist_bridge_specs[*bridge_index];
                                let bridge_index_ts = int2sliceindex(spec.tuple_index as u32);
                                generate_bridge_rx_execution_tokens(
                                    step,
                                    spec,
                                    *channel_index,
                                    StepGenerationContext::new(
                                        &output_pack_sizes,
                                        false,
                                        &mission_mod,
                                        parallel_lifecycle_placements
                                            .as_ref()
                                            .expect("parallel lifecycle placements missing")
                                            [step_index],
                                        true,
                                    ),
                                    quote! {
                                        let _bridge_lock = step_rt.bridge_locks.#bridge_index_ts.lock().expect("parallel bridge lock poisoned");
                                        let bridge = unsafe { step_rt.bridge_ptrs.#bridge_index_ts.as_mut() };
                                    },
                                )
                            }
                            ExecutionEntityKind::BridgeTx {
                                bridge_index,
                                channel_index,
                            } => {
                                let spec = &culist_bridge_specs[*bridge_index];
                                let bridge_index_ts = int2sliceindex(spec.tuple_index as u32);
                                generate_bridge_tx_execution_tokens(
                                    step,
                                    spec,
                                    *channel_index,
                                    StepGenerationContext::new(
                                        &output_pack_sizes,
                                        false,
                                        &mission_mod,
                                        parallel_lifecycle_placements
                                            .as_ref()
                                            .expect("parallel lifecycle placements missing")[step_index],
                                        true,
                                    ),
                                    quote! {
                                        let _bridge_lock = step_rt.bridge_locks.#bridge_index_ts.lock().expect("parallel bridge lock poisoned");
                                        let bridge = unsafe { step_rt.bridge_ptrs.#bridge_index_ts.as_mut() };
                                    },
                                )
                            }
                        },
                        CuExecutionUnit::Loop(_) => {
                            panic!("Execution loops are not supported in runtime generation");
                        }
                    })
                    .collect(),
            )
        } else {
            None
        };

        let sim_support = if sim_mode {
            Some(gen_sim_support(
                &culist_plan,
                &culist_exec_entities,
                &culist_bridge_specs,
            ))
        } else {
            None
        };

        let recorded_replay_support = if sim_mode {
            Some(gen_recorded_replay_support(
                &culist_plan,
                &culist_exec_entities,
                &culist_bridge_specs,
            ))
        } else {
            None
        };

        let (run_one_iteration, start_all_tasks, stop_all_tasks, run) = if sim_mode {
            (
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

        let sim_callback_on_new_bridges = culist_bridge_specs.iter().map(|spec| {
            let enum_ident = Ident::new(
                &config_id_to_enum(&format!("{}_bridge", spec.id)),
                Span::call_site(),
            );
            let cfg_index = syn::Index::from(spec.config_index);
            quote! {
                sim_callback(SimStep::#enum_ident(
                    cu29::simulation::CuBridgeLifecycleState::New(config.bridges[#cfg_index].config.clone())
                ));
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
                #(#sim_callback_on_new_bridges)*
            })
        } else {
            None
        };

        let (runtime_plan_code, preprocess_logging_calls): (Vec<_>, Vec<_>) =
            itertools::multiunzip(runtime_plan_code_and_logging);
        let process_step_tasks_type = if sim_mode {
            quote!(CuSimTasks)
        } else {
            quote!(CuTasks)
        };
        let (
            parallel_process_step_idents,
            parallel_process_step_fn_defs,
            parallel_stage_worker_spawns,
        ): (
            Vec<Ident>,
            Vec<proc_macro2::TokenStream>,
            Vec<proc_macro2::TokenStream>,
        ) = if let Some(runtime_plan_parallel_code_and_logging) =
            &runtime_plan_parallel_code_and_logging
        {
            let (runtime_plan_parallel_step_code, _): (Vec<_>, Vec<_>) =
                itertools::multiunzip(runtime_plan_parallel_code_and_logging.clone());
            let parallel_process_step_idents: Vec<Ident> = (0..runtime_plan_parallel_step_code
                .len())
                .map(|index| format_ident!("__cu_parallel_process_step_{index}"))
                .collect();
            let parallel_process_step_fn_defs: Vec<proc_macro2::TokenStream> =
                parallel_process_step_idents
                    .iter()
                    .zip(runtime_plan_parallel_step_code.iter())
                    .map(|(step_ident, step_code)| {
                        quote! {
                            #[inline(always)]
                            fn #step_ident(
                                step_rt: &mut ParallelProcessStepRuntime<'_>,
                            ) -> cu29::curuntime::ProcessStepResult {
                                let clock = step_rt.clock;
                                let execution_probe = step_rt.execution_probe;
                                let monitor = step_rt.monitor;
                                let kf_manager = ParallelKeyFrameAccessor::new(
                                    step_rt.kf_manager_ptr,
                                    step_rt.kf_lock,
                                );
                                let culist = &mut *step_rt.culist;
                                let clid = step_rt.clid;
                                let ctx = &mut step_rt.ctx;
                                let msgs = &mut culist.msgs.0;
                                #step_code
                            }
                        }
                    })
                    .collect();
            let parallel_stage_worker_spawns: Vec<proc_macro2::TokenStream> =
                parallel_process_step_idents
                    .iter()
                    .enumerate()
                    .map(|(stage_index, step_ident)| {
                        let stage_index_lit = syn::Index::from(stage_index);
                        let receiver_ident =
                            format_ident!("__cu_parallel_stage_rx_{stage_index}");
                        quote! {
                            {
                                let mut #receiver_ident = stage_receivers
                                    .next()
                                    .expect("parallel stage receiver missing");
                                let mut next_stage_tx = stage_senders.next();
                                let done_tx = done_tx.clone();
                                let shutdown = std::sync::Arc::clone(&shutdown);
                                let clock = clock.clone();
                                let instance_id = instance_id;
                                let subsystem_code = subsystem_code;
                                let execution_probe_ptr = execution_probe_ptr;
                                let monitor_ptr = monitor_ptr;
                                let task_ptrs = task_ptrs;
                                let task_locks = std::sync::Arc::clone(&task_locks);
                                let bridge_ptrs = bridge_ptrs;
                                let bridge_locks = std::sync::Arc::clone(&bridge_locks);
                                let kf_manager_ptr = kf_manager_ptr;
                                let kf_lock = std::sync::Arc::clone(&kf_lock);
                                scope.spawn(move || {
                                    loop {
                                        let job = match #receiver_ident.recv() {
                                            Ok(job) => job,
                                            Err(_) => break,
                                        };
                                        let clid = job.clid;
                                        let culist = job.culist;

                                        let terminal_result = if shutdown.load(Ordering::Acquire) {
                                            #mission_mod::ParallelWorkerResult {
                                                clid,
                                                culist: Some(culist),
                                                outcome: Err(CuError::from(
                                                    "Parallel runtime shutting down after an earlier stage failure",
                                                )),
                                                raw_payload_bytes: 0,
                                                handle_bytes: 0,
                                            }
                                        } else {
                                            match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                                                let execution_probe = unsafe { execution_probe_ptr.as_ref() };
                                                let monitor = unsafe { monitor_ptr.as_ref() };
                                                let mut culist = culist;
                                                let mut step_rt = #mission_mod::ParallelProcessStepRuntime {
                                                    clock: &clock,
                                                    execution_probe,
                                                    monitor,
                                                    task_ptrs: &task_ptrs,
                                                    task_locks: task_locks.as_ref(),
                                                    bridge_ptrs: &bridge_ptrs,
                                                    bridge_locks: bridge_locks.as_ref(),
                                                    kf_manager_ptr,
                                                    kf_lock: kf_lock.as_ref(),
                                                    culist: culist.as_mut(),
                                                    clid,
                                                    ctx: cu29::context::CuContext::from_runtime_metadata(
                                                        clock.clone(),
                                                        clid,
                                                        instance_id,
                                                        subsystem_code,
                                                        #mission_mod::TASK_IDS,
                                                    ),
                                                };
                                                let outcome = #step_ident(&mut step_rt);
                                                drop(step_rt);
                                                (culist, outcome)
                                            })) {
                                                Ok((culist, Ok(cu29::curuntime::ProcessStepOutcome::Continue))) => {
                                                    if shutdown.load(Ordering::Acquire) {
                                                        #mission_mod::ParallelWorkerResult {
                                                            clid,
                                                            culist: Some(culist),
                                                            outcome: Err(CuError::from(
                                                                "Parallel runtime shutting down after an earlier stage failure",
                                                            )),
                                                            raw_payload_bytes: 0,
                                                            handle_bytes: 0,
                                                        }
                                                    } else if let Some(next_stage_tx) = next_stage_tx.as_mut() {
                                                        let forwarded_job = #mission_mod::ParallelWorkerJob { clid, culist };
                                                        match next_stage_tx.send(forwarded_job) {
                                                            Ok(()) => continue,
                                                            Err(send_error) => {
                                                                let failed_job = send_error.0;
                                                                shutdown.store(true, Ordering::Release);
                                                                #mission_mod::ParallelWorkerResult {
                                                                    clid,
                                                                    culist: Some(failed_job.culist),
                                                                    outcome: Err(CuError::from(format!(
                                                                        "Parallel stage {} could not hand CopperList #{} to the next stage",
                                                                        #stage_index_lit,
                                                                        clid
                                                                    ))),
                                                                    raw_payload_bytes: 0,
                                                                    handle_bytes: 0,
                                                                }
                                                            }
                                                        }
                                                    } else {
                                                        #mission_mod::ParallelWorkerResult {
                                                            clid,
                                                            culist: Some(culist),
                                                            outcome: Ok(cu29::curuntime::ProcessStepOutcome::Continue),
                                                            raw_payload_bytes: 0,
                                                            handle_bytes: 0,
                                                        }
                                                    }
                                                }
                                                Ok((culist, Ok(cu29::curuntime::ProcessStepOutcome::AbortCopperList))) => {
                                                    #mission_mod::ParallelWorkerResult {
                                                        clid,
                                                        culist: Some(culist),
                                                        outcome: Ok(cu29::curuntime::ProcessStepOutcome::AbortCopperList),
                                                        raw_payload_bytes: 0,
                                                        handle_bytes: 0,
                                                    }
                                                }
                                                Ok((culist, Err(error))) => {
                                                    shutdown.store(true, Ordering::Release);
                                                    #mission_mod::ParallelWorkerResult {
                                                        clid,
                                                        culist: Some(culist),
                                                        outcome: Err(error),
                                                        raw_payload_bytes: 0,
                                                        handle_bytes: 0,
                                                    }
                                                }
                                                Err(payload) => {
                                                    shutdown.store(true, Ordering::Release);
                                                    let panic_message =
                                                        cu29::monitoring::panic_payload_to_string(payload.as_ref());
                                                    #mission_mod::ParallelWorkerResult {
                                                        clid,
                                                        culist: None,
                                                        outcome: Err(CuError::from(format!(
                                                            "Panic while processing CopperList #{} in stage {}: {}",
                                                            clid,
                                                            #stage_index_lit,
                                                            panic_message
                                                        ))),
                                                        raw_payload_bytes: 0,
                                                        handle_bytes: 0,
                                                    }
                                                }
                                            }
                                        };

                                        if done_tx.send(terminal_result).is_err() {
                                            break;
                                        }
                                    }
                                });
                            }
                        }
                    })
                    .collect();
            (
                parallel_process_step_idents,
                parallel_process_step_fn_defs,
                parallel_stage_worker_spawns,
            )
        } else {
            (Vec::new(), Vec::new(), Vec::new())
        };
        let parallel_process_stage_count_tokens =
            proc_macro2::Literal::usize_unsuffixed(parallel_process_step_idents.len());
        let parallel_task_ptrs_type = if task_types.is_empty() {
            quote! { () }
        } else {
            let elems = task_types
                .iter()
                .map(|ty| quote! { ParallelSharedPtr<#ty> });
            quote! { (#(#elems),*,) }
        };
        let parallel_task_locks_type = if task_types.is_empty() {
            quote! { () }
        } else {
            let elems = (0..task_types.len()).map(|_| quote! { std::sync::Mutex<()> });
            quote! { (#(#elems),*,) }
        };
        let parallel_task_ptr_values = if task_types.is_empty() {
            quote! { () }
        } else {
            let elems = (0..task_types.len()).map(|index| {
                let index = syn::Index::from(index);
                quote! { ParallelSharedPtr::new(&mut runtime.tasks.#index as *mut _) }
            });
            quote! { (#(#elems),*,) }
        };
        let parallel_task_lock_values = if task_types.is_empty() {
            quote! { () }
        } else {
            let elems = (0..task_types.len()).map(|_| quote! { std::sync::Mutex::new(()) });
            quote! { (#(#elems),*,) }
        };
        let parallel_bridge_ptrs_type = if bridge_runtime_types.is_empty() {
            quote! { () }
        } else {
            let elems = bridge_runtime_types
                .iter()
                .map(|ty| quote! { ParallelSharedPtr<#ty> });
            quote! { (#(#elems),*,) }
        };
        let parallel_bridge_locks_type = if bridge_runtime_types.is_empty() {
            quote! { () }
        } else {
            let elems = (0..bridge_runtime_types.len()).map(|_| quote! { std::sync::Mutex<()> });
            quote! { (#(#elems),*,) }
        };
        let parallel_bridge_ptr_values = if bridge_runtime_types.is_empty() {
            quote! { () }
        } else {
            let elems = (0..bridge_runtime_types.len()).map(|index| {
                let index = syn::Index::from(index);
                quote! { ParallelSharedPtr::new(&mut runtime.bridges.#index as *mut _) }
            });
            quote! { (#(#elems),*,) }
        };
        let parallel_bridge_lock_values = if bridge_runtime_types.is_empty() {
            quote! { () }
        } else {
            let elems =
                (0..bridge_runtime_types.len()).map(|_| quote! { std::sync::Mutex::new(()) });
            quote! { (#(#elems),*,) }
        };
        let parallel_rt_support_tokens = if parallel_rt_run_supported {
            quote! {
                type ParallelTaskPtrs = #parallel_task_ptrs_type;
                type ParallelTaskLocks = #parallel_task_locks_type;
                type ParallelBridgePtrs = #parallel_bridge_ptrs_type;
                type ParallelBridgeLocks = #parallel_bridge_locks_type;

                struct ParallelSharedPtr<T>(*mut T);

                impl<T> Clone for ParallelSharedPtr<T> {
                    #[inline(always)]
                    fn clone(&self) -> Self {
                        *self
                    }
                }

                impl<T> Copy for ParallelSharedPtr<T> {}

                impl<T> ParallelSharedPtr<T> {
                    #[inline(always)]
                    const fn new(ptr: *mut T) -> Self {
                        Self(ptr)
                    }

                    #[inline(always)]
                    const fn from_ref(ptr: *const T) -> Self {
                        Self(ptr as *mut T)
                    }

                    #[inline(always)]
                    unsafe fn as_mut<'a>(self) -> &'a mut T {
                        unsafe { &mut *self.0 }
                    }

                    #[inline(always)]
                    unsafe fn as_ref<'a>(self) -> &'a T {
                        unsafe { &*self.0 }
                    }
                }

                unsafe impl<T: Send> Send for ParallelSharedPtr<T> {}
                unsafe impl<T: Send> Sync for ParallelSharedPtr<T> {}

                struct ParallelKeyFrameAccessor<'a> {
                    ptr: ParallelSharedPtr<cu29::curuntime::KeyFramesManager>,
                    lock: &'a std::sync::Mutex<()>,
                }

                impl<'a> ParallelKeyFrameAccessor<'a> {
                    #[inline(always)]
                    fn new(
                        ptr: ParallelSharedPtr<cu29::curuntime::KeyFramesManager>,
                        lock: &'a std::sync::Mutex<()>,
                    ) -> Self {
                        Self { ptr, lock }
                    }

                    #[inline(always)]
                    fn freeze_task(
                        &self,
                        culistid: u64,
                        task: &impl cu29::cutask::Freezable,
                    ) -> CuResult<usize> {
                        let _guard = self.lock.lock().expect("parallel keyframe lock poisoned");
                        let manager = unsafe { self.ptr.as_mut() };
                        manager.freeze_task(culistid, task)
                    }

                    #[inline(always)]
                    fn freeze_any(
                        &self,
                        culistid: u64,
                        item: &impl cu29::cutask::Freezable,
                    ) -> CuResult<usize> {
                        let _guard = self.lock.lock().expect("parallel keyframe lock poisoned");
                        let manager = unsafe { self.ptr.as_mut() };
                        manager.freeze_any(culistid, item)
                    }
                }

                struct ParallelProcessStepRuntime<'a> {
                    clock: &'a RobotClock,
                    execution_probe: &'a cu29::monitoring::RuntimeExecutionProbe,
                    monitor: &'a #monitor_type,
                    task_ptrs: &'a ParallelTaskPtrs,
                    task_locks: &'a ParallelTaskLocks,
                    bridge_ptrs: &'a ParallelBridgePtrs,
                    bridge_locks: &'a ParallelBridgeLocks,
                    kf_manager_ptr: ParallelSharedPtr<cu29::curuntime::KeyFramesManager>,
                    kf_lock: &'a std::sync::Mutex<()>,
                    culist: &'a mut CuList,
                    clid: u64,
                    ctx: cu29::context::CuContext,
                }

                struct ParallelWorkerJob {
                    clid: u64,
                    culist: Box<CuList>,
                }

                struct ParallelWorkerResult {
                    clid: u64,
                    culist: Option<Box<CuList>>,
                    outcome: cu29::curuntime::ProcessStepResult,
                    raw_payload_bytes: u64,
                    handle_bytes: u64,
                }

                #[inline(always)]
                fn assert_parallel_rt_send_bounds()
                where
                    CuList: Send,
                    #process_step_tasks_type: Send,
                    CuBridges: Send,
                    #monitor_type: Sync,
                {
                }

                #(#parallel_process_step_fn_defs)*
            }
        } else {
            quote! {}
        };

        let config_load_stmt =
            build_config_load_stmt(std, application_name, subsystem_id.as_deref());

        let copperlist_count_check = quote! {
            let configured_copperlist_count = config
                .logging
                .as_ref()
                .and_then(|logging| logging.copperlist_count)
                .unwrap_or(#copperlist_count_tokens);
            if configured_copperlist_count != #copperlist_count_tokens {
                return Err(CuError::from(format!(
                    "Configured logging.copperlist_count ({configured_copperlist_count}) does not match the runtime compiled into this binary ({})",
                    #copperlist_count_tokens
                )));
            }
        };

        let prepare_config_sig = if std {
            quote! {
                fn prepare_config(
                    instance_id: u32,
                    config_override: Option<CuConfig>,
                ) -> CuResult<(CuConfig, RuntimeLifecycleConfigSource)>
            }
        } else {
            quote! {
                fn prepare_config() -> CuResult<(CuConfig, RuntimeLifecycleConfigSource)>
            }
        };

        let prepare_config_call = if std {
            quote! { Self::prepare_config(instance_id, config_override)? }
        } else {
            quote! { Self::prepare_config()? }
        };

        let prepare_resources_sig = if std {
            quote! {
                pub fn prepare_resources_for_instance(
                    instance_id: u32,
                    config_override: Option<CuConfig>,
                ) -> CuResult<AppResources>
            }
        } else {
            quote! {
                pub fn prepare_resources() -> CuResult<AppResources>
            }
        };

        let prepare_resources_compat_fn = if std {
            Some(quote! {
                pub fn prepare_resources(
                    config_override: Option<CuConfig>,
                ) -> CuResult<AppResources> {
                    Self::prepare_resources_for_instance(0, config_override)
                }
            })
        } else {
            None
        };

        let init_resources_compat_fn = if std {
            Some(quote! {
                pub fn init_resources_for_instance(
                    instance_id: u32,
                    config_override: Option<CuConfig>,
                ) -> CuResult<AppResources> {
                    Self::prepare_resources_for_instance(instance_id, config_override)
                }

                pub fn init_resources(
                    config_override: Option<CuConfig>,
                ) -> CuResult<AppResources> {
                    Self::prepare_resources(config_override)
                }
            })
        } else {
            Some(quote! {
                pub fn init_resources() -> CuResult<AppResources> {
                    Self::prepare_resources()
                }
            })
        };

        let build_with_resources_sig = if sim_mode {
            quote! {
                fn build_with_resources<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>(
                    clock: RobotClock,
                    unified_logger: Arc<Mutex<L>>,
                    app_resources: AppResources,
                    instance_id: u32,
                    sim_callback: &mut impl FnMut(SimStep) -> SimOverride,
                ) -> CuResult<Self>
            }
        } else {
            quote! {
                fn build_with_resources<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>(
                    clock: RobotClock,
                    unified_logger: Arc<Mutex<L>>,
                    app_resources: AppResources,
                    instance_id: u32,
                ) -> CuResult<Self>
            }
        };
        let parallel_rt_metadata_arg = if std && parallel_rt_enabled {
            Some(quote! {
                &#mission_mod::PARALLEL_RT_METADATA,
            })
        } else {
            None
        };

        let kill_handler = if std && signal_handler {
            Some(quote! {
                ctrlc::set_handler(move || {
                    STOP_FLAG.store(true, Ordering::SeqCst);
                }).expect("Error setting Ctrl-C handler");
            })
        } else {
            None
        };

        let run_loop = if std {
            quote! {{
                let mut rate_limiter = self
                    .copper_runtime
                    .runtime_config
                    .rate_target_hz
                    .map(|rate| cu29::curuntime::LoopRateLimiter::from_rate_target_hz(
                        rate,
                        &self.copper_runtime.clock,
                    ))
                    .transpose()?;
                loop  {
                    let result = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(
                        || <Self as #app_trait<S, L>>::run_one_iteration(self, #sim_callback_arg)
                    )) {
                        Ok(result) => result,
                        Err(payload) => {
                            let panic_message = cu29::monitoring::panic_payload_to_string(payload.as_ref());
                            self.copper_runtime.monitor.process_panic(&panic_message);
                            let _ = self.log_runtime_lifecycle_event(RuntimeLifecycleEvent::Panic {
                                message: panic_message.clone(),
                                file: None,
                                line: None,
                                column: None,
                            });
                            Err(CuError::from(format!(
                                "Panic while running one iteration: {}",
                                panic_message
                            )))
                        }
                    };

                    if let Some(rate_limiter) = rate_limiter.as_mut() {
                        rate_limiter.limit(&self.copper_runtime.clock);
                    }

                    if STOP_FLAG.load(Ordering::SeqCst) || result.is_err() {
                        break result;
                    }
                }
            }}
        } else {
            quote! {{
                let mut rate_limiter = self
                    .copper_runtime
                    .runtime_config
                    .rate_target_hz
                    .map(|rate| cu29::curuntime::LoopRateLimiter::from_rate_target_hz(
                        rate,
                        &self.copper_runtime.clock,
                    ))
                    .transpose()?;
                loop  {
                    let result = <Self as #app_trait<S, L>>::run_one_iteration(self, #sim_callback_arg);
                    if let Some(rate_limiter) = rate_limiter.as_mut() {
                        rate_limiter.limit(&self.copper_runtime.clock);
                    }

                    if STOP_FLAG.load(Ordering::SeqCst) || result.is_err() {
                        break result;
                    }
                }
            }}
        };

        #[cfg(feature = "macro_debug")]
        eprintln!("[build the run methods]");
        let run_body: proc_macro2::TokenStream = if parallel_rt_run_supported {
            quote! {
                static STOP_FLAG: AtomicBool = AtomicBool::new(false);

                #kill_handler

                <Self as #app_trait<S, L>>::start_all_tasks(self)?;
                let result = std::thread::scope(|scope| -> CuResult<()> {
                    #mission_mod::assert_parallel_rt_send_bounds();

                    let runtime = &mut self.copper_runtime;
                    let clock = &runtime.clock;
                    let instance_id = runtime.instance_id();
                    let subsystem_code = runtime.subsystem_code();
                    let execution_probe = runtime.execution_probe.as_ref();
                    let monitor = &runtime.monitor;
                    let cl_manager = &mut runtime.copperlists_manager;
                    let parallel_rt = &runtime.parallel_rt;
                    let execution_probe_ptr =
                        #mission_mod::ParallelSharedPtr::from_ref(execution_probe as *const _);
                    let monitor_ptr =
                        #mission_mod::ParallelSharedPtr::from_ref(monitor as *const _);
                    let task_ptrs: #mission_mod::ParallelTaskPtrs = #parallel_task_ptr_values;
                    let task_locks = std::sync::Arc::new(#parallel_task_lock_values);
                    let bridge_ptrs: #mission_mod::ParallelBridgePtrs = #parallel_bridge_ptr_values;
                    let bridge_locks = std::sync::Arc::new(#parallel_bridge_lock_values);
                    let kf_manager_ptr =
                        #mission_mod::ParallelSharedPtr::new(&mut runtime.keyframes_manager as *mut _);
                    let kf_lock = std::sync::Arc::new(std::sync::Mutex::new(()));
                    let mut free_copperlists =
                        cu29::curuntime::allocate_boxed_copperlists::<CuStampedDataSet, #copperlist_count_tokens>();
                    let start_clid = cl_manager.next_cl_id();
                    parallel_rt.reset_cursors(start_clid);

                    let stage_count = #parallel_process_stage_count_tokens;
                    debug_assert_eq!(parallel_rt.metadata().process_stage_count(), stage_count);
                    if stage_count == 0 {
                        return Err(CuError::from(
                            "Parallel runtime requires at least one generated process stage",
                        ));
                    }

                    let queue_capacity = parallel_rt.in_flight_limit().max(1);
                    let mut stage_senders = Vec::with_capacity(stage_count);
                    let mut stage_receivers = Vec::with_capacity(stage_count);
                    for _stage_index in 0..stage_count {
                        let (stage_tx, stage_rx) =
                            cu29::parallel_queue::stage_queue::<#mission_mod::ParallelWorkerJob>(
                                queue_capacity,
                            );
                        stage_senders.push(stage_tx);
                        stage_receivers.push(stage_rx);
                    }
                    let (done_tx, done_rx) =
                        std::sync::mpsc::channel::<#mission_mod::ParallelWorkerResult>();
                    let shutdown = std::sync::Arc::new(AtomicBool::new(false));
                    let mut stage_senders = stage_senders.into_iter();
                    let mut entry_stage_tx = stage_senders
                        .next()
                        .expect("parallel stage pipeline has no entry queue");
                    let mut stage_receivers = stage_receivers.into_iter();
                    #(#parallel_stage_worker_spawns)*
                    drop(done_tx);

                    let mut dispatch_limiter = runtime
                        .runtime_config
                        .rate_target_hz
                        .map(|rate| cu29::curuntime::LoopRateLimiter::from_rate_target_hz(rate, clock))
                        .transpose()?;
                    let mut in_flight = 0usize;
                    let mut stop_launching = false;
                    let mut next_launch_clid = start_clid;
                    let mut next_commit_clid = start_clid;
                    let mut pending_results =
                        std::collections::BTreeMap::<u64, #mission_mod::ParallelWorkerResult>::new();
                    let mut active_keyframe_clid: Option<u64> = None;
                    let mut fatal_error: Option<CuError> = None;

                    loop {
                        while let Some(recycled_culist) = cl_manager.try_reclaim_boxed()? {
                            free_copperlists.push(recycled_culist);
                        }

                        if !stop_launching && fatal_error.is_none() {
                            let next_clid = next_launch_clid;
                            let rate_ready = dispatch_limiter
                                .as_ref()
                                .map(|limiter| limiter.is_ready(clock))
                                .unwrap_or(true);
                            let keyframe_ready = {
                                let _keyframe_lock = kf_lock.lock().expect("parallel keyframe lock poisoned");
                                let kf_manager = unsafe { kf_manager_ptr.as_mut() };
                                active_keyframe_clid.is_none() || !kf_manager.captures_keyframe(next_clid)
                            };

                            if in_flight < parallel_rt.in_flight_limit()
                                && rate_ready
                                && keyframe_ready
                                && !free_copperlists.is_empty()
                            {
                                // Parallel lifecycle is attached to component-local stage work,
                                // so dispatch itself can launch the next CopperList immediately.
                                let should_launch = true;

                                if should_launch {
                                    let mut culist = free_copperlists
                                        .pop()
                                        .expect("parallel CopperList pool unexpectedly empty");
                                    let clid = next_clid;
                                    culist.id = clid;
                                    culist.change_state(cu29::copperlist::CopperListState::Initialized);
                                    {
                                        let _keyframe_lock =
                                            kf_lock.lock().expect("parallel keyframe lock poisoned");
                                        let kf_manager = unsafe { kf_manager_ptr.as_mut() };
                                        kf_manager.reset(clid, clock);
                                        if kf_manager.captures_keyframe(clid) {
                                            active_keyframe_clid = Some(clid);
                                        }
                                    }
                                    culist.change_state(cu29::copperlist::CopperListState::Processing);
                                    culist.msgs.init_zeroed();
                                    entry_stage_tx
                                        .send(#mission_mod::ParallelWorkerJob {
                                            clid,
                                            culist,
                                        })
                                        .map_err(|e| {
                                            shutdown.store(true, Ordering::Release);
                                            CuError::from("Failed to enqueue CopperList for parallel stage processing")
                                                .add_cause(e.to_string().as_str())
                                        })?;
                                    next_launch_clid += 1;
                                    in_flight += 1;
                                    if let Some(limiter) = dispatch_limiter.as_mut() {
                                        limiter.mark_tick(clock);
                                    }
                                }

                                if STOP_FLAG.load(Ordering::SeqCst) {
                                    stop_launching = true;
                                }
                                continue;
                            }
                        }

                        if in_flight == 0 {
                            if stop_launching || fatal_error.is_some() {
                                break;
                            }

                            if free_copperlists.is_empty() {
                                free_copperlists.push(cl_manager.wait_reclaim_boxed()?);
                                continue;
                            }

                            if let Some(limiter) = dispatch_limiter.as_ref()
                                && !limiter.is_ready(clock)
                            {
                                limiter.wait_until_ready(clock);
                                continue;
                            }
                        }

                        let recv_result = if !stop_launching && fatal_error.is_none() {
                            if let Some(limiter) = dispatch_limiter.as_ref() {
                                if let Some(remaining) = limiter.remaining(clock)
                                    && in_flight > 0
                                {
                                    done_rx.recv_timeout(std::time::Duration::from(remaining))
                                } else {
                                    done_rx
                                        .recv()
                                        .map_err(|_| std::sync::mpsc::RecvTimeoutError::Disconnected)
                                }
                            } else {
                                done_rx
                                    .recv()
                                    .map_err(|_| std::sync::mpsc::RecvTimeoutError::Disconnected)
                            }
                        } else {
                            done_rx
                                .recv()
                                .map_err(|_| std::sync::mpsc::RecvTimeoutError::Disconnected)
                        };

                        let worker_result = match recv_result {
                            Ok(worker_result) => worker_result,
                            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                                if STOP_FLAG.load(Ordering::SeqCst) {
                                    stop_launching = true;
                                }
                                continue;
                            }
                            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                                shutdown.store(true, Ordering::Release);
                                return Err(CuError::from(
                                    "Parallel stage worker disconnected unexpectedly",
                                ));
                            }
                        };
                        in_flight = in_flight.saturating_sub(1);
                        pending_results.insert(worker_result.clid, worker_result);

                        while let Some(worker_result) = pending_results.remove(&next_commit_clid) {
                            if fatal_error.is_none()
                                && parallel_rt.current_commit_clid() != worker_result.clid
                            {
                                shutdown.store(true, Ordering::Release);
                                fatal_error = Some(CuError::from(format!(
                                    "Parallel commit checkpoint out of sync: expected {}, got {}",
                                    parallel_rt.current_commit_clid(),
                                    worker_result.clid
                                )));
                                stop_launching = true;
                            }

                            let mut worker_result = worker_result;
                            if fatal_error.is_none() {
                                match worker_result.outcome {
                                    Ok(cu29::curuntime::ProcessStepOutcome::AbortCopperList) => {
                                        let mut culist = worker_result
                                            .culist
                                            .take()
                                            .expect("parallel abort result missing CopperList ownership");
                                        let mut commit_ctx = cu29::context::CuContext::from_runtime_metadata(
                                            clock.clone(),
                                            worker_result.clid,
                                            instance_id,
                                            subsystem_code,
                                            #mission_mod::TASK_IDS,
                                        );
                                        commit_ctx.clear_current_task();
                                        let monitor_result = monitor.process_copperlist(
                                            &commit_ctx,
                                            #mission_mod::MONITOR_LAYOUT.view(&#mission_mod::collect_metadata(&culist)),
                                        );
                                        match cl_manager.end_of_processing_boxed(culist)? {
                                            cu29::curuntime::OwnedCopperListSubmission::Recycled(culist) => {
                                                free_copperlists.push(culist);
                                            }
                                            cu29::curuntime::OwnedCopperListSubmission::Pending => {}
                                        }
                                        monitor_result?;
                                    }
                                    Ok(cu29::curuntime::ProcessStepOutcome::Continue) => {
                                        let mut culist = worker_result
                                            .culist
                                            .take()
                                            .expect("parallel worker result missing CopperList ownership");
                                        let mut commit_ctx = cu29::context::CuContext::from_runtime_metadata(
                                            clock.clone(),
                                            worker_result.clid,
                                            instance_id,
                                            subsystem_code,
                                            #mission_mod::TASK_IDS,
                                        );
                                        commit_ctx.clear_current_task();
                                        let monitor_result = monitor.process_copperlist(
                                            &commit_ctx,
                                            #mission_mod::MONITOR_LAYOUT.view(&#mission_mod::collect_metadata(&culist)),
                                        );

                                        #(#preprocess_logging_calls)*

                                        match cl_manager.end_of_processing_boxed(culist)? {
                                            cu29::curuntime::OwnedCopperListSubmission::Recycled(culist) => {
                                                free_copperlists.push(culist);
                                            }
                                            cu29::curuntime::OwnedCopperListSubmission::Pending => {}
                                        }
                                        let keyframe_bytes = {
                                            let _keyframe_lock =
                                                kf_lock.lock().expect("parallel keyframe lock poisoned");
                                            let kf_manager = unsafe { kf_manager_ptr.as_mut() };
                                            kf_manager.end_of_processing(worker_result.clid)?;
                                            kf_manager.last_encoded_bytes
                                        };
                                        monitor_result?;
                                        let stats = cu29::monitoring::CopperListIoStats {
                                            raw_culist_bytes: core::mem::size_of::<CuList>() as u64
                                                + cl_manager.last_handle_bytes,
                                            handle_bytes: cl_manager.last_handle_bytes,
                                            encoded_culist_bytes: cl_manager.last_encoded_bytes,
                                            keyframe_bytes,
                                            structured_log_bytes_total: ::cu29::prelude::structured_log_bytes_total(),
                                            culistid: worker_result.clid,
                                        };
                                        monitor.observe_copperlist_io(stats);

                                        // Postprocess, when present, now runs inside the owning
                                        // component stage instead of on the ordered commit path.
                                    }
                                    Err(error) => {
                                        shutdown.store(true, Ordering::Release);
                                        stop_launching = true;
                                        fatal_error = Some(error);
                                        if let Some(mut culist) = worker_result.culist.take() {
                                            culist.change_state(cu29::copperlist::CopperListState::Free);
                                            free_copperlists.push(culist);
                                        }
                                    }
                                }
                            } else if let Some(mut culist) = worker_result.culist.take() {
                                culist.change_state(cu29::copperlist::CopperListState::Free);
                                free_copperlists.push(culist);
                            }

                            if active_keyframe_clid == Some(worker_result.clid) {
                                active_keyframe_clid = None;
                            }
                            parallel_rt.release_commit(worker_result.clid + 1);
                            next_commit_clid += 1;
                        }

                        if STOP_FLAG.load(Ordering::SeqCst) {
                            stop_launching = true;
                        }
                    }

                    drop(entry_stage_tx);
                    free_copperlists.extend(cl_manager.finish_pending_boxed()?);
                    if let Some(error) = fatal_error {
                        Err(error)
                    } else {
                        Ok(())
                    }
                });

                if result.is_err() {
                    error!("A task errored out: {}", &result);
                }
                <Self as #app_trait<S, L>>::stop_all_tasks(self, #sim_callback_arg)?;
                let _ = self.log_shutdown_completed();
                result
            }
        } else {
            quote! {
                static STOP_FLAG: AtomicBool = AtomicBool::new(false);

                #kill_handler

                <Self as #app_trait<S, L>>::start_all_tasks(self, #sim_callback_arg)?;
                let result = #run_loop;

                if result.is_err() {
                    error!("A task errored out: {}", &result);
                }
                <Self as #app_trait<S, L>>::stop_all_tasks(self, #sim_callback_arg)?;
                let _ = self.log_shutdown_completed();
                result
            }
        };
        let run_methods: proc_macro2::TokenStream = quote! {

            #run_one_iteration {

                // Pre-explode the runtime to avoid complexity with partial borrowing in the generated code.
                let runtime = &mut self.copper_runtime;
                let clock = &runtime.clock;
                let instance_id = runtime.instance_id();
                let subsystem_code = runtime.subsystem_code();
                let execution_probe = &runtime.execution_probe;
                let monitor = &mut runtime.monitor;
                let tasks = &mut runtime.tasks;
                let __cu_bridges = &mut runtime.bridges;
                let cl_manager = &mut runtime.copperlists_manager;
                let kf_manager = &mut runtime.keyframes_manager;
                let iteration_clid = cl_manager.next_cl_id();
                let mut ctx = cu29::context::CuContext::from_runtime_metadata(
                    clock.clone(),
                    iteration_clid,
                    instance_id,
                    subsystem_code,
                    #mission_mod::TASK_IDS,
                );
                let mut __cu_abort_copperlist = false;

                // Preprocess calls can happen at any time, just packed them up front.
                #(#preprocess_calls)*

                let culist = cl_manager.create()?;
                let clid = culist.id;
                debug_assert_eq!(clid, iteration_clid);
                kf_manager.reset(clid, clock); // beginning of processing, we empty the serialized frozen states of the tasks.
                culist.change_state(cu29::copperlist::CopperListState::Processing);
                culist.msgs.init_zeroed();
                let mut ctx = cu29::context::CuContext::from_runtime_metadata(
                    clock.clone(),
                    iteration_clid,
                    instance_id,
                    subsystem_code,
                    #mission_mod::TASK_IDS,
                );
                {
                    let msgs = &mut culist.msgs.0;
                    '__cu_process_steps: {
                    #(#runtime_plan_code)*
                    }
                } // drop(msgs);
                if __cu_abort_copperlist {
                    ctx.clear_current_task();
                    let monitor_result = monitor.process_copperlist(&ctx, #mission_mod::MONITOR_LAYOUT.view(&#mission_mod::collect_metadata(&culist)));
                    cl_manager.end_of_processing(clid)?;
                    monitor_result?;
                    return Ok(());
                }
                ctx.clear_current_task();
                let monitor_result = monitor.process_copperlist(&ctx, #mission_mod::MONITOR_LAYOUT.view(&#mission_mod::collect_metadata(&culist)));

                // here drop the payloads if we don't want them to be logged.
                #(#preprocess_logging_calls)*

                cl_manager.end_of_processing(clid)?;
                kf_manager.end_of_processing(clid)?;
                monitor_result?;
                let stats = cu29::monitoring::CopperListIoStats {
                    raw_culist_bytes: core::mem::size_of::<CuList>() as u64 + cl_manager.last_handle_bytes,
                    handle_bytes: cl_manager.last_handle_bytes,
                    encoded_culist_bytes: cl_manager.last_encoded_bytes,
                    keyframe_bytes: kf_manager.last_encoded_bytes,
                    structured_log_bytes_total: ::cu29::prelude::structured_log_bytes_total(),
                    culistid: clid,
                };
                monitor.observe_copperlist_io(stats);

                // Postprocess calls can happen at any time, just packed them up at the end.
                #(#postprocess_calls)*
                Ok(())
            }

            fn restore_keyframe(&mut self, keyframe: &KeyFrame) -> CuResult<()> {
                let runtime = &mut self.copper_runtime;
                let clock = &runtime.clock;
                let tasks = &mut runtime.tasks;
                let __cu_bridges = &mut runtime.bridges;
                let config = cu29::bincode::config::standard();
                let reader = cu29::bincode::de::read::SliceReader::new(&keyframe.serialized_tasks);
                let mut decoder = DecoderImpl::new(reader, config, ());
                #(#task_restore_code);*;
                #(#bridge_restore_code);*;
                Ok(())
            }

            #start_all_tasks {
                let _ = self.log_runtime_lifecycle_event(RuntimeLifecycleEvent::MissionStarted {
                    mission: #mission.to_string(),
                });
                let lifecycle_clid = self.copper_runtime.copperlists_manager.last_cl_id();
                let mut ctx = cu29::context::CuContext::from_runtime_metadata(
                    self.copper_runtime.clock.clone(),
                    lifecycle_clid,
                    self.copper_runtime.instance_id(),
                    self.copper_runtime.subsystem_code(),
                    #mission_mod::TASK_IDS,
                );
                #(#start_calls)*
                ctx.clear_current_task();
                self.copper_runtime.monitor.start(&ctx)?;
                Ok(())
            }

            #stop_all_tasks {
                let lifecycle_clid = self.copper_runtime.copperlists_manager.last_cl_id();
                let mut ctx = cu29::context::CuContext::from_runtime_metadata(
                    self.copper_runtime.clock.clone(),
                    lifecycle_clid,
                    self.copper_runtime.instance_id(),
                    self.copper_runtime.subsystem_code(),
                    #mission_mod::TASK_IDS,
                );
                #(#stop_calls)*
                ctx.clear_current_task();
                self.copper_runtime.monitor.stop(&ctx)?;
                self.copper_runtime.copperlists_manager.finish_pending()?;
                // TODO(lifecycle): emit typed stop reasons (completed/error/panic/requested)
                // once panic/reporting flow is finalized for std and no-std.
                let _ = self.log_runtime_lifecycle_event(RuntimeLifecycleEvent::MissionStopped {
                    mission: #mission.to_string(),
                    reason: "stop_all_tasks".to_string(),
                });
                Ok(())
            }

            #run {
                #run_body
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

        let mission_id_method = if sim_mode {
            quote! {
                fn mission_id() -> Option<&'static str> {
                    Some(#mission)
                }
            }
        } else {
            quote!()
        };

        let app_resources_struct = quote! {
            pub struct AppResources {
                pub config: CuConfig,
                pub config_source: RuntimeLifecycleConfigSource,
                pub resources: ResourceManager,
            }
        };

        let prepare_config_fn = quote! {
            #prepare_config_sig {
                let config_filename = #config_file;

                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp init: config file {}", config_filename);
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp init: loading config");
                #config_load_stmt
                #copperlist_count_check
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp init: config loaded");
                if let Some(runtime) = &config.runtime {
                    #[cfg(target_os = "none")]
                    ::cu29::prelude::info!(
                        "CuApp init: rate_target_hz={}",
                        runtime.rate_target_hz.unwrap_or(0)
                    );
                } else {
                    #[cfg(target_os = "none")]
                    ::cu29::prelude::info!("CuApp init: rate_target_hz=none");
                }

                Ok((config, config_source))
            }
        };

        let prepare_resources_fn = quote! {
            #prepare_resources_sig {
                let (config, config_source) = #prepare_config_call;

                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp init: building resources");
                let resources = #mission_mod::resources_instanciator(&config)?;
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp init: resources ready");

                Ok(AppResources {
                    config,
                    config_source,
                    resources,
                })
            }
        };

        let new_with_resources_compat_fn = if sim_mode {
            quote! {
                pub fn new_with_resources<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>(
                    clock: RobotClock,
                    unified_logger: Arc<Mutex<L>>,
                    app_resources: AppResources,
                    instance_id: u32,
                    sim_callback: &mut impl FnMut(SimStep) -> SimOverride,
                ) -> CuResult<Self> {
                    Self::build_with_resources(
                        clock,
                        unified_logger,
                        app_resources,
                        instance_id,
                        sim_callback,
                    )
                }
            }
        } else {
            quote! {
                pub fn new_with_resources<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>(
                    clock: RobotClock,
                    unified_logger: Arc<Mutex<L>>,
                    app_resources: AppResources,
                    instance_id: u32,
                ) -> CuResult<Self> {
                    Self::build_with_resources(clock, unified_logger, app_resources, instance_id)
                }
            }
        };

        let build_with_resources_fn = quote! {
            #build_with_resources_sig {
                let AppResources {
                    config,
                    config_source,
                    resources,
                } = app_resources;

                let structured_stream = ::cu29::prelude::stream_write::<
                    ::cu29::prelude::CuLogEntry,
                    S,
                >(
                    unified_logger.clone(),
                    ::cu29::prelude::UnifiedLogType::StructuredLogLine,
                    4096 * 10,
                )?;
                let logger_runtime = ::cu29::prelude::LoggerRuntime::init(
                    clock.clone(),
                    structured_stream,
                    None::<::cu29::prelude::NullLog>,
                );

                // For simple cases we can say the section is just a bunch of Copper Lists.
                // But we can now have allocations outside of it so we can override it from the config.
                let mut default_section_size = size_of::<super::#mission_mod::CuList>() * 64;
                // Check if there is a logging configuration with section_size_mib
                if let Some(section_size_mib) = config.logging.as_ref().and_then(|l| l.section_size_mib) {
                    // Convert MiB to bytes
                    default_section_size = section_size_mib as usize * 1024usize * 1024usize;
                }
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!(
                    "CuApp new: copperlist section size={}",
                    default_section_size
                );
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: creating copperlist stream");
                let copperlist_stream = stream_write::<#mission_mod::CuList, S>(
                    unified_logger.clone(),
                    UnifiedLogType::CopperList,
                    default_section_size,
                    // the 2 sizes are not directly related as we encode the CuList but we can
                    // assume the encoded size is close or lower than the non encoded one
                    // This is to be sure we have the size of at least a Culist and some.
                )?;
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: copperlist stream ready");

                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: creating keyframes stream");
                let keyframes_stream = stream_write::<KeyFrame, S>(
                    unified_logger.clone(),
                    UnifiedLogType::FrozenTasks,
                    1024 * 1024 * 10, // 10 MiB
                )?;
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: keyframes stream ready");

                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: creating runtime lifecycle stream");
                let mut runtime_lifecycle_stream = stream_write::<RuntimeLifecycleRecord, S>(
                    unified_logger.clone(),
                    UnifiedLogType::RuntimeLifecycle,
                    1024 * 64, // 64 KiB
                )?;
                let effective_config_ron = config
                    .serialize_ron()
                    .unwrap_or_else(|_| "<failed to serialize config>".to_string());
                ::cu29::logcodec::set_effective_config_ron::<super::#mission_mod::CuStampedDataSet>(&effective_config_ron);
                let stack_info = RuntimeLifecycleStackInfo {
                    app_name: env!("CARGO_PKG_NAME").to_string(),
                    app_version: env!("CARGO_PKG_VERSION").to_string(),
                    git_commit: #git_commit_tokens,
                    git_dirty: #git_dirty_tokens,
                    subsystem_id: #application_name::subsystem().id().map(str::to_string),
                    subsystem_code: #application_name::subsystem().code(),
                    instance_id,
                };
                runtime_lifecycle_stream.log(&RuntimeLifecycleRecord {
                    timestamp: clock.now(),
                    event: RuntimeLifecycleEvent::Instantiated {
                        config_source,
                        effective_config_ron,
                        stack: stack_info,
                    },
                })?;
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: runtime lifecycle stream ready");

                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: building runtime");
                let copper_runtime = CuRuntimeBuilder::<#mission_mod::#tasks_type, #mission_mod::CuBridges, #mission_mod::CuStampedDataSet, #monitor_type, #copperlist_count_tokens, _, _, _, _, _>::new(
                    clock,
                    &config,
                    #mission,
                    CuRuntimeParts::new(
                        #mission_mod::#tasks_instanciator_fn,
                        #mission_mod::MONITORED_COMPONENTS,
                        #mission_mod::CULIST_COMPONENT_MAPPING,
                        #parallel_rt_metadata_arg
                        #mission_mod::monitor_instanciator,
                        #mission_mod::bridges_instanciator,
                    ),
                    copperlist_stream,
                    keyframes_stream,
                )
                .with_subsystem(#application_name::subsystem())
                .with_instance_id(instance_id)
                .with_resources(resources)
                .build()?;
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: runtime built");

                let application = Ok(#application_name {
                    copper_runtime,
                    runtime_lifecycle_stream: Some(Box::new(runtime_lifecycle_stream)),
                    logger_runtime,
                });

                #sim_callback_on_new

                application
            }
        };

        let app_inherent_impl = quote! {
            impl #application_name {
                const SUBSYSTEM: cu29::prelude::app::Subsystem =
                    cu29::prelude::app::Subsystem::new(#subsystem_id_tokens, #subsystem_code_literal);

                #[inline]
                pub fn subsystem() -> cu29::prelude::app::Subsystem {
                    Self::SUBSYSTEM
                }

                pub fn original_config() -> String {
                    #copper_config_content.to_string()
                }

                pub fn register_reflect_types(registry: &mut cu29::reflect::TypeRegistry) {
                    #(#reflect_type_registration_calls)*
                }

                /// Returns a clone of the runtime clock handle.
                #[inline]
                pub fn clock(&self) -> cu29::clock::RobotClock {
                    self.copper_runtime.clock()
                }

                /// Log one runtime lifecycle event with the current runtime timestamp.
                pub fn log_runtime_lifecycle_event(
                    &mut self,
                    event: RuntimeLifecycleEvent,
                ) -> CuResult<()> {
                    let timestamp = self.copper_runtime.clock.now();
                    let Some(stream) = self.runtime_lifecycle_stream.as_mut() else {
                        return Err(CuError::from("Runtime lifecycle stream is not initialized"));
                    };
                    stream.log(&RuntimeLifecycleRecord { timestamp, event })
                }

                /// Convenience helper for manual execution loops to mark graceful shutdown.
                // TODO(lifecycle): add helper(s) for panic/error stop reporting once we wire
                // RuntimeLifecycleEvent::Panic across std/no-std execution models.
                pub fn log_shutdown_completed(&mut self) -> CuResult<()> {
                    self.log_runtime_lifecycle_event(RuntimeLifecycleEvent::ShutdownCompleted)
                }

                #prepare_config_fn
                #prepare_resources_compat_fn
                #prepare_resources_fn
                #init_resources_compat_fn
                #new_with_resources_compat_fn
                #build_with_resources_fn

                /// Mutable access to the underlying runtime (used by tools such as deterministic re-sim).
                #[inline]
                pub fn copper_runtime_mut(&mut self) -> &mut CuRuntime<#mission_mod::#tasks_type, #mission_mod::CuBridges, #mission_mod::CuStampedDataSet, #monitor_type, #copperlist_count_tokens> {
                    &mut self.copper_runtime
                }
            }
        };

        let app_metadata_impl = quote! {
            impl cu29::prelude::app::CuSubsystemMetadata for #application_name {
                fn subsystem() -> cu29::prelude::app::Subsystem {
                    #application_name::subsystem()
                }
            }
        };

        let app_reflect_impl = quote! {
            impl cu29::reflect::ReflectTaskIntrospection for #application_name {
                fn reflect_task(&self, task_id: &str) -> Option<&dyn cu29::reflect::Reflect> {
                    match task_id {
                        #(#task_reflect_read_arms)*
                        _ => None,
                    }
                }

                fn reflect_task_mut(
                    &mut self,
                    task_id: &str,
                ) -> Option<&mut dyn cu29::reflect::Reflect> {
                    match task_id {
                        #(#task_reflect_write_arms)*
                        _ => None,
                    }
                }

                fn register_reflect_types(registry: &mut cu29::reflect::TypeRegistry) {
                    #application_name::register_reflect_types(registry);
                }
            }
        };

        let app_runtime_copperlist_impl = quote! {
            impl cu29::app::CurrentRuntimeCopperList<#mission_mod::CuStampedDataSet>
                for #application_name
            {
                fn current_runtime_copperlist_bytes(&self) -> Option<&[u8]> {
                    self.copper_runtime.copperlists_manager.last_completed_encoded()
                }

                fn set_current_runtime_copperlist_bytes(
                    &mut self,
                    snapshot: Option<Vec<u8>>,
                ) {
                    self.copper_runtime
                        .copperlists_manager
                        .set_last_completed_encoded(snapshot);
                }
            }
        };

        #[cfg(feature = "std")]
        #[cfg(feature = "macro_debug")]
        eprintln!("[build result]");
        let application_impl = quote! {
            #app_impl_decl {
                #simstep_type_decl

                fn get_original_config() -> String {
                    Self::original_config()
                }

                #mission_id_method

                #run_methods
            }
        };

        let recorded_replay_app_impl = if sim_mode {
            Some(quote! {
                impl<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>
                    CuRecordedReplayApplication<S, L> for #application_name
                {
                    type RecordedDataSet = #mission_mod::CuStampedDataSet;

                    fn replay_recorded_copperlist(
                        &mut self,
                        clock_mock: &RobotClockMock,
                        copperlist: &CopperList<Self::RecordedDataSet>,
                        keyframe: Option<&KeyFrame>,
                    ) -> CuResult<()> {
                        if let Some(keyframe) = keyframe {
                            if keyframe.culistid != copperlist.id {
                                return Err(CuError::from(format!(
                                    "Recorded keyframe culistid {} does not match copperlist {}",
                                    keyframe.culistid, copperlist.id
                                )));
                            }

                            if !self.copper_runtime_mut().captures_keyframe(copperlist.id) {
                                return Err(CuError::from(format!(
                                    "CopperList {} is not configured to capture a keyframe in this runtime",
                                    copperlist.id
                                )));
                            }

                            self.copper_runtime_mut()
                                .set_forced_keyframe_timestamp(keyframe.timestamp);
                            self.copper_runtime_mut().lock_keyframe(keyframe);
                            clock_mock.set_value(keyframe.timestamp.as_nanos());
                        } else {
                            let timestamp =
                                cu29::simulation::recorded_copperlist_timestamp(copperlist)
                                    .ok_or_else(|| {
                                        CuError::from(format!(
                                            "Recorded copperlist {} has no process_time.start timestamps",
                                            copperlist.id
                                        ))
                                    })?;
                            clock_mock.set_value(timestamp.as_nanos());
                        }

                        let mut sim_callback = |step: SimStep<'_>| -> SimOverride {
                            #mission_mod::recorded_replay_step(step, copperlist)
                        };
                        <Self as CuSimApplication<S, L>>::run_one_iteration(self, &mut sim_callback)
                    }
                }
            })
        } else {
            None
        };

        let distributed_replay_app_impl = if sim_mode {
            Some(quote! {
                impl<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>
                    cu29::prelude::app::CuDistributedReplayApplication<S, L> for #application_name
                {
                    fn build_distributed_replay(
                        clock: cu29::clock::RobotClock,
                        unified_logger: std::sync::Arc<std::sync::Mutex<L>>,
                        instance_id: u32,
                        config_override: Option<cu29::config::CuConfig>,
                    ) -> CuResult<Self> {
                        let mut noop =
                            |_step: SimStep<'_>| cu29::simulation::SimOverride::ExecuteByRuntime;
                        let builder = Self::builder()
                            .with_logger::<S, L>(unified_logger)
                            .with_clock(clock)
                            .with_instance_id(instance_id);
                        let builder = if let Some(config_override) = config_override {
                            builder.with_config(config_override)
                        } else {
                            builder
                        };
                        builder.with_sim_callback(&mut noop).build()
                    }
                }
            })
        } else {
            None
        };

        let builder_prepare_config_call = if std {
            quote! { #application_name::prepare_config(self.instance_id, self.config_override)? }
        } else {
            quote! {{
                let _ = self.config_override;
                #application_name::prepare_config()?
            }}
        };

        let builder_with_config_method = if std {
            Some(quote! {
                #[allow(dead_code)]
                pub fn with_config(mut self, config_override: CuConfig) -> Self {
                    self.config_override = Some(config_override);
                    self
                }
            })
        } else {
            None
        };

        let builder_default_clock = if std {
            quote! { Some(RobotClock::default()) }
        } else {
            quote! { None }
        };

        let (
            builder_struct,
            builder_impl,
            builder_ctor,
            builder_log_path_generics,
            builder_sim_callback_method,
            builder_build_sim_callback_arg,
        ) = if sim_mode {
            (
                quote! {
                    #[allow(dead_code)]
                    pub struct #builder_name<'a, F, S, L, R>
                    where
                        S: SectionStorage + 'static,
                        L: UnifiedLogWrite<S> + 'static,
                        R: FnOnce(&CuConfig) -> CuResult<ResourceManager>,
                        F: FnMut(SimStep) -> SimOverride,
                    {
                        clock: Option<RobotClock>,
                        unified_logger: Arc<Mutex<L>>,
                        instance_id: u32,
                        config_override: Option<CuConfig>,
                        resources_factory: R,
                        sim_callback: Option<&'a mut F>,
                        _storage: core::marker::PhantomData<S>,
                    }
                },
                quote! {
                    impl<'a, F, S, L, R> #builder_name<'a, F, S, L, R>
                    where
                        S: SectionStorage + 'static,
                        L: UnifiedLogWrite<S> + 'static,
                        R: FnOnce(&CuConfig) -> CuResult<ResourceManager>,
                        F: FnMut(SimStep) -> SimOverride,
                },
                quote! {
                    #[allow(dead_code)]
                    pub fn builder<'a, F>() -> #builder_name<'a, F, cu29::prelude::NoopSectionStorage, cu29::prelude::NoopLogger, fn(&CuConfig) -> CuResult<ResourceManager>>
                    where
                        F: FnMut(SimStep) -> SimOverride,
                    {
                        #builder_name {
                            clock: #builder_default_clock,
                            unified_logger: Arc::new(Mutex::new(cu29::prelude::NoopLogger::new())),
                            instance_id: 0,
                            config_override: None,
                            resources_factory: #mission_mod::resources_instanciator as fn(&CuConfig) -> CuResult<ResourceManager>,
                            sim_callback: None,
                            _storage: core::marker::PhantomData,
                        }
                    }
                },
                quote! {'a, F, MmapSectionStorage, UnifiedLoggerWrite, R},
                Some(quote! {
                    #[allow(dead_code)]
                    pub fn with_sim_callback(mut self, sim_callback: &'a mut F) -> Self {
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
                    pub struct #builder_name<S, L, R>
                    where
                        S: SectionStorage + 'static,
                        L: UnifiedLogWrite<S> + 'static,
                        R: FnOnce(&CuConfig) -> CuResult<ResourceManager>,
                    {
                        clock: Option<RobotClock>,
                        unified_logger: Arc<Mutex<L>>,
                        instance_id: u32,
                        config_override: Option<CuConfig>,
                        resources_factory: R,
                        _storage: core::marker::PhantomData<S>,
                    }
                },
                quote! {
                    impl<S, L, R> #builder_name<S, L, R>
                    where
                        S: SectionStorage + 'static,
                        L: UnifiedLogWrite<S> + 'static,
                        R: FnOnce(&CuConfig) -> CuResult<ResourceManager>,
                },
                quote! {
                    #[allow(dead_code)]
                    pub fn builder() -> #builder_name<cu29::prelude::NoopSectionStorage, cu29::prelude::NoopLogger, fn(&CuConfig) -> CuResult<ResourceManager>> {
                        #builder_name {
                            clock: #builder_default_clock,
                            unified_logger: Arc::new(Mutex::new(cu29::prelude::NoopLogger::new())),
                            instance_id: 0,
                            config_override: None,
                            resources_factory: #mission_mod::resources_instanciator as fn(&CuConfig) -> CuResult<ResourceManager>,
                            _storage: core::marker::PhantomData,
                        }
                    }
                },
                quote! {MmapSectionStorage, UnifiedLoggerWrite, R},
                None,
                None,
            )
        };

        let builder_with_logger_generics = if sim_mode {
            quote! {'a, F, S2, L2, R}
        } else {
            quote! {S2, L2, R}
        };

        let builder_with_resources_generics = if sim_mode {
            quote! {'a, F, S, L, R2}
        } else {
            quote! {S, L, R2}
        };

        let builder_sim_callback_field_copy = if sim_mode {
            Some(quote! {
                sim_callback: self.sim_callback,
            })
        } else {
            None
        };

        let builder_with_log_path_method = if std {
            Some(quote! {
                #[allow(dead_code)]
                pub fn with_log_path(
                    self,
                    path: impl AsRef<std::path::Path>,
                    slab_size: Option<usize>,
                ) -> CuResult<#builder_name<#builder_log_path_generics>> {
                    let preallocated_size = slab_size.unwrap_or(1024 * 1024 * 10);
                    let logger = cu29::prelude::UnifiedLoggerBuilder::new()
                        .write(true)
                        .create(true)
                        .file_base_name(path.as_ref())
                        .preallocated_size(preallocated_size)
                        .build()
                        .map_err(|e| CuError::new_with_cause("Failed to create unified logger", e))?;
                    let logger = match logger {
                        cu29::prelude::UnifiedLogger::Write(logger) => logger,
                        cu29::prelude::UnifiedLogger::Read(_) => {
                            return Err(CuError::from(
                                "UnifiedLoggerBuilder did not create a write-capable logger",
                            ));
                        }
                    };
                    Ok(self.with_logger::<MmapSectionStorage, UnifiedLoggerWrite>(Arc::new(Mutex::new(
                        logger,
                    ))))
                }
            })
        } else {
            None
        };

        let builder_with_unified_logger_method = if std {
            Some(quote! {
                #[allow(dead_code)]
                pub fn with_unified_logger(
                    self,
                    unified_logger: Arc<Mutex<UnifiedLoggerWrite>>,
                ) -> #builder_name<#builder_log_path_generics> {
                    self.with_logger::<MmapSectionStorage, UnifiedLoggerWrite>(unified_logger)
                }
            })
        } else {
            None
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
                            pub fn replay_recorded_copperlist(
                                &mut self,
                                clock_mock: &RobotClockMock,
                                copperlist: &CopperList<CuStampedDataSet>,
                                keyframe: Option<&KeyFrame>,
                            ) -> CuResult<()> {
                                <Self as CuRecordedReplayApplication<MmapSectionStorage, UnifiedLoggerWrite>>::replay_recorded_copperlist(
                                    self,
                                    clock_mock,
                                    copperlist,
                                    keyframe,
                                )
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

        let application_builder = Some(quote! {
            #builder_struct

            #builder_impl
            {
                #[allow(dead_code)]
                pub fn with_clock(mut self, clock: RobotClock) -> Self {
                    self.clock = Some(clock);
                    self
                }

                #[allow(dead_code)]
                pub fn with_logger<S2, L2>(
                    self,
                    unified_logger: Arc<Mutex<L2>>,
                ) -> #builder_name<#builder_with_logger_generics>
                where
                    S2: SectionStorage + 'static,
                    L2: UnifiedLogWrite<S2> + 'static,
                {
                    #builder_name {
                        clock: self.clock,
                        unified_logger,
                        instance_id: self.instance_id,
                        config_override: self.config_override,
                        resources_factory: self.resources_factory,
                        #builder_sim_callback_field_copy
                        _storage: core::marker::PhantomData,
                    }
                }

                #builder_with_unified_logger_method

                #[allow(dead_code)]
                pub fn with_instance_id(mut self, instance_id: u32) -> Self {
                    self.instance_id = instance_id;
                    self
                }

                pub fn with_resources<R2>(self, resources_factory: R2) -> #builder_name<#builder_with_resources_generics>
                where
                    R2: FnOnce(&CuConfig) -> CuResult<ResourceManager>,
                {
                    #builder_name {
                        clock: self.clock,
                        unified_logger: self.unified_logger,
                        instance_id: self.instance_id,
                        config_override: self.config_override,
                        resources_factory,
                        #builder_sim_callback_field_copy
                        _storage: core::marker::PhantomData,
                    }
                }

                #builder_with_config_method
                #builder_with_log_path_method
                #builder_sim_callback_method

                #[allow(dead_code)]
                pub fn build(self) -> CuResult<#application_name> {
                    let clock = self
                        .clock
                        .ok_or(CuError::from("Clock missing from builder"))?;
                    let (config, config_source) = #builder_prepare_config_call;
                    let resources = (self.resources_factory)(&config)?;
                    let app_resources = AppResources {
                        config,
                        config_source,
                        resources,
                    };
                    #application_name::build_with_resources(
                        clock,
                        self.unified_logger,
                        app_resources,
                        self.instance_id,
                        #builder_build_sim_callback_arg
                    )
                }
            }
        });

        let app_builder_inherent_impl = quote! {
            impl #application_name {
                #builder_ctor
            }
        };

        let sim_imports = if sim_mode {
            Some(quote! {
                use cu29::simulation::SimOverride;
                use cu29::simulation::CuTaskCallbackState;
                use cu29::simulation::CuSimSrcTask;
                use cu29::simulation::CuSimSinkTask;
                use cu29::simulation::CuSimBridge;
                use cu29::prelude::app::CuSimApplication;
                use cu29::prelude::app::CuRecordedReplayApplication;
                use cu29::cubridge::BridgeChannelSet;
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
            quote! {
                let _ = resources;
                Ok(())
            }
        } else {
            quote! { Ok(( #(#task_sim_instances_init_code),*, )) }
        };

        let sim_tasks_instanciator = if sim_mode {
            Some(quote! {
                pub fn tasks_instanciator_sim(
                    all_instances_configs: Vec<Option<&ComponentConfig>>,
                    resources: &mut ResourceManager,
                ) -> CuResult<CuSimTasks> {
                    #sim_inst_body
            }})
        } else {
            None
        };

        let tasks_inst_body_std = if task_instances_init_code.is_empty() {
            quote! {
                let _ = resources;
                Ok(())
            }
        } else {
            quote! { Ok(( #(#task_instances_init_code),*, )) }
        };

        let tasks_inst_body_nostd = if task_instances_init_code.is_empty() {
            quote! {
                let _ = resources;
                Ok(())
            }
        } else {
            quote! { Ok(( #(#task_instances_init_code),*, )) }
        };

        let tasks_instanciator = if std {
            quote! {
                pub fn tasks_instanciator<'c>(
                    all_instances_configs: Vec<Option<&'c ComponentConfig>>,
                    resources: &mut ResourceManager,
                ) -> CuResult<CuTasks> {
                    #tasks_inst_body_std
                }
            }
        } else {
            // no thread pool in the no-std impl
            quote! {
                pub fn tasks_instanciator<'c>(
                    all_instances_configs: Vec<Option<&'c ComponentConfig>>,
                    resources: &mut ResourceManager,
                ) -> CuResult<CuTasks> {
                    #tasks_inst_body_nostd
                }
            }
        };

        let imports = if std {
            quote! {
                use cu29::rayon::ThreadPool;
                use cu29::cuasynctask::CuAsyncSrcTask;
                use cu29::cuasynctask::CuAsyncTask;
                use cu29::resource::{ResourceBindings, ResourceManager};
                use cu29::prelude::SectionStorage;
                use cu29::prelude::UnifiedLoggerWrite;
                use cu29::prelude::memmap::MmapSectionStorage;
                use std::fmt::{Debug, Formatter};
                use std::fmt::Result as FmtResult;
                use std::mem::size_of;
                use std::boxed::Box;
                use std::sync::Arc;
                use std::sync::atomic::{AtomicBool, Ordering};
                use std::sync::Mutex;
            }
        } else {
            quote! {
                use alloc::boxed::Box;
                use alloc::sync::Arc;
                use alloc::string::String;
                use alloc::string::ToString;
                use core::sync::atomic::{AtomicBool, Ordering};
                use core::fmt::{Debug, Formatter};
                use core::fmt::Result as FmtResult;
                use core::mem::size_of;
                use spin::Mutex;
                use cu29::prelude::SectionStorage;
                use cu29::resource::{ResourceBindings, ResourceManager};
            }
        };

        let task_mapping_defs = task_resource_mappings.defs.clone();
        let bridge_mapping_defs = bridge_resource_mappings.defs.clone();

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
                use cu29::clock::RobotClockMock;
                use cu29::config::CuConfig;
                use cu29::config::ComponentConfig;
                use cu29::curuntime::CuRuntime;
                use cu29::curuntime::CuRuntimeBuilder;
                use cu29::curuntime::CuRuntimeParts;
                use cu29::curuntime::KeyFrame;
                use cu29::curuntime::RuntimeLifecycleConfigSource;
                use cu29::curuntime::RuntimeLifecycleEvent;
                use cu29::curuntime::RuntimeLifecycleRecord;
                use cu29::curuntime::RuntimeLifecycleStackInfo;
                use cu29::CuResult;
                use cu29::CuError;
                use cu29::cutask::CuSrcTask;
                use cu29::cutask::CuSinkTask;
                use cu29::cutask::CuTask;
                use cu29::cutask::CuMsg;
                use cu29::cutask::CuMsgMetadata;
                use cu29::copperlist::CopperList;
                use cu29::monitoring::CuMonitor; // Trait import.
                use cu29::monitoring::CuComponentState;
                use cu29::monitoring::Decision;
                use cu29::prelude::app::CuApplication;
                use cu29::prelude::debug;
                use cu29::prelude::stream_write;
                use cu29::prelude::UnifiedLogType;
                use cu29::prelude::UnifiedLogWrite;
                use cu29::prelude::WriteStream;

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
                #sim_bridge_channel_defs
                #resources_module
                #resources_instanciator_fn
                #task_mapping_defs
                #bridge_mapping_defs

                #sim_tasks
                #sim_support
                #recorded_replay_support
                #sim_tasks_instanciator

                pub const TASK_IDS: &'static [&'static str] = &[#( #task_ids ),*];
                pub const MONITORED_COMPONENTS: &'static [cu29::monitoring::MonitorComponentMetadata] =
                    &[#( #monitored_component_entries ),*];
                pub const CULIST_COMPONENT_MAPPING: &'static [cu29::monitoring::ComponentId] =
                    &[#( cu29::monitoring::ComponentId::new(#culist_component_mapping) ),*];
                pub const MONITOR_LAYOUT: cu29::monitoring::CopperListLayout =
                    cu29::monitoring::CopperListLayout::new(
                        MONITORED_COMPONENTS,
                        CULIST_COMPONENT_MAPPING,
                    );
                #parallel_rt_metadata_defs

                #[inline]
                pub fn monitor_component_label(
                    component_id: cu29::monitoring::ComponentId,
                ) -> &'static str {
                    MONITORED_COMPONENTS[component_id.index()].id()
                }

                #culist_support
                #parallel_rt_support_tokens

                #tasks_instanciator
                #bridges_instanciator

                pub fn monitor_instanciator(
                    config: &CuConfig,
                    metadata: ::cu29::monitoring::CuMonitoringMetadata,
                    runtime: ::cu29::monitoring::CuMonitoringRuntime,
                ) -> #monitor_type {
                    #monitor_instanciator_body
                }

                // The application for this mission
                #app_resources_struct
                pub #application_struct

                #app_inherent_impl
                #app_builder_inherent_impl
                #app_metadata_impl
                #app_reflect_impl
                #app_runtime_copperlist_impl
                #application_impl
                #recorded_replay_app_impl
                #distributed_replay_app_impl

                #std_application_impl

                #application_builder
            }

        };
        all_missions_tokens.push(mission_mod_tokens);
    }

    let default_application_tokens = if all_missions.contains_key("default") {
        let default_builder = quote! {
            #[allow(unused_imports)]
            use default::#builder_name;
        };
        quote! {
            #default_builder

            #[allow(unused_imports)]
            use default::AppResources;

            #[allow(unused_imports)]
            use default::resources as app_resources;

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

    result.into()
}

fn resolve_runtime_config(args: &CopperRuntimeArgs) -> CuResult<ResolvedRuntimeConfig> {
    let caller_root = utils::caller_crate_root();
    resolve_runtime_config_with_root(args, &caller_root)
}

fn resolve_runtime_config_with_root(
    args: &CopperRuntimeArgs,
    caller_root: &Path,
) -> CuResult<ResolvedRuntimeConfig> {
    let filename = config_full_path_from_root(caller_root, &args.config_path);
    if !Path::new(&filename).exists() {
        return Err(CuError::from(format!(
            "The configuration file `{}` does not exist. Please provide a valid path.",
            args.config_path
        )));
    }

    if let Some(subsystem_id) = args.subsystem_id.as_deref() {
        let multi_config = cu29_runtime::config::read_multi_configuration(filename.as_str())
            .map_err(|e| {
                CuError::from(format!(
                    "When `subsystem = \"{subsystem_id}\"` is provided, `config = \"{}\"` must point to a valid multi-Copper configuration: {e}",
                    args.config_path
                ))
            })?;
        let subsystem = multi_config.subsystem(subsystem_id).ok_or_else(|| {
            CuError::from(format!(
                "Subsystem '{subsystem_id}' was not found in multi-Copper configuration '{}'.",
                args.config_path
            ))
        })?;
        let bundled_local_config_content = read_to_string(&subsystem.config_path).map_err(|e| {
            CuError::from(format!(
                "Failed to read bundled local configuration for subsystem '{subsystem_id}' from '{}'.",
                subsystem.config_path
            ))
            .add_cause(e.to_string().as_str())
        })?;

        Ok(ResolvedRuntimeConfig {
            local_config: subsystem.config.clone(),
            bundled_local_config_content,
            subsystem_id: Some(subsystem_id.to_string()),
            subsystem_code: subsystem.subsystem_code,
        })
    } else {
        Ok(ResolvedRuntimeConfig {
            local_config: read_configuration(filename.as_str())?,
            bundled_local_config_content: read_to_string(&filename).map_err(|e| {
                CuError::from(format!(
                    "Could not read the configuration file '{}'.",
                    args.config_path
                ))
                .add_cause(e.to_string().as_str())
            })?,
            subsystem_id: None,
            subsystem_code: 0,
        })
    }
}

fn build_config_load_stmt(
    std_enabled: bool,
    application_name: &Ident,
    subsystem_id: Option<&str>,
) -> proc_macro2::TokenStream {
    if std_enabled {
        if let Some(subsystem_id) = subsystem_id {
            quote! {
                let (config, config_source) = if let Some(overridden_config) = config_override {
                    debug!("CuConfig: Overridden programmatically.");
                    (overridden_config, RuntimeLifecycleConfigSource::ProgrammaticOverride)
                } else if ::std::path::Path::new(config_filename).exists() {
                    let subsystem_id = #application_name::subsystem()
                        .id()
                        .expect("generated multi-Copper runtime is missing a subsystem id");
                    debug!(
                        "CuConfig: Reading multi-Copper configuration from file: {} (subsystem={})",
                        config_filename,
                        subsystem_id
                    );
                    let multi_config = cu29::config::read_multi_configuration(config_filename)?;
                    (
                        multi_config.resolve_subsystem_config_for_instance(subsystem_id, instance_id)?,
                        RuntimeLifecycleConfigSource::ExternalFile,
                    )
                } else {
                    let original_config = Self::original_config();
                    debug!(
                        "CuConfig: Using the bundled subsystem configuration compiled into the binary (subsystem={}).",
                        #subsystem_id
                    );
                    if instance_id != 0 {
                        debug!(
                            "CuConfig: runtime file '{}' is missing, so instance-specific overrides for instance_id={} cannot be resolved; using bundled subsystem defaults.",
                            config_filename,
                            instance_id
                        );
                    }
                    (
                        cu29::config::read_configuration_str(original_config, None)?,
                        RuntimeLifecycleConfigSource::BundledDefault,
                    )
                };
            }
        } else {
            quote! {
                let _ = instance_id;
                let (config, config_source) = if let Some(overridden_config) = config_override {
                    debug!("CuConfig: Overridden programmatically.");
                    (overridden_config, RuntimeLifecycleConfigSource::ProgrammaticOverride)
                } else if ::std::path::Path::new(config_filename).exists() {
                    debug!("CuConfig: Reading configuration from file: {}", config_filename);
                    (
                        cu29::config::read_configuration(config_filename)?,
                        RuntimeLifecycleConfigSource::ExternalFile,
                    )
                } else {
                    let original_config = Self::original_config();
                    debug!("CuConfig: Using the bundled configuration compiled into the binary.");
                    (
                        cu29::config::read_configuration_str(original_config, None)?,
                        RuntimeLifecycleConfigSource::BundledDefault,
                    )
                };
            }
        }
    } else {
        quote! {
            // Only the original config is available in no-std
            let original_config = Self::original_config();
            debug!("CuConfig: Using the bundled configuration compiled into the binary.");
            let config = cu29::config::read_configuration_str(original_config, None)?;
            let config_source = RuntimeLifecycleConfigSource::BundledDefault;
        }
    }
}

fn config_full_path(config_file: &str) -> String {
    config_full_path_from_root(&utils::caller_crate_root(), config_file)
}

fn config_full_path_from_root(caller_root: &Path, config_file: &str) -> String {
    let mut config_full_path = caller_root.to_path_buf();
    config_full_path.push(config_file);
    let filename = config_full_path
        .as_os_str()
        .to_str()
        .expect("Could not interpret the config file name");
    filename.to_string()
}

fn read_config(config_file: &str) -> CuResult<CuConfig> {
    let filename = config_full_path(config_file);
    read_configuration(filename.as_str())
}

fn extract_tasks_output_types(graph: &CuGraph) -> Vec<Option<Type>> {
    graph
        .get_all_nodes()
        .iter()
        .map(|(_, node)| {
            let id = node.get_id();
            let type_str = graph.get_node_output_msg_type(id.as_str());
            type_str.map(|type_str| {
                parse_str::<Type>(type_str.as_str()).expect("Could not parse output message type.")
            })
        })
        .collect()
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

        let cutypes: Vec<CuTaskType> = all_id_nodes
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
            .zip(cutypes.iter())
            .zip(background_flags.iter())
            .zip(output_types.iter())
            .map(|(((name, cutype), &background), output_type)| {
                let name_type = parse_str::<Type>(name).unwrap_or_else(|error| {
                    panic!("Could not transform {name} into a Task Rust type: {error}");
                });
                if background {
                    if let Some(output_type) = output_type {
                        match cutype {
                            CuTaskType::Source => {
                                parse_quote!(CuAsyncSrcTask<#name_type, #output_type>)
                            }
                            CuTaskType::Regular => {
                                parse_quote!(CuAsyncTask<#name_type, #output_type>)
                            }
                            CuTaskType::Sink => {
                                panic!("CuSinkTask {name} cannot be a background task, it should be a regular task.");
                            }
                        }
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
            .zip(cutypes.iter())
            .zip(background_flags.iter())
            .zip(output_types.iter())
            .map(|(((name, cutype), &background), output_type)| {
                let name_type = parse_str::<Type>(name).unwrap_or_else(|error| {
                    panic!("Could not transform {name} into a Task Rust type: {error}");
                });
                if background {
                    if let Some(output_type) = output_type {
                        match cutype {
                            CuTaskType::Source => {
                                parse_quote!(CuAsyncSrcTask::<#name_type, #output_type>)
                            }
                            CuTaskType::Regular => {
                                parse_quote!(CuAsyncTask::<#name_type, #output_type>)
                            }
                            CuTaskType::Sink => {
                                panic!("CuSinkTask {name} cannot be a background task, it should be a regular task.");
                            }
                        }
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

#[derive(Clone)]
struct OutputPack {
    msg_types: Vec<Type>,
    msg_type_names: Vec<String>,
}

impl OutputPack {
    fn slot_type(&self) -> Type {
        build_output_slot_type(&self.msg_types)
    }

    fn is_multi(&self) -> bool {
        self.msg_types.len() > 1
    }
}

fn build_output_slot_type(msg_types: &[Type]) -> Type {
    if msg_types.is_empty() {
        parse_quote! { () }
    } else if msg_types.len() == 1 {
        let msg_type = msg_types.first().unwrap();
        parse_quote! { CuMsg<#msg_type> }
    } else {
        parse_quote! { ( #( CuMsg<#msg_types> ),* ) }
    }
}

fn flatten_slot_origin_ids(
    output_packs: &[OutputPack],
    slot_origin_ids: &[Option<String>],
) -> Vec<String> {
    let mut ids = Vec::new();
    for (slot, pack) in output_packs.iter().enumerate() {
        if pack.msg_types.is_empty() {
            continue;
        }
        let origin = slot_origin_ids
            .get(slot)
            .and_then(|origin| origin.as_ref())
            .unwrap_or_else(|| panic!("Missing slot origin id for copperlist output slot {slot}"));
        for _ in 0..pack.msg_types.len() {
            ids.push(origin.clone());
        }
    }
    ids
}

fn flatten_task_output_specs(
    output_packs: &[OutputPack],
    slot_origin_ids: &[Option<String>],
) -> Vec<(String, String, String)> {
    let mut specs = Vec::new();
    for (slot, pack) in output_packs.iter().enumerate() {
        if pack.msg_types.is_empty() {
            continue;
        }
        let origin = slot_origin_ids
            .get(slot)
            .and_then(|origin| origin.as_ref())
            .unwrap_or_else(|| panic!("Missing slot origin id for copperlist output slot {slot}"));
        for (msg_type, payload_type) in pack.msg_type_names.iter().zip(pack.msg_types.iter()) {
            specs.push((
                origin.clone(),
                msg_type.clone(),
                rust_type_path_literal(payload_type),
            ));
        }
    }
    specs
}

fn rust_type_path_literal(ty: &Type) -> String {
    ty.to_token_stream()
        .to_string()
        .chars()
        .filter(|ch| !ch.is_whitespace())
        .collect()
}

fn extract_output_packs(runtime_plan: &CuExecutionLoop) -> Vec<OutputPack> {
    let mut packs: Vec<(u32, OutputPack)> = runtime_plan
        .steps
        .iter()
        .filter_map(|unit| match unit {
            CuExecutionUnit::Step(step) => {
                if let Some(output_pack) = &step.output_msg_pack {
                    let msg_types: Vec<Type> = output_pack
                        .msg_types
                        .iter()
                        .map(|output_msg_type| {
                            parse_str::<Type>(output_msg_type.as_str()).unwrap_or_else(|_| {
                                panic!(
                                    "Could not transform {output_msg_type} into a message Rust type."
                                )
                            })
                        })
                        .collect();
                    Some((
                        output_pack.culist_index,
                        OutputPack {
                            msg_types,
                            msg_type_names: output_pack.msg_types.clone(),
                        },
                    ))
                } else {
                    None
                }
            }
            CuExecutionUnit::Loop(_) => todo!("Needs to be implemented"),
        })
        .collect();

    packs.sort_by_key(|(index, _)| *index);
    packs.into_iter().map(|(_, pack)| pack).collect()
}

#[derive(Clone)]
struct SlotCodecBinding {
    payload_type: Type,
    task_id: String,
    msg_type: String,
    codec_type: syn::Path,
    codec_type_path: String,
}

fn build_flat_slot_codec_bindings(
    cuconfig: &CuConfig,
    mission_label: Option<&str>,
    output_packs: &[OutputPack],
    node_output_positions: &HashMap<NodeId, usize>,
    task_names: &[(NodeId, String, String)],
) -> CuResult<Vec<Option<SlotCodecBinding>>> {
    let mut slot_task_ids: Vec<Option<String>> = vec![None; output_packs.len()];
    for (node_id, task_id, _) in task_names {
        let Some(output_position) = node_output_positions.get(node_id) else {
            continue;
        };
        slot_task_ids[*output_position] = Some(task_id.clone());
    }

    let mut bindings =
        Vec::with_capacity(output_packs.iter().map(|pack| pack.msg_types.len()).sum());
    for (slot_idx, pack) in output_packs.iter().enumerate() {
        let task_id = slot_task_ids.get(slot_idx).and_then(|id| id.as_ref());
        for (port_idx, payload_type) in pack.msg_types.iter().enumerate() {
            let Some(task_id) = task_id else {
                bindings.push(None);
                continue;
            };
            let Some(msg_type) = pack.msg_type_names.get(port_idx) else {
                return Err(CuError::from(format!(
                    "Missing message type name for task '{task_id}' slot {slot_idx} port {port_idx}."
                )));
            };

            let spec = cuconfig
                .find_task_node(mission_label, task_id)
                .and_then(|node| node.get_logging())
                .and_then(|logging| logging.codec_for_msg_type(msg_type))
                .map(|codec_id| {
                    cuconfig.find_logging_codec_spec(codec_id).ok_or_else(|| {
                        CuError::from(format!(
                            "Task '{task_id}' binds output '{msg_type}' to unknown logging codec '{codec_id}'."
                        ))
                    })
                })
                .transpose()?;

            if let Some(spec) = spec {
                let codec_type = parse_str::<syn::Path>(&spec.type_).map_err(|_| {
                    CuError::from(format!(
                        "Logging codec '{}' for task '{task_id}' output '{msg_type}' is not a valid Rust type path.",
                        spec.type_
                    ))
                })?;
                bindings.push(Some(SlotCodecBinding {
                    payload_type: payload_type.clone(),
                    task_id: task_id.clone(),
                    msg_type: msg_type.clone(),
                    codec_type,
                    codec_type_path: spec.type_.clone(),
                }));
            } else {
                bindings.push(None);
            }
        }
    }

    Ok(bindings)
}

fn build_culist_codec_helpers(
    flat_codec_bindings: &[Option<SlotCodecBinding>],
    default_config_ron_ident: &Ident,
    mission_label: Option<&str>,
) -> (
    Vec<proc_macro2::TokenStream>,
    Vec<Option<Ident>>,
    Vec<Option<Ident>>,
) {
    let mission_tokens = if let Some(mission) = mission_label {
        let lit = LitStr::new(mission, Span::call_site());
        quote! { Some(#lit) }
    } else {
        quote! { None }
    };

    let mut helpers = Vec::new();
    let mut encode_helper_names = Vec::with_capacity(flat_codec_bindings.len());
    let mut decode_helper_names = Vec::with_capacity(flat_codec_bindings.len());

    for (flat_idx, binding) in flat_codec_bindings.iter().enumerate() {
        let Some(binding) = binding else {
            encode_helper_names.push(None);
            decode_helper_names.push(None);
            continue;
        };

        let encode_fn = format_ident!("__cu_logcodec_encode_slot_{flat_idx}");
        let decode_fn = format_ident!("__cu_logcodec_decode_slot_{flat_idx}");
        let payload_type = &binding.payload_type;
        let codec_type = &binding.codec_type;
        let task_id = LitStr::new(&binding.task_id, Span::call_site());
        let msg_type = LitStr::new(&binding.msg_type, Span::call_site());
        let codec_type_path = LitStr::new(&binding.codec_type_path, Span::call_site());

        helpers.push(quote! {
            fn #encode_fn<E: Encoder>(msg: &CuMsg<#payload_type>, encoder: &mut E) -> Result<(), EncodeError> {
                static STATE: ::cu29::logcodec::CodecState<#codec_type> = ::cu29::logcodec::CodecState::new();
                let config_entry = ::cu29::logcodec::effective_config_entry::<CuStampedDataSet>(#default_config_ron_ident);
                ::cu29::logcodec::with_codec_for_encode(
                    &STATE,
                    config_entry,
                    |effective_config_ron| {
                        ::cu29::logcodec::instantiate_codec::<#codec_type, #payload_type>(
                            effective_config_ron,
                            #mission_tokens,
                            #task_id,
                            #msg_type,
                            #codec_type_path,
                        )
                    },
                    |codec| ::cu29::logcodec::encode_msg_with_codec(msg, codec, encoder),
                )
            }

            fn #decode_fn<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<CuMsg<#payload_type>, DecodeError> {
                static STATE: ::cu29::logcodec::CodecState<#codec_type> = ::cu29::logcodec::CodecState::new();
                let config_entry = ::cu29::logcodec::effective_config_entry::<CuStampedDataSet>(#default_config_ron_ident);
                ::cu29::logcodec::with_codec_for_decode(
                    &STATE,
                    config_entry,
                    |effective_config_ron| {
                        ::cu29::logcodec::instantiate_codec::<#codec_type, #payload_type>(
                            effective_config_ron,
                            #mission_tokens,
                            #task_id,
                            #msg_type,
                            #codec_type_path,
                        )
                    },
                    |codec| ::cu29::logcodec::decode_msg_with_codec(decoder, codec),
                )
            }
        });
        encode_helper_names.push(Some(encode_fn));
        decode_helper_names.push(Some(decode_fn));
    }

    (helpers, encode_helper_names, decode_helper_names)
}

fn collect_output_pack_sizes(runtime_plan: &CuExecutionLoop) -> Vec<usize> {
    let mut sizes: Vec<(u32, usize)> = runtime_plan
        .steps
        .iter()
        .filter_map(|unit| match unit {
            CuExecutionUnit::Step(step) => step
                .output_msg_pack
                .as_ref()
                .map(|output_pack| (output_pack.culist_index, output_pack.msg_types.len())),
            CuExecutionUnit::Loop(_) => todo!("Needs to be implemented"),
        })
        .collect();

    sizes.sort_by_key(|(index, _)| *index);
    sizes.into_iter().map(|(_, size)| size).collect()
}

/// Builds the tuple of the CuList as a tuple off all the output slots.
fn build_culist_tuple(slot_types: &[Type]) -> TypeTuple {
    if slot_types.is_empty() {
        parse_quote! { () }
    } else {
        parse_quote! { ( #( #slot_types ),* ) }
    }
}

/// This is the bincode encoding part of the CuStampedDataSet
fn build_culist_tuple_encode(
    output_packs: &[OutputPack],
    encode_helper_names: &[Option<Ident>],
) -> ItemImpl {
    let mut flat_idx = 0usize;
    let mut encode_fields = Vec::new();

    for (slot_idx, pack) in output_packs.iter().enumerate() {
        let slot_index = syn::Index::from(slot_idx);
        if pack.is_multi() {
            for port_idx in 0..pack.msg_types.len() {
                let port_index = syn::Index::from(port_idx);
                let cache_index = flat_idx;
                let encode_helper = encode_helper_names[flat_idx].clone();
                flat_idx += 1;
                if let Some(encode_helper) = encode_helper {
                    encode_fields.push(quote! {
                        __cu_capture.select_slot(#cache_index);
                        #encode_helper(&self.0.#slot_index.#port_index, encoder)?;
                    });
                } else {
                    encode_fields.push(quote! {
                        __cu_capture.select_slot(#cache_index);
                        self.0.#slot_index.#port_index.encode(encoder)?;
                    });
                }
            }
        } else {
            let cache_index = flat_idx;
            let encode_helper = encode_helper_names[flat_idx].clone();
            flat_idx += 1;
            if let Some(encode_helper) = encode_helper {
                encode_fields.push(quote! {
                    __cu_capture.select_slot(#cache_index);
                    #encode_helper(&self.0.#slot_index, encoder)?;
                });
            } else {
                encode_fields.push(quote! {
                    __cu_capture.select_slot(#cache_index);
                    self.0.#slot_index.encode(encoder)?;
                });
            }
        }
    }

    parse_quote! {
        impl Encode for CuStampedDataSet {
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                let __cu_capture = cu29::monitoring::start_copperlist_io_capture(&self.1);
                #(#encode_fields)*
                Ok(())
            }
        }
    }
}

/// This is the bincode decoding part of the CuStampedDataSet
fn build_culist_tuple_decode(
    output_packs: &[OutputPack],
    slot_types: &[Type],
    cumsg_count: usize,
    decode_helper_names: &[Option<Ident>],
) -> ItemImpl {
    let mut flat_idx = 0usize;
    let mut decode_fields = Vec::with_capacity(slot_types.len());
    for (slot_idx, pack) in output_packs.iter().enumerate() {
        let slot_type = &slot_types[slot_idx];
        if pack.is_multi() {
            let mut slot_fields = Vec::with_capacity(pack.msg_types.len());
            for _ in 0..pack.msg_types.len() {
                let decode_helper = decode_helper_names[flat_idx].clone();
                flat_idx += 1;
                if let Some(decode_helper) = decode_helper {
                    slot_fields.push(quote! { #decode_helper(decoder)? });
                } else {
                    let msg_type = &pack.msg_types[slot_fields.len()];
                    slot_fields.push(quote! { <CuMsg<#msg_type> as Decode<()>>::decode(decoder)? });
                }
            }
            decode_fields.push(quote! { ( #(#slot_fields),* ) });
        } else if let Some(decode_helper) = decode_helper_names[flat_idx].clone() {
            flat_idx += 1;
            decode_fields.push(quote! { #decode_helper(decoder)? });
        } else {
            flat_idx += 1;
            decode_fields.push(quote! { <#slot_type as Decode<()>>::decode(decoder)? });
        }
    }

    parse_quote! {
        impl Decode<()> for CuStampedDataSet {
            fn decode<D: Decoder<Context=()>>(decoder: &mut D) -> Result<Self, DecodeError> {
                Ok(CuStampedDataSet(
                    (
                        #(#decode_fields),*
                    ),
                    cu29::monitoring::CuMsgIoCache::<#cumsg_count>::default(),
                ))
            }
        }
    }
}

fn build_culist_erasedcumsgs(output_packs: &[OutputPack]) -> ItemImpl {
    let mut casted_fields: Vec<proc_macro2::TokenStream> = Vec::new();
    for (idx, pack) in output_packs.iter().enumerate() {
        let slot_index = syn::Index::from(idx);
        if pack.is_multi() {
            for port_idx in 0..pack.msg_types.len() {
                let port_index = syn::Index::from(port_idx);
                casted_fields.push(quote! {
                    &self.0.#slot_index.#port_index as &dyn ErasedCuStampedData
                });
            }
        } else {
            casted_fields.push(quote! { &self.0.#slot_index as &dyn ErasedCuStampedData });
        }
    }
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

fn build_culist_tuple_debug(slot_types: &[Type]) -> ItemImpl {
    let indices: Vec<usize> = (0..slot_types.len()).collect();

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
fn build_culist_tuple_serialize(slot_types: &[Type]) -> ItemImpl {
    let indices: Vec<usize> = (0..slot_types.len()).collect();
    let tuple_len = slot_types.len();

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
fn build_culist_tuple_default(slot_types: &[Type], cumsg_count: usize) -> ItemImpl {
    let default_fields: Vec<_> = slot_types
        .iter()
        .map(|slot_type| quote! { <#slot_type as Default>::default() })
        .collect();

    parse_quote! {
        impl Default for CuStampedDataSet {
            fn default() -> CuStampedDataSet
            {
                CuStampedDataSet(
                    (
                        #(#default_fields),*
                    ),
                    cu29::monitoring::CuMsgIoCache::<#cumsg_count>::default(),
                )
            }
        }
    }
}

fn collect_bridge_channel_usage(graph: &CuGraph) -> HashMap<BridgeChannelKey, String> {
    let mut usage = HashMap::new();
    for cnx in graph.edges() {
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
                        let msg_type_name = msg_type.clone();
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
                            msg_type_name,
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
                        let msg_type_name = msg_type.clone();
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
                            msg_type_name,
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
            run_in_sim: bridge_cfg.is_run_in_sim(),
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

fn collect_task_names(graph: &CuGraph) -> Vec<(NodeId, String, String)> {
    graph
        .get_all_nodes()
        .iter()
        .filter(|(_, node)| node.get_flavor() == Flavor::Task)
        .map(|(node_id, node)| {
            (
                *node_id,
                node.get_id().to_string(),
                config_id_to_struct_member(node.get_id().as_str()),
            )
        })
        .collect()
}

#[derive(Clone, Copy)]
enum ResourceOwner {
    Task(usize),
    Bridge(usize),
}

#[derive(Clone)]
struct ResourceKeySpec {
    bundle_index: usize,
    provider_path: syn::Path,
    resource_name: String,
    binding_name: String,
    owner: ResourceOwner,
}

fn parse_resource_path(path: &str) -> CuResult<(String, String)> {
    let (bundle_id, name) = path.split_once('.').ok_or_else(|| {
        CuError::from(format!(
            "Resource '{path}' is missing a bundle prefix (expected bundle.resource)"
        ))
    })?;

    if bundle_id.is_empty() || name.is_empty() {
        return Err(CuError::from(format!(
            "Resource '{path}' must use the 'bundle.resource' format"
        )));
    }

    Ok((bundle_id.to_string(), name.to_string()))
}

fn collect_resource_specs(
    graph: &CuGraph,
    task_specs: &CuTaskSpecSet,
    bridge_specs: &[BridgeSpec],
    bundle_specs: &[BundleSpec],
) -> CuResult<Vec<ResourceKeySpec>> {
    let mut bridge_lookup: BTreeMap<String, usize> = BTreeMap::new();
    for (idx, spec) in bridge_specs.iter().enumerate() {
        bridge_lookup.insert(spec.id.clone(), idx);
    }

    let mut bundle_lookup: HashMap<String, (usize, syn::Path)> = HashMap::new();
    for (index, bundle) in bundle_specs.iter().enumerate() {
        bundle_lookup.insert(bundle.id.clone(), (index, bundle.provider_path.clone()));
    }

    let mut specs = Vec::new();

    for (node_id, node) in graph.get_all_nodes() {
        let resources = node.get_resources();
        if let Some(resources) = resources {
            let task_index = task_specs.node_id_to_task_index[node_id as usize];
            let owner = if let Some(task_index) = task_index {
                ResourceOwner::Task(task_index)
            } else if node.get_flavor() == Flavor::Bridge {
                let bridge_index = bridge_lookup.get(&node.get_id()).ok_or_else(|| {
                    CuError::from(format!(
                        "Resource mapping attached to unknown bridge node '{}'",
                        node.get_id()
                    ))
                })?;
                ResourceOwner::Bridge(*bridge_index)
            } else {
                return Err(CuError::from(format!(
                    "Resource mapping attached to non-task node '{}'",
                    node.get_id()
                )));
            };

            for (binding_name, path) in resources {
                let (bundle_id, resource_name) = parse_resource_path(path)?;
                let (bundle_index, provider_path) =
                    bundle_lookup.get(&bundle_id).ok_or_else(|| {
                        CuError::from(format!(
                            "Resource '{}' references unknown bundle '{}'",
                            path, bundle_id
                        ))
                    })?;
                specs.push(ResourceKeySpec {
                    bundle_index: *bundle_index,
                    provider_path: provider_path.clone(),
                    resource_name,
                    binding_name: binding_name.clone(),
                    owner,
                });
            }
        }
    }

    Ok(specs)
}

fn build_bundle_list<'a>(config: &'a CuConfig, mission: &str) -> Vec<&'a ResourceBundleConfig> {
    config
        .resources
        .iter()
        .filter(|bundle| {
            bundle
                .missions
                .as_ref()
                .is_none_or(|missions| missions.iter().any(|m| m == mission))
        })
        .collect()
}

struct BundleSpec {
    id: String,
    provider_path: syn::Path,
}

fn build_bundle_specs(config: &CuConfig, mission: &str) -> CuResult<Vec<BundleSpec>> {
    build_bundle_list(config, mission)
        .into_iter()
        .map(|bundle| {
            let provider_path: syn::Path =
                syn::parse_str(bundle.provider.as_str()).map_err(|err| {
                    CuError::from(format!(
                        "Failed to parse provider path '{}' for bundle '{}': {err}",
                        bundle.provider, bundle.id
                    ))
                })?;
            Ok(BundleSpec {
                id: bundle.id.clone(),
                provider_path,
            })
        })
        .collect()
}

fn build_resources_module(
    bundle_specs: &[BundleSpec],
) -> CuResult<(proc_macro2::TokenStream, proc_macro2::TokenStream)> {
    let bundle_consts = bundle_specs.iter().enumerate().map(|(index, bundle)| {
        let const_ident = Ident::new(
            &config_id_to_bridge_const(bundle.id.as_str()),
            Span::call_site(),
        );
        quote! { pub const #const_ident: BundleIndex = BundleIndex::new(#index); }
    });

    let resources_module = quote! {
        pub mod resources {
            #![allow(dead_code)]
            use cu29::resource::BundleIndex;

            pub mod bundles {
                use super::BundleIndex;
                #(#bundle_consts)*
            }
        }
    };

    let bundle_counts = bundle_specs.iter().map(|bundle| {
        let provider_path = &bundle.provider_path;
        quote! { <#provider_path as cu29::resource::ResourceBundleDecl>::Id::COUNT }
    });

    let bundle_inits = bundle_specs
        .iter()
        .enumerate()
        .map(|(index, bundle)| {
            let bundle_id = LitStr::new(bundle.id.as_str(), Span::call_site());
            let provider_path = &bundle.provider_path;
            quote! {
                let bundle_cfg = config
                    .resources
                    .iter()
                    .find(|b| b.id == #bundle_id)
                    .unwrap_or_else(|| panic!("Resource bundle '{}' missing from configuration", #bundle_id));
                let bundle_ctx = cu29::resource::BundleContext::<#provider_path>::new(
                    cu29::resource::BundleIndex::new(#index),
                    #bundle_id,
                );
                <#provider_path as cu29::resource::ResourceBundle>::build(
                    bundle_ctx,
                    bundle_cfg.config.as_ref(),
                    &mut manager,
                )?;
            }
            })
            .collect::<Vec<_>>();

    let resources_instanciator = quote! {
        pub fn resources_instanciator(config: &CuConfig) -> CuResult<cu29::resource::ResourceManager> {
            let bundle_counts: &[usize] = &[ #(#bundle_counts),* ];
            let mut manager = cu29::resource::ResourceManager::new(bundle_counts);
            #(#bundle_inits)*
            Ok(manager)
        }
    };

    Ok((resources_module, resources_instanciator))
}

struct ResourceMappingTokens {
    defs: proc_macro2::TokenStream,
    refs: Vec<proc_macro2::TokenStream>,
}

fn build_task_resource_mappings(
    resource_specs: &[ResourceKeySpec],
    task_specs: &CuTaskSpecSet,
) -> CuResult<ResourceMappingTokens> {
    let mut per_task: Vec<Vec<&ResourceKeySpec>> = vec![Vec::new(); task_specs.ids.len()];

    for spec in resource_specs {
        let ResourceOwner::Task(task_index) = spec.owner else {
            continue;
        };
        per_task
            .get_mut(task_index)
            .ok_or_else(|| {
                CuError::from(format!(
                    "Resource '{}' mapped to invalid task index {}",
                    spec.binding_name, task_index
                ))
            })?
            .push(spec);
    }

    let mut mapping_defs = Vec::new();
    let mut mapping_refs = Vec::new();

    for (idx, entries) in per_task.iter().enumerate() {
        if entries.is_empty() {
            mapping_refs.push(quote! { None });
            continue;
        }

        let binding_task_type = if task_specs.background_flags[idx] {
            &task_specs.sim_task_types[idx]
        } else {
            &task_specs.task_types[idx]
        };

        let binding_trait = match task_specs.cutypes[idx] {
            CuTaskType::Source => quote! { CuSrcTask },
            CuTaskType::Regular => quote! { CuTask },
            CuTaskType::Sink => quote! { CuSinkTask },
        };

        let entries_ident = format_ident!("TASK{}_RES_ENTRIES", idx);
        let map_ident = format_ident!("TASK{}_RES_MAPPING", idx);
        let binding_type = quote! {
            <<#binding_task_type as #binding_trait>::Resources<'_> as ResourceBindings>::Binding
        };
        let entry_tokens = entries.iter().map(|spec| {
            let binding_ident =
                Ident::new(&config_id_to_enum(spec.binding_name.as_str()), Span::call_site());
            let resource_ident =
                Ident::new(&config_id_to_enum(spec.resource_name.as_str()), Span::call_site());
            let bundle_index = spec.bundle_index;
            let provider_path = &spec.provider_path;
            quote! {
                (#binding_type::#binding_ident, cu29::resource::ResourceKey::new(
                    cu29::resource::BundleIndex::new(#bundle_index),
                    <#provider_path as cu29::resource::ResourceBundleDecl>::Id::#resource_ident as usize,
                ))
            }
        });

        mapping_defs.push(quote! {
            const #entries_ident: &[(#binding_type, cu29::resource::ResourceKey)] = &[ #(#entry_tokens),* ];
            const #map_ident: cu29::resource::ResourceBindingMap<#binding_type> =
                cu29::resource::ResourceBindingMap::new(#entries_ident);
        });
        mapping_refs.push(quote! { Some(&#map_ident) });
    }

    Ok(ResourceMappingTokens {
        defs: quote! { #(#mapping_defs)* },
        refs: mapping_refs,
    })
}

fn build_bridge_resource_mappings(
    resource_specs: &[ResourceKeySpec],
    bridge_specs: &[BridgeSpec],
    sim_mode: bool,
) -> ResourceMappingTokens {
    let mut per_bridge: Vec<Vec<&ResourceKeySpec>> = vec![Vec::new(); bridge_specs.len()];

    for spec in resource_specs {
        let ResourceOwner::Bridge(bridge_index) = spec.owner else {
            continue;
        };
        if sim_mode && !bridge_specs[bridge_index].run_in_sim {
            continue;
        }
        per_bridge[bridge_index].push(spec);
    }

    let mut mapping_defs = Vec::new();
    let mut mapping_refs = Vec::new();

    for (idx, entries) in per_bridge.iter().enumerate() {
        if entries.is_empty() {
            mapping_refs.push(quote! { None });
            continue;
        }

        let bridge_type = &bridge_specs[idx].type_path;
        let binding_type = quote! {
            <<#bridge_type as cu29::cubridge::CuBridge>::Resources<'_> as ResourceBindings>::Binding
        };
        let entries_ident = format_ident!("BRIDGE{}_RES_ENTRIES", idx);
        let map_ident = format_ident!("BRIDGE{}_RES_MAPPING", idx);
        let entry_tokens = entries.iter().map(|spec| {
            let binding_ident =
                Ident::new(&config_id_to_enum(spec.binding_name.as_str()), Span::call_site());
            let resource_ident =
                Ident::new(&config_id_to_enum(spec.resource_name.as_str()), Span::call_site());
            let bundle_index = spec.bundle_index;
            let provider_path = &spec.provider_path;
            quote! {
                (#binding_type::#binding_ident, cu29::resource::ResourceKey::new(
                    cu29::resource::BundleIndex::new(#bundle_index),
                    <#provider_path as cu29::resource::ResourceBundleDecl>::Id::#resource_ident as usize,
                ))
            }
        });

        mapping_defs.push(quote! {
            const #entries_ident: &[(#binding_type, cu29::resource::ResourceKey)] = &[ #(#entry_tokens),* ];
            const #map_ident: cu29::resource::ResourceBindingMap<#binding_type> =
                cu29::resource::ResourceBindingMap::new(#entries_ident);
        });
        mapping_refs.push(quote! { Some(&#map_ident) });
    }

    ResourceMappingTokens {
        defs: quote! { #(#mapping_defs)* },
        refs: mapping_refs,
    }
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

    for cnx in graph.edges() {
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
            .connect_ext_with_order(
                src_plan,
                dst_plan,
                &cnx.msg,
                cnx.missions.clone(),
                None,
                None,
                cnx.order,
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
        if let CuExecutionUnit::Step(step) = unit
            && let Some(output_pack) = &step.output_msg_pack
        {
            let output_idx = output_pack.culist_index;
            culist_order.push(output_idx as usize);
            match &exec_entities[step.node_id as usize].kind {
                ExecutionEntityKind::Task { .. } => {
                    if let Some(original_node_id) = plan_to_original.get(&step.node_id) {
                        node_output_positions.insert(*original_node_id, output_idx as usize);
                    }
                }
                ExecutionEntityKind::BridgeRx {
                    bridge_index,
                    channel_index,
                } => {
                    bridge_specs[*bridge_index].rx_channels[*channel_index].culist_index =
                        Some(output_idx as usize);
                }
                ExecutionEntityKind::BridgeTx {
                    bridge_index,
                    channel_index,
                } => {
                    bridge_specs[*bridge_index].tx_channels[*channel_index].culist_index =
                        Some(output_idx as usize);
                }
            }
        }
    }

    (culist_order, node_output_positions)
}

fn build_monitor_culist_component_mapping(
    runtime_plan: &CuExecutionLoop,
    exec_entities: &[ExecutionEntity],
    bridge_specs: &[BridgeSpec],
) -> Result<Vec<usize>, String> {
    let mut mapping = Vec::new();
    for unit in &runtime_plan.steps {
        if let CuExecutionUnit::Step(step) = unit
            && step.output_msg_pack.is_some()
        {
            let Some(entity) = exec_entities.get(step.node_id as usize) else {
                return Err(format!(
                    "Missing execution entity for plan node {} while building monitor mapping",
                    step.node_id
                ));
            };
            let component_index = match &entity.kind {
                ExecutionEntityKind::Task { task_index } => *task_index,
                ExecutionEntityKind::BridgeRx {
                    bridge_index,
                    channel_index,
                } => bridge_specs
                    .get(*bridge_index)
                    .and_then(|spec| spec.rx_channels.get(*channel_index))
                    .and_then(|channel| channel.monitor_index)
                    .ok_or_else(|| {
                        format!(
                            "Missing monitor index for bridge rx {}:{}",
                            bridge_index, channel_index
                        )
                    })?,
                ExecutionEntityKind::BridgeTx {
                    bridge_index,
                    channel_index,
                } => bridge_specs
                    .get(*bridge_index)
                    .and_then(|spec| spec.tx_channels.get(*channel_index))
                    .and_then(|channel| channel.monitor_index)
                    .ok_or_else(|| {
                        format!(
                            "Missing monitor index for bridge tx {}:{}",
                            bridge_index, channel_index
                        )
                    })?,
            };
            mapping.push(component_index);
        }
    }
    Ok(mapping)
}

fn build_parallel_rt_stage_entries(
    runtime_plan: &CuExecutionLoop,
    exec_entities: &[ExecutionEntity],
    task_specs: &CuTaskSpecSet,
    bridge_specs: &[BridgeSpec],
) -> CuResult<Vec<proc_macro2::TokenStream>> {
    let mut entries = Vec::new();

    for unit in &runtime_plan.steps {
        let CuExecutionUnit::Step(step) = unit else {
            todo!("parallel runtime metadata for nested loops is not implemented yet")
        };

        let entity = exec_entities.get(step.node_id as usize).ok_or_else(|| {
            CuError::from(format!(
                "Missing execution entity for runtime plan node {} while building parallel runtime metadata",
                step.node_id
            ))
        })?;

        let (label, kind_tokens, component_index) = match &entity.kind {
            ExecutionEntityKind::Task { task_index } => (
                task_specs
                    .ids
                    .get(*task_index)
                    .cloned()
                    .ok_or_else(|| {
                        CuError::from(format!(
                            "Missing task id for task index {} while building parallel runtime metadata",
                            task_index
                        ))
                    })?,
                quote! { cu29::parallel_rt::ParallelRtStageKind::Task },
                *task_index,
            ),
            ExecutionEntityKind::BridgeRx {
                bridge_index,
                channel_index,
            } => {
                let bridge = bridge_specs.get(*bridge_index).ok_or_else(|| {
                    CuError::from(format!(
                        "Missing bridge spec {} while building parallel runtime metadata",
                        bridge_index
                    ))
                })?;
                let channel = bridge.rx_channels.get(*channel_index).ok_or_else(|| {
                    CuError::from(format!(
                        "Missing bridge rx channel {}:{} while building parallel runtime metadata",
                        bridge_index, channel_index
                    ))
                })?;
                let component_index = channel.monitor_index.ok_or_else(|| {
                    CuError::from(format!(
                        "Missing monitor index for bridge rx {}:{} while building parallel runtime metadata",
                        bridge_index, channel_index
                    ))
                })?;
                (
                    format!("bridge::{}::rx::{}", bridge.id, channel.id),
                    quote! { cu29::parallel_rt::ParallelRtStageKind::BridgeRx },
                    component_index,
                )
            }
            ExecutionEntityKind::BridgeTx {
                bridge_index,
                channel_index,
            } => {
                let bridge = bridge_specs.get(*bridge_index).ok_or_else(|| {
                    CuError::from(format!(
                        "Missing bridge spec {} while building parallel runtime metadata",
                        bridge_index
                    ))
                })?;
                let channel = bridge.tx_channels.get(*channel_index).ok_or_else(|| {
                    CuError::from(format!(
                        "Missing bridge tx channel {}:{} while building parallel runtime metadata",
                        bridge_index, channel_index
                    ))
                })?;
                let component_index = channel.monitor_index.ok_or_else(|| {
                    CuError::from(format!(
                        "Missing monitor index for bridge tx {}:{} while building parallel runtime metadata",
                        bridge_index, channel_index
                    ))
                })?;
                (
                    format!("bridge::{}::tx::{}", bridge.id, channel.id),
                    quote! { cu29::parallel_rt::ParallelRtStageKind::BridgeTx },
                    component_index,
                )
            }
        };

        let node_id = step.node_id;
        entries.push(quote! {
            cu29::parallel_rt::ParallelRtStageMetadata::new(
                #label,
                #kind_tokens,
                #node_id,
                cu29::monitoring::ComponentId::new(#component_index),
            )
        });
    }

    Ok(entries)
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

fn wrap_process_step_tokens(
    wrap_process_step: bool,
    body: proc_macro2::TokenStream,
) -> proc_macro2::TokenStream {
    if wrap_process_step {
        quote! {{
            let __cu_process_step_result: cu29::curuntime::ProcessStepResult = (|| {
                #body
                Ok(cu29::curuntime::ProcessStepOutcome::Continue)
            })();
            __cu_process_step_result
        }}
    } else {
        body
    }
}

fn abort_process_step_tokens(wrap_process_step: bool) -> proc_macro2::TokenStream {
    if wrap_process_step {
        quote! {
            return Ok(cu29::curuntime::ProcessStepOutcome::AbortCopperList);
        }
    } else {
        quote! {
            __cu_abort_copperlist = true;
            break '__cu_process_steps;
        }
    }
}

fn parallel_task_lifecycle_tokens(
    task_kind: CuTaskType,
    task_type: &Type,
    component_index: usize,
    mission_mod: &Ident,
    task_instance: &proc_macro2::TokenStream,
    placement: ParallelLifecyclePlacement,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let rt_guard = rtsan_guard_tokens();
    let abort_process_step = abort_process_step_tokens(true);
    let task_trait = match task_kind {
        CuTaskType::Source => quote! { cu29::cutask::CuSrcTask },
        CuTaskType::Sink => quote! { cu29::cutask::CuSinkTask },
        CuTaskType::Regular => quote! { cu29::cutask::CuTask },
    };

    let preprocess = if placement.preprocess {
        quote! {
            execution_probe.record(cu29::monitoring::ExecutionMarker {
                component_id: cu29::monitoring::ComponentId::new(#component_index),
                step: CuComponentState::Preprocess,
                culistid: Some(clid),
            });
            ctx.set_current_task(#component_index);
            let maybe_error = {
                #rt_guard
                <#task_type as #task_trait>::preprocess(&mut #task_instance, &ctx)
            };
            if let Err(error) = maybe_error {
                let decision = monitor.process_error(
                    cu29::monitoring::ComponentId::new(#component_index),
                    CuComponentState::Preprocess,
                    &error,
                );
                match decision {
                    Decision::Abort => {
                        debug!(
                            "Preprocess: ABORT decision from monitoring. Component '{}' errored out during preprocess. Aborting CopperList {}.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index)),
                            clid
                        );
                        #abort_process_step
                    }
                    Decision::Ignore => {
                        debug!(
                            "Preprocess: IGNORE decision from monitoring. Component '{}' errored out during preprocess. The runtime will continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Shutdown => {
                        debug!(
                            "Preprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during preprocess. The runtime cannot continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                        return Err(CuError::new_with_cause(
                            "Component errored out during preprocess.",
                            error,
                        ));
                    }
                }
            }
        }
    } else {
        quote! {}
    };

    let postprocess = if placement.postprocess {
        quote! {
            execution_probe.record(cu29::monitoring::ExecutionMarker {
                component_id: cu29::monitoring::ComponentId::new(#component_index),
                step: CuComponentState::Postprocess,
                culistid: Some(clid),
            });
            ctx.set_current_task(#component_index);
            let maybe_error = {
                #rt_guard
                <#task_type as #task_trait>::postprocess(&mut #task_instance, &ctx)
            };
            if let Err(error) = maybe_error {
                let decision = monitor.process_error(
                    cu29::monitoring::ComponentId::new(#component_index),
                    CuComponentState::Postprocess,
                    &error,
                );
                match decision {
                    Decision::Abort => {
                        debug!(
                            "Postprocess: ABORT decision from monitoring. Component '{}' errored out during postprocess. Continuing with the completed CopperList.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Ignore => {
                        debug!(
                            "Postprocess: IGNORE decision from monitoring. Component '{}' errored out during postprocess. The runtime will continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Shutdown => {
                        debug!(
                            "Postprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during postprocess. The runtime cannot continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                        return Err(CuError::new_with_cause(
                            "Component errored out during postprocess.",
                            error,
                        ));
                    }
                }
            }
        }
    } else {
        quote! {}
    };

    (preprocess, postprocess)
}

fn parallel_bridge_lifecycle_tokens(
    bridge_type: &Type,
    component_index: usize,
    mission_mod: &Ident,
    placement: ParallelLifecyclePlacement,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let rt_guard = rtsan_guard_tokens();
    let abort_process_step = abort_process_step_tokens(true);

    let preprocess = if placement.preprocess {
        quote! {
            execution_probe.record(cu29::monitoring::ExecutionMarker {
                component_id: cu29::monitoring::ComponentId::new(#component_index),
                step: CuComponentState::Preprocess,
                culistid: Some(clid),
            });
            ctx.clear_current_task();
            let maybe_error = {
                #rt_guard
                <#bridge_type as cu29::cubridge::CuBridge>::preprocess(bridge, &ctx)
            };
            if let Err(error) = maybe_error {
                let decision = monitor.process_error(
                    cu29::monitoring::ComponentId::new(#component_index),
                    CuComponentState::Preprocess,
                    &error,
                );
                match decision {
                    Decision::Abort => {
                        debug!(
                            "Preprocess: ABORT decision from monitoring. Component '{}' errored out during preprocess. Aborting CopperList {}.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index)),
                            clid
                        );
                        #abort_process_step
                    }
                    Decision::Ignore => {
                        debug!(
                            "Preprocess: IGNORE decision from monitoring. Component '{}' errored out during preprocess. The runtime will continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Shutdown => {
                        debug!(
                            "Preprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during preprocess. The runtime cannot continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                        return Err(CuError::new_with_cause(
                            "Component errored out during preprocess.",
                            error,
                        ));
                    }
                }
            }
        }
    } else {
        quote! {}
    };

    let postprocess = if placement.postprocess {
        quote! {
            kf_manager.freeze_any(clid, bridge)?;
            execution_probe.record(cu29::monitoring::ExecutionMarker {
                component_id: cu29::monitoring::ComponentId::new(#component_index),
                step: CuComponentState::Postprocess,
                culistid: Some(clid),
            });
            ctx.clear_current_task();
            let maybe_error = {
                #rt_guard
                <#bridge_type as cu29::cubridge::CuBridge>::postprocess(bridge, &ctx)
            };
            if let Err(error) = maybe_error {
                let decision = monitor.process_error(
                    cu29::monitoring::ComponentId::new(#component_index),
                    CuComponentState::Postprocess,
                    &error,
                );
                match decision {
                    Decision::Abort => {
                        debug!(
                            "Postprocess: ABORT decision from monitoring. Component '{}' errored out during postprocess. Continuing with the completed CopperList.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Ignore => {
                        debug!(
                            "Postprocess: IGNORE decision from monitoring. Component '{}' errored out during postprocess. The runtime will continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Shutdown => {
                        debug!(
                            "Postprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during postprocess. The runtime cannot continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                        return Err(CuError::new_with_cause(
                            "Component errored out during postprocess.",
                            error,
                        ));
                    }
                }
            }
        }
    } else {
        quote! {}
    };

    (preprocess, postprocess)
}

#[derive(Clone, Copy)]
struct StepGenerationContext<'a> {
    output_pack_sizes: &'a [usize],
    sim_mode: bool,
    mission_mod: &'a Ident,
    lifecycle_placement: ParallelLifecyclePlacement,
    wrap_process_step: bool,
}

impl<'a> StepGenerationContext<'a> {
    fn new(
        output_pack_sizes: &'a [usize],
        sim_mode: bool,
        mission_mod: &'a Ident,
        lifecycle_placement: ParallelLifecyclePlacement,
        wrap_process_step: bool,
    ) -> Self {
        Self {
            output_pack_sizes,
            sim_mode,
            mission_mod,
            lifecycle_placement,
            wrap_process_step,
        }
    }
}

struct TaskExecutionTokens {
    setup: proc_macro2::TokenStream,
    instance: proc_macro2::TokenStream,
}

impl TaskExecutionTokens {
    fn new(setup: proc_macro2::TokenStream, instance: proc_macro2::TokenStream) -> Self {
        Self { setup, instance }
    }
}

fn generate_task_execution_tokens(
    step: &CuExecutionStep,
    task_index: usize,
    task_specs: &CuTaskSpecSet,
    ctx: StepGenerationContext<'_>,
    task_tokens: TaskExecutionTokens,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let StepGenerationContext {
        output_pack_sizes,
        sim_mode,
        mission_mod,
        lifecycle_placement,
        wrap_process_step,
    } = ctx;
    let TaskExecutionTokens {
        setup: task_setup,
        instance: task_instance,
    } = task_tokens;
    let abort_process_step = abort_process_step_tokens(wrap_process_step);
    let comment_str = format!(
        "DEBUG ->> {} ({:?}) Id:{} I:{:?} O:{:?}",
        step.node.get_id(),
        step.task_type,
        step.node_id,
        step.input_msg_indices_types,
        step.output_msg_pack
    );
    let comment_tokens = quote! {{
        let _ = stringify!(#comment_str);
    }};
    let tid = task_index;
    let task_enum_name = config_id_to_enum(&task_specs.ids[tid]);
    let enum_name = Ident::new(&task_enum_name, Span::call_site());
    let task_hint = config_id_to_struct_member(&task_specs.ids[tid]);
    let source_slot_match_trait_ident = format_ident!(
        "__CuOutputSlotMustMatchTaskOutput__Task_{}__Add_dst___nc___connections_for_unused_outputs",
        task_hint
    );
    let source_slot_match_fn_ident = format_ident!(
        "__cu_source_output_slot_or_add_dst___nc___for_unused_outputs__task_{}",
        task_hint
    );
    let regular_slot_match_trait_ident = format_ident!(
        "__CuOutputSlotMustMatchTaskOutput__Task_{}__Add_dst___nc___connections_for_unused_outputs",
        task_hint
    );
    let regular_slot_match_fn_ident = format_ident!(
        "__cu_task_output_slot_or_add_dst___nc___for_unused_outputs__task_{}",
        task_hint
    );
    let rt_guard = rtsan_guard_tokens();
    let run_in_sim_flag = task_specs.run_in_sim_flags[tid];
    let task_type = &task_specs.task_types[tid];
    let (parallel_task_preprocess, parallel_task_postprocess) = parallel_task_lifecycle_tokens(
        step.task_type,
        task_type,
        tid,
        mission_mod,
        &task_instance,
        lifecycle_placement,
    );
    let maybe_sim_tick = if sim_mode && !run_in_sim_flag {
        quote! {
            if !doit {
                #task_instance.sim_tick();
            }
        }
    } else {
        quote!()
    };

    let output_pack = step
        .output_msg_pack
        .as_ref()
        .expect("Task should have an output message pack.");
    let output_culist_index = int2sliceindex(output_pack.culist_index);
    let output_ports: Vec<syn::Index> = (0..output_pack.msg_types.len())
        .map(syn::Index::from)
        .collect();
    let output_clear_payload = if output_ports.len() == 1 {
        quote! { cumsg_output.clear_payload(); }
    } else {
        quote! { #(cumsg_output.#output_ports.clear_payload();)* }
    };
    let output_start_time = if output_ports.len() == 1 {
        quote! {
            if cumsg_output.metadata.process_time.start.is_none() {
                cumsg_output.metadata.process_time.start = cu29::curuntime::perf_now(clock).into();
            }
        }
    } else {
        quote! {
            let start_time = cu29::curuntime::perf_now(clock).into();
            #( if cumsg_output.#output_ports.metadata.process_time.start.is_none() {
                cumsg_output.#output_ports.metadata.process_time.start = start_time;
            } )*
        }
    };
    let output_end_time = if output_ports.len() == 1 {
        quote! {
            if cumsg_output.metadata.process_time.end.is_none() {
                cumsg_output.metadata.process_time.end = cu29::curuntime::perf_now(clock).into();
            }
        }
    } else {
        quote! {
            let end_time = cu29::curuntime::perf_now(clock).into();
            #( if cumsg_output.#output_ports.metadata.process_time.end.is_none() {
                cumsg_output.#output_ports.metadata.process_time.end = end_time;
            } )*
        }
    };

    match step.task_type {
        CuTaskType::Source => {
            let monitoring_action = quote! {
                debug!("Component {}: Error during process: {}", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)), &error);
                let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#tid), CuComponentState::Process, &error);
                match decision {
                    Decision::Abort => {
                        debug!("Process: ABORT decision from monitoring. Component '{}' errored out \
                                during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)), clid);
                        #abort_process_step
                    }
                    Decision::Ignore => {
                        debug!("Process: IGNORE decision from monitoring. Component '{}' errored out \
                                during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)));
                        let cumsg_output = &mut msgs.#output_culist_index;
                        #output_clear_payload
                    }
                    Decision::Shutdown => {
                        debug!("Process: SHUTDOWN decision from monitoring. Component '{}' errored out \
                                during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)));
                        return Err(CuError::new_with_cause("Component errored out during process.", error));
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
                quote! {
                    let mut cumsg_output = &mut culist.msgs.0.#output_culist_index;
                    #output_clear_payload
                }
            } else {
                quote!()
            };
            let source_process_tokens = quote! {
                #[allow(non_camel_case_types)]
                trait #source_slot_match_trait_ident<Expected> {
                    fn __cu_cast_output_slot(slot: &mut Self) -> &mut Expected;
                }
                impl<T> #source_slot_match_trait_ident<T> for T {
                    fn __cu_cast_output_slot(slot: &mut Self) -> &mut T {
                        slot
                    }
                }

                fn #source_slot_match_fn_ident<'a, Task, Slot>(
                    _task: &Task,
                    slot: &'a mut Slot,
                ) -> &'a mut Task::Output<'static>
                where
                    Task: cu29::cutask::CuSrcTask,
                    Slot: #source_slot_match_trait_ident<Task::Output<'static>>,
                {
                    <Slot as #source_slot_match_trait_ident<Task::Output<'static>>>::__cu_cast_output_slot(slot)
                }

                #output_start_time
                let result = {
                    let cumsg_output = #source_slot_match_fn_ident::<
                        _,
                        _,
                    >(&#task_instance, cumsg_output);
                    #rt_guard
                    ctx.set_current_task(#tid);
                    #task_instance.process(&ctx, cumsg_output)
                };
                #output_end_time
                result
            };

            (
                wrap_process_step_tokens(
                    wrap_process_step,
                    quote! {
                        #task_setup
                        #parallel_task_preprocess
                        #comment_tokens
                        kf_manager.freeze_task(clid, &#task_instance)?;
                        #call_sim_callback
                        let cumsg_output = &mut msgs.#output_culist_index;
                        #maybe_sim_tick
                        let maybe_error = if doit {
                            execution_probe.record(cu29::monitoring::ExecutionMarker {
                                component_id: cu29::monitoring::ComponentId::new(#tid),
                                step: CuComponentState::Process,
                                culistid: Some(clid),
                            });
                            #source_process_tokens
                        } else {
                            Ok(())
                        };
                        if let Err(error) = maybe_error {
                            #monitoring_action
                        }
                        #parallel_task_postprocess
                    },
                ),
                logging_tokens,
            )
        }
        CuTaskType::Sink => {
            let input_exprs: Vec<proc_macro2::TokenStream> = step
                .input_msg_indices_types
                .iter()
                .map(|input| {
                    let input_index = int2sliceindex(input.culist_index);
                    let output_size = output_pack_sizes
                        .get(input.culist_index as usize)
                        .copied()
                        .unwrap_or_else(|| {
                            panic!(
                                "Missing output pack size for culist index {}",
                                input.culist_index
                            )
                        });
                    if output_size > 1 {
                        let port_index = syn::Index::from(input.src_port);
                        quote! { msgs.#input_index.#port_index }
                    } else {
                        quote! { msgs.#input_index }
                    }
                })
                .collect();
            let inputs_type = if input_exprs.len() == 1 {
                let input = input_exprs.first().unwrap();
                quote! { #input }
            } else {
                quote! { (#(&#input_exprs),*) }
            };

            let monitoring_action = quote! {
                debug!("Component {}: Error during process: {}", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)), &error);
                let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#tid), CuComponentState::Process, &error);
                match decision {
                    Decision::Abort => {
                        debug!("Process: ABORT decision from monitoring. Component '{}' errored out \
                                during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)), clid);
                        #abort_process_step
                    }
                    Decision::Ignore => {
                        debug!("Process: IGNORE decision from monitoring. Component '{}' errored out \
                                during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)));
                        let cumsg_output = &mut msgs.#output_culist_index;
                        #output_clear_payload
                    }
                    Decision::Shutdown => {
                        debug!("Process: SHUTDOWN decision from monitoring. Component '{}' errored out \
                                during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)));
                        return Err(CuError::new_with_cause("Component errored out during process.", error));
                    }
                }
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
                wrap_process_step_tokens(
                    wrap_process_step,
                    quote! {
                        #task_setup
                        #parallel_task_preprocess
                        #comment_tokens
                        kf_manager.freeze_task(clid, &#task_instance)?;
                        #call_sim_callback
                        let cumsg_input = &#inputs_type;
                        let cumsg_output = &mut msgs.#output_culist_index;
                        let maybe_error = if doit {
                            execution_probe.record(cu29::monitoring::ExecutionMarker {
                                component_id: cu29::monitoring::ComponentId::new(#tid),
                                step: CuComponentState::Process,
                                culistid: Some(clid),
                            });
                            #output_start_time
                            let result = {
                                #rt_guard
                                ctx.set_current_task(#tid);
                                #task_instance.process(&ctx, cumsg_input)
                            };
                            #output_end_time
                            result
                        } else {
                            Ok(())
                        };
                        if let Err(error) = maybe_error {
                            #monitoring_action
                        }
                        #parallel_task_postprocess
                    },
                ),
                quote! {},
            )
        }
        CuTaskType::Regular => {
            let input_exprs: Vec<proc_macro2::TokenStream> = step
                .input_msg_indices_types
                .iter()
                .map(|input| {
                    let input_index = int2sliceindex(input.culist_index);
                    let output_size = output_pack_sizes
                        .get(input.culist_index as usize)
                        .copied()
                        .unwrap_or_else(|| {
                            panic!(
                                "Missing output pack size for culist index {}",
                                input.culist_index
                            )
                        });
                    if output_size > 1 {
                        let port_index = syn::Index::from(input.src_port);
                        quote! { msgs.#input_index.#port_index }
                    } else {
                        quote! { msgs.#input_index }
                    }
                })
                .collect();
            let inputs_type = if input_exprs.len() == 1 {
                let input = input_exprs.first().unwrap();
                quote! { #input }
            } else {
                quote! { (#(&#input_exprs),*) }
            };

            let monitoring_action = quote! {
                debug!("Component {}: Error during process: {}", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)), &error);
                let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#tid), CuComponentState::Process, &error);
                match decision {
                    Decision::Abort => {
                        debug!("Process: ABORT decision from monitoring. Component '{}' errored out \
                                during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)), clid);
                        #abort_process_step
                    }
                    Decision::Ignore => {
                        debug!("Process: IGNORE decision from monitoring. Component '{}' errored out \
                                during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)));
                        let cumsg_output = &mut msgs.#output_culist_index;
                        #output_clear_payload
                    }
                    Decision::Shutdown => {
                        debug!("Process: SHUTDOWN decision from monitoring. Component '{}' errored out \
                                during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)));
                        return Err(CuError::new_with_cause("Component errored out during process.", error));
                    }
                }
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
                quote! {
                    let mut cumsg_output = &mut culist.msgs.0.#output_culist_index;
                    #output_clear_payload
                }
            } else {
                quote!()
            };
            let regular_process_tokens = quote! {
                #[allow(non_camel_case_types)]
                trait #regular_slot_match_trait_ident<Expected> {
                    fn __cu_cast_output_slot(slot: &mut Self) -> &mut Expected;
                }
                impl<T> #regular_slot_match_trait_ident<T> for T {
                    fn __cu_cast_output_slot(slot: &mut Self) -> &mut T {
                        slot
                    }
                }

                fn #regular_slot_match_fn_ident<'a, Task, Slot>(
                    _task: &Task,
                    slot: &'a mut Slot,
                ) -> &'a mut Task::Output<'static>
                where
                    Task: cu29::cutask::CuTask,
                    Slot: #regular_slot_match_trait_ident<Task::Output<'static>>,
                {
                    <Slot as #regular_slot_match_trait_ident<Task::Output<'static>>>::__cu_cast_output_slot(slot)
                }

                #output_start_time
                let result = {
                    let cumsg_output = #regular_slot_match_fn_ident::<
                        _,
                        _,
                    >(&#task_instance, cumsg_output);
                    #rt_guard
                    ctx.set_current_task(#tid);
                    #task_instance.process(&ctx, cumsg_input, cumsg_output)
                };
                #output_end_time
                result
            };

            (
                wrap_process_step_tokens(
                    wrap_process_step,
                    quote! {
                        #task_setup
                        #parallel_task_preprocess
                        #comment_tokens
                        kf_manager.freeze_task(clid, &#task_instance)?;
                        #call_sim_callback
                        let cumsg_input = &#inputs_type;
                        let cumsg_output = &mut msgs.#output_culist_index;
                        let maybe_error = if doit {
                            execution_probe.record(cu29::monitoring::ExecutionMarker {
                                component_id: cu29::monitoring::ComponentId::new(#tid),
                                step: CuComponentState::Process,
                                culistid: Some(clid),
                            });
                            #regular_process_tokens
                        } else {
                            Ok(())
                        };
                        if let Err(error) = maybe_error {
                            #monitoring_action
                        }
                        #parallel_task_postprocess
                    },
                ),
                logging_tokens,
            )
        }
    }
}

fn generate_bridge_rx_execution_tokens(
    step: &CuExecutionStep,
    bridge_spec: &BridgeSpec,
    channel_index: usize,
    ctx: StepGenerationContext<'_>,
    bridge_setup: proc_macro2::TokenStream,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let StepGenerationContext {
        output_pack_sizes: _,
        sim_mode,
        mission_mod,
        lifecycle_placement,
        wrap_process_step,
    } = ctx;
    let rt_guard = rtsan_guard_tokens();
    let abort_process_step = abort_process_step_tokens(wrap_process_step);
    let channel = &bridge_spec.rx_channels[channel_index];
    let output_pack = step
        .output_msg_pack
        .as_ref()
        .expect("Bridge Rx channel missing output pack");
    let port_index = output_pack
        .msg_types
        .iter()
        .position(|msg| msg == &channel.msg_type_name)
        .unwrap_or_else(|| {
            panic!(
                "Bridge Rx channel '{}' missing output port for '{}'",
                channel.id, channel.msg_type_name
            )
        });
    let culist_index_ts = int2sliceindex(output_pack.culist_index);
    let output_ref = if output_pack.msg_types.len() == 1 {
        quote! { &mut msgs.#culist_index_ts }
    } else {
        let port_index = syn::Index::from(port_index);
        quote! { &mut msgs.#culist_index_ts.#port_index }
    };
    let monitor_index = syn::Index::from(
        channel
            .monitor_index
            .expect("Bridge Rx channel missing monitor index"),
    );
    let bridge_type = runtime_bridge_type_for_spec(bridge_spec, sim_mode);
    let (parallel_bridge_preprocess, parallel_bridge_postprocess) =
        parallel_bridge_lifecycle_tokens(
            &bridge_type,
            bridge_spec
                .monitor_index
                .expect("Bridge missing monitor index for lifecycle"),
            mission_mod,
            lifecycle_placement,
        );
    let const_ident = &channel.const_ident;
    let enum_ident = Ident::new(
        &config_id_to_enum(&format!("{}_rx_{}", bridge_spec.id, channel.id)),
        Span::call_site(),
    );

    let call_sim_callback = if sim_mode {
        quote! {
            let doit = {
                let state = SimStep::#enum_ident {
                    channel: &<#bridge_type as cu29::cubridge::CuBridge>::Rx::#const_ident,
                    msg: cumsg_output,
                };
                let ovr = sim_callback(state);
                if let SimOverride::Errored(reason) = ovr {
                    let error: CuError = reason.into();
                    let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Process, &error);
                    match decision {
                        Decision::Abort => {
                            debug!("Process: ABORT decision from monitoring. Component '{}' errored out during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)), clid);
                            #abort_process_step
                        }
                        Decision::Ignore => {
                            debug!("Process: IGNORE decision from monitoring. Component '{}' errored out during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                            cumsg_output.clear_payload();
                            false
                        }
                        Decision::Shutdown => {
                            debug!("Process: SHUTDOWN decision from monitoring. Component '{}' errored out during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                            return Err(CuError::new_with_cause("Component errored out during process.", error));
                        }
                    }
                } else {
                    ovr == SimOverride::ExecuteByRuntime
                }
            };
        }
    } else {
        quote! { let doit = true; }
    };
    (
        wrap_process_step_tokens(
            wrap_process_step,
            quote! {
                #bridge_setup
                #parallel_bridge_preprocess
                let cumsg_output = #output_ref;
                #call_sim_callback
                if doit {
                    execution_probe.record(cu29::monitoring::ExecutionMarker {
                        component_id: cu29::monitoring::ComponentId::new(#monitor_index),
                        step: CuComponentState::Process,
                        culistid: Some(clid),
                    });
                    cumsg_output.metadata.process_time.start = cu29::curuntime::perf_now(clock).into();
                    let maybe_error = {
                        #rt_guard
                        ctx.clear_current_task();
                        bridge.receive(
                            &ctx,
                            &<#bridge_type as cu29::cubridge::CuBridge>::Rx::#const_ident,
                            cumsg_output,
                        )
                    };
                    cumsg_output.metadata.process_time.end = cu29::curuntime::perf_now(clock).into();
                    if let Err(error) = maybe_error {
                        let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Process, &error);
                        match decision {
                            Decision::Abort => {
                                debug!("Process: ABORT decision from monitoring. Component '{}' errored out during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)), clid);
                                #abort_process_step
                            }
                            Decision::Ignore => {
                                debug!("Process: IGNORE decision from monitoring. Component '{}' errored out during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                cumsg_output.clear_payload();
                            }
                            Decision::Shutdown => {
                                debug!("Process: SHUTDOWN decision from monitoring. Component '{}' errored out during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                return Err(CuError::new_with_cause("Component errored out during process.", error));
                            }
                        }
                    }
                }
                #parallel_bridge_postprocess
            },
        ),
        quote! {},
    )
}

fn generate_bridge_tx_execution_tokens(
    step: &CuExecutionStep,
    bridge_spec: &BridgeSpec,
    channel_index: usize,
    ctx: StepGenerationContext<'_>,
    bridge_setup: proc_macro2::TokenStream,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let StepGenerationContext {
        output_pack_sizes,
        sim_mode,
        mission_mod,
        lifecycle_placement,
        wrap_process_step,
    } = ctx;
    let rt_guard = rtsan_guard_tokens();
    let abort_process_step = abort_process_step_tokens(wrap_process_step);
    let channel = &bridge_spec.tx_channels[channel_index];
    let monitor_index = syn::Index::from(
        channel
            .monitor_index
            .expect("Bridge Tx channel missing monitor index"),
    );
    let input = step
        .input_msg_indices_types
        .first()
        .expect("Bridge Tx channel should have exactly one input");
    let input_index = int2sliceindex(input.culist_index);
    let output_size = output_pack_sizes
        .get(input.culist_index as usize)
        .copied()
        .unwrap_or_else(|| {
            panic!(
                "Missing output pack size for culist index {}",
                input.culist_index
            )
        });
    let input_ref = if output_size > 1 {
        let port_index = syn::Index::from(input.src_port);
        quote! { &mut msgs.#input_index.#port_index }
    } else {
        quote! { &mut msgs.#input_index }
    };
    let output_pack = step
        .output_msg_pack
        .as_ref()
        .expect("Bridge Tx channel missing output pack");
    if output_pack.msg_types.len() != 1 {
        panic!(
            "Bridge Tx channel '{}' expected a single output message slot, got {}",
            channel.id,
            output_pack.msg_types.len()
        );
    }
    let output_index = int2sliceindex(output_pack.culist_index);
    let output_ref = quote! { &mut msgs.#output_index };
    let bridge_type = runtime_bridge_type_for_spec(bridge_spec, sim_mode);
    let (parallel_bridge_preprocess, parallel_bridge_postprocess) =
        parallel_bridge_lifecycle_tokens(
            &bridge_type,
            bridge_spec
                .monitor_index
                .expect("Bridge missing monitor index for lifecycle"),
            mission_mod,
            lifecycle_placement,
        );
    let const_ident = &channel.const_ident;
    let enum_ident = Ident::new(
        &config_id_to_enum(&format!("{}_tx_{}", bridge_spec.id, channel.id)),
        Span::call_site(),
    );

    let call_sim_callback = if sim_mode {
        quote! {
            let doit = {
                let state = SimStep::#enum_ident {
                    channel: &<#bridge_type as cu29::cubridge::CuBridge>::Tx::#const_ident,
                    msg: &*cumsg_input,
                    output: cumsg_output,
                };
                let ovr = sim_callback(state);
                if let SimOverride::Errored(reason) = ovr  {
                    let error: CuError = reason.into();
                    let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Process, &error);
                    match decision {
                        Decision::Abort => {
                            debug!("Process: ABORT decision from monitoring. Component '{}' errored out during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)), clid);
                            #abort_process_step
                        }
                        Decision::Ignore => {
                            debug!("Process: IGNORE decision from monitoring. Component '{}' errored out during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                            false
                        }
                        Decision::Shutdown => {
                            debug!("Process: SHUTDOWN decision from monitoring. Component '{}' errored out during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                            return Err(CuError::new_with_cause("Component errored out during process.", error));
                        }
                    }
                } else {
                    ovr == SimOverride::ExecuteByRuntime
                }
            };
        }
    } else {
        quote! { let doit = true; }
    };
    (
        wrap_process_step_tokens(
            wrap_process_step,
            quote! {
                #bridge_setup
                #parallel_bridge_preprocess
                let cumsg_input = #input_ref;
                let cumsg_output = #output_ref;
                let bridge_channel = &<#bridge_type as cu29::cubridge::CuBridge>::Tx::#const_ident;
                #call_sim_callback
                if doit {
                    execution_probe.record(cu29::monitoring::ExecutionMarker {
                        component_id: cu29::monitoring::ComponentId::new(#monitor_index),
                        step: CuComponentState::Process,
                        culistid: Some(clid),
                    });
                    cumsg_output.metadata.process_time.start = cu29::curuntime::perf_now(clock).into();
                    let maybe_error = if bridge_channel.should_send(cumsg_input.payload().is_some()) {
                        {
                            #rt_guard
                            ctx.clear_current_task();
                            bridge.send(
                                &ctx,
                                bridge_channel,
                                &*cumsg_input,
                            )
                        }
                    } else {
                        Ok(())
                    };
                    if let Err(error) = maybe_error {
                        let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Process, &error);
                        match decision {
                            Decision::Abort => {
                                debug!("Process: ABORT decision from monitoring. Component '{}' errored out during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)), clid);
                                #abort_process_step
                            }
                            Decision::Ignore => {
                                debug!("Process: IGNORE decision from monitoring. Component '{}' errored out during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                            }
                            Decision::Shutdown => {
                                debug!("Process: SHUTDOWN decision from monitoring. Component '{}' errored out during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                return Err(CuError::new_with_cause("Component errored out during process.", error));
                            }
                        }
                    }
                    cumsg_output.metadata.process_time.end = cu29::curuntime::perf_now(clock).into();
                }
                #parallel_bridge_postprocess
            },
        ),
        quote! {},
    )
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
    msg_type_name: String,
    config_index: usize,
    plan_node_id: Option<NodeId>,
    culist_index: Option<usize>,
    monitor_index: Option<usize>,
}

#[derive(Clone)]
struct BridgeSpec {
    id: String,
    type_path: Type,
    run_in_sim: bool,
    config_index: usize,
    tuple_index: usize,
    monitor_index: Option<usize>,
    rx_channels: Vec<BridgeChannelSpec>,
    tx_channels: Vec<BridgeChannelSpec>,
}

#[derive(Clone, Copy, Debug, Default)]
struct ParallelLifecyclePlacement {
    preprocess: bool,
    postprocess: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum ParallelLifecycleKey {
    Task(usize),
    Bridge(usize),
}

fn build_parallel_lifecycle_placements(
    culist_plan: &CuExecutionLoop,
    culist_exec_entities: &[ExecutionEntity],
) -> Vec<ParallelLifecyclePlacement> {
    let step_keys: Vec<Option<ParallelLifecycleKey>> = culist_plan
        .steps
        .iter()
        .map(|unit| match unit {
            CuExecutionUnit::Step(step) => {
                match &culist_exec_entities[step.node_id as usize].kind {
                    ExecutionEntityKind::Task { task_index } => {
                        Some(ParallelLifecycleKey::Task(*task_index))
                    }
                    ExecutionEntityKind::BridgeRx { bridge_index, .. }
                    | ExecutionEntityKind::BridgeTx { bridge_index, .. } => {
                        Some(ParallelLifecycleKey::Bridge(*bridge_index))
                    }
                }
            }
            CuExecutionUnit::Loop(_) => None,
        })
        .collect();

    let mut placements = vec![ParallelLifecyclePlacement::default(); step_keys.len()];
    let mut seen_forward = std::collections::HashSet::new();
    for (index, key) in step_keys.iter().enumerate() {
        let Some(key) = key else {
            continue;
        };
        if seen_forward.insert(*key) {
            placements[index].preprocess = true;
        }
    }

    let mut seen_reverse = std::collections::HashSet::new();
    for (index, key) in step_keys.iter().enumerate().rev() {
        let Some(key) = key else {
            continue;
        };
        if seen_reverse.insert(*key) {
            placements[index].postprocess = true;
        }
    }

    placements
}

fn sim_bridge_channel_set_idents(bridge_tuple_index: usize) -> (Ident, Ident, Ident, Ident) {
    (
        format_ident!("__CuSimBridge{}TxChannels", bridge_tuple_index),
        format_ident!("__CuSimBridge{}TxId", bridge_tuple_index),
        format_ident!("__CuSimBridge{}RxChannels", bridge_tuple_index),
        format_ident!("__CuSimBridge{}RxId", bridge_tuple_index),
    )
}

fn runtime_bridge_type_for_spec(bridge_spec: &BridgeSpec, sim_mode: bool) -> Type {
    if sim_mode && !bridge_spec.run_in_sim {
        let (tx_set_ident, _tx_id_ident, rx_set_ident, _rx_id_ident) =
            sim_bridge_channel_set_idents(bridge_spec.tuple_index);
        let tx_type: Type = if bridge_spec.tx_channels.is_empty() {
            parse_quote!(cu29::simulation::CuNoBridgeChannels)
        } else {
            parse_quote!(#tx_set_ident)
        };
        let rx_type: Type = if bridge_spec.rx_channels.is_empty() {
            parse_quote!(cu29::simulation::CuNoBridgeChannels)
        } else {
            parse_quote!(#rx_set_ident)
        };
        parse_quote!(cu29::simulation::CuSimBridge<#tx_type, #rx_type>)
    } else {
        bridge_spec.type_path.clone()
    }
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

#[cfg(test)]
mod tests {
    use std::fs;
    use std::path::{Path, PathBuf};

    fn unique_test_dir(name: &str) -> PathBuf {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("system clock before unix epoch")
            .as_nanos();
        std::env::temp_dir().join(format!("cu29_derive_{name}_{nanos}"))
    }

    fn write_file(path: &Path, content: &str) {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).expect("create parent dirs");
        }
        fs::write(path, content).expect("write file");
    }

    // See tests/compile_file directory for more information
    #[test]
    fn test_compile_fail() {
        use rustc_version::{Channel, version_meta};
        use std::{env, fs, path::Path};

        let log_index_dir = env::temp_dir()
            .join("cu29_derive_trybuild_log_index")
            .join("a")
            .join("b")
            .join("c");
        fs::create_dir_all(&log_index_dir).unwrap();
        unsafe {
            env::set_var("LOG_INDEX_DIR", &log_index_dir);
        }

        let dir = Path::new("tests/compile_fail");
        for entry in fs::read_dir(dir).unwrap() {
            let entry = entry.unwrap();
            if !entry.file_type().unwrap().is_dir() {
                continue;
            }
            for file in fs::read_dir(entry.path()).unwrap() {
                let file = file.unwrap();
                let p = file.path();
                if p.extension().and_then(|x| x.to_str()) != Some("rs") {
                    continue;
                }

                let base = p.with_extension("stderr"); // the file trybuild reads
                let src = match version_meta().unwrap().channel {
                    Channel::Beta => Path::new(&format!("{}.beta", base.display())).to_path_buf(),
                    _ => Path::new(&format!("{}.stable", base.display())).to_path_buf(),
                };

                if src.exists() {
                    fs::copy(src, &base).unwrap();
                }
            }
        }

        let t = trybuild::TestCases::new();
        t.compile_fail("tests/compile_fail/*/*.rs");
        t.pass("tests/compile_pass/*/*.rs");
    }

    #[test]
    fn runtime_plan_keeps_nc_order_for_non_first_connected_output() {
        use super::*;
        use cu29::config::CuConfig;
        use cu29::curuntime::{CuExecutionUnit, compute_runtime_plan};

        let config: CuConfig =
            read_config("tests/config/multi_output_source_non_first_connected_valid.ron")
                .expect("failed to read test config");
        let graph = config.get_graph(None).expect("missing graph");
        let src_id = graph.get_node_id_by_name("src").expect("missing src node");

        let runtime = compute_runtime_plan(graph).expect("runtime plan failed");
        let src_step = runtime
            .steps
            .iter()
            .find_map(|step| match step {
                CuExecutionUnit::Step(step) if step.node_id == src_id => Some(step),
                _ => None,
            })
            .expect("missing source step");

        assert_eq!(
            src_step.output_msg_pack.as_ref().unwrap().msg_types,
            vec!["i32", "bool"]
        );
    }

    #[test]
    fn matching_task_ids_are_flattened_per_output_message() {
        use super::*;
        use cu29::config::CuConfig;

        let config: CuConfig =
            read_config("tests/config/multi_output_source_non_first_connected_valid.ron")
                .expect("failed to read test config");
        let graph = config.get_graph(None).expect("missing graph");
        let task_specs = CuTaskSpecSet::from_graph(graph);
        let channel_usage = collect_bridge_channel_usage(graph);
        let mut bridge_specs = build_bridge_specs(&config, graph, &channel_usage);
        let (runtime_plan, exec_entities, plan_to_original) =
            build_execution_plan(graph, &task_specs, &mut bridge_specs)
                .expect("runtime plan failed");
        let output_packs = extract_output_packs(&runtime_plan);
        let task_names = collect_task_names(graph);
        let (_, node_output_positions) = collect_culist_metadata(
            &runtime_plan,
            &exec_entities,
            &mut bridge_specs,
            &plan_to_original,
        );

        // Rebuild per-slot origin ids like `gen_culist_support` does.
        let mut slot_origin_ids: Vec<Option<String>> = vec![None; output_packs.len()];
        for (node_id, task_id, _) in task_names {
            let output_position = node_output_positions
                .get(&node_id)
                .unwrap_or_else(|| panic!("Task {task_id} (node id: {node_id}) not found"));
            slot_origin_ids[*output_position] = Some(task_id);
        }

        let flattened_ids = flatten_slot_origin_ids(&output_packs, &slot_origin_ids);

        // src emits two messages (i32 + bool), both map to src.
        // sink contributes its own output slot (CuMsg<()>), mapped to sink.
        assert_eq!(
            flattened_ids,
            vec!["src".to_string(), "src".to_string(), "sink".to_string()]
        );
    }

    #[test]
    fn bridge_resources_are_collected() {
        use super::*;
        use cu29::config::{CuGraph, Flavor, Node};
        use std::collections::HashMap;
        use syn::parse_str;

        let mut graph = CuGraph::default();
        let mut node = Node::new_with_flavor("radio", "bridge::Dummy", Flavor::Bridge);
        let mut res = HashMap::new();
        res.insert("serial".to_string(), "fc.serial0".to_string());
        node.set_resources(Some(res));
        graph.add_node(node).expect("bridge node");

        let task_specs = CuTaskSpecSet::from_graph(&graph);
        let bridge_spec = BridgeSpec {
            id: "radio".to_string(),
            type_path: parse_str("bridge::Dummy").unwrap(),
            run_in_sim: true,
            config_index: 0,
            tuple_index: 0,
            monitor_index: None,
            rx_channels: Vec::new(),
            tx_channels: Vec::new(),
        };

        let mut config = cu29::config::CuConfig::default();
        config.resources.push(ResourceBundleConfig {
            id: "fc".to_string(),
            provider: "board::Bundle".to_string(),
            config: None,
            missions: None,
        });
        let bundle_specs = build_bundle_specs(&config, "default").expect("bundle specs");
        let specs = collect_resource_specs(&graph, &task_specs, &[bridge_spec], &bundle_specs)
            .expect("collect specs");
        assert_eq!(specs.len(), 1);
        assert!(matches!(specs[0].owner, ResourceOwner::Bridge(0)));
        assert_eq!(specs[0].binding_name, "serial");
        assert_eq!(specs[0].bundle_index, 0);
        assert_eq!(specs[0].resource_name, "serial0");
    }

    #[test]
    fn copper_runtime_args_parse_subsystem_mode() {
        use super::*;
        use quote::quote;

        let args = CopperRuntimeArgs::parse_tokens(quote!(
            config = "multi_copper.ron",
            subsystem = "ping",
            sim_mode,
            ignore_resources
        ))
        .expect("parse runtime args");

        assert_eq!(args.config_path, "multi_copper.ron");
        assert_eq!(args.subsystem_id.as_deref(), Some("ping"));
        assert!(args.sim_mode);
        assert!(args.ignore_resources);
    }

    #[test]
    fn resolve_runtime_config_from_multi_config_selects_local_subsystem() {
        use super::*;

        let root = unique_test_dir("multi_runtime_resolve");
        let alpha_config = root.join("alpha.ron");
        let beta_config = root.join("beta.ron");
        let network_config = root.join("multi.ron");

        write_file(
            &alpha_config,
            r#"
(
    tasks: [
        (id: "src", type: "AlphaSource", run_in_sim: true),
        (id: "sink", type: "AlphaSink", run_in_sim: true),
    ],
    cnx: [
        (src: "src", dst: "sink", msg: "u32"),
    ],
)
"#,
        );
        write_file(
            &beta_config,
            r#"
(
    tasks: [
        (id: "src", type: "BetaSource", run_in_sim: true),
        (id: "sink", type: "BetaSink", run_in_sim: true),
    ],
    cnx: [
        (src: "src", dst: "sink", msg: "u64"),
    ],
)
"#,
        );
        write_file(
            &network_config,
            r#"
(
    subsystems: [
        (id: "beta", config: "beta.ron"),
        (id: "alpha", config: "alpha.ron"),
    ],
    interconnects: [],
)
"#,
        );

        let args = CopperRuntimeArgs {
            config_path: "multi.ron".to_string(),
            subsystem_id: Some("beta".to_string()),
            sim_mode: false,
            ignore_resources: false,
        };

        let resolved =
            resolve_runtime_config_with_root(&args, &root).expect("resolve multi runtime config");

        assert_eq!(resolved.subsystem_id.as_deref(), Some("beta"));
        assert_eq!(resolved.subsystem_code, 1);
        let graph = resolved
            .local_config
            .get_graph(None)
            .expect("resolved local config graph");
        assert!(graph.get_node_id_by_name("src").is_some());
        assert!(resolved.bundled_local_config_content.contains("BetaSource"));
    }

    #[test]
    fn resolve_runtime_config_rejects_missing_subsystem() {
        use super::*;

        let root = unique_test_dir("multi_runtime_missing_subsystem");
        let alpha_config = root.join("alpha.ron");
        let network_config = root.join("multi.ron");

        write_file(
            &alpha_config,
            r#"
(
    tasks: [
        (id: "src", type: "AlphaSource", run_in_sim: true),
        (id: "sink", type: "AlphaSink", run_in_sim: true),
    ],
    cnx: [
        (src: "src", dst: "sink", msg: "u32"),
    ],
)
"#,
        );
        write_file(
            &network_config,
            r#"
(
    subsystems: [
        (id: "alpha", config: "alpha.ron"),
    ],
    interconnects: [],
)
"#,
        );

        let args = CopperRuntimeArgs {
            config_path: "multi.ron".to_string(),
            subsystem_id: Some("missing".to_string()),
            sim_mode: false,
            ignore_resources: false,
        };

        let err = resolve_runtime_config_with_root(&args, &root).expect_err("missing subsystem");
        assert!(err.to_string().contains("Subsystem 'missing'"));
    }
}
