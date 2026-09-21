mod live_twin;
use proc_macro::TokenStream;
use quote::{ToTokens, format_ident, quote};
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::Path;
use std::process::Command;
use syn::Fields::{Named, Unnamed};
use syn::ext::IdentExt;
use syn::meta::parser;
use syn::parse::Parser;
use syn::{
    Field, Fields, ItemImpl, ItemStruct, LitStr, Type, TypeTuple, parse_macro_input, parse_quote,
    parse_str,
};

use crate::utils::{config_id_to_bridge_const, config_id_to_enum, config_id_to_struct_member};
use cu29_build::COPPER_CFG_FEATURES_ENV;
use cu29_runtime::config::CuConfig;
use cu29_runtime::config::DEFAULT_MISSION_ID;
use cu29_runtime::config::{
    AnytimeConfig, BridgeChannelConfigRepresentation, ConfigGraphs, CuGraph, Flavor, HandleContent,
    Node, NodeId, PlannerKind, RT_POOL, ResourceBundleConfig, SchedulingPolicy, TaskKind,
    read_configuration_with_features, read_configuration_with_resolved_ron_and_features,
};
use cu29_runtime::curuntime::{
    CuExecutionLoop, CuExecutionStep, CuExecutionUnit, CuStepPhase, CuTaskType,
    find_task_type_for_id,
};
use cu29_runtime::planner::{
    CuPlanBackground, CuPlanBackgroundResult, CuPlanPlacement, CuPlanThread,
    DEFAULT_COPPERLIST_COUNT, LanePlan, PlanEntityKind, assemble_runtime_plan_for_mission,
};
use cu29_traits::{CuError, CuResult};
use proc_macro2::{Ident, Span};

mod bundle_resources;
mod constants;
mod resources;
mod safety;
mod utils;

use constants::build_constant_modules;

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
/// Opens a heap-allocation accounting scope (binds `__cu_alloc_scope`) iff the
/// `memory_monitoring` feature is enabled. Otherwise expands to nothing, so a
/// default build emits zero extra code per task step.
fn alloc_scope_open_tokens() -> proc_macro2::TokenStream {
    if cfg!(feature = "memory_monitoring") {
        quote! {
            let __cu_alloc_scope = cu29::monitoring::ScopedAllocCounter::new();
        }
    } else {
        quote! {}
    }
}

/// Forwards the scope's accumulated delta to `monitor.observe_alloc(...)` and
/// drops the scope. `monitor_expr` is whatever expression reaches the monitor
/// in the current code block (`monitor` for preprocess/process/postprocess,
/// `self.copper_runtime.monitor` for start/stop). `component_index` and `step`
/// are the tokens to be passed through to `ComponentId::new(...)` and the
/// `CuComponentState` variant.
fn alloc_scope_close_tokens(
    monitor_expr: proc_macro2::TokenStream,
    component_index: proc_macro2::TokenStream,
    step: proc_macro2::TokenStream,
) -> proc_macro2::TokenStream {
    if cfg!(feature = "memory_monitoring") {
        quote! {
            #monitor_expr.observe_alloc(
                cu29::monitoring::ComponentId::new(#component_index),
                #step,
                __cu_alloc_scope.allocated(),
                __cu_alloc_scope.deallocated(),
            );
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
    active_features: Vec<String>,
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

#[proc_macro_attribute]
pub fn safety_case(args: TokenStream, input: TokenStream) -> TokenStream {
    safety::expand(args, input)
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
    let channel_usage = collect_bridge_channel_usage(graph);
    let mut bridge_specs = build_bridge_specs(cuconfig, graph, &channel_usage);
    let (culist_plan, exec_entities, plan_to_original) = build_execution_plan(
        cuconfig,
        graph,
        mission_label.unwrap_or(DEFAULT_MISSION_ID),
        &mut bridge_specs,
    )
    .map_err(|e| {
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
        CuListSupportOptions {
            cuconfig,
            distributed: false,
        },
        mission_label,
        &culist_plan,
        &culist_order,
        &node_output_positions,
        &task_names,
        &bridge_specs,
    ))
}

/// Build the inner support of the copper list.
struct CuListSupportOptions<'a> {
    cuconfig: &'a CuConfig,
    distributed: bool,
}

fn gen_culist_support(
    options: CuListSupportOptions<'_>,
    mission_label: Option<&str>,
    runtime_plan: &CuExecutionLoop,
    culist_indices_in_plan_order: &[usize],
    node_output_positions: &HashMap<NodeId, usize>,
    task_names: &[(NodeId, String, String)],
    bridge_specs: &[BridgeSpec],
) -> proc_macro2::TokenStream {
    let CuListSupportOptions {
        cuconfig,
        distributed,
    } = options;
    #[cfg(feature = "macro_debug")]
    eprintln!("[Extract msgs types]");
    let output_packs = extract_output_packs(runtime_plan);
    let slot_types: Vec<Type> = output_packs.iter().map(|pack| pack.slot_type()).collect();

    let culist_size = output_packs.len();

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist struct]");
    let msgs_types_tuple: Type = if distributed {
        parse_quote!(CuMessageRegions)
    } else {
        let tuple = build_culist_tuple(&slot_types);
        parse_quote!(#tuple)
    };
    let isolated_regions = if distributed {
        quote! {
            #[repr(C)]
            pub struct CuMessageRegions(
                #(pub cu29::copperlist::MessageRegion<#slot_types>),*
            );
        }
    } else {
        quote! {}
    };
    let dataset_alignment = distributed.then(|| quote!(#[repr(align(128))]));
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
    let slot_handle_modes = build_slot_handle_modes(
        cuconfig,
        mission_label,
        &output_packs,
        node_output_positions,
        task_names,
    );
    let compressed = !cfg!(feature = "flat-copperlist-encoding");
    let (msgs_types_tuple_encode, msgs_types_tuple_decode) = if compressed {
        (
            build_compressed_culist_tuple_encode(
                &output_packs,
                &encode_helper_names,
                &slot_handle_modes,
                cumsg_count,
                None,
            ),
            build_compressed_culist_tuple_decode(
                &output_packs,
                &slot_types,
                cumsg_count,
                &decode_helper_names,
                distributed,
            ),
        )
    } else {
        (
            build_culist_tuple_encode(&output_packs, &encode_helper_names, &slot_handle_modes),
            build_culist_tuple_decode(
                &output_packs,
                &slot_types,
                cumsg_count,
                &decode_helper_names,
                distributed,
            ),
        )
    };

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist tuple debug support]");
    let msgs_types_tuple_debug = build_culist_tuple_debug(&slot_types);

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the copperlist tuple serialize support]");
    let msgs_types_tuple_serialize = build_culist_tuple_serialize(&slot_types);

    #[cfg(feature = "macro_debug")]
    eprintln!("[build the default tuple support]");
    let msgs_types_tuple_default =
        build_culist_tuple_default(&slot_types, cumsg_count, distributed);

    #[cfg(feature = "macro_debug")]
    eprintln!("[build erasedcumsgs]");

    let erasedmsg_trait_impl = build_culist_erasedcumsgs(&output_packs, distributed);

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
    let mut in_place_init_tokens: Vec<proc_macro2::TokenStream> = Vec::new();
    for idx in culist_indices_in_plan_order {
        let slot_index = syn::Index::from(*idx);
        let pack = output_packs
            .get(*idx)
            .unwrap_or_else(|| panic!("Missing output pack for index {idx}"));
        let region_value = distributed.then(|| quote!(.value));
        if pack.is_multi() {
            for port_idx in 0..pack.msg_types.len() {
                let port_index = syn::Index::from(port_idx);
                in_place_init_tokens.push(quote! {
                    CuMsg::init_in_place(core::ptr::addr_of_mut!((*dst).0.#slot_index #region_value .#port_index));
                });
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
            in_place_init_tokens.push(quote! {
                CuMsg::init_in_place(core::ptr::addr_of_mut!((*dst).0.#slot_index #region_value));
            });
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
        .map(|(task_id, msg_type, payload_type)| {
            let task_id = LitStr::new(task_id, Span::call_site());
            let msg_type = LitStr::new(msg_type, Span::call_site());
            quote! {
                cu29::TaskOutputSpec::new::<#payload_type>(#task_id, #msg_type)
            }
        })
        .collect();

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

    let capture_support = if cfg!(feature = "logstream") {
        live_twin::dataset_support(
            runtime_plan,
            &output_packs,
            &encode_helper_names,
            &slot_handle_modes,
            cumsg_count,
            &flat_codec_bindings,
        )
    } else {
        quote! {}
    };

    // This generates a way to get the metadata of every single message of a culist at low cost
    quote! {
        #capture_support
        #collect_metadata_function
        #compute_payload_bytes_fn
        #default_config_ron_const
        #(#codec_helper_fns)*

        #isolated_regions
        #dataset_alignment
        pub struct CuStampedDataSet(pub #msgs_types_tuple, cu29::monitoring::CuMsgIoCache<#cumsg_count>);

        pub type CuList = CopperList<CuStampedDataSet>;

        const TASK_OUTPUT_SPECS: &[cu29::TaskOutputSpec] = &[
            #(#task_output_spec_literals),*
        ];

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
        impl MatchingTasks for CuStampedDataSet {
            #[allow(dead_code)]
            fn get_all_task_ids() -> &'static [&'static str] {
                &[#(#task_name_literals),*]
            }

            #[allow(dead_code)]
            fn get_output_specs() -> &'static [cu29::TaskOutputSpec] {
                TASK_OUTPUT_SPECS
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
            const DISTRIBUTED: bool = #distributed;

            fn init_in_place(storage: &mut core::mem::MaybeUninit<Self>) -> &mut Self {
                let dst = storage.as_mut_ptr();
                // SAFETY: The pool supplies uninitialized dataset storage. Every
                // message and the I/O cache are written before it is borrowed.
                unsafe {
                    #(#in_place_init_tokens)*
                    core::ptr::addr_of_mut!((*dst).1)
                        .write(cu29::monitoring::CuMsgIoCache::default());
                    storage.assume_init_mut()
                }
            }

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
        .filter(|unit| {
            // The sim callback fires once per node, in the base step; refine
            // steps get no SimStep variant of their own.
            !matches!(unit, CuExecutionUnit::Step(step) if step.phase == CuStepPhase::AnytimeRefine)
        })
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

    if cfg!(feature = "logstream") {
        variants.push(quote! { CopperListCompleted(&'a mut CopperList<CuStampedDataSet>) });
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
        .filter(|unit| {
            // One replay arm per node: refine steps have no SimStep variant.
            !matches!(unit, CuExecutionUnit::Step(step) if step.phase == CuStepPhase::AnytimeRefine)
        })
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
    let debug_replay_arms: Vec<proc_macro2::TokenStream> =
        runtime_plan
            .steps
            .iter()
            .filter_map(|unit| match unit {
                CuExecutionUnit::Step(step) => match &exec_entities[step.node_id as usize].kind {
                    ExecutionEntityKind::Task { .. } => {
                        if step.task_type == CuTaskType::Regular {
                            return None;
                        }
                        let enum_entry_name = config_id_to_enum(step.node.get_id().as_str());
                        let enum_ident = Ident::new(&enum_entry_name, Span::call_site());
                        let output_pack = step
                            .output_msg_pack
                            .as_ref()
                            .expect("Task step missing output pack for recorded debug replay");
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
                        let output_pack = step.output_msg_pack.as_ref().expect(
                            "Bridge Rx channel missing output pack for recorded debug replay",
                        );
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
                        let output_pack = step.output_msg_pack.as_ref().expect(
                            "Bridge Tx channel missing output pack for recorded debug replay",
                        );
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
        /// Exact-output replay callback: every recorded task and bridge output is
        /// copied from the CopperList and the runtime implementation is skipped.
        ///
        /// This is for deterministic log reproduction, not for debugger state replay.
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

        /// Debugger state replay callback: recorded external inputs are injected,
        /// regular Copper tasks execute normally, and external sink/bridge effects
        /// are suppressed.
        ///
        /// This preserves task-state evolution after restoring an intra-CL keyframe.
        #[allow(dead_code)]
        pub fn recorded_debug_replay_step<'a>(
            step: SimStep<'a>,
            recorded: &CopperList<CuStampedDataSet>,
        ) -> SimOverride {
            match step {
                #(#debug_replay_arms),*,
                _ => SimOverride::ExecuteByRuntime,
            }
        }
    }
}

include!("runtime_macro.rs");
fn resolve_runtime_config(args: &CopperRuntimeArgs) -> CuResult<ResolvedRuntimeConfig> {
    let caller_root = utils::caller_crate_root();
    resolve_runtime_config_with_root(args, &caller_root)
}

fn resolve_runtime_config_with_root(
    args: &CopperRuntimeArgs,
    caller_root: &Path,
) -> CuResult<ResolvedRuntimeConfig> {
    let active_features = active_config_features();
    let active_feature_refs: Vec<_> = active_features.iter().map(String::as_str).collect();
    resolve_runtime_config_with_root_and_features(args, caller_root, &active_feature_refs)
}

fn resolve_runtime_config_with_root_and_features(
    args: &CopperRuntimeArgs,
    caller_root: &Path,
    active_features: &[&str],
) -> CuResult<ResolvedRuntimeConfig> {
    let filename = config_full_path_from_root(caller_root, &args.config_path);
    if !Path::new(&filename).exists() {
        return Err(CuError::from(format!(
            "The configuration file `{}` does not exist. Please provide a valid path.",
            args.config_path
        )));
    }

    if let Some(subsystem_id) = args.subsystem_id.as_deref() {
        let multi_config = cu29_runtime::config::read_multi_configuration_with_features(
            filename.as_str(),
            active_features,
        )
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
        // Bundle the include-expanded source representation. Serializing the lowered
        // mission graphs would lose the source task order because missions use hash maps.
        let (local_config, bundled_local_config_content) =
            read_configuration_with_resolved_ron_and_features(
                &subsystem.config_path,
                active_features,
            )
            .map_err(|e| {
                CuError::from(format!(
                    "Failed to prepare bundled local configuration for subsystem '{subsystem_id}' from '{}'.",
                    subsystem.config_path
                ))
                .add_cause(e.to_string().as_str())
            })?;

        Ok(ResolvedRuntimeConfig {
            local_config,
            bundled_local_config_content,
            active_features: active_features
                .iter()
                .map(|feature| (*feature).to_string())
                .collect(),
            subsystem_id: Some(subsystem_id.to_string()),
            subsystem_code: subsystem.subsystem_code,
        })
    } else {
        let (local_config, bundled_local_config_content) =
            read_configuration_with_resolved_ron_and_features(filename.as_str(), active_features)?;
        Ok(ResolvedRuntimeConfig {
            local_config,
            bundled_local_config_content,
            active_features: active_features
                .iter()
                .map(|feature| (*feature).to_string())
                .collect(),
            subsystem_id: None,
            subsystem_code: 0,
        })
    }
}

fn active_config_features() -> Vec<String> {
    let mut features: Vec<_> = std::env::var(COPPER_CFG_FEATURES_ENV)
        .unwrap_or_default()
        .split(',')
        .filter(|feature| !feature.is_empty())
        .map(str::to_owned)
        .collect();
    features.sort_unstable();
    features.dedup();
    features
}

fn build_config_load_stmt(
    std_enabled: bool,
    application_name: &Ident,
    subsystem_id: Option<&str>,
    active_features: &[String],
) -> proc_macro2::TokenStream {
    let active_features = active_features.iter();
    if std_enabled {
        if let Some(subsystem_id) = subsystem_id {
            quote! {
                const COPPER_CFG_FEATURES: &[&str] = &[#(#active_features),*];
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
                    let multi_config = cu29::config::read_multi_configuration_with_features(
                        config_filename,
                        COPPER_CFG_FEATURES,
                    )?;
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
                const COPPER_CFG_FEATURES: &[&str] = &[#(#active_features),*];
                let _ = instance_id;
                let (config, config_source) = if let Some(overridden_config) = config_override {
                    debug!("CuConfig: Overridden programmatically.");
                    (overridden_config, RuntimeLifecycleConfigSource::ProgrammaticOverride)
                } else if ::std::path::Path::new(config_filename).exists() {
                    debug!("CuConfig: Reading configuration from file: {}", config_filename);
                    (
                        cu29::config::read_configuration_with_features(
                            config_filename,
                            COPPER_CFG_FEATURES,
                        )?,
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
    let active_features = active_config_features();
    let active_feature_refs: Vec<_> = active_features.iter().map(String::as_str).collect();
    read_configuration_with_features(filename.as_str(), &active_feature_refs)
}

fn inferred_single_output_payload_type(
    task_type: &Type,
    task_kind: CuTaskType,
    is_anytime: bool,
    is_stateless: bool,
) -> Type {
    if is_anytime {
        return parse_quote! {
            <<#task_type as cu29::cutask_anytime::CuAnytimeTask>::Output<'static> as cu29::cutask::CuSingleOutputMsg>::Payload
        };
    }
    if is_stateless {
        return parse_quote! {
            <<#task_type as cu29::cutask::CuStatelessTask>::Output<'static> as cu29::cutask::CuSingleOutputMsg>::Payload
        };
    }
    match task_kind {
        CuTaskType::Source => parse_quote! {
            <<#task_type as cu29::cutask::CuSrcTask>::Output<'static> as cu29::cutask::CuSingleOutputMsg>::Payload
        },
        CuTaskType::Regular => parse_quote! {
            <<#task_type as cu29::cutask::CuTask>::Output<'static> as cu29::cutask::CuSingleOutputMsg>::Payload
        },
        CuTaskType::Sink => panic!("Sinks do not have output payload types"),
    }
}

fn task_trait_for_kind(task_kind: CuTaskType) -> proc_macro2::TokenStream {
    match task_kind {
        CuTaskType::Source => quote! { cu29::cutask::CuSrcTask },
        CuTaskType::Regular => quote! { cu29::cutask::CuTask },
        CuTaskType::Sink => quote! { cu29::cutask::CuSinkTask },
    }
}

/// Like [`task_trait_for_kind`], but resolves stateless and anytime nodes to
/// their declared task traits while they retain the regular graph role.
///
/// Background anytime nodes are driven through `CuAsyncTask`, which is a plain
/// `CuTask`; stateless nodes reject background and anytime configuration.
fn task_trait_for_specs(task_specs: &CuTaskSpecSet, index: usize) -> proc_macro2::TokenStream {
    if task_specs.stateless_flags[index] {
        return quote! { cu29::cutask::CuStatelessTask };
    }
    let foreground_anytime =
        task_specs.anytime_configs[index].is_some() && !task_specs.background_flags[index];
    if foreground_anytime {
        quote! { cu29::cutask_anytime::CuAnytimeTask }
    } else {
        task_trait_for_kind(task_specs.cutypes[index])
    }
}

fn task_output_payload_type(
    graph: &CuGraph,
    node: &Node,
    task_kind: CuTaskType,
    task_type: &Type,
) -> Option<Type> {
    if task_kind == CuTaskType::Sink {
        return None;
    }

    let id = node.get_id();
    if let Some(type_str) = graph.get_node_output_msg_type(id.as_str()) {
        return Some(
            parse_str::<Type>(type_str.as_str()).expect("Could not parse output message type."),
        );
    }

    node.get_declared_task_kind().map(|_| {
        inferred_single_output_payload_type(
            task_type,
            task_kind,
            node.anytime().is_some(),
            node.get_declared_task_kind() == Some(TaskKind::Stateless),
        )
    })
}

#[cfg(test)]
fn synthesized_single_output_msg_name(
    task_type: &Type,
    task_kind: CuTaskType,
    is_anytime: bool,
    is_stateless: bool,
) -> String {
    inferred_single_output_payload_type(task_type, task_kind, is_anytime, is_stateless)
        .to_token_stream()
        .to_string()
}

struct CuTaskSpecSet {
    pub ids: Vec<String>,
    pub cutypes: Vec<CuTaskType>,
    pub stateless_flags: Vec<bool>,
    pub background_flags: Vec<bool>,
    /// Fixed CopperList lag selected for a deterministic background result.
    pub background_result_lags: Vec<Option<u32>>,
    /// Thread pool name each task runs on when backgrounded (defaults to the
    /// `"background"` pool). Only meaningful where `background_flags` is true.
    pub background_pools: Vec<String>,
    /// The `anytime:` policy of each task, if any. An anytime task keeps its
    /// raw `CuAnytimeTask` type in the tuple (no wrapper); the policy values
    /// are baked into a per-node `AnytimePolicy` ZST and the emitted
    /// base/refine steps.
    pub anytime_configs: Vec<Option<AnytimeConfig>>,
    /// The task type inside the optional async wrapper: the anytime runner
    /// for a background anytime node, the declared task type for everything
    /// else — including every foreground task, where nothing wraps it.
    pub async_inner_task_types: Vec<Type>,
    pub logging_enabled: Vec<bool>,
    pub type_names: Vec<String>,
    pub task_types: Vec<Type>,
    pub instantiation_types: Vec<Type>,
    pub sim_task_types: Vec<Type>,
    pub run_in_sim_flags: Vec<bool>,
    #[allow(dead_code)]
    pub output_types: Vec<Option<Type>>,
    pub autogenerated_output_flags: Vec<bool>,
    pub node_id_to_task_index: Vec<Option<usize>>,
}

impl CuTaskSpecSet {
    pub fn from_graph(graph: &CuGraph) -> CuResult<Self> {
        let all_id_nodes: Vec<(NodeId, &Node)> = graph
            .get_all_nodes()
            .into_iter()
            .filter(|(_, node)| node.get_flavor() == Flavor::Task)
            .collect();

        let ids: Vec<String> = all_id_nodes
            .iter()
            .map(|(_, node)| node.get_id().to_string())
            .collect();

        let cutypes: Vec<CuTaskType> = all_id_nodes
            .iter()
            .map(|(id, _)| find_task_type_for_id(graph, *id))
            .collect::<CuResult<Vec<_>>>()?;

        let stateless_flags: Vec<bool> = all_id_nodes
            .iter()
            .map(|(_, node)| node.get_declared_task_kind() == Some(TaskKind::Stateless))
            .collect();

        let background_flags: Vec<bool> = all_id_nodes
            .iter()
            .map(|(_, node)| node.is_background())
            .collect();

        let background_pools: Vec<String> = all_id_nodes
            .iter()
            .map(|(_, node)| node.background_pool().to_string())
            .collect();

        let anytime_configs: Vec<Option<AnytimeConfig>> = all_id_nodes
            .iter()
            .map(|(_, node)| node.anytime().cloned())
            .collect();

        let logging_enabled: Vec<bool> = all_id_nodes
            .iter()
            .map(|(_, node)| node.is_logging_enabled())
            .collect();

        let type_names: Vec<String> = all_id_nodes
            .iter()
            .map(|(_, node)| node.get_type().to_string())
            .collect();

        let declared_task_types: Vec<Type> = type_names
            .iter()
            .map(|name| {
                parse_str::<Type>(name).unwrap_or_else(|error| {
                    panic!("Could not transform {name} into a Task Rust type: {error}");
                })
            })
            .collect();

        let output_types: Vec<Option<Type>> = all_id_nodes
            .iter()
            .zip(cutypes.iter())
            .zip(declared_task_types.iter())
            .map(|(((_, node), &task_kind), task_type)| {
                task_output_payload_type(graph, node, task_kind, task_type)
            })
            .collect();

        let autogenerated_output_flags: Vec<bool> = all_id_nodes
            .iter()
            .zip(cutypes.iter())
            .map(|((node_id, node), &task_kind)| {
                task_kind != CuTaskType::Sink
                    && node.get_declared_task_kind().is_some()
                    && graph
                        .get_node_output_msg_types_by_id(*node_id)
                        .expect("missing output type lookup")
                        .is_empty()
            })
            .collect();

        // A background anytime node is handed to `CuAsyncTask` wrapped in the
        // runner, which turns one whole job into a single `process` call.
        let async_inner_task_types: Vec<Type> = declared_task_types
            .iter()
            .zip(ids.iter())
            .zip(background_flags.iter())
            .zip(anytime_configs.iter())
            .map(|(((task_type, id), &background), anytime)| {
                if background && anytime.is_some() {
                    let policy_ident = anytime_policy_ident(id.as_str());
                    parse_quote!(cu29::cutask_anytime::CuAnytimeRunner<#task_type, #policy_ident>)
                } else {
                    task_type.clone()
                }
            })
            .collect();

        let task_types = declared_task_types
            .iter()
            .zip(type_names.iter())
            .zip(cutypes.iter())
            .zip(background_flags.iter())
            .zip(output_types.iter())
            .zip(async_inner_task_types.iter())
            .map(|(((((name_type, name), cutype), &background), output_type), inner_type)| {
                if background {
                    if let Some(output_type) = output_type {
                        match cutype {
                            CuTaskType::Source => {
                                parse_quote!(CuAsyncSrcTask<#inner_type, #output_type>)
                            }
                            CuTaskType::Regular => {
                                parse_quote!(CuAsyncTask<#inner_type, #output_type>)
                            }
                            CuTaskType::Sink => {
                                panic!("CuSinkTask {name} cannot be a background task, it should be a regular task.");
                            }
                        }
                    } else {
                        panic!(
                            "{}: If a task is background, it has to have an output",
                            name_type.to_token_stream()
                        );
                    }
                } else {
                    name_type.clone()
                }
            })
            .collect();

        let instantiation_types = declared_task_types
            .iter()
            .zip(type_names.iter())
            .zip(cutypes.iter())
            .zip(background_flags.iter())
            .zip(output_types.iter())
            .zip(async_inner_task_types.iter())
            .map(|(((((name_type, name), cutype), &background), output_type), inner_type)| {
                if background {
                    if let Some(output_type) = output_type {
                        match cutype {
                            CuTaskType::Source => {
                                parse_quote!(CuAsyncSrcTask::<#inner_type, #output_type>)
                            }
                            CuTaskType::Regular => {
                                parse_quote!(CuAsyncTask::<#inner_type, #output_type>)
                            }
                            CuTaskType::Sink => {
                                panic!("CuSinkTask {name} cannot be a background task, it should be a regular task.");
                            }
                        }
                    } else {
                        panic!(
                            "{}: If a task is background, it has to have an output",
                            name_type.to_token_stream()
                        );
                    }
                } else {
                    name_type.clone()
                }
            })
            .collect();

        let sim_task_types = declared_task_types;

        let run_in_sim_flags = all_id_nodes
            .iter()
            .map(|(_, node)| node.is_run_in_sim())
            .collect();

        let mut node_id_to_task_index = vec![None; graph.node_count()];
        for (index, (node_id, _)) in all_id_nodes.iter().enumerate() {
            node_id_to_task_index[*node_id as usize] = Some(index);
        }
        let background_result_lags = vec![None; ids.len()];

        Ok(Self {
            ids,
            cutypes,
            stateless_flags,
            background_flags,
            background_result_lags,
            background_pools,
            anytime_configs,
            async_inner_task_types,
            logging_enabled,
            type_names,
            task_types,
            instantiation_types,
            sim_task_types,
            run_in_sim_flags,
            output_types,
            autogenerated_output_flags,
            node_id_to_task_index,
        })
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
) -> Vec<(String, String, Type)> {
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
            specs.push((origin.clone(), msg_type.clone(), payload_type.clone()));
        }
    }
    specs
}

/// Compute the per-slot [`HandleContent`] policy, reading each slot's producing task's
/// `NodeLogging.handle_content` from the config. Slots produced by bridges (or whose
/// producing task can't be located in the config) default to [`HandleContent::All`] —
/// matches the existing, payload-preserving behavior.
fn build_slot_handle_modes(
    cuconfig: &CuConfig,
    mission_label: Option<&str>,
    output_packs: &[OutputPack],
    node_output_positions: &HashMap<NodeId, usize>,
    task_names: &[(NodeId, String, String)],
) -> Vec<HandleContent> {
    let mut slot_modes: Vec<HandleContent> = vec![HandleContent::default(); output_packs.len()];
    for (node_id, task_id, _member) in task_names {
        let Some(pos) = node_output_positions.get(node_id) else {
            continue;
        };
        if let Some(node) = cuconfig.find_task_node(mission_label, task_id) {
            slot_modes[*pos] = node.handle_content_policy();
        }
    }
    slot_modes
}

fn extract_output_packs(runtime_plan: &CuExecutionLoop) -> Vec<OutputPack> {
    let mut packs: Vec<(u32, OutputPack)> = runtime_plan
        .steps
        .iter()
        .filter_map(|unit| match unit {
            CuExecutionUnit::Step(step) => {
                // Refine steps reuse the base step's slot: one slot per node.
                if step.phase == CuStepPhase::AnytimeRefine {
                    return None;
                }
                let output_pack = step.output_msg_pack.as_ref()?;
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
    let compressed = !cfg!(feature = "flat-copperlist-encoding");
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

        let helper = if compressed {
            quote! {
            fn #encode_fn<E: Encoder>(payload: &#payload_type, encoder: &mut E) -> Result<(), EncodeError> {
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
                    |codec| ::cu29::logcodec::encode_payload_with_codec(payload, codec, encoder),
                )
            }

            fn #decode_fn<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<#payload_type, DecodeError> {
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
                    |codec| ::cu29::logcodec::decode_payload_with_codec(decoder, codec),
                )
            }
            }
        } else {
            quote! {
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
            }
        };
        helpers.push(helper);
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
            CuExecutionUnit::Step(step) => {
                // Refine steps reuse the base step's slot: one slot per node.
                if step.phase == CuStepPhase::AnytimeRefine {
                    return None;
                }
                step.output_msg_pack
                    .as_ref()
                    .map(|output_pack| (output_pack.culist_index, output_pack.msg_types.len()))
            }
            CuExecutionUnit::Loop(_) => todo!("Needs to be implemented"),
        })
        .collect();

    sizes.sort_by_key(|(index, _)| *index);
    sizes.into_iter().map(|(_, size)| size).collect()
}

fn sorted_mission_graphs(copper_config: &CuConfig) -> Vec<(String, CuGraph)> {
    let mut all_missions: Vec<_> = copper_config
        .graphs
        .get_all_missions_graphs()
        .into_iter()
        .collect();
    all_missions.sort_by(|(left, _), (right, _)| left.cmp(right));
    all_missions
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct CanonicalTaskInputSlot {
    msg_type: String,
    connection_orders: BTreeSet<usize>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct MissionTaskInput {
    msg_type: String,
    connection_order: usize,
}

#[derive(Debug, Clone)]
struct TaskInputLayout {
    slots: Vec<CanonicalTaskInputSlot>,
    mission_slot_mappings: HashMap<String, Vec<Option<usize>>>,
}

#[derive(Clone, Copy)]
enum AlignmentStep {
    Match {
        canonical_slot_index: usize,
        mission_input_index: usize,
    },
    ExistingGap {
        canonical_slot_index: usize,
    },
    Insert {
        mission_input_index: usize,
    },
}

#[derive(Clone, Copy)]
enum AlignmentTransition {
    Match,
    ExistingGap,
    Insert,
}

#[derive(Clone, Copy)]
struct AlignmentBackpointer {
    prev_i: usize,
    prev_j: usize,
    transition: AlignmentTransition,
}

#[derive(Clone, Copy)]
struct AlignmentCell {
    score: i32,
    paths: u8,
    backpointer: Option<AlignmentBackpointer>,
}

impl AlignmentCell {
    fn unreachable() -> Self {
        Self {
            score: i32::MIN,
            paths: 0,
            backpointer: None,
        }
    }
}

fn collect_mission_task_inputs(
    graph: &CuGraph,
    node_id: NodeId,
    task_id: &str,
) -> CuResult<Vec<MissionTaskInput>> {
    let mut edge_ids = graph.get_dst_edges(node_id)?;
    edge_ids.sort_by_key(|edge_id| {
        graph
            .edge(*edge_id)
            .map(|edge| edge.order)
            .unwrap_or(usize::MAX)
    });

    edge_ids
        .into_iter()
        .map(|edge_id| {
            let edge = graph.edge(edge_id).ok_or_else(|| {
                CuError::from(format!(
                    "Missing edge {edge_id} while collecting inputs for task '{task_id}'"
                ))
            })?;
            Ok(MissionTaskInput {
                msg_type: edge.msg.clone(),
                connection_order: edge.order,
            })
        })
        .collect()
}

fn format_canonical_input_slots(slots: &[CanonicalTaskInputSlot]) -> String {
    let parts: Vec<String> = slots
        .iter()
        .map(|slot| {
            let orders = slot
                .connection_orders
                .iter()
                .map(|order| order.to_string())
                .collect::<Vec<_>>()
                .join("|");
            format!("{}@{}", slot.msg_type, orders)
        })
        .collect();
    format!("[{}]", parts.join(", "))
}

fn format_mission_task_inputs(inputs: &[MissionTaskInput]) -> String {
    let parts: Vec<String> = inputs
        .iter()
        .map(|input| format!("{}@{}", input.msg_type, input.connection_order))
        .collect();
    format!("[{}]", parts.join(", "))
}

const INPUT_MATCH_SCORE: i32 = 100;
const ANCHORED_INPUT_MATCH_BONUS: i32 = 1;

fn task_input_match_score(slot: &CanonicalTaskInputSlot, input: &MissionTaskInput) -> Option<i32> {
    if slot.msg_type != input.msg_type {
        return None;
    }

    let anchored_bonus = if slot.connection_orders.contains(&input.connection_order) {
        ANCHORED_INPUT_MATCH_BONUS
    } else {
        0
    };

    Some(INPUT_MATCH_SCORE + anchored_bonus)
}

fn update_alignment_cell(
    cell: &mut AlignmentCell,
    candidate_score: i32,
    candidate_paths: u8,
    backpointer: Option<AlignmentBackpointer>,
) {
    if candidate_paths == 0 {
        return;
    }

    if candidate_score > cell.score {
        cell.score = candidate_score;
        cell.paths = candidate_paths.min(2);
        cell.backpointer = if candidate_paths == 1 {
            backpointer
        } else {
            None
        };
    } else if candidate_score == cell.score {
        cell.paths = cell.paths.saturating_add(candidate_paths).min(2);
        cell.backpointer = None;
    }
}

fn align_task_inputs(
    task_id: &str,
    mission_name: &str,
    canonical_slots: &[CanonicalTaskInputSlot],
    mission_inputs: &[MissionTaskInput],
) -> CuResult<Vec<AlignmentStep>> {
    let canonical_len = canonical_slots.len();
    let mission_len = mission_inputs.len();
    let mut table = vec![vec![AlignmentCell::unreachable(); mission_len + 1]; canonical_len + 1];
    table[0][0] = AlignmentCell {
        score: 0,
        paths: 1,
        backpointer: None,
    };

    for i in 0..=canonical_len {
        for j in 0..=mission_len {
            let cell = table[i][j];
            if cell.paths == 0 {
                continue;
            }

            if i < canonical_len {
                update_alignment_cell(
                    &mut table[i + 1][j],
                    cell.score,
                    cell.paths,
                    if cell.paths == 1 {
                        Some(AlignmentBackpointer {
                            prev_i: i,
                            prev_j: j,
                            transition: AlignmentTransition::ExistingGap,
                        })
                    } else {
                        None
                    },
                );
            }

            if j < mission_len {
                update_alignment_cell(
                    &mut table[i][j + 1],
                    cell.score,
                    cell.paths,
                    if cell.paths == 1 {
                        Some(AlignmentBackpointer {
                            prev_i: i,
                            prev_j: j,
                            transition: AlignmentTransition::Insert,
                        })
                    } else {
                        None
                    },
                );
            }

            if i < canonical_len
                && j < mission_len
                && let Some(match_score) =
                    task_input_match_score(&canonical_slots[i], &mission_inputs[j])
            {
                update_alignment_cell(
                    &mut table[i + 1][j + 1],
                    cell.score + match_score,
                    cell.paths,
                    if cell.paths == 1 {
                        Some(AlignmentBackpointer {
                            prev_i: i,
                            prev_j: j,
                            transition: AlignmentTransition::Match,
                        })
                    } else {
                        None
                    },
                );
            }
        }
    }

    let final_cell = table[canonical_len][mission_len];
    if final_cell.paths > 1 {
        return Err(CuError::from(format!(
            "Task '{task_id}' has ambiguous input alignment while merging mission '{mission_name}'. Existing canonical inputs {} and mission inputs {} admit multiple equally valid alignments.",
            format_canonical_input_slots(canonical_slots),
            format_mission_task_inputs(mission_inputs),
        )));
    }

    let mut steps = Vec::new();
    let (mut i, mut j) = (canonical_len, mission_len);
    while i > 0 || j > 0 {
        let backpointer = table[i][j].backpointer.unwrap_or_else(|| {
            panic!(
                "Missing backpointer while aligning task '{task_id}' for mission '{mission_name}'"
            )
        });

        match backpointer.transition {
            AlignmentTransition::Match => steps.push(AlignmentStep::Match {
                canonical_slot_index: i - 1,
                mission_input_index: j - 1,
            }),
            AlignmentTransition::ExistingGap => steps.push(AlignmentStep::ExistingGap {
                canonical_slot_index: i - 1,
            }),
            AlignmentTransition::Insert => steps.push(AlignmentStep::Insert {
                mission_input_index: j - 1,
            }),
        }

        i = backpointer.prev_i;
        j = backpointer.prev_j;
    }
    steps.reverse();
    Ok(steps)
}

fn merge_task_input_layout(
    task_id: &str,
    layout: &mut TaskInputLayout,
    mission_name: String,
    mission_inputs: Vec<MissionTaskInput>,
) -> CuResult<()> {
    let alignment = align_task_inputs(task_id, &mission_name, &layout.slots, &mission_inputs)?;
    let mut new_slots = Vec::with_capacity(alignment.len());
    let mut old_to_new = vec![None; layout.slots.len()];
    let mut mission_mapping = Vec::with_capacity(alignment.len());

    for step in alignment {
        match step {
            AlignmentStep::Match {
                canonical_slot_index,
                mission_input_index,
            } => {
                let mut slot = layout.slots[canonical_slot_index].clone();
                slot.connection_orders
                    .insert(mission_inputs[mission_input_index].connection_order);
                let new_index = new_slots.len();
                old_to_new[canonical_slot_index] = Some(new_index);
                new_slots.push(slot);
                mission_mapping.push(Some(mission_input_index));
            }
            AlignmentStep::ExistingGap {
                canonical_slot_index,
            } => {
                let new_index = new_slots.len();
                old_to_new[canonical_slot_index] = Some(new_index);
                new_slots.push(layout.slots[canonical_slot_index].clone());
                mission_mapping.push(None);
            }
            AlignmentStep::Insert {
                mission_input_index,
            } => {
                new_slots.push(CanonicalTaskInputSlot {
                    msg_type: mission_inputs[mission_input_index].msg_type.clone(),
                    connection_orders: BTreeSet::from([
                        mission_inputs[mission_input_index].connection_order
                    ]),
                });
                mission_mapping.push(Some(mission_input_index));
            }
        }
    }

    let mut remapped_mission_slot_mappings =
        HashMap::with_capacity(layout.mission_slot_mappings.len() + 1);
    for (existing_mission, existing_mapping) in &layout.mission_slot_mappings {
        let mut remapped = vec![None; new_slots.len()];
        for (old_slot_index, maybe_local_input_index) in existing_mapping.iter().enumerate() {
            let new_slot_index = old_to_new[old_slot_index].unwrap_or_else(|| {
                panic!("Missing remap for task '{task_id}' canonical slot {old_slot_index}")
            });
            remapped[new_slot_index] = *maybe_local_input_index;
        }
        remapped_mission_slot_mappings.insert(existing_mission.clone(), remapped);
    }
    remapped_mission_slot_mappings.insert(mission_name, mission_mapping);

    layout.slots = new_slots;
    layout.mission_slot_mappings = remapped_mission_slot_mappings;
    Ok(())
}

fn collect_task_input_layouts(
    all_missions: &[(String, CuGraph)],
) -> CuResult<HashMap<String, TaskInputLayout>> {
    let mut task_mission_inputs: BTreeMap<String, Vec<(String, Vec<MissionTaskInput>)>> =
        BTreeMap::new();

    for (mission_name, graph) in all_missions {
        for (node_id, node) in graph.get_all_nodes() {
            if node.get_flavor() != Flavor::Task {
                continue;
            }

            if find_task_type_for_id(graph, node_id)? == CuTaskType::Source {
                continue;
            }

            let task_id = node.get_id().to_string();
            let mission_inputs = collect_mission_task_inputs(graph, node_id, task_id.as_str())?;
            task_mission_inputs
                .entry(task_id)
                .or_default()
                .push((mission_name.clone(), mission_inputs));
        }
    }

    let mut layouts = HashMap::new();
    for (task_id, mission_inputs) in task_mission_inputs {
        let mut mission_iter = mission_inputs.into_iter();
        let Some((first_mission, first_inputs)) = mission_iter.next() else {
            continue;
        };

        let slots: Vec<CanonicalTaskInputSlot> = first_inputs
            .iter()
            .map(|input| CanonicalTaskInputSlot {
                msg_type: input.msg_type.clone(),
                connection_orders: BTreeSet::from([input.connection_order]),
            })
            .collect();
        let mut mission_slot_mappings = HashMap::new();
        mission_slot_mappings.insert(
            first_mission,
            (0..first_inputs.len()).map(Some).collect::<Vec<_>>(),
        );

        let mut layout = TaskInputLayout {
            slots,
            mission_slot_mappings,
        };
        for (mission_name, mission_inputs) in mission_iter {
            merge_task_input_layout(&task_id, &mut layout, mission_name, mission_inputs)?;
        }

        layouts.insert(task_id, layout);
    }

    Ok(layouts)
}

struct GeneratedTaskInput {
    setup: proc_macro2::TokenStream,
    expr: proc_macro2::TokenStream,
}

fn present_task_input_expr(
    input: &cu29_runtime::curuntime::CuInputMsg,
    output_pack_sizes: &[usize],
) -> proc_macro2::TokenStream {
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
        quote! { __cu_input!(#input_index, #port_index) }
    } else {
        quote! { __cu_input!(#input_index) }
    }
}

fn generate_task_input_binding(
    step: &CuExecutionStep,
    mission_name: &str,
    output_pack_sizes: &[usize],
    task_input_layouts: &HashMap<String, TaskInputLayout>,
) -> GeneratedTaskInput {
    let task_id = step.node.get_id().to_string();
    let layout = task_input_layouts
        .get(&task_id)
        .unwrap_or_else(|| panic!("Missing canonical input layout for task '{task_id}'"));
    let slot_mapping = layout
        .mission_slot_mappings
        .get(mission_name)
        .unwrap_or_else(|| {
            panic!("Missing input slot mapping for task '{task_id}' in mission '{mission_name}'")
        });

    let mut setup = Vec::new();
    let mut refs = Vec::new();

    for (slot_index, slot) in layout.slots.iter().enumerate() {
        if let Some(input_index) = slot_mapping.get(slot_index).copied().flatten() {
            let input = step.input_msg_indices_types.get(input_index).unwrap_or_else(|| {
                panic!(
                    "Task '{task_id}' mission '{mission_name}' input slot {slot_index} mapped to missing input index {input_index}"
                )
            });
            refs.push(present_task_input_expr(input, output_pack_sizes));
            continue;
        }

        let empty_input_ident = format_ident!("__cu_missing_input_{slot_index}");
        let input_ty: Type = parse_str(slot.msg_type.as_str()).unwrap_or_else(|err| {
            panic!(
                "Could not parse canonical input message type '{}' for task '{}': {err}",
                slot.msg_type, task_id
            )
        });
        setup.push(quote! {
            let #empty_input_ident = cu29::cutask::CuMsg::<#input_ty>::new(None);
        });
        refs.push(quote! { &#empty_input_ident });
    }

    let expr = match refs.len() {
        0 => quote! { &() },
        1 => refs
            .into_iter()
            .next()
            .expect("single input expression missing"),
        _ => quote! { &( #(#refs),* ) },
    };

    GeneratedTaskInput {
        setup: quote! { #(#setup)* },
        expr,
    }
}

/// Builds the tuple of the CuList as a tuple off all the output slots.
fn build_culist_tuple(slot_types: &[Type]) -> TypeTuple {
    if slot_types.is_empty() {
        parse_quote! { () }
    } else {
        parse_quote! { ( #( #slot_types ),*, ) }
    }
}

/// Builds the canonical generated encoding: one common metadata block followed
/// by the payload bytes selected for the local log view.
fn build_compressed_culist_tuple_encode(
    output_packs: &[OutputPack],
    encode_helper_names: &[Option<Ident>],
    slot_handle_modes: &[HandleContent],
    cumsg_count: usize,
    capture_slots: Option<&[bool]>,
) -> ItemImpl {
    let mut flat_idx = 0usize;
    let mut metadata_refs = Vec::with_capacity(cumsg_count);
    let mut original_idents = Vec::with_capacity(cumsg_count);
    let mut captured_idents = Vec::with_capacity(cumsg_count);
    let mut presence_initializers = Vec::with_capacity(cumsg_count);
    let mut payload_encoders = Vec::with_capacity(cumsg_count);

    for (slot_idx, pack) in output_packs.iter().enumerate() {
        let slot_index = syn::Index::from(slot_idx);
        let mode = slot_handle_modes.get(slot_idx).copied().unwrap_or_default();
        for (port_idx, payload_ty) in pack.msg_types.iter().enumerate() {
            let access = if pack.is_multi() {
                let port_index = syn::Index::from(port_idx);
                quote! { self.0.#slot_index.#port_index }
            } else {
                quote! { self.0.#slot_index }
            };
            let original_ident = format_ident!("__cu_original_payload_{flat_idx}");
            let captured_ident = format_ident!("__cu_captured_payload_{flat_idx}");
            let helper = encode_helper_names[flat_idx].clone();
            let cache_index = flat_idx;
            flat_idx += 1;

            metadata_refs.push(quote! {
                ::cu29::copperlist_codec::CommonMetadataRef::new(
                    &#access.tov,
                    &#access.metadata,
                )
            });
            original_idents.push(original_ident.clone());
            captured_idents.push(captured_ident.clone());

            let captured_value = if capture_slots.is_some_and(|slots| !slots[slot_idx]) {
                quote! { false }
            } else if mode == HandleContent::default() {
                quote! { #original_ident }
            } else {
                let mode_u8 = mode as u8;
                quote! {
                    match #access.payload() {
                        Some(__cu_payload) => {
                            use ::cu29::pool::PayloadDefaultHandlePolicyApply as _;
                            use ::cu29::pool::PayloadDefaultLoggingPolicy as _;
                            __cu_payload.apply_handle_content_policy(
                                ::cu29::config::HandleContent::from_u8(#mode_u8),
                            );
                            __cu_payload.payload_should_log()
                        }
                        None => false,
                    }
                }
            };
            let awareness_check = if mode == HandleContent::default() {
                quote! {}
            } else {
                quote! {
                    const _: fn() = || {
                        fn assert_aware<__T: ::cu29::pool::HandleContentAware + ?::core::marker::Sized>() {}
                        assert_aware::<#payload_ty>();
                    };
                }
            };
            presence_initializers.push(quote! {
                #awareness_check
                let #original_ident = #access.payload().is_some();
                let #captured_ident = #captured_value;
            });

            let encode_payload = if let Some(helper) = helper {
                quote! { #helper(__cu_payload, encoder)?; }
            } else {
                quote! {
                    ::cu29::copperlist_codec::encode_payload(__cu_payload, encoder)?;
                }
            };
            payload_encoders.push(quote! {
                if #captured_ident {
                    __cu_capture.select_slot(#cache_index);
                    let __cu_payload = #access.payload().ok_or(
                        EncodeError::Other("CopperList captured-payload plane mismatch")
                    )?;
                    #encode_payload
                }
            });
        }
    }

    let (impl_header, method) = if capture_slots.is_some() {
        (
            quote! { impl CuStampedDataSet },
            format_ident!("__encode_capture"),
        )
    } else {
        (
            quote! { impl Encode for CuStampedDataSet },
            format_ident!("encode"),
        )
    };
    parse_quote! {
        #impl_header {
            fn #method<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                let __cu_capture = cu29::monitoring::start_copperlist_io_capture(&self.1);
                #(#presence_initializers)*
                let __cu_metadata = [#(#metadata_refs),*];
                let __cu_original_payload_presence = [#(#original_idents),*];
                let __cu_captured_payload_presence = [#(#captured_idents),*];
                ::cu29::copperlist_codec::encode_common_metadata::<#cumsg_count, _>(
                    &__cu_metadata,
                    &__cu_original_payload_presence,
                    &__cu_captured_payload_presence,
                    encoder,
                )?;
                #(#payload_encoders)*
                Ok(())
            }
        }
    }
}

fn build_compressed_culist_tuple_decode(
    output_packs: &[OutputPack],
    slot_types: &[Type],
    cumsg_count: usize,
    decode_helper_names: &[Option<Ident>],
    distributed: bool,
) -> ItemImpl {
    let mut flat_idx = 0usize;
    let mut decode_fields = Vec::with_capacity(slot_types.len());

    for pack in output_packs {
        if pack.is_multi() {
            let mut fields = Vec::with_capacity(pack.msg_types.len());
            for payload_ty in &pack.msg_types {
                let metadata_index = flat_idx;
                let helper = decode_helper_names[flat_idx].clone();
                flat_idx += 1;
                let decode_payload = if let Some(helper) = helper {
                    quote! { #helper(decoder)? }
                } else {
                    quote! { ::cu29::copperlist_codec::decode_payload::<#payload_ty, _>(decoder)? }
                };
                fields.push(quote! {
                    {
                        let __cu_slot = __cu_metadata.take_slot(#metadata_index);
                        let __cu_payload = if __cu_slot.captured_payload_present {
                            Some(#decode_payload)
                        } else {
                            None
                        };
                        ::cu29::copperlist_codec::restore_msg(__cu_payload, __cu_slot)
                    }
                });
            }
            decode_fields.push(quote! { ( #(#fields),* ) });
        } else {
            let payload_ty = pack
                .msg_types
                .first()
                .expect("single-port pack must have a payload type");
            let metadata_index = flat_idx;
            let helper = decode_helper_names[flat_idx].clone();
            flat_idx += 1;
            let decode_payload = if let Some(helper) = helper {
                quote! { #helper(decoder)? }
            } else {
                quote! { ::cu29::copperlist_codec::decode_payload::<#payload_ty, _>(decoder)? }
            };
            decode_fields.push(quote! {
                {
                    let __cu_slot = __cu_metadata.take_slot(#metadata_index);
                    let __cu_payload = if __cu_slot.captured_payload_present {
                        Some(#decode_payload)
                    } else {
                        None
                    };
                    ::cu29::copperlist_codec::restore_msg(__cu_payload, __cu_slot)
                }
            });
        }
    }

    let constructor = distributed.then(|| quote!(CuMessageRegions));
    parse_quote! {
        impl Decode<()> for CuStampedDataSet {
            fn decode<D: Decoder<Context=()>>(decoder: &mut D) -> Result<Self, DecodeError> {
                let mut __cu_metadata =
                    ::cu29::copperlist_codec::decode_common_metadata::<#cumsg_count, _>(decoder)?;
                Ok(CuStampedDataSet(
                    #constructor (
                        #((#decode_fields).into()),*,
                    ),
                    cu29::monitoring::CuMsgIoCache::<#cumsg_count>::default(),
                ))
            }
        }
    }
}

/// This is the bincode encoding part of the CuStampedDataSet
fn build_culist_tuple_encode(
    output_packs: &[OutputPack],
    encode_helper_names: &[Option<Ident>],
    slot_handle_modes: &[HandleContent],
) -> ItemImpl {
    let mut flat_idx = 0usize;
    let mut encode_fields = Vec::new();

    for (slot_idx, pack) in output_packs.iter().enumerate() {
        let slot_index = syn::Index::from(slot_idx);
        let mode = slot_handle_modes.get(slot_idx).copied();

        if pack.is_multi() {
            for (port_idx, payload_ty) in pack.msg_types.iter().enumerate() {
                let port_index = syn::Index::from(port_idx);
                let cache_index = flat_idx;
                let encode_helper = encode_helper_names[flat_idx].clone();
                flat_idx += 1;
                let normal_encode = if let Some(helper) = encode_helper {
                    quote! { #helper(&self.0.#slot_index.#port_index, encoder)?; }
                } else {
                    quote! { self.0.#slot_index.#port_index.encode(encoder)?; }
                };
                let slot_access = quote! { self.0.#slot_index.#port_index };
                let slot_block =
                    build_per_slot_encode_block(mode, payload_ty, &slot_access, &normal_encode);
                encode_fields.push(quote! {
                    __cu_capture.select_slot(#cache_index);
                    #slot_block
                });
            }
        } else {
            let cache_index = flat_idx;
            let encode_helper = encode_helper_names[flat_idx].clone();
            flat_idx += 1;
            let normal_encode = if let Some(helper) = encode_helper {
                quote! { #helper(&self.0.#slot_index, encoder)?; }
            } else {
                quote! { self.0.#slot_index.encode(encoder)?; }
            };
            let slot_access = quote! { self.0.#slot_index };
            let payload_ty = pack
                .msg_types
                .first()
                .expect("single-port pack must have a payload type");
            let slot_block =
                build_per_slot_encode_block(mode, payload_ty, &slot_access, &normal_encode);
            encode_fields.push(quote! {
                __cu_capture.select_slot(#cache_index);
                #slot_block
            });
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

/// Build the per-slot encode block. Mode `All` returns the existing encode call
/// (zero codegen change). Non-default modes wrap it with a `HandleContentAware`
/// bound check on the payload type and an autoref-specialized policy check that
/// routes to `encode_metadata_only` when the payload says skip.
fn build_per_slot_encode_block(
    mode: Option<HandleContent>,
    payload_ty: &Type,
    slot_access: &proc_macro2::TokenStream,
    normal_encode: &proc_macro2::TokenStream,
) -> proc_macro2::TokenStream {
    let mode = match mode {
        Some(m) if m != HandleContent::default() => m,
        _ => return normal_encode.clone(),
    };
    let mode_u8 = mode as u8;
    quote! {
        {
            // Catches the silent-no-op footgun: non-default handle_content on an
            // unmarked payload fails here with `HandleContentAware not satisfied`.
            const _: fn() = || {
                fn assert_aware<__T: ::cu29::pool::HandleContentAware + ?::core::marker::Sized>() {}
                assert_aware::<#payload_ty>();
            };
            use ::cu29::pool::PayloadDefaultHandlePolicyApply as _;
            use ::cu29::pool::PayloadDefaultLoggingPolicy as _;
            // Stamp the source's configured policy on whatever handles live in the
            // payload, then ask the (now-policy-aware) payload whether to log.
            let __cu_should_log = match #slot_access.payload() {
                Some(__cu_p) => {
                    __cu_p.apply_handle_content_policy(
                        ::cu29::config::HandleContent::from_u8(#mode_u8),
                    );
                    __cu_p.payload_should_log()
                }
                None => false,
            };
            if __cu_should_log {
                #normal_encode
            } else {
                ::cu29::cutask::encode_metadata_only(&#slot_access, encoder)?;
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
    distributed: bool,
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

    let constructor = distributed.then(|| quote!(CuMessageRegions));
    parse_quote! {
        impl Decode<()> for CuStampedDataSet {
            fn decode<D: Decoder<Context=()>>(decoder: &mut D) -> Result<Self, DecodeError> {
                Ok(CuStampedDataSet(
                    #constructor (
                        #((#decode_fields).into()),*,
                    ),
                    cu29::monitoring::CuMsgIoCache::<#cumsg_count>::default(),
                ))
            }
        }
    }
}

fn build_culist_erasedcumsgs(output_packs: &[OutputPack], distributed: bool) -> ItemImpl {
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
            let value = distributed.then(|| quote!(.value));
            casted_fields.push(quote! { &self.0.#slot_index #value as &dyn ErasedCuStampedData });
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
        impl cu29::serde::ser::Serialize for CuStampedDataSet {
            fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
            where
                S: cu29::serde::Serializer,
            {
                use cu29::serde::ser::SerializeTuple;
                let mut tuple = serializer.serialize_tuple(#tuple_len)?;
                #(tuple.serialize_element(#serialize_fields)?;)*
                tuple.end()
            }
        }
    }
}

/// This is the default implementation for CuStampedDataSet
fn build_culist_tuple_default(
    slot_types: &[Type],
    cumsg_count: usize,
    distributed: bool,
) -> ItemImpl {
    let default_fields: Vec<_> = slot_types
        .iter()
        .map(|slot_type| quote! { <#slot_type as Default>::default() })
        .collect();

    let constructor = distributed.then(|| quote!(CuMessageRegions));
    parse_quote! {
        impl Default for CuStampedDataSet {
            fn default() -> CuStampedDataSet
            {
                CuStampedDataSet(
                    #constructor (
                        #((#default_fields).into()),*,
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
    inputs: Vec<(String, String)>,
    id: String,
    provider_path: syn::Path,
}

struct LogStreamResourceSpec {
    destination_index: usize,
    stream: LogStreamEndpointSpec,
    feedback: Option<LogStreamEndpointSpec>,
}

struct LogStreamEndpointSpec {
    bundle_index: usize,
    provider_path: syn::Path,
    resource_name: String,
    transport_type: syn::Type,
}

fn build_logstream_resource_specs(
    config: &CuConfig,
    mission: &str,
    component_resources: &[ResourceKeySpec],
) -> CuResult<Vec<LogStreamResourceSpec>> {
    let Some(streaming) = &config.log_streaming else {
        return Ok(Vec::new());
    };
    let bundle_specs = build_bundle_specs(config, mission)?;
    let bundle_lookup: HashMap<_, _> = bundle_specs
        .iter()
        .enumerate()
        .map(|(index, bundle)| (bundle.id.as_str(), (index, &bundle.provider_path)))
        .collect();
    let endpoint = |destination_id: &str,
                    transport: &cu29_runtime::config::LogStreamTransportConfig|
     -> CuResult<LogStreamEndpointSpec> {
        let (bundle_id, resource_name) = parse_resource_path(&transport.resource)?;
        let (bundle_index, provider_path) = bundle_lookup.get(bundle_id.as_str()).ok_or_else(|| CuError::from(format!(
            "Log-stream destination '{destination_id}' references resource bundle '{bundle_id}' which is not active in mission '{mission}'"
        )))?;
        if component_resources.iter().any(|resource| {
            resource.bundle_index == *bundle_index && resource.resource_name == resource_name
        }) {
            return Err(CuError::from(format!(
                "Log-stream destination '{destination_id}' and a task or bridge both require exclusive resource '{}'",
                transport.resource
            )));
        }
        if bundle_specs.iter().any(|bundle| {
            bundle
                .inputs
                .iter()
                .any(|(_, path)| path == &transport.resource)
        }) {
            return Err(CuError::from(format!(
                "Log-stream destination '{destination_id}' and a resource bundle both require exclusive resource '{}'",
                transport.resource
            )));
        }
        let transport_type = parse_str::<Type>(&transport.type_).map_err(|error| CuError::from(format!(
            "Log-stream destination '{destination_id}' transport type '{}' is not a valid Rust type: {error}", transport.type_
        )))?;
        Ok(LogStreamEndpointSpec {
            bundle_index: *bundle_index,
            provider_path: (*provider_path).clone(),
            resource_name,
            transport_type,
        })
    };
    streaming
        .destinations
        .iter()
        .enumerate()
        .map(|(destination_index, destination)| {
            Ok(LogStreamResourceSpec {
                destination_index,
                stream: endpoint(&destination.id, &destination.transport)?,
                feedback: destination
                    .feedback
                    .as_ref()
                    .map(|f| endpoint(&destination.id, &f.transport))
                    .transpose()?,
            })
        })
        .collect()
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
            let mut inputs: Vec<_> = bundle
                .resources
                .iter()
                .flat_map(|map| map.iter())
                .map(|(name, path)| (name.clone(), path.clone()))
                .collect();
            inputs.sort();
            Ok(BundleSpec {
                inputs,
                id: bundle.id.clone(),
                provider_path,
            })
        })
        .collect()
}

/// Compute construction order while preserving declaration-based resource keys.
fn resource_bundle_build_order(bundles: &[BundleSpec]) -> CuResult<Vec<usize>> {
    let mut lookup = HashMap::new();
    for (index, bundle) in bundles.iter().enumerate() {
        if lookup.insert(bundle.id.as_str(), index).is_some() {
            return Err(CuError::from(format!(
                "Duplicate resource bundle '{}'",
                bundle.id
            )));
        }
    }
    let dependencies = bundles.iter().map(|bundle| {
        bundle.inputs.iter().map(|(_, path)| {
            let (source, _) = parse_resource_path(path)?;
            lookup.get(source.as_str()).copied().ok_or_else(|| CuError::from(format!(
                "Resource bundle '{}' depends on '{}' which is not active in this mission", bundle.id, source
            )))
        }).collect::<CuResult<Vec<_>>>()
    }).collect::<CuResult<Vec<_>>>()?;
    let mut built = vec![false; bundles.len()];
    let mut order = Vec::with_capacity(bundles.len());
    while order.len() < bundles.len() {
        let Some(index) = (0..bundles.len()).find(|&index| {
            !built[index]
                && dependencies[index]
                    .iter()
                    .all(|&dependency| built[dependency])
        }) else {
            let cycle = bundles
                .iter()
                .enumerate()
                .filter(|(index, _)| !built[*index])
                .map(|(_, bundle)| bundle.id.as_str())
                .collect::<Vec<_>>()
                .join(", ");
            return Err(CuError::from(format!(
                "Resource dependency cycle involving: {cycle}"
            )));
        };
        built[index] = true;
        order.push(index);
    }
    Ok(order)
}

#[cfg(test)]
mod resource_stack_tests {
    use super::*;
    type InputSpecs<'a> = &'a [(&'a str, &'a str)];
    fn specs(entries: &[(&str, InputSpecs<'_>)]) -> Vec<BundleSpec> {
        entries
            .iter()
            .map(|(id, inputs)| BundleSpec {
                id: (*id).into(),
                provider_path: syn::parse_str("Provider").unwrap(),
                inputs: inputs
                    .iter()
                    .map(|(name, path)| ((*name).into(), (*path).into()))
                    .collect(),
            })
            .collect()
    }
    #[test]
    fn resource_dependencies_validate_and_keep_indices() {
        assert_eq!(
            resource_bundle_build_order(&specs(&[
                ("stream", &[("serial", "radio.serial")]),
                ("board", &[]),
                ("radio", &[("serial", "board.uart")]),
            ]))
            .unwrap(),
            [1, 2, 0]
        );
        for invalid in [
            specs(&[("a", &[("x", "a.x")])]),
            specs(&[("a", &[("x", "b.x")]), ("b", &[("x", "a.x")])]),
            specs(&[("a", &[("x", "inactive.x")])]),
            specs(&[("a", &[]), ("a", &[])]),
        ] {
            assert!(resource_bundle_build_order(&invalid).is_err());
        }
    }
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

    let order = resource_bundle_build_order(bundle_specs)?;
    let mut validations = Vec::new();
    let mut bundle_inits = Vec::new();
    for index in order {
        let bundle = &bundle_specs[index];
        let bundle_id = LitStr::new(&bundle.id, Span::call_site());
        let provider_path = &bundle.provider_path;
        let input_count = bundle.inputs.len();
        let checks = bundle.inputs.iter().map(|(name, path)| {
            quote! {
                bundle_cfg.resources.as_ref().and_then(|map| map.get(#name))
                    .is_some_and(|value| value == #path)
            }
        });
        validations.push(quote! {
            let bundle_cfg = config.resources.iter().find(|b| b.id == #bundle_id)
                .ok_or_else(|| cu29::CuError::from(concat!("Missing resource bundle: ", #bundle_id)))?;
            if bundle_cfg.resources.as_ref().map_or(0, |map| map.len()) != #input_count
                #(|| !(#checks))* {
                return Err(cu29::CuError::from(concat!("Resource input topology differs from compiled configuration: ", #bundle_id)));
            }
        });
        let mut entries = Vec::new();
        for (name, path) in &bundle.inputs {
            let (source_id, slot) = parse_resource_path(path)?;
            let source_index = bundle_specs
                .iter()
                .position(|source| source.id == source_id)
                .expect("validated resource dependency");
            let source_provider = &bundle_specs[source_index].provider_path;
            entries.push(quote! {
                (#name, cu29::resource::ResourceKey::new(
                    cu29::resource::BundleIndex::new(#source_index),
                    cu29::resource::resource_index_by_name::<#source_provider>(#slot),
                ))
            });
        }
        bundle_inits.push(quote! {
            {
                const INPUT_KEYS: [cu29::resource::ResourceKey;
                    <#provider_path as cu29::resource::ResourceBundle>::INPUT_NAMES.len()] =
                    cu29::resource::resource_input_keys(
                        <#provider_path as cu29::resource::ResourceBundle>::INPUT_NAMES,
                        &[#(#entries),*],
                    );
                let bundle_cfg = config.resources.iter().find(|b| b.id == #bundle_id)
                    .expect("validated resource bundle");
                let bundle_ctx = cu29::resource::BundleContext::<#provider_path>::new(
                    cu29::resource::BundleIndex::new(#index), #bundle_id,
                ).with_input_keys(&INPUT_KEYS);
                <#provider_path as cu29::resource::ResourceBundle>::build(
                    bundle_ctx, bundle_cfg.config.as_ref(), &mut manager,
                )?;
            }
        });
    }

    let resources_instanciator = quote! {
        pub fn resources_instanciator(config: &CuConfig) -> CuResult<cu29::resource::ResourceManager> {
            #(#validations)*
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
    sim_mode: bool,
) -> CuResult<ResourceMappingTokens> {
    let mut per_task: Vec<Vec<&ResourceKeySpec>> = vec![Vec::new(); task_specs.ids.len()];

    for spec in resource_specs {
        let ResourceOwner::Task(task_index) = spec.owner else {
            continue;
        };
        if sim_mode
            && !task_specs.run_in_sim_flags[task_index]
            && task_specs.cutypes[task_index] != CuTaskType::Regular
        {
            continue;
        }
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

        // A backgrounded task binds the resources of what the wrapper wraps —
        // the runner for an anytime node, the task itself otherwise.
        let binding_task_type = &task_specs.async_inner_task_types[idx];

        let binding_trait = task_trait_for_specs(task_specs, idx);

        let entries_ident = format_ident!("TASK{}_RES_ENTRIES", idx);
        let map_ident = format_ident!("TASK{}_RES_MAPPING", idx);
        let binding_type = quote! {
            <<#binding_task_type as #binding_trait>::Resources<'_> as ResourceBindings>::Binding
        };
        let entry_tokens = entries.iter().map(|spec| {
            let binding_ident = Ident::new(
                &config_id_to_enum(spec.binding_name.as_str()),
                Span::call_site(),
            );
            let resource_name = LitStr::new(spec.resource_name.as_str(), Span::call_site());
            let bundle_index = spec.bundle_index;
            let provider_path = &spec.provider_path;
            quote! {
                (#binding_type::#binding_ident, cu29::resource::ResourceKey::new(
                    cu29::resource::BundleIndex::new(#bundle_index),
                    cu29::resource::resource_index_by_name::<#provider_path>(#resource_name),
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
            let binding_ident = Ident::new(
                &config_id_to_enum(spec.binding_name.as_str()),
                Span::call_site(),
            );
            let resource_name = LitStr::new(spec.resource_name.as_str(), Span::call_site());
            let bundle_index = spec.bundle_index;
            let provider_path = &spec.provider_path;
            quote! {
                (#binding_type::#binding_ident, cu29::resource::ResourceKey::new(
                    cu29::resource::BundleIndex::new(#bundle_index),
                    cu29::resource::resource_index_by_name::<#provider_path>(#resource_name),
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

struct ExecutionSchedule {
    background: Vec<CuPlanBackground>,
    lanes: Option<LanePlan>,
}

type ExecutionPlanBuild = (
    CuExecutionLoop,
    Vec<ExecutionEntity>,
    HashMap<NodeId, NodeId>,
    ExecutionSchedule,
);

fn build_execution_plan(
    config: &CuConfig,
    graph: &CuGraph,
    mission: &str,
    bridge_specs: &mut [BridgeSpec],
) -> CuResult<(
    CuExecutionLoop,
    Vec<ExecutionEntity>,
    HashMap<NodeId, NodeId>,
)> {
    build_execution_plan_with_schedule(config, graph, mission, bridge_specs)
        .map(|(execution, entities, plan_to_original, _)| (execution, entities, plan_to_original))
}

fn build_execution_plan_with_schedule(
    config: &CuConfig,
    graph: &CuGraph,
    mission: &str,
    bridge_specs: &mut [BridgeSpec],
) -> CuResult<ExecutionPlanBuild> {
    let assembled = assemble_runtime_plan_for_mission(config, graph, mission)?;
    let mut exec_entities = Vec::with_capacity(assembled.entities.len());
    for (plan_node_id, entity) in assembled.entities.iter().enumerate() {
        let kind = match entity.kind {
            PlanEntityKind::Task { task_index, .. } => ExecutionEntityKind::Task { task_index },
            PlanEntityKind::BridgeRx {
                bridge_config_index,
                channel_config_index,
            } => {
                let bridge_index = bridge_specs
                    .iter()
                    .position(|bridge| bridge.config_index == bridge_config_index)
                    .expect("shared planner returned an unknown bridge");
                let channel_index = bridge_specs[bridge_index]
                    .rx_channels
                    .iter()
                    .position(|channel| channel.config_index == channel_config_index)
                    .expect("shared planner returned an unknown bridge rx channel");
                bridge_specs[bridge_index].rx_channels[channel_index].plan_node_id =
                    Some(plan_node_id as NodeId);
                ExecutionEntityKind::BridgeRx {
                    bridge_index,
                    channel_index,
                }
            }
            PlanEntityKind::BridgeTx {
                bridge_config_index,
                channel_config_index,
            } => {
                let bridge_index = bridge_specs
                    .iter()
                    .position(|bridge| bridge.config_index == bridge_config_index)
                    .expect("shared planner returned an unknown bridge");
                let channel_index = bridge_specs[bridge_index]
                    .tx_channels
                    .iter()
                    .position(|channel| channel.config_index == channel_config_index)
                    .expect("shared planner returned an unknown bridge tx channel");
                bridge_specs[bridge_index].tx_channels[channel_index].plan_node_id =
                    Some(plan_node_id as NodeId);
                ExecutionEntityKind::BridgeTx {
                    bridge_index,
                    channel_index,
                }
            }
        };
        exec_entities.push(ExecutionEntity { kind });
    }
    let plan_to_original = assembled
        .plan_to_original
        .iter()
        .enumerate()
        .filter_map(|(plan_node_id, original)| {
            original.map(|original| (plan_node_id as NodeId, original))
        })
        .collect();
    let schedule = ExecutionSchedule {
        background: assembled.background,
        lanes: assembled.lanes,
    };
    Ok((
        assembled.execution,
        exec_entities,
        plan_to_original,
        schedule,
    ))
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
            && step.phase != CuStepPhase::AnytimeRefine // refine steps reuse the base step's slot
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
            && step.phase != CuStepPhase::AnytimeRefine // refine steps reuse the base step's slot
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

        // An anytime node is one parallel-rt stage: its base and refine steps
        // all run inside the base step's stage, on the same worker thread
        // (memory locality), so refine steps produce no stage of their own.
        if step.phase == CuStepPhase::AnytimeRefine {
            continue;
        }

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
    task_trait: proc_macro2::TokenStream,
    task_type: &Type,
    stateless: bool,
    component_index: usize,
    mission_mod: &Ident,
    task_instance: &proc_macro2::TokenStream,
    placement: ParallelLifecyclePlacement,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let rt_guard = rtsan_guard_tokens();
    let abort_process_step = abort_process_step_tokens(true);

    let preprocess_alloc_open = alloc_scope_open_tokens();
    let preprocess_alloc_close = alloc_scope_close_tokens(
        quote! { monitor },
        quote! { #component_index },
        quote! { CuComponentState::Preprocess },
    );
    let postprocess_alloc_open = alloc_scope_open_tokens();
    let postprocess_alloc_close = alloc_scope_close_tokens(
        quote! { monitor },
        quote! { #component_index },
        quote! { CuComponentState::Postprocess },
    );
    let task_borrow = if stateless {
        quote! { &#task_instance }
    } else {
        quote! { &mut #task_instance }
    };
    let preprocess = if placement.preprocess {
        quote! {
            execution_probe.record(cu29::monitoring::ExecutionMarker {
                component_id: cu29::monitoring::ComponentId::new(#component_index),
                step: CuComponentState::Preprocess,
                culistid: Some(clid),
            });
            ctx.set_current_task(#component_index);
            #preprocess_alloc_open
            let maybe_error = {
                #rt_guard
                <#task_type as #task_trait>::preprocess(#task_borrow, &ctx)
            };
            #preprocess_alloc_close
            if let Err(error) = maybe_error {
                let decision = monitor.process_error(
                    cu29::monitoring::ComponentId::new(#component_index),
                    CuComponentState::Preprocess,
                    &error,
                );
                match decision {
                    Decision::Abort => {
                        debug!(ctx,
                            "Preprocess: ABORT decision from monitoring. Component '{}' errored out during preprocess. Aborting CopperList {}.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index)),
                            clid
                        );
                        #abort_process_step
                    }
                    Decision::Ignore => {
                        debug!(ctx,
                            "Preprocess: IGNORE decision from monitoring. Component '{}' errored out during preprocess. The runtime will continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Shutdown => {
                        debug!(ctx,
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
            #postprocess_alloc_open
            let maybe_error = {
                #rt_guard
                <#task_type as #task_trait>::postprocess(#task_borrow, &ctx)
            };
            #postprocess_alloc_close
            if let Err(error) = maybe_error {
                let decision = monitor.process_error(
                    cu29::monitoring::ComponentId::new(#component_index),
                    CuComponentState::Postprocess,
                    &error,
                );
                match decision {
                    Decision::Abort => {
                        debug!(ctx,
                            "Postprocess: ABORT decision from monitoring. Component '{}' errored out during postprocess. Continuing with the completed CopperList.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Ignore => {
                        debug!(ctx,
                            "Postprocess: IGNORE decision from monitoring. Component '{}' errored out during postprocess. The runtime will continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Shutdown => {
                        debug!(ctx,
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
    keyframe_logging_enabled: bool,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let rt_guard = rtsan_guard_tokens();
    let abort_process_step = abort_process_step_tokens(true);
    let freeze_bridge = keyframe_freeze_bridge_tokens(keyframe_logging_enabled);

    let preprocess_alloc_open = alloc_scope_open_tokens();
    let preprocess_alloc_close = alloc_scope_close_tokens(
        quote! { monitor },
        quote! { #component_index },
        quote! { CuComponentState::Preprocess },
    );
    let postprocess_alloc_open = alloc_scope_open_tokens();
    let postprocess_alloc_close = alloc_scope_close_tokens(
        quote! { monitor },
        quote! { #component_index },
        quote! { CuComponentState::Postprocess },
    );
    let preprocess = if placement.preprocess {
        quote! {
            execution_probe.record(cu29::monitoring::ExecutionMarker {
                component_id: cu29::monitoring::ComponentId::new(#component_index),
                step: CuComponentState::Preprocess,
                culistid: Some(clid),
            });
            ctx.set_current_component(#component_index);
            ctx.clear_current_task();
            #preprocess_alloc_open
            let maybe_error = {
                #rt_guard
                <#bridge_type as cu29::cubridge::CuBridge>::preprocess(bridge, &ctx)
            };
            #preprocess_alloc_close
            if let Err(error) = maybe_error {
                let decision = monitor.process_error(
                    cu29::monitoring::ComponentId::new(#component_index),
                    CuComponentState::Preprocess,
                    &error,
                );
                match decision {
                    Decision::Abort => {
                        debug!(ctx,
                            "Preprocess: ABORT decision from monitoring. Component '{}' errored out during preprocess. Aborting CopperList {}.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index)),
                            clid
                        );
                        #abort_process_step
                    }
                    Decision::Ignore => {
                        debug!(ctx,
                            "Preprocess: IGNORE decision from monitoring. Component '{}' errored out during preprocess. The runtime will continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Shutdown => {
                        debug!(ctx,
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
            #freeze_bridge
            execution_probe.record(cu29::monitoring::ExecutionMarker {
                component_id: cu29::monitoring::ComponentId::new(#component_index),
                step: CuComponentState::Postprocess,
                culistid: Some(clid),
            });
            ctx.set_current_component(#component_index);
            ctx.clear_current_task();
            #postprocess_alloc_open
            let maybe_error = {
                #rt_guard
                <#bridge_type as cu29::cubridge::CuBridge>::postprocess(bridge, &ctx)
            };
            #postprocess_alloc_close
            if let Err(error) = maybe_error {
                let decision = monitor.process_error(
                    cu29::monitoring::ComponentId::new(#component_index),
                    CuComponentState::Postprocess,
                    &error,
                );
                match decision {
                    Decision::Abort => {
                        debug!(ctx,
                            "Postprocess: ABORT decision from monitoring. Component '{}' errored out during postprocess. Continuing with the completed CopperList.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Ignore => {
                        debug!(ctx,
                            "Postprocess: IGNORE decision from monitoring. Component '{}' errored out during postprocess. The runtime will continue.",
                            #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#component_index))
                        );
                    }
                    Decision::Shutdown => {
                        debug!(ctx,
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

/// Mission-module name of the `AnytimePolicy` ZST emitted for one anytime node.
fn anytime_policy_ident(task_id: &str) -> Ident {
    format_ident!("__CuAnytimePolicy_{}", config_id_to_struct_member(task_id))
}

/// Name of the hoisted `Option<AnytimeJob>` local for one anytime node — the
/// only value crossing its base/refine steps (is the job still live).
fn anytime_job_ident(task_id: &str) -> Ident {
    format_ident!("__cu_anytime_job_{}", config_id_to_struct_member(task_id))
}

fn anytime_ms_to_nanos(ms: f64) -> u64 {
    (ms * 1_000_000.0).round() as u64
}

/// The `__cu_any_now` binding shared by the base and refine emitters: one clock
/// read per base call and per refine quantum. Without a time knob every timed
/// check const-folds away and `now` only feeds the debug-only elapsed, so the
/// read is skipped entirely (`CuTime` subtraction saturates, so the zero
/// stand-in is safe).
fn anytime_now_binding_tokens(anytime: &AnytimeConfig) -> proc_macro2::TokenStream {
    if anytime.time_budget_ms.is_some() || anytime.max_age_ms.is_some() {
        quote! { let __cu_any_now = clock.now(); }
    } else {
        quote! { let __cu_any_now = cu29::clock::CuTime::default(); }
    }
}

fn anytime_option_duration_tokens(ms: Option<f64>) -> proc_macro2::TokenStream {
    match ms {
        Some(ms) => {
            let nanos = anytime_ms_to_nanos(ms);
            quote! { Some(cu29::clock::CuDuration(#nanos)) }
        }
        None => quote! { None },
    }
}

/// One `AnytimePolicy` ZST per anytime node, its RON policy baked in as
/// consts and overrides so every unset knob const-folds away.
///
/// A quality knob (`quality_target`/`quality_floor`/`max_stall`) pins the
/// implementation to the shared `Quality` scale, which is the compile-time
/// gate rejecting those knobs on `Quality = ()` tasks (the job's
/// `AnytimePolicy<T::Quality>` bound fails). Without quality knobs the
/// implementation is generic over any comparable quality.
fn build_anytime_policy_defs(task_specs: &CuTaskSpecSet) -> Vec<proc_macro2::TokenStream> {
    task_specs
        .anytime_configs
        .iter()
        .enumerate()
        .filter_map(|(index, anytime)| {
            let anytime = anytime.as_ref()?;
            let policy_ident = anytime_policy_ident(&task_specs.ids[index]);
            let time_budget = anytime_option_duration_tokens(anytime.time_budget_ms);
            let max_age = anytime_option_duration_tokens(anytime.max_age_ms);
            let max_stall = match anytime.max_stall {
                Some(stall) => quote! { Some(#stall) },
                None => quote! { None },
            };
            // Only the background runner reads MAX_REFINES: a foreground node
            // carries the count as the number of refine steps in its plan.
            let max_refines = match anytime.max_refines {
                Some(refines) => quote! { Some(#refines) },
                None => quote! { None },
            };
            let consts = quote! {
                const TIME_BUDGET: Option<cu29::clock::CuDuration> = #time_budget;
                const MAX_AGE: Option<cu29::clock::CuDuration> = #max_age;
                const MAX_STALL: Option<u32> = #max_stall;
                const MAX_REFINES: Option<u32> = #max_refines;
            };
            let has_quality_knob = anytime.quality_target.is_some()
                || anytime.quality_floor.is_some()
                || anytime.max_stall.is_some();
            let policy_impl = if has_quality_knob {
                let target_met = anytime.quality_target.map(|target| {
                    quote! {
                        #[inline(always)]
                        fn target_met(q: cu29::cutask_anytime::Quality) -> bool {
                            q >= cu29::cutask_anytime::quality_from_f32(#target)
                        }
                    }
                });
                let below_floor = anytime.quality_floor.map(|floor| {
                    quote! {
                        #[inline(always)]
                        fn below_floor(q: cu29::cutask_anytime::Quality) -> bool {
                            // NaN fails closed: an unordered quality counts as below the floor.
                            q.partial_cmp(&cu29::cutask_anytime::quality_from_f32(#floor))
                                .is_none_or(core::cmp::Ordering::is_lt)
                        }
                    }
                });
                quote! {
                    impl cu29::cutask_anytime::AnytimePolicy<cu29::cutask_anytime::Quality>
                        for #policy_ident
                    {
                        #consts
                        #target_met
                        #below_floor
                    }
                }
            } else {
                quote! {
                    impl<Q: Copy + PartialOrd> cu29::cutask_anytime::AnytimePolicy<Q>
                        for #policy_ident
                    {
                        #consts
                    }
                }
            };
            Some(quote! {
                #[allow(non_camel_case_types)]
                pub struct #policy_ident;
                #policy_impl
            })
        })
        .collect()
}

/// The hoisted per-node job local, declared above the plan splice: step bodies
/// can be wrapped in closures, so the local cannot live inside a step. The
/// `Option` answers the single genuinely dynamic cross-step question — is the
/// job still live; nothing else crosses steps.
fn anytime_job_local_tokens(task_specs: &CuTaskSpecSet, index: usize) -> proc_macro2::TokenStream {
    let task_type = &task_specs.task_types[index];
    let policy_ident = anytime_policy_ident(&task_specs.ids[index]);
    let job_ident = anytime_job_ident(&task_specs.ids[index]);
    quote! {
        #[allow(non_snake_case, unused_mut)]
        let mut #job_ident: Option<
            cu29::cutask_anytime::AnytimeJob<
                <#task_type as cu29::cutask_anytime::CuAnytimeTask>::Quality,
                #policy_ident,
            >,
        > = None;
    }
}

fn build_anytime_job_locals(task_specs: &CuTaskSpecSet) -> Vec<proc_macro2::TokenStream> {
    task_specs
        .anytime_configs
        .iter()
        .enumerate()
        // A background anytime node keeps its single `Whole` step and runs its
        // job inside the runner, so it has no plan-level job to carry.
        .filter(|(index, anytime)| anytime.is_some() && !task_specs.background_flags[*index])
        .map(|(index, _)| anytime_job_local_tokens(task_specs, index))
        .collect()
}

/// The output-slot cast trait/fn pair shared by the source, regular, and
/// anytime step emitters: `kind_label`/`fn_prefix` keep each emitter's
/// diagnostic idents, `task_trait` is the trait whose `Output` the slot must
/// match. Each emitted block is its own scope, so the fixed names never
/// collide.
fn output_slot_cast_tokens(
    task_hint: &str,
    kind_label: &str,
    fn_prefix: &str,
    task_trait: &proc_macro2::TokenStream,
) -> (proc_macro2::TokenStream, Ident) {
    let trait_ident = format_ident!(
        "__CuOutputSlotMustMatchTaskOutput__{}_{}__Add_dst___nc___connections_for_unused_outputs",
        kind_label,
        task_hint
    );
    let fn_ident = format_ident!(
        "__cu_{}_output_slot_or_add_dst___nc___for_unused_outputs__task_{}",
        fn_prefix,
        task_hint
    );
    let defs = quote! {
        #[allow(non_camel_case_types)]
        trait #trait_ident<Expected> {
            fn __cu_cast_output_slot(slot: &mut Self) -> &mut Expected;
        }
        impl<T> #trait_ident<T> for T {
            fn __cu_cast_output_slot(slot: &mut Self) -> &mut T {
                slot
            }
        }

        fn #fn_ident<'a, Task, Slot>(
            _task: &Task,
            slot: &'a mut Slot,
        ) -> &'a mut Task::Output<'static>
        where
            Task: #task_trait,
            Slot: #trait_ident<Task::Output<'static>>,
        {
            <Slot as #trait_ident<Task::Output<'static>>>::__cu_cast_output_slot(slot)
        }
    };
    (defs, fn_ident)
}

/// Anytime flavor of [`output_slot_cast_tokens`].
fn anytime_slot_cast_tokens(task_hint: &str) -> (proc_macro2::TokenStream, Ident) {
    output_slot_cast_tokens(
        task_hint,
        "AnytimeTask",
        "anytime",
        &quote! { cu29::cutask_anytime::CuAnytimeTask },
    )
}

/// The process-error monitoring match shared by every task step emitter
/// (source, sink, regular, and anytime base/refine blocks).
fn process_monitoring_action_tokens(
    tid: usize,
    mission_mod: &Ident,
    output_culist_index: &syn::Index,
    output_clear_payload: &proc_macro2::TokenStream,
    wrap_process_step: bool,
) -> proc_macro2::TokenStream {
    let abort_process_step = abort_process_step_tokens(wrap_process_step);
    quote! {
        debug!(ctx, "Component {}: Error during process: {}", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)), &error);
        let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#tid), CuComponentState::Process, &error);
        match decision {
            Decision::Abort => {
                debug!(ctx, "Process: ABORT decision from monitoring. Component '{}' errored out \
                        during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)), clid);
                #abort_process_step
            }
            Decision::Ignore => {
                debug!(ctx, "Process: IGNORE decision from monitoring. Component '{}' errored out \
                        during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)));
                let cumsg_output = __cu_output!(#output_culist_index);
                #output_clear_payload
            }
            Decision::Shutdown => {
                debug!(ctx, "Process: SHUTDOWN decision from monitoring. Component '{}' errored out \
                        during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#tid)));
                return Err(CuError::new_with_cause("Component errored out during process.", error));
            }
        }
    }
}

/// The sim-callback `doit` gate shared by the sink, regular, and anytime-base
/// step emitters. `input_binding` is the `let cumsg_input = <expr>;` line;
/// sources have none and pass `()` to the callback state.
fn process_sim_callback_tokens(
    sim_mode: bool,
    enum_name: &Ident,
    input_binding: Option<&proc_macro2::TokenStream>,
    output_culist_index: &syn::Index,
    monitoring_action: &proc_macro2::TokenStream,
) -> proc_macro2::TokenStream {
    if !sim_mode {
        return quote! { let doit = true; };
    }
    let (input_line, state_input) = match input_binding {
        Some(binding) => ((*binding).clone(), quote! { cumsg_input }),
        None => (quote! {}, quote! { () }),
    };
    quote! {
        let doit = {
            #input_line
            let cumsg_output = __cu_output!(#output_culist_index);
            let state = CuTaskCallbackState::Process(#state_input, cumsg_output);
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
}

/// Emits the `[base <node>]` block of a foreground anytime node: the standard
/// step wrapper (probe, keyframe freeze, sim callback, monitoring, alloc
/// scopes) around the DOA age check — emitted only when `max_age_ms` is
/// configured — and one `base()` call. On `Improved` the job lands in the
/// node's hoisted local; `Converged`/`Aborted` finish on the spot and the
/// local stays `None`, so every refine block no-ops through its liveness test.
///
/// An anytime node is single-output, so the monitoring clear is the one-slot
/// form. The job local needs no explicit kill on error: a job is only
/// (re)stored on an `Ok` status, so an erroring block leaves it `None`/taken
/// and later quanta no-op.
///
/// Returns the block plus the payload-drop logging tokens (same contract as
/// `generate_task_execution_tokens`).
fn generate_anytime_base_block(
    step: &CuExecutionStep,
    task_index: usize,
    task_specs: &CuTaskSpecSet,
    ctx: &StepGenerationContext<'_>,
    task_instance: &proc_macro2::TokenStream,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let tid = task_index;
    let anytime = task_specs.anytime_configs[tid]
        .as_ref()
        .expect("anytime base block emitted for a task without an anytime policy");
    let now_binding = anytime_now_binding_tokens(anytime);
    let task_id = &task_specs.ids[tid];
    let policy_ident = anytime_policy_ident(task_id);
    let job_ident = anytime_job_ident(task_id);
    let enum_name = Ident::new(&config_id_to_enum(task_id), Span::call_site());
    let task_hint = config_id_to_struct_member(task_id);
    let (slot_cast_defs, slot_cast_fn) = anytime_slot_cast_tokens(&task_hint);
    let rt_guard = rtsan_guard_tokens();
    let mission_mod = ctx.mission_mod;
    let freeze_task = keyframe_freeze_task_tokens(ctx.keyframe_logging_enabled, task_instance);

    let comment_str = format!(
        "DEBUG ->> {} ({:?}/{:?}) Id:{} I:{:?} O:{:?}",
        step.node.get_id(),
        step.task_type,
        step.phase,
        step.node_id,
        step.input_msg_indices_types,
        step.output_msg_pack
    );
    let comment_tokens = quote! {{
        let _ = stringify!(#comment_str);
    }};

    let output_pack = step
        .output_msg_pack
        .as_ref()
        .expect("Anytime task should have an output message pack.");
    let output_culist_index = int2sliceindex(output_pack.culist_index);
    let monitoring_action = process_monitoring_action_tokens(
        tid,
        mission_mod,
        &output_culist_index,
        &quote! { cumsg_output.clear_payload(); },
        ctx.wrap_process_step,
    );

    let GeneratedTaskInput {
        setup: task_input_setup,
        expr: task_input_expr,
    } = generate_task_input_binding(
        step,
        ctx.mission_name,
        ctx.output_pack_sizes,
        ctx.task_input_layouts,
    );

    // The sim callback fires once per node, here in the base block, with the
    // standard Process state. On ExecutedBySim the job local simply stays
    // `None` — no refine-side sim machinery exists or is needed.
    let call_sim_callback = process_sim_callback_tokens(
        ctx.sim_mode,
        &enum_name,
        Some(&quote! { let cumsg_input = #task_input_expr; }),
        &output_culist_index,
        &monitoring_action,
    );

    // Anchor extraction and the DOA skip exist in the text only when max_age
    // is configured; otherwise the anchor degenerates to the budget anchor.
    let base_call = quote! {
        match #task_instance.base(&ctx, cumsg_input, cumsg_output) {
            Ok(cu29::cutask_anytime::AnytimeStatus::Improved(q)) => {
                #job_ident = Some(cu29::cutask_anytime::AnytimeJob::new(__cu_any_now, __cu_any_anchor, q));
                Ok(())
            }
            Ok(cu29::cutask_anytime::AnytimeStatus::Converged(q)) => {
                let __cu_any_job: cu29::cutask_anytime::AnytimeJob<_, #policy_ident> =
                    cu29::cutask_anytime::AnytimeJob::new(__cu_any_now, __cu_any_anchor, q);
                // The job's clock read is reused: `finish` spends `now` only on
                // the debug-only elapsed, so a terminal base pays no second read.
                let __cu_any_outcome = __cu_any_job.finish(
                    __cu_any_now,
                    cu29::cutask_anytime::AnytimeStopCause::Converged,
                    0u32,
                    cumsg_output,
                );
                debug!(ctx, "Anytime task {}: {} after {} refinement(s).", #task_id, __cu_any_outcome.stop.label(), __cu_any_outcome.iterations);
                Ok(())
            }
            Ok(cu29::cutask_anytime::AnytimeStatus::Aborted) => {
                let __cu_any_outcome = cu29::cutask_anytime::abort_at_base(__cu_any_now, __cu_any_now, cumsg_output);
                debug!(ctx, "Anytime task {}: {} after {} refinement(s).", #task_id, __cu_any_outcome.stop.label(), __cu_any_outcome.iterations);
                Ok(())
            }
            Err(error) => Err(error),
        }
    };
    let base_dispatch = if let Some(max_age_ms) = anytime.max_age_ms {
        let max_age_nanos = anytime_ms_to_nanos(max_age_ms);
        quote! {
            let __cu_any_anchor = cu29::cutask_anytime::anchor_from_tov(cumsg_input.tov, __cu_any_now);
            if __cu_any_now >= __cu_any_anchor + cu29::clock::CuDuration(#max_age_nanos) {
                let __cu_any_outcome = cu29::cutask_anytime::skip_stale(cumsg_output);
                debug!(ctx, "Anytime task {}: input dead on arrival, job skipped.", #task_id);
                let _ = __cu_any_outcome;
                Ok(())
            } else {
                #base_call
            }
        }
    } else {
        quote! {
            let __cu_any_anchor = __cu_any_now;
            #base_call
        }
    };

    let alloc_open = alloc_scope_open_tokens();
    let alloc_close = alloc_scope_close_tokens(
        quote! { monitor },
        quote! { #tid },
        quote! { CuComponentState::Process },
    );

    let block = quote! {
        {
            #comment_tokens
            // One snapshot per copperlist, before the job: refine blocks must
            // not re-freeze.
            #freeze_task
            #task_input_setup
            #call_sim_callback
            let cumsg_input = #task_input_expr;
            let cumsg_output = __cu_output!(#output_culist_index);
            let maybe_error = if doit {
                execution_probe.record(cu29::monitoring::ExecutionMarker {
                    component_id: cu29::monitoring::ComponentId::new(#tid),
                    step: CuComponentState::Process,
                    culistid: Some(clid),
                });
                #slot_cast_defs
                if cumsg_output.metadata.process_time.start.is_none() {
                    cumsg_output.metadata.process_time.start = cu29::curuntime::perf_now(clock).into();
                }
                #alloc_open
                let result = {
                    let cumsg_output = #slot_cast_fn(&#task_instance, cumsg_output);
                    #rt_guard
                    ctx.set_current_task(#tid);
                    #now_binding
                    #base_dispatch
                };
                // A live job means a refine block runs and stamps `end`; only
                // a job that terminated at the base site stamps it here.
                if #job_ident.is_none() {
                    cumsg_output.metadata.process_time.end = cu29::curuntime::perf_now(clock).into();
                }
                #alloc_close
                result
            } else {
                Ok(())
            };
            if let Err(error) = maybe_error {
                #monitoring_action
            }
        }
    };

    let logging_tokens = if !task_specs.logging_enabled[tid] {
        quote! {
            let cumsg_output = &mut culist.msgs.0.#output_culist_index;
            cumsg_output.clear_payload();
        }
    } else {
        quote!()
    };

    (block, logging_tokens)
}

/// Emits the `k`-th of `total` refine blocks of a foreground anytime node:
/// liveness test on the hoisted local, then the configured between-quanta
/// `check()`, then one `refine()` quantum. `k` and last-ness are generation
/// time facts — the runtime carries no counter; the LAST block never puts the
/// job back and stops a still-live job with `MaxRefines` purely by position.
fn generate_anytime_refine_block(
    step: &CuExecutionStep,
    task_index: usize,
    task_specs: &CuTaskSpecSet,
    ctx: &StepGenerationContext<'_>,
    task_instance: &proc_macro2::TokenStream,
    k: u32,
    total: u32,
) -> proc_macro2::TokenStream {
    let tid = task_index;
    let task_id = &task_specs.ids[tid];
    let job_ident = anytime_job_ident(task_id);
    let task_hint = config_id_to_struct_member(task_id);
    let (slot_cast_defs, slot_cast_fn) = anytime_slot_cast_tokens(&task_hint);
    let rt_guard = rtsan_guard_tokens();

    let comment_str = format!(
        "DEBUG ->> {} ({:?}/{:?} {}/{}) Id:{} O:{:?}",
        step.node.get_id(),
        step.task_type,
        step.phase,
        k,
        total,
        step.node_id,
        step.output_msg_pack
    );
    let comment_tokens = quote! {{
        let _ = stringify!(#comment_str);
    }};

    let output_pack = step
        .output_msg_pack
        .as_ref()
        .expect("Anytime refine step should carry the base step's output pack.");
    let output_culist_index = int2sliceindex(output_pack.culist_index);
    let monitoring_action = process_monitoring_action_tokens(
        tid,
        ctx.mission_mod,
        &output_culist_index,
        &quote! { cumsg_output.clear_payload(); },
        ctx.wrap_process_step,
    );

    // A live job at block k has run exactly k-1 quanta — every earlier block
    // either ran its quantum or killed the job — so these are plain literals.
    let iters_on_check_stop = k - 1;
    let iters_with_quantum = k;
    let finish_log = quote! {
        debug!(ctx, "Anytime task {}: {} after {} refinement(s).", #task_id, __cu_any_outcome.stop.label(), __cu_any_outcome.iterations);
    };
    let anytime = task_specs.anytime_configs[tid]
        .as_ref()
        .expect("anytime refine block emitted for a task without an anytime policy");
    let now_binding = anytime_now_binding_tokens(anytime);
    // Only the Improved tail differs between blocks: put the job back while
    // the plan has more quanta; the LAST block stops a still-live job with
    // MaxRefines BY POSITION — the plan has no more quanta for it.
    let improved_tail = if k < total {
        quote! {
            #job_ident = Some(__cu_any_job); // still live: put it back
            Ok(())
        }
    } else {
        quote! {
            let __cu_any_outcome = __cu_any_job.finish(__cu_any_now, cu29::cutask_anytime::AnytimeStopCause::MaxRefines, #iters_with_quantum, cumsg_output);
            #finish_log
            Ok(())
        }
    };
    let quantum = quote! {
        #now_binding
        if let Some(__cu_any_cause) = __cu_any_job.check(__cu_any_now) {
            let __cu_any_outcome = __cu_any_job.finish(__cu_any_now, __cu_any_cause, #iters_on_check_stop, cumsg_output);
            #finish_log
            Ok(())
        } else {
            match #task_instance.refine(&ctx, cumsg_output) {
                Ok(cu29::cutask_anytime::AnytimeStatus::Improved(q)) => {
                    __cu_any_job.record(q);
                    #improved_tail
                }
                Ok(cu29::cutask_anytime::AnytimeStatus::Converged(q)) => {
                    __cu_any_job.record(q);
                    let __cu_any_outcome = __cu_any_job.finish(__cu_any_now, cu29::cutask_anytime::AnytimeStopCause::Converged, #iters_with_quantum, cumsg_output);
                    #finish_log
                    Ok(())
                }
                Ok(cu29::cutask_anytime::AnytimeStatus::Aborted) => {
                    let __cu_any_outcome = __cu_any_job.finish(__cu_any_now, cu29::cutask_anytime::AnytimeStopCause::Aborted, #iters_with_quantum, cumsg_output);
                    #finish_log
                    Ok(())
                }
                Err(error) => Err(error),
            }
        }
    };

    let alloc_open = alloc_scope_open_tokens();
    let alloc_close = alloc_scope_close_tokens(
        quote! { monitor },
        quote! { #tid },
        quote! { CuComponentState::Process },
    );

    quote! {
        {
            #comment_tokens
            // Liveness test: an earlier block that finished (or errored) the
            // job left the local empty, so this quantum no-ops.
            if let Some(mut __cu_any_job) = #job_ident.take() {
                execution_probe.record(cu29::monitoring::ExecutionMarker {
                    component_id: cu29::monitoring::ComponentId::new(#tid),
                    step: CuComponentState::Process,
                    culistid: Some(clid),
                });
                let cumsg_output = __cu_output!(#output_culist_index);
                #slot_cast_defs
                #alloc_open
                let maybe_error = {
                    let cumsg_output = #slot_cast_fn(&#task_instance, cumsg_output);
                    #rt_guard
                    ctx.set_current_task(#tid);
                    #quantum
                };
                // A still-live job means a later block runs and stamps `end`;
                // only the job's terminal quantum pays the clock read.
                if #job_ident.is_none() {
                    cumsg_output.metadata.process_time.end = cu29::curuntime::perf_now(clock).into();
                }
                #alloc_close
                if let Err(error) = maybe_error {
                    // No explicit job kill needed: the job was taken and is
                    // only put back on Ok(Improved), so later quanta no-op.
                    #monitoring_action
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
struct StepGenerationContext<'a> {
    output_pack_sizes: &'a [usize],
    task_input_layouts: &'a HashMap<String, TaskInputLayout>,
    mission_name: &'a str,
    sim_mode: bool,
    keyframe_logging_enabled: bool,
    mission_mod: &'a Ident,
    lifecycle_placement: ParallelLifecyclePlacement,
    wrap_process_step: bool,
}

impl<'a> StepGenerationContext<'a> {
    #[allow(clippy::too_many_arguments)]
    fn new(
        output_pack_sizes: &'a [usize],
        task_input_layouts: &'a HashMap<String, TaskInputLayout>,
        mission_name: &'a str,
        sim_mode: bool,
        keyframe_logging_enabled: bool,
        mission_mod: &'a Ident,
        lifecycle_placement: ParallelLifecyclePlacement,
        wrap_process_step: bool,
    ) -> Self {
        Self {
            output_pack_sizes,
            task_input_layouts,
            mission_name,
            sim_mode,
            keyframe_logging_enabled,
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

fn keyframe_freeze_task_tokens(
    enabled: bool,
    task_instance: &proc_macro2::TokenStream,
) -> proc_macro2::TokenStream {
    if enabled {
        quote! { kf_manager.freeze_task(clid, &#task_instance)?; }
    } else {
        quote! {}
    }
}

fn keyframe_freeze_bridge_tokens(enabled: bool) -> proc_macro2::TokenStream {
    if enabled {
        quote! { kf_manager.freeze_any(clid, bridge)?; }
    } else {
        quote! {}
    }
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
    runtime_task_type: &Type,
    ctx: StepGenerationContext<'_>,
    task_tokens: TaskExecutionTokens,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    let StepGenerationContext {
        output_pack_sizes,
        task_input_layouts,
        mission_name,
        sim_mode,
        keyframe_logging_enabled,
        mission_mod,
        lifecycle_placement,
        wrap_process_step,
    } = ctx;
    let TaskExecutionTokens {
        setup: task_setup,
        instance: task_instance,
    } = task_tokens;
    let freeze_task = keyframe_freeze_task_tokens(keyframe_logging_enabled, &task_instance);
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
    let rt_guard = rtsan_guard_tokens();
    let run_in_sim_flag = task_specs.run_in_sim_flags[tid];
    let (parallel_task_preprocess, parallel_task_postprocess) = parallel_task_lifecycle_tokens(
        task_trait_for_specs(task_specs, tid),
        runtime_task_type,
        task_specs.stateless_flags[tid],
        tid,
        mission_mod,
        &task_instance,
        lifecycle_placement,
    );
    let lag_wait = if task_specs.background_result_lags[tid].is_some() && !sim_mode {
        quote! { #task_instance.wait_for_job()?; }
    } else {
        quote! {}
    };
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
            let monitoring_action = process_monitoring_action_tokens(
                tid,
                mission_mod,
                &output_culist_index,
                &output_clear_payload,
                wrap_process_step,
            );

            let call_sim_callback = process_sim_callback_tokens(
                sim_mode,
                &enum_name,
                None,
                &output_culist_index,
                &monitoring_action,
            );

            let logging_tokens = if !task_specs.logging_enabled[tid] {
                quote! {
                    let mut cumsg_output = &mut culist.msgs.0.#output_culist_index;
                    #output_clear_payload
                }
            } else {
                quote!()
            };
            let alloc_open = alloc_scope_open_tokens();
            let alloc_close = alloc_scope_close_tokens(
                quote! { monitor },
                quote! { #tid },
                quote! { CuComponentState::Process },
            );
            let (slot_cast_defs, source_slot_match_fn_ident) = output_slot_cast_tokens(
                &task_hint,
                "Task",
                "source",
                &quote! { cu29::cutask::CuSrcTask },
            );
            let source_process_tokens = quote! {
                #slot_cast_defs

                #output_start_time
                #alloc_open
                let result = {
                    let cumsg_output = #source_slot_match_fn_ident::<
                        _,
                        _,
                    >(&#task_instance, cumsg_output);
                    #rt_guard
                    ctx.set_current_task(#tid);
                    #lag_wait
                    #task_instance.process(&ctx, cumsg_output)
                };
                #output_end_time
                #alloc_close
                result
            };

            (
                wrap_process_step_tokens(
                    wrap_process_step,
                    quote! {
                        #task_setup
                        #parallel_task_preprocess
                        #comment_tokens
                        #freeze_task
                        #call_sim_callback
                        let cumsg_output = __cu_output!(#output_culist_index);
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
            let GeneratedTaskInput {
                setup: task_input_setup,
                expr: task_input_expr,
            } = generate_task_input_binding(
                step,
                mission_name,
                output_pack_sizes,
                task_input_layouts,
            );

            let monitoring_action = process_monitoring_action_tokens(
                tid,
                mission_mod,
                &output_culist_index,
                &output_clear_payload,
                wrap_process_step,
            );

            let call_sim_callback = process_sim_callback_tokens(
                sim_mode,
                &enum_name,
                Some(&quote! { let cumsg_input = #task_input_expr; }),
                &output_culist_index,
                &monitoring_action,
            );

            let alloc_open = alloc_scope_open_tokens();
            let alloc_close = alloc_scope_close_tokens(
                quote! { monitor },
                quote! { #tid },
                quote! { CuComponentState::Process },
            );
            (
                wrap_process_step_tokens(
                    wrap_process_step,
                    quote! {
                        #task_setup
                        #parallel_task_preprocess
                        #comment_tokens
                        #freeze_task
                        #task_input_setup
                        #call_sim_callback
                        let cumsg_input = #task_input_expr;
                        let cumsg_output = __cu_output!(#output_culist_index);
                        let maybe_error = if doit {
                            execution_probe.record(cu29::monitoring::ExecutionMarker {
                                component_id: cu29::monitoring::ComponentId::new(#tid),
                                step: CuComponentState::Process,
                                culistid: Some(clid),
                            });
                            #output_start_time
                            #alloc_open
                            let result = {
                                #rt_guard
                                ctx.set_current_task(#tid);
                                #task_instance.process(&ctx, cumsg_input)
                            };
                            #output_end_time
                            #alloc_close
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
            let GeneratedTaskInput {
                setup: task_input_setup,
                expr: task_input_expr,
            } = generate_task_input_binding(
                step,
                mission_name,
                output_pack_sizes,
                task_input_layouts,
            );

            let monitoring_action = process_monitoring_action_tokens(
                tid,
                mission_mod,
                &output_culist_index,
                &output_clear_payload,
                wrap_process_step,
            );

            let call_sim_callback = process_sim_callback_tokens(
                sim_mode,
                &enum_name,
                Some(&quote! { let cumsg_input = #task_input_expr; }),
                &output_culist_index,
                &monitoring_action,
            );

            let logging_tokens = if !task_specs.logging_enabled[tid] {
                quote! {
                    let mut cumsg_output = &mut culist.msgs.0.#output_culist_index;
                    #output_clear_payload
                }
            } else {
                quote!()
            };
            let alloc_open = alloc_scope_open_tokens();
            let alloc_close = alloc_scope_close_tokens(
                quote! { monitor },
                quote! { #tid },
                quote! { CuComponentState::Process },
            );
            let regular_trait = task_trait_for_specs(task_specs, tid);
            let (slot_cast_defs, regular_slot_match_fn_ident) =
                output_slot_cast_tokens(&task_hint, "Task", "task", &regular_trait);
            let live_process_completed = (sim_mode && cfg!(feature = "logstream")).then(|| quote! {
                let _ = sim_callback(SimStep::#enum_name(CuTaskCallbackState::ProcessCompleted(cumsg_output)));
            });
            let regular_process_tokens = quote! {
                #slot_cast_defs

                #output_start_time
                #alloc_open
                let result = {
                    let cumsg_output = #regular_slot_match_fn_ident::<
                        _,
                        _,
                    >(&#task_instance, cumsg_output);
                    #rt_guard
                    ctx.set_current_task(#tid);
                    #lag_wait
                    #task_instance.process(&ctx, cumsg_input, cumsg_output)
                };
                #output_end_time
                #live_process_completed
                #alloc_close
                result
            };

            (
                wrap_process_step_tokens(
                    wrap_process_step,
                    quote! {
                        #task_setup
                        #parallel_task_preprocess
                        #comment_tokens
                        #freeze_task
                        #task_input_setup
                        #call_sim_callback
                        let cumsg_input = #task_input_expr;
                        let cumsg_output = __cu_output!(#output_culist_index);
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
        task_input_layouts: _,
        mission_name: _,
        sim_mode,
        keyframe_logging_enabled,
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
        quote! { __cu_output!(#culist_index_ts) }
    } else {
        let port_index = syn::Index::from(port_index);
        quote! { __cu_output!(#culist_index_ts, #port_index) }
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
            keyframe_logging_enabled,
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
                            debug!(ctx, "Process: ABORT decision from monitoring. Component '{}' errored out during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)), clid);
                            #abort_process_step
                        }
                        Decision::Ignore => {
                            debug!(ctx, "Process: IGNORE decision from monitoring. Component '{}' errored out during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                            cumsg_output.clear_payload();
                            false
                        }
                        Decision::Shutdown => {
                            debug!(ctx, "Process: SHUTDOWN decision from monitoring. Component '{}' errored out during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
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
    let alloc_open = alloc_scope_open_tokens();
    let alloc_close = alloc_scope_close_tokens(
        quote! { monitor },
        quote! { #monitor_index },
        quote! { CuComponentState::Process },
    );
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
                    #alloc_open
                    let maybe_error = {
                        #rt_guard
                        ctx.set_current_component(#monitor_index);
                        ctx.clear_current_task();
                        bridge.receive(
                            &ctx,
                            &<#bridge_type as cu29::cubridge::CuBridge>::Rx::#const_ident,
                            cumsg_output,
                        )
                    };
                    cumsg_output.metadata.process_time.end = cu29::curuntime::perf_now(clock).into();
                    #alloc_close
                    if let Err(error) = maybe_error {
                        let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Process, &error);
                        match decision {
                            Decision::Abort => {
                                debug!(ctx, "Process: ABORT decision from monitoring. Component '{}' errored out during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)), clid);
                                #abort_process_step
                            }
                            Decision::Ignore => {
                                debug!(ctx, "Process: IGNORE decision from monitoring. Component '{}' errored out during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                cumsg_output.clear_payload();
                            }
                            Decision::Shutdown => {
                                debug!(ctx, "Process: SHUTDOWN decision from monitoring. Component '{}' errored out during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
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
        task_input_layouts: _,
        mission_name: _,
        sim_mode,
        keyframe_logging_enabled,
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
        quote! { __cu_input!(#input_index, #port_index) }
    } else {
        quote! { __cu_input!(#input_index) }
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
    let output_ref = quote! { __cu_output!(#output_index) };
    let bridge_type = runtime_bridge_type_for_spec(bridge_spec, sim_mode);
    let (parallel_bridge_preprocess, parallel_bridge_postprocess) =
        parallel_bridge_lifecycle_tokens(
            &bridge_type,
            bridge_spec
                .monitor_index
                .expect("Bridge missing monitor index for lifecycle"),
            mission_mod,
            lifecycle_placement,
            keyframe_logging_enabled,
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
                            debug!(ctx, "Process: ABORT decision from monitoring. Component '{}' errored out during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)), clid);
                            #abort_process_step
                        }
                        Decision::Ignore => {
                            debug!(ctx, "Process: IGNORE decision from monitoring. Component '{}' errored out during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                            false
                        }
                        Decision::Shutdown => {
                            debug!(ctx, "Process: SHUTDOWN decision from monitoring. Component '{}' errored out during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
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
    let alloc_open = alloc_scope_open_tokens();
    let alloc_close = alloc_scope_close_tokens(
        quote! { monitor },
        quote! { #monitor_index },
        quote! { CuComponentState::Process },
    );
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
                    #alloc_open
                    let maybe_error = if bridge_channel.should_send(cumsg_input.payload().is_some()) {
                        {
                            #rt_guard
                            ctx.set_current_component(#monitor_index);
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
                    #alloc_close
                    if let Err(error) = maybe_error {
                        let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Process, &error);
                        match decision {
                            Decision::Abort => {
                                debug!(ctx, "Process: ABORT decision from monitoring. Component '{}' errored out during process. Skipping the processing of CL {}.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)), clid);
                                #abort_process_step
                            }
                            Decision::Ignore => {
                                debug!(ctx, "Process: IGNORE decision from monitoring. Component '{}' errored out during process. The runtime will continue with a forced empty message.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                            }
                            Decision::Shutdown => {
                                debug!(ctx, "Process: SHUTDOWN decision from monitoring. Component '{}' errored out during process. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
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
                // Anytime refine steps emit no parallel stage (they run inside
                // the base step's stage), so they must not claim the task's
                // postprocess placement.
                if step.phase == CuStepPhase::AnytimeRefine {
                    return None;
                }
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

struct LaneExecutorTokens {
    worker_spawns: Vec<proc_macro2::TokenStream>,
    dispatcher_scheduling: proc_macro2::TokenStream,
    completion_ready: proc_macro2::TokenStream,
    copperlists_per_cycle: proc_macro2::Literal,
    max_in_flight: proc_macro2::Literal,
    workers: proc_macro2::Literal,
}

fn scheduling_policy_tokens(policy: SchedulingPolicy) -> proc_macro2::TokenStream {
    match policy {
        SchedulingPolicy::Fair => quote! { cu29::config::SchedulingPolicy::Fair },
        SchedulingPolicy::Nice(nice) => quote! { cu29::config::SchedulingPolicy::Nice(#nice) },
        SchedulingPolicy::Fifo { priority } => {
            quote! { cu29::config::SchedulingPolicy::Fifo { priority: #priority } }
        }
        SchedulingPolicy::RoundRobin { priority } => {
            quote! { cu29::config::SchedulingPolicy::RoundRobin { priority: #priority } }
        }
    }
}

fn plan_thread_spec_tokens(
    id: &str,
    cpu: Option<usize>,
    policy: SchedulingPolicy,
) -> proc_macro2::TokenStream {
    let affinity = match cpu {
        Some(cpu) => quote! { Some(vec![#cpu]) },
        None => quote! { None },
    };
    let policy = scheduling_policy_tokens(policy);
    quote! {
        cu29::config::ThreadPoolConfig {
            id: #id.to_string(),
            threads: 1,
            affinity: #affinity,
            policy: #policy,
            on_error: cu29::config::OnError::Strict,
        }
    }
}

fn build_lane_executor_tokens(
    lane_plan: &LanePlan,
    culist_plan: &CuExecutionLoop,
    stage_idents: &[Ident],
    mission_mod: &Ident,
    keyframe_logging_enabled: bool,
) -> CuResult<LaneExecutorTokens> {
    let compiled = cu29_runtime::planner::DistributedSchedule::compile(lane_plan, culist_plan)?;
    if compiled.stages != stage_idents.len() {
        return Err(CuError::from(
            "Worker executor stages do not match the generated execution plan",
        ));
    }

    let mut worker_spawns = Vec::with_capacity(lane_plan.workers.len());
    let copperlists_per_cycle = u64::from(lane_plan.copperlists_per_cycle);
    for (worker_index, worker) in lane_plan.workers.iter().enumerate() {
        let CuPlanPlacement::Thread { cpu, policy } = &worker.placement else {
            return Err(CuError::from(format!(
                "Worker executor requires thread placement for '{}'",
                worker.id
            )));
        };
        let worker_id = worker.id.as_str();
        let spec = plan_thread_spec_tokens(worker_id, *cpu, *policy);
        let worker_length = compiled.worker_lengths[worker_index];
        let mut blocks = Vec::new();

        for &index in &worker.occurrences {
            if compiled.base_of[index].is_some() {
                continue;
            }
            let occurrence = lane_plan.occurrences[index];
            let stage = compiled.stage_of_step[occurrence.step]
                .expect("executable occurrence must have a generated stage");
            let stage_ident = &stage_idents[stage];
            let copperlist_offset = u64::from(occurrence.copperlist);
            let waits: Vec<_> = compiled
                .dependencies
                .iter()
                .filter(|(_, to, _)| *to == index)
                .map(|(from, _, lag)| {
                    let from_worker = compiled.owner[*from];
                    let from_length = compiled.worker_lengths[from_worker];
                    let from_progress = compiled.progress[*from];
                    let lag = u64::from(*lag);
                    quote! {
                        if cycle >= #lag {
                            let required_progress =
                                (cycle - #lag) * #from_length + #from_progress;
                            if !lanes.wait_progress(#from_worker, required_progress) {
                                break '__cu_worker;
                            }
                        }
                    }
                })
                .collect();
            let dependent_workers: BTreeSet<usize> = compiled
                .dependencies
                .iter()
                .filter(|(from, _, _)| *from == index)
                .map(|(_, to, _)| compiled.owner[*to])
                .collect();
            let progress = compiled.progress[index];
            let terminal = compiled.finish_counts[index] != 0;
            let publish = (!dependent_workers.is_empty() || terminal).then(|| {
                let dependent_workers = dependent_workers.iter();
                quote! {
                    lanes.publish_progress(
                        #worker_index,
                        cycle * #worker_length + #progress,
                        &[#(#dependent_workers),*],
                        #terminal,
                    );
                }
            });
            let keyframe_fields = keyframe_logging_enabled.then(|| {
                quote! {
                    let (keyframe_ptr, keyframe_len) = lanes.keyframe_capture(clid);
                }
            });
            let keyframe_runtime_fields = keyframe_logging_enabled.then(|| {
                quote! { keyframe_ptr, keyframe_len, }
            });

            blocks.push(quote! {
                {
                    let clid = first_clid + cycle * #copperlists_per_cycle + #copperlist_offset;
                    let Some(culist_ptr) = lanes.wait_admitted(clid) else {
                        break '__cu_worker;
                    };
                    #(#waits)*
                    if lanes.is_shut_down() {
                        break '__cu_worker;
                    }
                    let outcome = if lanes.is_aborted(clid) {
                        Ok(cu29::curuntime::ProcessStepOutcome::Continue)
                    } else {
                        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            let execution_probe = unsafe { execution_probe_ptr.as_ref() };
                            let monitor = unsafe { monitor_ptr.as_ref() };
                            #keyframe_fields
                            let mut step_rt = #mission_mod::ParallelProcessStepRuntime {
                                clock: &clock,
                                execution_probe,
                                monitor,
                                task_ptrs: &task_ptrs,
                                bridge_ptrs: &bridge_ptrs,
                                #keyframe_runtime_fields
                                culist: culist_ptr.cast::<CuList>(),
                                clid,
                                ctx: cu29::context::CuContext::from_runtime_metadata(
                                    clock.clone(), clid, instance_id, subsystem_code,
                                    #mission_mod::TASK_IDS,
                                ),
                            };
                            #stage_ident(&mut step_rt)
                        })) {
                            Ok(outcome) => outcome,
                            Err(payload) => Err(CuError::from(format!(
                                "Panic while processing CopperList #{} on worker '{}': {}",
                                clid,
                                #worker_id,
                                cu29::monitoring::panic_payload_to_string(payload.as_ref()),
                            ))),
                        }
                    };
                    match outcome {
                        Ok(cu29::curuntime::ProcessStepOutcome::Continue) => {}
                        Ok(cu29::curuntime::ProcessStepOutcome::AbortCopperList) => {
                            lanes.abort(clid);
                        }
                        Err(error) => {
                            lanes.publish_error(clid, error);
                            break '__cu_worker;
                        }
                    }
                    #publish
                }
            });
        }

        worker_spawns.push(quote! {
            {
                let lanes = std::sync::Arc::clone(&lanes);
                let clock = clock.clone();
                let execution_probe_ptr = execution_probe_ptr;
                let monitor_ptr = monitor_ptr;
                let task_ptrs = task_ptrs;
                let bridge_ptrs = bridge_ptrs;
                lane_handles.push(scope.spawn(move || {
                    lanes.register_worker(#worker_index);
                    let first_clid = lanes.first_clid();
                    let spec = #spec;
                    if let Err(error) = cu29::thread_pool::apply_current_thread_scheduling(&spec, 0) {
                        lanes.publish_error(
                            first_clid,
                            CuError::new_with_cause(
                                &format!("Worker '{}' could not apply its placement", #worker_id),
                                error,
                            ),
                        );
                        return;
                    }
                    let mut cycle = 0u64;
                    '__cu_worker: loop {
                        #(#blocks)*
                        cycle += 1;
                    }
                }));
            }
        });
    }

    let completion_arms = compiled
        .completion
        .iter()
        .enumerate()
        .map(|(offset, entries)| {
            let checks = entries.iter().map(|(worker, progress)| {
                let worker_length = compiled.worker_lengths[*worker];
                quote! {
                    && lanes.worker_reached(#worker, cycle * #worker_length + #progress)
                }
            });
            quote! { #offset => true #(#checks)*, }
        });
    let completion_ready = quote! {
        {
            let cycle = lanes.cycle_of(next_commit_clid);
            match ((next_commit_clid - lanes.first_clid()) % #copperlists_per_cycle) as usize {
                #(#completion_arms)*
                _ => unreachable!("CopperList offset outside exact schedule cycle"),
            }
        }
    };
    let dispatcher_scheduling = match &lane_plan.dispatcher {
        Some(CuPlanThread { cpu, policy }) => {
            let spec = plan_thread_spec_tokens("dispatcher", *cpu, *policy);
            quote! {
                {
                    let spec = #spec;
                    cu29::thread_pool::apply_current_thread_scheduling(&spec, 0)?;
                }
            }
        }
        None => quote! {},
    };

    Ok(LaneExecutorTokens {
        worker_spawns,
        dispatcher_scheduling,
        completion_ready,
        copperlists_per_cycle: proc_macro2::Literal::u64_unsuffixed(copperlists_per_cycle),
        max_in_flight: proc_macro2::Literal::usize_unsuffixed(lane_plan.max_in_flight as usize),
        workers: proc_macro2::Literal::usize_unsuffixed(lane_plan.workers.len()),
    })
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

fn runtime_task_type_for_index(
    task_specs: &CuTaskSpecSet,
    graph: &CuGraph,
    index: usize,
    sim_mode: bool,
) -> Type {
    let task_id = &task_specs.ids[index];
    let declared_task_type = &task_specs.sim_task_types[index];
    let background = task_specs.background_flags[index];
    let run_in_sim = task_specs.run_in_sim_flags[index];
    let output_type = &task_specs.output_types[index];

    match task_specs.cutypes[index] {
        CuTaskType::Source => {
            if sim_mode && !run_in_sim {
                let msg_types = graph
                    .get_node_output_msg_types(task_id.as_str())
                    .unwrap_or_else(|| {
                        panic!(
                            "CuSrcTask {task_id} should have an outgoing connection with a valid output msg type"
                        )
                    });
                let sim_task_name = if msg_types.len() == 1 {
                    format!("CuSimSrcTask<{}>", msg_types[0])
                } else {
                    let messages = msg_types
                        .iter()
                        .map(|msg_type| format!("cu29::prelude::CuMsg<{msg_type}>"))
                        .collect::<Vec<_>>()
                        .join(", ");
                    format!("CuSimSrcTaskPack<({messages})>")
                };
                parse_str(sim_task_name.as_str()).unwrap_or_else(|_| {
                    panic!("Could not build the placeholder for simulation: {sim_task_name}")
                })
            } else if background {
                if let Some(out_ty) = output_type {
                    parse_quote!(CuAsyncSrcTask<#declared_task_type, #out_ty>)
                } else {
                    panic!("{task_id}: If a source is background, it has to have an output");
                }
            } else {
                declared_task_type.clone()
            }
        }
        CuTaskType::Regular => {
            if background {
                if let Some(out_ty) = output_type {
                    let inner = &task_specs.async_inner_task_types[index];
                    parse_quote!(CuAsyncTask<#inner, #out_ty>)
                } else {
                    panic!("{task_id}: If a task is background, it has to have an output");
                }
            } else {
                // run_in_sim has no effect for regular tasks; they always run as themselves in sim.
                declared_task_type.clone()
            }
        }
        CuTaskType::Sink => {
            if background {
                panic!(
                    "CuSinkTask {task_id} cannot be a background task, it should be a regular task."
                );
            }

            if sim_mode && !run_in_sim {
                let msg_types = graph.get_node_input_msg_types(task_id.as_str()).unwrap_or_else(|| {
                    panic!(
                        "CuSinkTask {task_id} should have an incoming connection with a valid input msg type"
                    )
                });
                let msg_type = if msg_types.len() == 1 {
                    format!("({},)", msg_types[0])
                } else {
                    format!("({})", msg_types.join(", "))
                };
                let sim_task_name = format!("CuSimSinkTask<{msg_type}>");
                parse_str(sim_task_name.as_str()).unwrap_or_else(|_| {
                    panic!("Could not build the placeholder for simulation: {sim_task_name}")
                })
            } else {
                declared_task_type.clone()
            }
        }
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
mod tests;
