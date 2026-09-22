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
    let logstream_enabled = cfg!(feature = "logstream");
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
    let config_features = resolved_runtime_config.active_features.clone();
    let copper_config_content = resolved_runtime_config.bundled_local_config_content.clone();
    let copper_config = resolved_runtime_config.local_config;
    if copper_config.log_streaming.is_some() && !logstream_enabled {
        return return_error(
            "copperconfig.ron declares log_streaming but the cu29 'logstream' feature is disabled"
                .to_string(),
        );
    }
    if copper_config.log_streaming.is_some() && ignore_resources {
        return return_error(
            "log_streaming resource bindings cannot be used with ignore_resources".to_string(),
        );
    }
    let copperlist_count = copper_config
        .logging
        .as_ref()
        .and_then(|logging| logging.copperlist_count)
        .unwrap_or(DEFAULT_COPPERLIST_COUNT);
    let pipeline_selected = copper_config.planner_kind() == PlannerKind::Pipeline;
    if pipeline_selected && !(std && parallel_rt_enabled) {
        return return_error(
            "runtime planner kind Pipeline requires the cu29 'parallel-rt' feature".to_string(),
        );
    }
    let pipeline_max_in_flight = match copper_config.planner_config() {
        Some(planner) => match planner.max_in_flight(copperlist_count) {
            Ok(limit) => limit,
            Err(error) => return return_error(error.to_string()),
        },
        None => 1,
    };
    let pipeline_max_in_flight_tokens =
        proc_macro2::Literal::usize_unsuffixed(pipeline_max_in_flight);
    let local_keyframe_logging_enabled = copper_config
        .logging
        .as_ref()
        .is_none_or(|logging| logging.enable_keyframe_logging && logging.enable_task_logging);
    // A feature-gated streaming destination may request keyframes even when the local archive
    // does not. Compile the capture path in that case; runtime output requirements keep it idle
    // when no logstream destination is configured.
    let keyframe_logging_enabled = local_keyframe_logging_enabled || logstream_enabled;
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
    let lifecycle_sink_field: Field = parse_quote! {
        runtime_lifecycle_sink: Option<Box<::cu29::curuntime::RuntimeLifecycleSink>>
    };
    let logger_runtime_field: Field = parse_quote! {
        logger_runtime: cu29::prelude::LoggerRuntime
    };

    #[cfg(feature = "macro_debug")]
    eprintln!("[match struct anonymity]");
    match &mut application_struct.fields {
        Named(fields_named) => {
            fields_named.named.push(runtime_field);
            fields_named.named.push(lifecycle_sink_field);
            fields_named.named.push(logger_runtime_field);
        }
        Unnamed(fields_unnamed) => {
            fields_unnamed.unnamed.push(runtime_field);
            fields_unnamed.unnamed.push(lifecycle_sink_field);
            fields_unnamed.unnamed.push(logger_runtime_field);
        }
        Fields::Unit => {
            panic!(
                "This struct is a unit struct, it should have named or unnamed fields. use struct Something {{}} and not struct Something;"
            )
        }
    };

    let all_missions = sorted_mission_graphs(&copper_config);
    let constant_modules = match build_constant_modules(&copper_config.constants) {
        Ok(modules) => modules,
        Err(error) => return return_error(error.to_string()),
    };
    let constant_fingerprints = match copper_config
        .constants
        .iter()
        .map(|constant| {
            constant.semantic_fingerprint().map(|fingerprint| {
                (
                    constant.module_path().to_string(),
                    constant.id().to_string(),
                    fingerprint,
                )
            })
        })
        .collect::<Result<Vec<_>, _>>()
    {
        Ok(fingerprints) => fingerprints,
        Err(error) => return return_error(error),
    };
    let task_input_layouts = match collect_task_input_layouts(&all_missions) {
        Ok(layouts) => layouts,
        Err(e) => return return_error(e.to_string()),
    };
    let mut all_missions_tokens = Vec::<proc_macro2::TokenStream>::new();
    for (mission, graph) in &all_missions {
        let git_commit_tokens = git_commit_tokens.clone();
        let git_dirty_tokens = git_dirty_tokens.clone();
        let mission_mod = parse_str::<Ident>(mission.as_str())
            .expect("Could not make an identifier of the mission name");
        let mission_constant_contents = constant_modules.child_contents(&mission_mod);

        #[cfg(feature = "macro_debug")]
        eprintln!("[extract tasks ids & types]");
        let task_specs = match CuTaskSpecSet::from_graph(graph) {
            Ok(specs) => specs,
            Err(e) => return return_error(e.to_string()),
        };

        let culist_channel_usage = collect_bridge_channel_usage(graph);
        let mut culist_bridge_specs =
            build_bridge_specs(&copper_config, graph, &culist_channel_usage);
        let (culist_plan, culist_exec_entities, culist_plan_to_original, culist_schedule) =
            match build_execution_plan_with_schedule(
                &copper_config,
                graph,
                mission,
                &mut culist_bridge_specs,
            ) {
                Ok(plan) => plan,
                Err(e) => {
                    return return_error(format!(
                        "Could not compute copperlist plan for mission '{mission}': {e}"
                    ));
                }
            };
        let lane_plan = culist_schedule.lanes.as_ref();
        let distributed_selected = lane_plan.is_some() && !sim_mode;
        if distributed_selected && !(std && parallel_rt_enabled) {
            return return_error(format!(
                "Mission '{mission}': this plan requires the cu29 'parallel-rt' feature"
            ));
        }
        let mut task_specs = task_specs;
        for entry in &culist_schedule.background {
            let Some(index) = task_specs.ids.iter().position(|id| *id == entry.task) else {
                return return_error(format!(
                    "Mission '{mission}': background plan entry names unknown task '{}'",
                    entry.task
                ));
            };
            if let CuPlanBackgroundResult::Lag { lag } = entry.result {
                if lag != 1 || entry.max_running != 1 {
                    return return_error(format!(
                        "Mission '{mission}': background task '{}' asks for result lag {lag} with max_running {}; this runtime implements lag 1 with max_running 1.",
                        entry.task, entry.max_running
                    ));
                }
                task_specs.background_result_lags[index] = Some(lag);
            }
        }
        let task_specs = task_specs;
        let schedule_max_in_flight = lane_plan
            .map(|plan| plan.max_in_flight as usize)
            .unwrap_or(1);
        let schedule_max_in_flight_tokens =
            proc_macro2::Literal::usize_unsuffixed(schedule_max_in_flight);

        // Single-input/single-output arity is validated at configuration time
        // (config.rs validate_anytime_graph), before the plan is built.

        // Per-node refine totals and each refine step's 1-based ordinal: the
        // emitter bakes `k` and last-ness in as literals while walking the
        // plan — the runtime carries no counter.
        // One walk: the running counter's final value per node is that node's total.
        let (anytime_refine_ordinals, anytime_refine_totals): (
            Vec<Option<u32>>,
            HashMap<NodeId, u32>,
        ) = {
            let mut running: HashMap<NodeId, u32> = HashMap::new();
            let ordinals = culist_plan
                .steps
                .iter()
                .map(|unit| match unit {
                    CuExecutionUnit::Step(step) if step.phase == CuStepPhase::AnytimeRefine => {
                        let ordinal = running.entry(step.node_id).or_insert(0u32);
                        *ordinal += 1;
                        Some(*ordinal)
                    }
                    _ => None,
                })
                .collect();
            (ordinals, running)
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
            CuListSupportOptions {
                cuconfig: &copper_config,
                distributed: distributed_selected,
            },
            Some(mission.as_str()),
            &culist_plan,
            &culist_call_order,
            &node_output_positions,
            &task_names,
            &culist_bridge_specs,
        );

        let (
            resources_module,
            resources_instanciator_fn,
            task_resource_mappings,
            bridge_resource_mappings,
            logstream_resource_specs,
        ) = if ignore_resources {
            let bundle_specs: Vec<BundleSpec> = Vec::new();
            let resource_specs: Vec<ResourceKeySpec> = Vec::new();
            let (resources_module, resources_instanciator_fn) =
                match build_resources_module(&bundle_specs) {
                    Ok(tokens) => tokens,
                    Err(e) => return return_error(e.to_string()),
                };
            let task_resource_mappings =
                match build_task_resource_mappings(&resource_specs, &task_specs, sim_mode) {
                    Ok(tokens) => tokens,
                    Err(e) => return return_error(e.to_string()),
                };
            let bridge_resource_mappings =
                build_bridge_resource_mappings(&resource_specs, &culist_bridge_specs, sim_mode);
            (
                resources_module,
                resources_instanciator_fn,
                task_resource_mappings,
                bridge_resource_mappings,
                Vec::new(),
            )
        } else {
            let bundle_specs = match build_bundle_specs(&copper_config, mission.as_str()) {
                Ok(specs) => specs,
                Err(e) => return return_error(e.to_string()),
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
                match build_task_resource_mappings(&resource_specs, &task_specs, sim_mode) {
                    Ok(tokens) => tokens,
                    Err(e) => return return_error(e.to_string()),
                };
            let bridge_resource_mappings =
                build_bridge_resource_mappings(&resource_specs, &culist_bridge_specs, sim_mode);
            let logstream_resource_specs = if sim_mode {
                Vec::new()
            } else {
                match build_logstream_resource_specs(
                    &copper_config,
                    mission.as_str(),
                    &resource_specs,
                ) {
                    Ok(specs) => specs,
                    Err(e) => return return_error(e.to_string()),
                }
            };
            (
                resources_module,
                resources_instanciator_fn,
                task_resource_mappings,
                bridge_resource_mappings,
                logstream_resource_specs,
            )
        };

        let task_ids = task_specs.ids.clone();
        let autogenerated_output_warnings: Vec<proc_macro2::TokenStream> = task_specs
            .ids
            .iter()
            .zip(task_specs.cutypes.iter())
            .zip(task_specs.stateless_flags.iter())
            .zip(task_specs.autogenerated_output_flags.iter())
            .filter_map(|(((task_id, task_kind), stateless), autogenerated)| {
                if !*autogenerated {
                    return None;
                }
                let warn_ident = format_ident!(
                    "__CU_AUTOGEN_FLOATING_OUTPUT_WARNING__{}",
                    config_id_to_enum(task_id)
                );
                let kind_str = match task_kind {
                    CuTaskType::Source => "source",
                    CuTaskType::Regular if *stateless => "stateless_task",
                    CuTaskType::Regular => "task",
                    CuTaskType::Sink => return None,
                };
                let note = format!(
                    "Task '{task_id}' is declared as kind '{kind_str}' but has no declared outputs. Copper synthesized a hidden floating output slot from the task trait. Add a real consumer or `dst: \"__nc__\"` if you want this to stay explicit."
                );
                Some(quote! {
                    #[allow(dead_code)]
                    #[deprecated(note = #note)]
                    const #warn_ident: () = ();
                    const _: () = {
                        let _ = #warn_ident;
                    };
                })
            })
            .collect();
        let ids = build_monitored_ids(&task_ids, &mut culist_bridge_specs);
        let parallel_rt_stage_entries = if distributed_selected {
            match build_parallel_rt_stage_entries(
                &culist_plan,
                &culist_exec_entities,
                &task_specs,
                &culist_bridge_specs,
            ) {
                Ok(entries) => entries,
                Err(e) => return return_error(e.to_string()),
            }
        } else {
            Vec::new()
        };
        let parallel_rt_metadata_defs = if std {
            Some(quote! {
                pub const PARALLEL_RT_STAGES: &'static [cu29::parallel_rt::ParallelRtStageMetadata] =
                    &[#( #parallel_rt_stage_entries ),*];
                pub const PARALLEL_RT_METADATA: cu29::parallel_rt::ParallelRtMetadata =
                    cu29::parallel_rt::ParallelRtMetadata::new(
                        PARALLEL_RT_STAGES,
                        #schedule_max_in_flight_tokens,
                    );
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

        let runtime_task_types: Vec<Type> = (0..task_specs.ids.len())
            .map(|index| runtime_task_type_for_index(&task_specs, graph, index, sim_mode))
            .collect();

        let task_reflect_read_arms: Vec<proc_macro2::TokenStream> = task_specs
            .ids
            .iter()
            .enumerate()
            .map(|(index, task_id)| {
                let task_index = syn::Index::from(index);
                let task_id_lit = LitStr::new(task_id, Span::call_site());
                let value = distributed_selected.then(|| quote!(.value));
                quote! {
                    #task_id_lit => Some(&self.copper_runtime.tasks.#task_index #value as &dyn cu29::reflect::Reflect),
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
                let value = distributed_selected.then(|| quote!(.value));
                quote! {
                    #task_id_lit => Some(&mut self.copper_runtime.tasks.#task_index #value as &mut dyn cu29::reflect::Reflect),
                }
            })
            .collect();

        let task_debug_state_type_path_arms: Vec<proc_macro2::TokenStream> = task_specs
            .ids
            .iter()
            .zip(runtime_task_types.iter())
            .enumerate()
            .map(|(index, (task_id, task_type))| {
                let task_id_lit = LitStr::new(task_id, Span::call_site());
                let task_trait = task_trait_for_specs(&task_specs, index);
                quote! {
                    #task_id_lit => Some(<#task_type as #task_trait>::debug_state_type_path()),
                }
            })
            .collect();

        let task_debug_state_read_arms: Vec<proc_macro2::TokenStream> = task_specs
            .ids
            .iter()
            .zip(runtime_task_types.iter())
            .enumerate()
            .map(|(index, (task_id, task_type))| {
                let task_index = syn::Index::from(index);
                let task_id_lit = LitStr::new(task_id, Span::call_site());
                let task_trait = task_trait_for_specs(&task_specs, index);
                let value = distributed_selected.then(|| quote!(.value));
                quote! {
                    #task_id_lit => Some(
                        <#task_type as #task_trait>::with_debug_state(
                            &self.copper_runtime.tasks.#task_index #value,
                            f,
                        )
                    ),
                }
            })
            .collect();

        let task_debug_state_registration_calls: Vec<proc_macro2::TokenStream> = task_specs
            .ids
            .iter()
            .enumerate()
            .map(|(index, _)| {
                let task_type = &runtime_task_types[index];
                let task_trait = task_trait_for_specs(&task_specs, index);
                quote! {
                    <#task_type as #task_trait>::register_debug_state_types(registry);
                }
            })
            .collect();

        let mut reflect_registry_types: BTreeMap<String, Type> = BTreeMap::new();
        let mut add_reflect_type = |ty: Type| {
            let key = quote! { #ty }.to_string();
            reflect_registry_types.entry(key).or_insert(ty);
        };

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

        let bridge_component_types: Vec<Type> = if distributed_selected {
            bridge_runtime_types
                .iter()
                .map(|ty| parse_quote! { cu29::copperlist::ComponentRegion<#ty> })
                .collect()
        } else {
            bridge_runtime_types.clone()
        };
        let bridges_type_tokens: proc_macro2::TokenStream = if bridge_component_types.is_empty() {
            quote! { () }
        } else {
            let tuple: TypeTuple = parse_quote! { (#(#bridge_component_types),*,) };
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
            let bridge_bindings = bridge_binding_idents.iter().map(|binding| {
                if distributed_selected {
                    quote! { cu29::copperlist::ComponentRegion::new(#binding) }
                } else {
                    quote! { #binding }
                }
            });
            quote! {
                pub fn bridges_instanciator(config: &CuConfig, resources: &mut ResourceManager) -> CuResult<CuBridges> {
                    #(#bridge_init_statements)*
                    Ok((#(#bridge_bindings),*,))
                }
            }
        };

        let all_sim_tasks_types = runtime_task_types.clone();

        #[cfg(feature = "macro_debug")]
        eprintln!("[build task tuples]");

        let task_types: Vec<Type> = if distributed_selected {
            task_specs
                .task_types
                .iter()
                .map(|ty| parse_quote! { cu29::copperlist::ComponentRegion<#ty> })
                .collect()
        } else {
            task_specs.task_types.clone()
        };
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

        // Resolve each background task's thread pool name to its index in
        // `runtime.thread_pools` (matching the slot order the runtime owns the
        // built pools in). Non-background tasks get a placeholder index that is
        // never used.
        let thread_pool_indices: HashMap<&str, usize> = copper_config
            .runtime
            .as_ref()
            .map(|runtime| {
                runtime
                    .thread_pools
                    .iter()
                    .enumerate()
                    .map(|(index, pool)| (pool.id.as_str(), index))
                    .collect()
            })
            .unwrap_or_default();
        for (task_index, pool_name) in task_specs.background_pools.iter().enumerate() {
            if !task_specs.background_flags[task_index] {
                continue;
            }
            // "rt" is the default pool applied to every task running under the
            // parallel-rt engine; a task only deviates from it by setting
            // `background: true` and picking a different pool here. The "rt"
            // pool itself is dedicated to the parallel-rt stage workers, so it
            // is rejected as a per-task override.
            if pool_name == RT_POOL {
                return return_error(format!(
                    "Background task '{}' may not use the reserved '{RT_POOL}' thread pool; it is dedicated to the parallel-rt execution engine.",
                    task_specs.ids[task_index]
                ));
            }
            if !thread_pool_indices.contains_key(pool_name.as_str()) {
                return return_error(format!(
                    "Background task '{}' references undefined thread pool '{}'. Define it under runtime.thread_pools.",
                    task_specs.ids[task_index], pool_name
                ));
            }
        }
        let task_pool_indices: Vec<usize> = task_specs
            .background_pools
            .iter()
            .map(|pool_name| {
                thread_pool_indices
                    .get(pool_name.as_str())
                    .copied()
                    .unwrap_or(0)
            })
            .collect();

        let task_sim_instances_init_code = all_sim_tasks_types
            .iter()
            .enumerate()
            .map(|(index, ty)| {
                let additional_error_info = format!(
                    "Failed to get create instance for {}, instance index {}.",
                    task_specs.type_names[index], index
                );
                let mapping_ref = task_resource_mappings.refs[index].clone();
                let background = task_specs.background_flags[index]
                    && !(sim_mode
                        && task_specs.cutypes[index] == CuTaskType::Source
                        && !task_specs.run_in_sim_flags[index]);
                let inner_task_type = &task_specs.async_inner_task_types[index];
                match task_specs.cutypes[index] {
                    CuTaskType::Source => {
                        if background {
                            let pool_index = task_pool_indices[index];
                            let pool_name = task_specs.background_pools[index].clone();
                            quote! {
                                {
                                    let inner_resources = <<#inner_task_type as CuSrcTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    let threadpool = thread_pools
                                        .get(#pool_index)
                                        .and_then(|slot| slot.clone())
                                        .ok_or_else(|| CuError::from(format!(
                                            "Background task at index {} requested thread pool '{}' but it was not provided",
                                            #index, #pool_name,
                                        )))?;
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
                            let pool_index = task_pool_indices[index];
                            let pool_name = task_specs.background_pools[index].clone();
                            quote! {
                                {
                                    let inner_resources = <<#inner_task_type as CuTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    let threadpool = thread_pools
                                        .get(#pool_index)
                                        .and_then(|slot| slot.clone())
                                        .ok_or_else(|| CuError::from(format!(
                                            "Background task at index {} requested thread pool '{}' but it was not provided",
                                            #index, #pool_name,
                                        )))?;
                                    let resources = cu29::cuasynctask::CuAsyncTaskResources {
                                        inner: inner_resources,
                                        threadpool,
                                    };
                                    <#ty as CuTask>::new(all_instances_configs[#index], resources)
                                        .map_err(|e| e.add_cause(#additional_error_info))?
                                }
                            }
                        } else {
                            let regular_trait = task_trait_for_specs(&task_specs, index);
                            quote! {
                                {
                                    let resources = <<#ty as #regular_trait>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    <#ty as #regular_trait>::new(all_instances_configs[#index], resources)
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
                let inner_task_type = &task_specs.async_inner_task_types[index];
                match task_specs.cutypes[index] {
                    CuTaskType::Source => {
                        if *background {
                            let pool_index = task_pool_indices[index];
                            let pool_name = task_specs.background_pools[index].clone();
                            quote! {
                                {
                                    let inner_resources = <<#inner_task_type as CuSrcTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    let threadpool = thread_pools
                                        .get(#pool_index)
                                        .and_then(|slot| slot.clone())
                                        .ok_or_else(|| CuError::from(format!(
                                            "Background task at index {} requested thread pool '{}' but it was not provided",
                                            #index, #pool_name,
                                        )))?;
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
                            let pool_index = task_pool_indices[index];
                            let pool_name = task_specs.background_pools[index].clone();
                            quote! {
                                {
                                    let inner_resources = <<#inner_task_type as CuTask>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    let threadpool = thread_pools
                                        .get(#pool_index)
                                        .and_then(|slot| slot.clone())
                                        .ok_or_else(|| CuError::from(format!(
                                            "Background task at index {} requested thread pool '{}' but it was not provided",
                                            #index, #pool_name,
                                        )))?;
                                    let resources = cu29::cuasynctask::CuAsyncTaskResources {
                                        inner: inner_resources,
                                        threadpool,
                                    };
                                    <#task_type as CuTask>::new(all_instances_configs[#index], resources)
                                        .map_err(|e| e.add_cause(#additional_error_info))?
                                }
                            }
                        } else {
                            let regular_trait = task_trait_for_specs(&task_specs, index);
                            quote! {
                                {
                                    let resources = <<#task_type as #regular_trait>::Resources<'_> as ResourceBindings>::from_bindings(
                                        resources,
                                        #mapping_ref,
                                    ).map_err(|e| e.add_cause(#additional_error_info))?;
                                    <#task_type as #regular_trait>::new(all_instances_configs[#index], resources)
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
        let task_instances_init_code: Vec<_> = if distributed_selected {
            task_instances_init_code
                .into_iter()
                .map(|instance| quote! { cu29::copperlist::ComponentRegion::new(#instance) })
                .collect()
        } else {
            task_instances_init_code
        };

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
                                    debug!(ctx, "Start: ABORT decision from monitoring. Component '{}' errored out \
                                during start. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                    return Ok(());

                                }
                                Decision::Ignore => {
                                    debug!(ctx, "Start: IGNORE decision from monitoring. Component '{}' errored out \
                                during start. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                }
                                Decision::Shutdown => {
                                    debug!(ctx, "Start: SHUTDOWN decision from monitoring. Component '{}' errored out \
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


                        let alloc_open = alloc_scope_open_tokens();
                        let alloc_close = alloc_scope_close_tokens(
                            quote! { self.copper_runtime.monitor },
                            quote! { #index },
                            quote! { CuComponentState::Start },
                        );
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
                                #alloc_open
                                let __cu_step_result = task.start(&ctx);
                                #alloc_close
                                if let Err(error) = __cu_step_result {
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
                                            debug!(ctx, "Stop: ABORT decision from monitoring. Component '{}' errored out \
                                    during stop. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                            return Ok(());

                                        }
                                        Decision::Ignore => {
                                            debug!(ctx, "Stop: IGNORE decision from monitoring. Component '{}' errored out \
                                    during stop. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                        }
                                        Decision::Shutdown => {
                                            debug!(ctx, "Stop: SHUTDOWN decision from monitoring. Component '{}' errored out \
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
                        let alloc_open = alloc_scope_open_tokens();
                        let alloc_close = alloc_scope_close_tokens(
                            quote! { self.copper_runtime.monitor },
                            quote! { #index },
                            quote! { CuComponentState::Stop },
                        );
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
                                #alloc_open
                                let __cu_step_result = task.stop(&ctx);
                                #alloc_close
                                if let Err(error) = __cu_step_result {
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
                                    debug!(ctx, "Preprocess: ABORT decision from monitoring. Component '{}' errored out \
                                during preprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                    return Ok(());

                                }
                                Decision::Ignore => {
                                    debug!(ctx, "Preprocess: IGNORE decision from monitoring. Component '{}' errored out \
                                during preprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                }
                                Decision::Shutdown => {
                                    debug!(ctx, "Preprocess: SHUTDOWN decision from monitoring. Component '{}' errored out \
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
                        let alloc_open = alloc_scope_open_tokens();
                        let alloc_close = alloc_scope_close_tokens(
                            quote! { monitor },
                            quote! { #index },
                            quote! { CuComponentState::Preprocess },
                        );
                        quote! {
                            #call_sim_callback
                            if doit {
                                execution_probe.record(cu29::monitoring::ExecutionMarker {
                                    component_id: cu29::monitoring::ComponentId::new(#index),
                                    step: CuComponentState::Preprocess,
                                    culistid: None,
                                });
                                ctx.set_current_task(#index);
                                #alloc_open
                                let maybe_error = {
                                    #rt_guard
                                    tasks.#task_index.preprocess(&ctx)
                                };
                                #alloc_close
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
                                    debug!(ctx, "Postprocess: ABORT decision from monitoring. Component '{}' errored out \
                                during postprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                    return Ok(());

                                }
                                Decision::Ignore => {
                                    debug!(ctx, "Postprocess: IGNORE decision from monitoring. Component '{}' errored out \
                                during postprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#index)));
                                }
                                Decision::Shutdown => {
                                    debug!(ctx, "Postprocess: SHUTDOWN decision from monitoring. Component '{}' errored out \
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
                        let alloc_open = alloc_scope_open_tokens();
                        let alloc_close = alloc_scope_close_tokens(
                            quote! { monitor },
                            quote! { #index },
                            quote! { CuComponentState::Postprocess },
                        );
                        quote! {
                            #call_sim_callback
                            if doit {
                                execution_probe.record(cu29::monitoring::ExecutionMarker {
                                    component_id: cu29::monitoring::ComponentId::new(#index),
                                    step: CuComponentState::Postprocess,
                                    culistid: None,
                                });
                                ctx.set_current_task(#index);
                                #alloc_open
                                let maybe_error = {
                                    #rt_guard
                                    tasks.#task_index.postprocess(&ctx)
                                };
                                #alloc_close
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
                                    Decision::Abort => { debug!(ctx, "Start: ABORT decision from monitoring. Component '{}' errored out during start. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Ok(()); }
                                    Decision::Ignore => { debug!(ctx, "Start: IGNORE decision from monitoring. Component '{}' errored out during start. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); false }
                                    Decision::Shutdown => { debug!(ctx, "Start: SHUTDOWN decision from monitoring. Component '{}' errored out during start. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Err(CuError::new_with_cause("Component errored out during start.", error)); }
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
                    quote! { self.copper_runtime.monitor },
                    quote! { #monitor_index },
                    quote! { CuComponentState::Start },
                );
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
                        ctx.set_current_component(#monitor_index);
                        ctx.clear_current_task();
                        let bridge = &mut self.copper_runtime.bridges.#bridge_index;
                        #alloc_open
                        let __cu_step_result = bridge.start(&ctx);
                        #alloc_close
                        if let Err(error) = __cu_step_result {
                            let decision = self.copper_runtime.monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Start, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!(ctx, "Start: ABORT decision from monitoring. Component '{}' errored out during start. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                    return Ok(());
                                }
                                Decision::Ignore => {
                                    debug!(ctx, "Start: IGNORE decision from monitoring. Component '{}' errored out during start. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                }
                                Decision::Shutdown => {
                                    debug!(ctx, "Start: SHUTDOWN decision from monitoring. Component '{}' errored out during start. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
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
                                    Decision::Abort => { debug!(ctx, "Stop: ABORT decision from monitoring. Component '{}' errored out during stop. Aborting all the other stops.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Ok(()); }
                                    Decision::Ignore => { debug!(ctx, "Stop: IGNORE decision from monitoring. Component '{}' errored out during stop. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); false }
                                    Decision::Shutdown => { debug!(ctx, "Stop: SHUTDOWN decision from monitoring. Component '{}' errored out during stop. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Err(CuError::new_with_cause("Component errored out during stop.", error)); }
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
                    quote! { self.copper_runtime.monitor },
                    quote! { #monitor_index },
                    quote! { CuComponentState::Stop },
                );
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
                        ctx.set_current_component(#monitor_index);
                        ctx.clear_current_task();
                        let bridge = &mut self.copper_runtime.bridges.#bridge_index;
                        #alloc_open
                        let __cu_step_result = bridge.stop(&ctx);
                        #alloc_close
                        if let Err(error) = __cu_step_result {
                            let decision = self.copper_runtime.monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Stop, &error);
                            match decision {
                                Decision::Abort => {
                                    debug!(ctx, "Stop: ABORT decision from monitoring. Component '{}' errored out during stop. Aborting all the other stops.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                    return Ok(());
                                }
                                Decision::Ignore => {
                                    debug!(ctx, "Stop: IGNORE decision from monitoring. Component '{}' errored out during stop. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                }
                                Decision::Shutdown => {
                                    debug!(ctx, "Stop: SHUTDOWN decision from monitoring. Component '{}' errored out during stop. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
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
                                    Decision::Abort => { debug!(ctx, "Preprocess: ABORT decision from monitoring. Component '{}' errored out during preprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Ok(()); }
                                    Decision::Ignore => { debug!(ctx, "Preprocess: IGNORE decision from monitoring. Component '{}' errored out during preprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); false }
                                    Decision::Shutdown => { debug!(ctx, "Preprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during preprocess. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Err(CuError::new_with_cause("Component errored out during preprocess.", error)); }
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
                    quote! { CuComponentState::Preprocess },
                );
                quote! {
                    {
                        #call_sim
                        if doit {
                            ctx.set_current_component(#monitor_index);
                            ctx.clear_current_task();
                            let bridge = &mut __cu_bridges.#bridge_index;
                            execution_probe.record(cu29::monitoring::ExecutionMarker {
                                component_id: cu29::monitoring::ComponentId::new(#monitor_index),
                                step: CuComponentState::Preprocess,
                                culistid: None,
                            });
                            #alloc_open
                            let maybe_error = {
                                #rt_guard
                                bridge.preprocess(&ctx)
                            };
                            #alloc_close
                            if let Err(error) = maybe_error {
                                let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Preprocess, &error);
                                match decision {
                                    Decision::Abort => {
                                        debug!(ctx, "Preprocess: ABORT decision from monitoring. Component '{}' errored out during preprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                        return Ok(());
                                    }
                                    Decision::Ignore => {
                                        debug!(ctx, "Preprocess: IGNORE decision from monitoring. Component '{}' errored out during preprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                    }
                                    Decision::Shutdown => {
                                        debug!(ctx, "Preprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during preprocess. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
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
                let freeze_bridge = keyframe_freeze_bridge_tokens(keyframe_logging_enabled);
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
                                    Decision::Abort => { debug!(ctx, "Postprocess: ABORT decision from monitoring. Component '{}' errored out during postprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Ok(()); }
                                    Decision::Ignore => { debug!(ctx, "Postprocess: IGNORE decision from monitoring. Component '{}' errored out during postprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); false }
                                    Decision::Shutdown => { debug!(ctx, "Postprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during postprocess. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index))); return Err(CuError::new_with_cause("Component errored out during postprocess.", error)); }
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
                    quote! { CuComponentState::Postprocess },
                );
                quote! {
                    {
                        #call_sim
                        if doit {
                            ctx.set_current_component(#monitor_index);
                            ctx.clear_current_task();
                            let bridge = &mut __cu_bridges.#bridge_index;
                            #freeze_bridge
                            execution_probe.record(cu29::monitoring::ExecutionMarker {
                                component_id: cu29::monitoring::ComponentId::new(#monitor_index),
                                step: CuComponentState::Postprocess,
                                culistid: Some(clid),
                            });
                            #alloc_open
                            let maybe_error = {
                                #rt_guard
                                bridge.postprocess(&ctx)
                            };
                            #alloc_close
                            if let Err(error) = maybe_error {
                                let decision = monitor.process_error(cu29::monitoring::ComponentId::new(#monitor_index), CuComponentState::Postprocess, &error);
                                match decision {
                                    Decision::Abort => {
                                        debug!(ctx, "Postprocess: ABORT decision from monitoring. Component '{}' errored out during postprocess. Aborting all the other starts.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                        return Ok(());
                                    }
                                    Decision::Ignore => {
                                        debug!(ctx, "Postprocess: IGNORE decision from monitoring. Component '{}' errored out during postprocess. The runtime will continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
                                    }
                                    Decision::Shutdown => {
                                        debug!(ctx, "Postprocess: SHUTDOWN decision from monitoring. Component '{}' errored out during postprocess. The runtime cannot continue.", #mission_mod::monitor_component_label(cu29::monitoring::ComponentId::new(#monitor_index)));
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
        let parallel_rt_run_supported = std && parallel_rt_enabled && distributed_selected;

        let output_pack_sizes = collect_output_pack_sizes(&culist_plan);
        let runtime_plan_code_and_logging: Vec<(
            proc_macro2::TokenStream,
            proc_macro2::TokenStream,
        )> = culist_plan
            .steps
            .iter()
            .enumerate()
            .map(|(step_index, unit)| match unit {
                CuExecutionUnit::Step(step) => {
                    #[cfg(feature = "macro_debug")]
                    eprintln!(
                        "{} -> {} as {:?}/{:?}. task_id: {} Input={:?}, Output={:?}",
                        step.node.get_id(),
                        step.node.get_type(),
                        step.task_type,
                        step.phase,
                        step.node_id,
                        step.input_msg_indices_types,
                        step.output_msg_pack
                    );

                    match &culist_exec_entities[step.node_id as usize].kind {
                        ExecutionEntityKind::Task { task_index } => {
                            let step_ctx = StepGenerationContext::new(
                                &output_pack_sizes,
                                &task_input_layouts,
                                mission.as_str(),
                                sim_mode,
                                keyframe_logging_enabled,
                                &mission_mod,
                                ParallelLifecyclePlacement::default(),
                                false,
                            );
                            let task_instance = {
                                let node_index = int2sliceindex(*task_index as u32);
                                quote! { tasks.#node_index }
                            };
                            match step.phase {
                                CuStepPhase::Whole => generate_task_execution_tokens(
                                    step,
                                    *task_index,
                                    &task_specs,
                                    &runtime_task_types[*task_index],
                                    step_ctx,
                                    TaskExecutionTokens::new(quote! {}, task_instance),
                                ),
                                CuStepPhase::AnytimeBase => generate_anytime_base_block(
                                    step,
                                    *task_index,
                                    &task_specs,
                                    &step_ctx,
                                    &task_instance,
                                ),
                                CuStepPhase::AnytimeRefine => {
                                    let ordinal = anytime_refine_ordinals[step_index]
                                        .expect("refine step without a precomputed ordinal");
                                    let total = anytime_refine_totals[&step.node_id];
                                    (
                                        generate_anytime_refine_block(
                                            step,
                                            *task_index,
                                            &task_specs,
                                            &step_ctx,
                                            &task_instance,
                                            ordinal,
                                            total,
                                        ),
                                        quote! {},
                                    )
                                }
                            }
                        }
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
                                    &task_input_layouts,
                                    mission.as_str(),
                                    sim_mode,
                                    keyframe_logging_enabled,
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
                                    &task_input_layouts,
                                    mission.as_str(),
                                    sim_mode,
                                    keyframe_logging_enabled,
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
        // A sim runtime does not execute parallel-rt, but it must decode the component
        // frames in the execution-wave order used by the matching recording runtime.
        let restore_parallel_placements = (std && parallel_rt_enabled && distributed_selected)
            .then(|| build_parallel_lifecycle_placements(&culist_plan, &culist_exec_entities));
        let mut keyframe_restore_order = Vec::<ParallelLifecycleKey>::new();
        if let Some(placements) = restore_parallel_placements.as_ref() {
            for (step_index, unit) in culist_plan.steps.iter().enumerate() {
                let CuExecutionUnit::Step(step) = unit else {
                    panic!("Execution loops are not supported in runtime generation");
                };
                match &culist_exec_entities[step.node_id as usize].kind {
                    ExecutionEntityKind::Task { task_index } => {
                        if step.phase != CuStepPhase::AnytimeRefine
                            && !keyframe_restore_order
                                .contains(&ParallelLifecycleKey::Task(*task_index))
                        {
                            keyframe_restore_order.push(ParallelLifecycleKey::Task(*task_index));
                        }
                    }
                    ExecutionEntityKind::BridgeRx { bridge_index, .. }
                    | ExecutionEntityKind::BridgeTx { bridge_index, .. } => {
                        if placements[step_index].postprocess {
                            keyframe_restore_order
                                .push(ParallelLifecycleKey::Bridge(*bridge_index));
                        }
                    }
                }
            }
        } else {
            keyframe_restore_order.extend(
                keyframe_task_restore_order
                    .iter()
                    .copied()
                    .map(ParallelLifecycleKey::Task),
            );
            keyframe_restore_order
                .extend((0..culist_bridge_specs.len()).map(ParallelLifecycleKey::Bridge));
        }
        let keyframe_restore_code: Vec<proc_macro2::TokenStream> = keyframe_restore_order
            .iter()
            .map(|component| match component {
                ParallelLifecycleKey::Task(index) => {
                    let task_tuple_index = syn::Index::from(*index);
                    let skip_substituted = sim_mode
                        && task_specs.cutypes[*index] != CuTaskType::Regular
                        && !task_specs.run_in_sim_flags[*index];
                    if skip_substituted {
                        quote! {
                            let _ = frames.next_frame()?;
                        }
                    } else {
                        quote! {
                            let frame = frames.next_frame()?;
                            cu29::curuntime::thaw_keyframe_component(
                                &mut tasks.#task_tuple_index,
                                frame,
                            )?;
                        }
                    }
                }
                ParallelLifecycleKey::Bridge(index) => {
                    let bridge_tuple_index = syn::Index::from(*index);
                    if sim_mode && !culist_bridge_specs[*index].run_in_sim {
                        quote! {
                            let _ = frames.next_frame()?;
                        }
                    } else {
                        quote! {
                            let frame = frames.next_frame()?;
                            cu29::curuntime::thaw_keyframe_component(
                                &mut __cu_bridges.#bridge_tuple_index,
                                frame,
                            )?;
                        }
                    }
                }
            })
            .collect();
        let keyframe_preallocation_code: Vec<proc_macro2::TokenStream> = keyframe_restore_order
            .iter()
            .map(|component| match component {
                ParallelLifecycleKey::Task(index) => {
                    let task_tuple_index = syn::Index::from(*index);
                    quote! {
                        kf_manager.include_capture_capacity(&tasks.#task_tuple_index)?;
                    }
                }
                ParallelLifecycleKey::Bridge(index) => {
                    let bridge_tuple_index = syn::Index::from(*index);
                    quote! {
                        kf_manager.include_capture_capacity(&__cu_bridges.#bridge_tuple_index)?;
                    }
                }
            })
            .collect();
        let runtime_plan_parallel_code_and_logging: Option<
            Vec<(proc_macro2::TokenStream, proc_macro2::TokenStream)>,
        > = if parallel_rt_run_supported {
            Some(
                culist_plan
                    .steps
                    .iter()
                    .enumerate()
                    .filter_map(|(step_index, unit)| match unit {
                        CuExecutionUnit::Step(step) => match &culist_exec_entities
                            [step.node_id as usize]
                            .kind
                        {
                            ExecutionEntityKind::Task { task_index } => {
                                let task_index_ts = int2sliceindex(*task_index as u32);
                                // Pipeline keeps an anytime base and every
                                // refine quantum contiguously, so the whole job
                                // stays on the same lane. Refine steps therefore
                                // emit no lane of their own.
                                if step.phase == CuStepPhase::AnytimeRefine {
                                    return None;
                                }
                                if step.phase == CuStepPhase::AnytimeBase {
                                    let step_ctx = StepGenerationContext::new(
                                        &output_pack_sizes,
                                        &task_input_layouts,
                                        mission.as_str(),
                                        false,
                                        keyframe_logging_enabled,
                                        &mission_mod,
                                        ParallelLifecyclePlacement::default(),
                                        true,
                                    );
                                    let task_instance = quote! { (*task) };
                                    let (base_block, logging) = generate_anytime_base_block(
                                        step,
                                        *task_index,
                                        &task_specs,
                                        &step_ctx,
                                        &task_instance,
                                    );
                                    let total = anytime_refine_totals[&step.node_id];
                                    let refine_blocks: Vec<proc_macro2::TokenStream> = (1..=total)
                                        .map(|ordinal| {
                                            generate_anytime_refine_block(
                                                step,
                                                *task_index,
                                                &task_specs,
                                                &step_ctx,
                                                &task_instance,
                                                ordinal,
                                                total,
                                            )
                                        })
                                        .collect();
                                    let (parallel_pre, parallel_post) = parallel_task_lifecycle_tokens(
                                        task_trait_for_specs(&task_specs, *task_index),
                                        &task_specs.task_types[*task_index],
                                        task_specs.stateless_flags[*task_index],
                                        *task_index,
                                        &mission_mod,
                                        &task_instance,
                                        parallel_lifecycle_placements
                                            .as_ref()
                                            .expect("parallel lifecycle placements missing")[step_index],
                                    );
                                    let job_local = anytime_job_local_tokens(&task_specs, *task_index);
                                    let body = quote! {
                                        let task = unsafe { step_rt.task_ptrs.#task_index_ts.as_mut() };
                                        #parallel_pre
                                        #job_local
                                        #base_block
                                        #(#refine_blocks)*
                                        #parallel_post
                                    };
                                    return Some((wrap_process_step_tokens(true, body), logging));
                                }
                                let task_access = if task_specs.stateless_flags[*task_index] {
                                    quote! { let task = unsafe { step_rt.task_ptrs.#task_index_ts.as_ref() }; }
                                } else {
                                    quote! { let task = unsafe { step_rt.task_ptrs.#task_index_ts.as_mut() }; }
                                };
                                Some(generate_task_execution_tokens(
                                    step,
                                    *task_index,
                                    &task_specs,
                                    &task_specs.task_types[*task_index],
                                    StepGenerationContext::new(
                                        &output_pack_sizes,
                                        &task_input_layouts,
                                        mission.as_str(),
                                        false,
                                        keyframe_logging_enabled,
                                        &mission_mod,
                                        parallel_lifecycle_placements
                                            .as_ref()
                                            .expect("parallel lifecycle placements missing")[step_index],
                                        true,
                                    ),
                                    TaskExecutionTokens::new(quote! {
                                        #task_access
                                    }, quote! { (*task) }),
                                ))
                            }
                            ExecutionEntityKind::BridgeRx {
                                bridge_index,
                                channel_index,
                            } => {
                                let spec = &culist_bridge_specs[*bridge_index];
                                let bridge_index_ts = int2sliceindex(spec.tuple_index as u32);
                                Some(generate_bridge_rx_execution_tokens(
                                    step,
                                    spec,
                                    *channel_index,
                                    StepGenerationContext::new(
                                        &output_pack_sizes,
                                        &task_input_layouts,
                                        mission.as_str(),
                                        false,
                                        keyframe_logging_enabled,
                                        &mission_mod,
                                        parallel_lifecycle_placements
                                            .as_ref()
                                            .expect("parallel lifecycle placements missing")
                                            [step_index],
                                        true,
                                    ),
                                    quote! {
                                        let bridge = unsafe { step_rt.bridge_ptrs.#bridge_index_ts.as_mut() };
                                    },
                                ))
                            }
                            ExecutionEntityKind::BridgeTx {
                                bridge_index,
                                channel_index,
                            } => {
                                let spec = &culist_bridge_specs[*bridge_index];
                                let bridge_index_ts = int2sliceindex(spec.tuple_index as u32);
                                Some(generate_bridge_tx_execution_tokens(
                                    step,
                                    spec,
                                    *channel_index,
                                    StepGenerationContext::new(
                                        &output_pack_sizes,
                                        &task_input_layouts,
                                        mission.as_str(),
                                        false,
                                        keyframe_logging_enabled,
                                        &mission_mod,
                                        parallel_lifecycle_placements
                                            .as_ref()
                                            .expect("parallel lifecycle placements missing")[step_index],
                                        true,
                                    ),
                                    quote! {
                                        let bridge = unsafe { step_rt.bridge_ptrs.#bridge_index_ts.as_mut() };
                                    },
                                ))
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
                    #[allow(deprecated)] // implements the deprecated raw trait method
                    fn run_one_iteration(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()>
                },
                quote! {
                    #[allow(deprecated)] // implements the deprecated raw trait method
                    fn start_all_tasks(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()>
                },
                quote! {
                    #[allow(deprecated)] // implements the deprecated raw trait method
                    fn stop_all_tasks(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()>
                },
                quote! {
                    #[allow(deprecated)] // implements the deprecated raw trait method
                    fn run(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()>
                },
            )
        } else {
            (
                quote! {
                    #[allow(deprecated)] // implements the deprecated raw trait method
                    fn run_one_iteration(&mut self) -> CuResult<()>
                },
                quote! {
                    #[allow(deprecated)] // implements the deprecated raw trait method
                    fn start_all_tasks(&mut self) -> CuResult<()>
                },
                quote! {
                    #[allow(deprecated)] // implements the deprecated raw trait method
                    fn stop_all_tasks(&mut self) -> CuResult<()>
                },
                quote! {
                    #[allow(deprecated)] // implements the deprecated raw trait method
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

        let anytime_job_locals = build_anytime_job_locals(&task_specs);
        let anytime_policy_defs = build_anytime_policy_defs(&task_specs);
        let (runtime_plan_code, preprocess_logging_calls): (Vec<_>, Vec<_>) =
            itertools::multiunzip(runtime_plan_code_and_logging);
        let process_step_tasks_type = if sim_mode {
            quote!(CuSimTasks)
        } else {
            quote!(CuTasks)
        };
        let parallel_keyframe_component_indices: Vec<usize> = culist_plan
            .steps
            .iter()
            .filter_map(|unit| {
                let CuExecutionUnit::Step(step) = unit else {
                    panic!("Execution loops are not supported in runtime generation");
                };
                if step.phase == CuStepPhase::AnytimeRefine {
                    return None;
                }
                let component = match &culist_exec_entities[step.node_id as usize].kind {
                    ExecutionEntityKind::Task { task_index } => {
                        ParallelLifecycleKey::Task(*task_index)
                    }
                    ExecutionEntityKind::BridgeRx { bridge_index, .. }
                    | ExecutionEntityKind::BridgeTx { bridge_index, .. } => {
                        ParallelLifecycleKey::Bridge(*bridge_index)
                    }
                };
                Some(
                    keyframe_restore_order
                        .iter()
                        .position(|candidate| *candidate == component)
                        .expect("parallel keyframe component missing from restore order"),
                )
            })
            .collect();
        let (parallel_process_step_idents, parallel_process_step_fn_defs): (
            Vec<Ident>,
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
                    .zip(parallel_keyframe_component_indices.iter())
                    .map(|((step_ident, step_code), component)| {
                        let keyframe_local = keyframe_logging_enabled.then(|| {
                            quote! {
                                let kf_manager = ParallelKeyFrameAccessor::new(
                                    step_rt.keyframe_ptr,
                                    step_rt.keyframe_len,
                                    #component,
                                );
                            }
                        });
                        quote! {
                            #[inline(always)]
                            fn #step_ident(
                                step_rt: &mut ParallelProcessStepRuntime<'_>,
                            ) -> cu29::curuntime::ProcessStepResult {
                                let clock = step_rt.clock;
                                let execution_probe = step_rt.execution_probe;
                                let monitor = step_rt.monitor;
                                #keyframe_local
                                let clid = step_rt.clid;
                                let ctx = &mut step_rt.ctx;
                                // SAFETY: each generated step projects only the message fields
                                // authorized by the validated schedule. Taking a raw pointer to
                                // the tuple avoids borrowing unrelated cache-isolated regions.
                                let msgs = unsafe {
                                    core::ptr::addr_of_mut!((*step_rt.culist).msgs.0)
                                };
                                #[allow(unused_macros)]
                                macro_rules! __cu_input {
                                    ($field:tt) => { unsafe { &(*msgs).$field.value } };
                                    ($field:tt, $port:tt) => {
                                        unsafe { &(*msgs).$field.value.$port }
                                    };
                                }
                                #[allow(unused_macros)]
                                macro_rules! __cu_output {
                                    ($field:tt) => { unsafe { &mut (*msgs).$field.value } };
                                    ($field:tt, $port:tt) => {
                                        unsafe { &mut (*msgs).$field.value.$port }
                                    };
                                }
                                #step_code
                            }
                        }
                    })
                    .collect();
            (parallel_process_step_idents, parallel_process_step_fn_defs)
        } else {
            (Vec::new(), Vec::new())
        };
        let parallel_process_stage_count_tokens =
            proc_macro2::Literal::usize_unsuffixed(parallel_process_step_idents.len());
        let parallel_task_ptrs_type = if runtime_task_types.is_empty() {
            quote! { () }
        } else {
            let elems = runtime_task_types
                .iter()
                .map(|ty| quote! { ParallelSharedPtr<#ty> });
            quote! { (#(#elems),*,) }
        };
        let parallel_task_ptr_values = if runtime_task_types.is_empty() {
            quote! { () }
        } else {
            let elems = (0..runtime_task_types.len()).map(|index| {
                let index = syn::Index::from(index);
                quote! { ParallelSharedPtr::new(&mut runtime.tasks.#index.value as *mut _) }
            });
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
        let parallel_bridge_ptr_values = if bridge_runtime_types.is_empty() {
            quote! { () }
        } else {
            let elems = (0..bridge_runtime_types.len()).map(|index| {
                let index = syn::Index::from(index);
                quote! { ParallelSharedPtr::new(&mut runtime.bridges.#index.value as *mut _) }
            });
            quote! { (#(#elems),*,) }
        };
        let parallel_keyframe_accessor = keyframe_logging_enabled.then(|| {
            quote! {
                struct ParallelKeyFrameAccessor {
                    ptr: *mut u8,
                    len: usize,
                    component: usize,
                }

                impl ParallelKeyFrameAccessor {
                    #[inline(always)]
                    fn new(
                        ptr: *mut u8,
                        len: usize,
                        component: usize,
                    ) -> Self {
                        Self { ptr, len, component }
                    }

                    #[inline(always)]
                    fn freeze_task(
                        &self,
                        _culistid: u64,
                        task: &impl cu29::cutask::Freezable,
                    ) -> CuResult<usize> {
                        // SAFETY: generated scheduling assigns this component's
                        // disjoint region to this stage and joins before output.
                        unsafe {
                            cu29::curuntime::KeyFrame::freeze_region(
                                self.ptr,
                                self.len,
                                self.component,
                                task,
                            )
                        }
                    }

                    #[inline(always)]
                    fn freeze_any(
                        &self,
                        culistid: u64,
                        item: &impl cu29::cutask::Freezable,
                    ) -> CuResult<usize> {
                        self.freeze_task(culistid, item)
                    }
                }
            }
        });
        let parallel_process_step_keyframe_fields = keyframe_logging_enabled.then(|| {
            quote! {
                keyframe_ptr: *mut u8,
                keyframe_len: usize,
            }
        });
        let parallel_rt_support_tokens = if parallel_rt_run_supported {
            quote! {
                type ParallelTaskPtrs = #parallel_task_ptrs_type;
                type ParallelBridgePtrs = #parallel_bridge_ptrs_type;

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

                #parallel_keyframe_accessor

                struct ParallelProcessStepRuntime<'a> {
                    clock: &'a RobotClock,
                    execution_probe: &'a cu29::monitoring::RuntimeExecutionProbe,
                    monitor: &'a #monitor_type,
                    task_ptrs: &'a ParallelTaskPtrs,
                    bridge_ptrs: &'a ParallelBridgePtrs,
                    #parallel_process_step_keyframe_fields
                    culist: *mut CuList,
                    clid: u64,
                    ctx: cu29::context::CuContext,
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

        let config_load_stmt = build_config_load_stmt(
            std,
            application_name,
            subsystem_id.as_deref(),
            &config_features,
        );
        let constant_override_warning = if std {
            let comparisons = constant_fingerprints
                .iter()
                .map(|(module, id, fingerprint)| {
                    quote! { (#module, #id) => fingerprint != #fingerprint, }
                });
            Some(quote! {
                for constant in &config.constants {
                    let changed = constant
                        .semantic_fingerprint()
                        .map_or(true, |fingerprint| match (constant.module_path(), constant.id()) {
                            #(#comparisons)*
                            _ => true,
                        });
                    if changed {
                        ::cu29::prelude::warning!(
                            "Runtime configuration tried to override compile-time constant '{}'; the value baked into this binary will be used.",
                            constant.qualified_id()
                        );
                    }
                }
            })
        } else {
            None
        };

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
        let planner_kind_tokens = match copper_config.planner_kind() {
            PlannerKind::Serial => quote! { cu29::config::PlannerKind::Serial },
            PlannerKind::TaskOrder => quote! { cu29::config::PlannerKind::TaskOrder },
            PlannerKind::ExplicitSchedule => {
                quote! { cu29::config::PlannerKind::ExplicitSchedule }
            }
            PlannerKind::Pipeline => quote! { cu29::config::PlannerKind::Pipeline },
        };
        let planner_config_check = quote! {
            let configured_planner_kind = config.planner_kind();
            if configured_planner_kind != #planner_kind_tokens {
                return Err(CuError::from(format!(
                    "Configured planner kind ({configured_planner_kind:?}) does not match the runtime compiled into this binary ({:?})",
                    #planner_kind_tokens,
                )));
            }
            if configured_planner_kind == cu29::config::PlannerKind::Pipeline {
                let configured_max_in_flight = config
                    .planner_config()
                    .expect("Pipeline planner configuration disappeared")
                    .max_in_flight(configured_copperlist_count)?;
                if configured_max_in_flight != #pipeline_max_in_flight_tokens {
                    return Err(CuError::from(format!(
                        "Configured Pipeline max_in_flight ({configured_max_in_flight}) does not match the runtime compiled into this binary ({})",
                        #pipeline_max_in_flight_tokens,
                    )));
                }
            }
        };
        let keyframe_logging_check = quote! {
            let configured_keyframe_logging = config
                .logging
                .as_ref()
                .is_none_or(|logging| logging.enable_keyframe_logging && logging.enable_task_logging);
            // A ground-side simulation may disable local output entirely while
            // retaining the generated freeze/thaw plan for received recovery points.
            if configured_keyframe_logging != #local_keyframe_logging_enabled
                && !(#sim_mode && #logstream_enabled && !configured_keyframe_logging) {
                return Err(CuError::from(format!(
                    "Configured keyframe logging ({configured_keyframe_logging}) does not match the runtime compiled into this binary ({})",
                    #local_keyframe_logging_enabled
                )));
            }
        };
        let logstream_topology_check = if logstream_enabled {
            let expected_count = logstream_resource_specs.len();
            let destination_checks = logstream_resource_specs.iter().map(|spec| {
                let index = syn::Index::from(spec.destination_index);
                let destination = &copper_config
                    .log_streaming
                    .as_ref()
                    .expect("logstream specs require config")
                    .destinations[spec.destination_index];
                let expected_id = LitStr::new(&destination.id, Span::call_site());
                let expected_type = LitStr::new(&destination.transport.type_, Span::call_site());
                let expected_resource =
                    LitStr::new(&destination.transport.resource, Span::call_site());
                let expected_feedback = match &destination.feedback {
                    Some(feedback) => {
                        let ty = LitStr::new(&feedback.transport.type_, Span::call_site());
                        let resource = LitStr::new(&feedback.transport.resource, Span::call_site());
                        quote! { Some((#ty, #resource)) }
                    }
                    None => quote! { None },
                };
                quote! {
                    let destination = &configured_logstream_destinations[#index];
                    if destination.id != #expected_id
                        || destination.transport.type_ != #expected_type
                        || destination.transport.resource != #expected_resource
                        || destination.feedback.as_ref().map(|f| (f.transport.type_.as_str(), f.transport.resource.as_str())) != #expected_feedback
                    {
                        return Err(CuError::from(format!(
                            "Configured log_streaming destination topology at index {} does not match the runtime compiled into this binary",
                            #index,
                        )));
                    }
                }
            });
            quote! {
                let configured_logstream_destinations = config
                    .log_streaming
                    .as_ref()
                    .map_or(&[][..], |streaming| streaming.destinations.as_slice());
                if configured_logstream_destinations.len() != #expected_count {
                    return Err(CuError::from(format!(
                        "Configured log_streaming destination count ({}) does not match the runtime compiled into this binary ({})",
                        configured_logstream_destinations.len(),
                        #expected_count,
                    )));
                }
                #(#destination_checks)*
            }
        } else {
            quote! {}
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

        let build_with_resources_logstream_param = if logstream_enabled {
            quote! {
                logstream: Option<(
                    Box<dyn ::cu29::logstream::CuStreamTx>,
                    ::cu29::logstream::LogStreamSenderConfig,
                )>,
            }
        } else {
            quote! {}
        };
        let build_with_resources_sig = if sim_mode {
            quote! {
                fn build_with_resources<S: SectionStorage + 'static, L: UnifiedLogWrite<S> + 'static>(
                    clock: RobotClock,
                    unified_logger: Arc<Mutex<L>>,
                    app_resources: AppResources,
                    instance_id: u32,
                    #build_with_resources_logstream_param
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
                    #build_with_resources_logstream_param
                ) -> CuResult<Self>
            }
        };
        let parallel_rt_metadata_arg = if std {
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
                        self.copper_runtime.clock_ref(),
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
                        rate_limiter.limit(self.copper_runtime.clock_ref());
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
                        self.copper_runtime.clock_ref(),
                    ))
                    .transpose()?;
                loop  {
                    let result = <Self as #app_trait<S, L>>::run_one_iteration(self, #sim_callback_arg);
                    if let Some(rate_limiter) = rate_limiter.as_mut() {
                        rate_limiter.limit(self.copper_runtime.clock_ref());
                    }

                    if STOP_FLAG.load(Ordering::SeqCst) || result.is_err() {
                        break result;
                    }
                }
            }}
        };

        let parallel_keyframe_runtime = keyframe_logging_enabled.then(|| {
            quote! {
                let keyframe_interval = runtime.keyframes_manager.capture_interval();
            }
        });
        let parallel_keyframe_reset = keyframe_logging_enabled.then(|| {
            quote! {
                if let Some(keyframe) = culist.keyframe_mut() {
                    keyframe.reset_distributed(clid, clock.now(), keyframe_interval);
                }
            }
        });
        let parallel_keyframe_capture = if keyframe_logging_enabled {
            quote! { let (keyframe_ptr, keyframe_len) = culist.keyframe_capture(); }
        } else {
            quote! {
                let keyframe_ptr = core::ptr::null_mut();
                let keyframe_len = 0usize;
            }
        };
        let parallel_keyframe_finish = if keyframe_logging_enabled {
            quote! { (cl_manager.last_keyframe_bytes, 0u64) }
        } else {
            quote! { (0u64, 0u64) }
        };

        #[cfg(feature = "macro_debug")]
        eprintln!("[build the run methods]");
        let lane_executor_tokens = if parallel_rt_run_supported {
            match build_lane_executor_tokens(
                lane_plan.expect("distributed plan"),
                &culist_plan,
                &parallel_process_step_idents,
                &mission_mod,
                keyframe_logging_enabled,
            ) {
                Ok(tokens) => Some(tokens),
                Err(error) => {
                    return return_error(format!("Mission '{mission}': {error}"));
                }
            }
        } else {
            None
        };
        let run_body: proc_macro2::TokenStream = if let Some(lane) = lane_executor_tokens {
            let LaneExecutorTokens {
                worker_spawns: lane_worker_spawns,
                dispatcher_scheduling: lane_dispatcher_scheduling,
                completion_ready: lane_completion_ready,
                copperlists_per_cycle: lane_copperlists_per_cycle,
                max_in_flight: lane_max_in_flight,
                workers: lane_workers,
            } = lane;
            quote! {
                static STOP_FLAG: AtomicBool = AtomicBool::new(false);

                #kill_handler

                let mut tasks_started = false;
                let result = std::thread::scope(|scope| -> CuResult<()> {
                    #mission_mod::assert_parallel_rt_send_bounds();
                    #lane_dispatcher_scheduling

                    let runtime = &mut self.copper_runtime;
                    let clock_handle = runtime.clock();
                    let clock = &clock_handle;
                    let instance_id = runtime.instance_id();
                    let subsystem_code = runtime.subsystem_code();
                    let execution_probe_ptr =
                        #mission_mod::ParallelSharedPtr::from_ref(
                            runtime.execution_probe.as_ref() as *const _,
                        );
                    let monitor_ptr =
                        #mission_mod::ParallelSharedPtr::from_ref(
                            &runtime.monitor as *const _,
                        );
                    let task_ptrs: #mission_mod::ParallelTaskPtrs = #parallel_task_ptr_values;
                    let bridge_ptrs: #mission_mod::ParallelBridgePtrs = #parallel_bridge_ptr_values;
                    let mut free_copperlists: Vec<Option<cu29::arena::CuSlotLease<CuStampedDataSet>>> =
                        (0..#copperlist_count_tokens).map(|_| None).collect();
                    for token in runtime.copperlists_manager.take_execution_slots() {
                        let index = token.index();
                        free_copperlists[index] = Some(token);
                    }
                    #parallel_keyframe_runtime
                    let start_clid = runtime.copperlists_manager.next_cl_id();
                    runtime.parallel_rt.reset_cursors(start_clid);
                    debug_assert_eq!(
                        runtime.parallel_rt.metadata().process_stage_count(),
                        #parallel_process_stage_count_tokens,
                    );
                    let max_in_flight = #lane_max_in_flight;
                    debug_assert_eq!(runtime.parallel_rt.in_flight_limit(), max_in_flight);
                    let copperlists_per_cycle = #lane_copperlists_per_cycle;
                    let lanes = std::sync::Arc::new(cu29::parallel_rt::LaneExecutor::new(
                        start_clid,
                        max_in_flight,
                        #lane_copperlists_per_cycle as u32,
                        #lane_workers,
                    ));
                    let mut in_flight_slots: Vec<Option<cu29::arena::CuSlotLease<CuStampedDataSet>>> =
                        (0..max_in_flight).map(|_| None).collect();
                    let mut returned_copperlists = Vec::with_capacity(#copperlist_count_tokens);
                    let mut lane_handles = Vec::with_capacity(#lane_workers);
                    #(#lane_worker_spawns)*

                    let execution_result = (|| -> CuResult<()> {
                        <Self as #app_trait<S, L>>::start_all_tasks(self)?;
                        tasks_started = true;

                        let runtime = &mut self.copper_runtime;
                        let monitor = &runtime.monitor;
                        let cl_manager = &mut runtime.copperlists_manager;
                        let parallel_rt = &runtime.parallel_rt;

                        let mut dispatch_limiter = runtime
                        .runtime_config
                        .rate_target_hz
                        .map(|rate| cu29::curuntime::LoopRateLimiter::from_rate_target_hz(rate, clock))
                        .transpose()?;
                    let mut in_flight = 0usize;
                    let mut stop_launching = false;
                    let mut next_launch_clid = start_clid;
                    let mut next_commit_clid = start_clid;
                    let mut fatal_error: Option<CuError> = None;
                    let mut failed_clid: Option<u64> = None;

                    loop {
                        let observed_progress = lanes.progress_generation();
                        while let Some(recycled) = cl_manager.try_reclaim_slot()? {
                            let index = recycled.index();
                            free_copperlists[index] = Some(recycled);
                        }
                        if let Some((clid, error)) = lanes.take_failure() {
                            fatal_error = Some(error);
                            failed_clid = Some(clid);
                            stop_launching = true;
                        }

                        while in_flight > 0
                            && failed_clid.is_none_or(|clid| next_commit_clid < clid)
                            && #lane_completion_ready
                        {
                            if parallel_rt.current_commit_clid() != next_commit_clid {
                                fatal_error = Some(CuError::from(
                                    "Parallel commit checkpoint out of sync",
                                ));
                                stop_launching = true;
                                break;
                            }
                            let slot = (next_commit_clid % max_in_flight as u64) as usize;
                            let mut culist = in_flight_slots[slot]
                                .take()
                                .expect("completed CopperList slot");
                            let clid = next_commit_clid;
                            debug_assert_eq!(culist.id, clid);
                            let outcome = lanes.completion_outcome(clid)?;
                            let mut commit_ctx = cu29::context::CuContext::from_runtime_metadata(
                                clock.clone(), clid, instance_id, subsystem_code,
                                #mission_mod::TASK_IDS,
                            );
                            commit_ctx.clear_current_component();
                            commit_ctx.clear_current_task();
                            let monitor_result = monitor.process_copperlist(
                                &commit_ctx,
                                #mission_mod::MONITOR_LAYOUT.view(
                                    &#mission_mod::collect_metadata(&culist),
                                ),
                            );
                            if outcome == cu29::curuntime::ProcessStepOutcome::AbortCopperList {
                                if let Some(keyframe) = culist.keyframe_mut() {
                                    keyframe.discard_distributed();
                                }
                            } else {
                                #(#preprocess_logging_calls)*
                            }
                            if let cu29::curuntime::OwnedCopperListSubmission::Recycled(culist) =
                                cl_manager.submit_slot(culist)?
                            {
                                let index = culist.index();
                                free_copperlists[index] = Some(culist);
                            }
                            monitor_result?;
                            if outcome == cu29::curuntime::ProcessStepOutcome::Continue {
                                let (keyframe_bytes, dropped_keyframes_total) = #parallel_keyframe_finish;
                                monitor.observe_copperlist_io(cu29::monitoring::CopperListIoStats {
                                    raw_culist_bytes: core::mem::size_of::<CuList>() as u64
                                        + cl_manager.last_handle_bytes,
                                    handle_bytes: cl_manager.last_handle_bytes,
                                    encoded_culist_bytes: cl_manager.last_encoded_bytes,
                                    keyframe_bytes,
                                    structured_log_bytes_total: ::cu29::prelude::structured_log_bytes_total(),
                                    culistid: clid,
                                    dropped_copperlists_total: cl_manager.dropped_copperlists_total(),
                                    dropped_keyframes_total,
                                });
                            }
                            parallel_rt.release_commit(clid + 1);
                            next_commit_clid += 1;
                            in_flight -= 1;
                        }

                        if fatal_error.is_some()
                            && failed_clid.is_none_or(|clid| next_commit_clid >= clid)
                        {
                            break;
                        }

                        if STOP_FLAG.load(Ordering::SeqCst)
                            && !stop_launching
                            && (next_launch_clid - start_clid).is_multiple_of(copperlists_per_cycle)
                        {
                            stop_launching = true;
                            lanes.stop_admitting();
                        }
                        if !stop_launching && fatal_error.is_none() {
                            let next_clid = next_launch_clid;
                            let rate_ready = dispatch_limiter
                                .as_ref()
                                .map(|limiter| limiter.is_ready(clock))
                                .unwrap_or(true);
                            let arena_index =
                                (next_clid % #copperlist_count_tokens as u64) as usize;
                            let execution_slot =
                                (next_clid % max_in_flight as u64) as usize;

                            if in_flight < max_in_flight
                                && rate_ready
                                && free_copperlists[arena_index].is_some()
                                && in_flight_slots[execution_slot].is_none()
                            {
                                let clid = next_clid;
                                let mut culist = free_copperlists[arena_index]
                                    .take()
                                    .expect("free CopperList slot disappeared");
                                culist.reset(clid);
                                #parallel_keyframe_reset
                                culist.change_state(cu29::copperlist::CopperListState::Processing);
                                let culist_ptr = culist.as_ptr().cast::<u8>();
                                #parallel_keyframe_capture
                                in_flight_slots[execution_slot] = Some(culist);
                                lanes.admit_with_keyframe(
                                    clid,
                                    culist_ptr,
                                    keyframe_ptr,
                                    keyframe_len,
                                );
                                cl_manager.note_admission(clid + 1);
                                next_launch_clid += 1;
                                in_flight += 1;
                                if let Some(limiter) = dispatch_limiter.as_mut() {
                                    limiter.mark_tick(clock);
                                }
                                continue;
                            }
                        }

                        if in_flight == 0 {
                            if stop_launching || fatal_error.is_some() {
                                break;
                            }
                        }
                        let arena_index =
                            (next_launch_clid % #copperlist_count_tokens as u64) as usize;
                        if !stop_launching
                            && free_copperlists[arena_index].is_none()
                            && cl_manager.has_pending_slots()
                        {
                            let token = cl_manager.wait_reclaim_slot()?;
                            let index = token.index();
                            free_copperlists[index] = Some(token);
                            continue;
                        }
                        let timeout = if stop_launching {
                            None
                        } else {
                            dispatch_limiter
                                .as_ref()
                                .and_then(|limiter| limiter.remaining(clock))
                                .map(std::time::Duration::from)
                        };
                        lanes.wait_for_progress(observed_progress, timeout);
                    }

                        fatal_error.map_or(Ok(()), Err)
                    })();

                    lanes.request_shutdown();
                    for handle in lane_handles.drain(..) {
                        let _ = handle.join();
                    }
                    for slot in in_flight_slots.into_iter().flatten() {
                        let index = slot.index();
                        free_copperlists[index] = Some(slot);
                    }
                    let cleanup_result = (|| -> CuResult<()> {
                        let cl_manager = &mut self.copper_runtime.copperlists_manager;
                        cl_manager.finish_pending_with(|slot| {
                            let index = slot.index();
                            free_copperlists[index] = Some(slot);
                        })?;
                        returned_copperlists.extend(free_copperlists.into_iter().flatten());
                        debug_assert_eq!(returned_copperlists.len(), #copperlist_count_tokens);
                        cl_manager.return_execution_slots(returned_copperlists);
                        Ok(())
                    })();
                    execution_result.and(cleanup_result)
                });

                if result.is_err() {
                    error!("A task errored out: {}", &result);
                }
                if tasks_started {
                    <Self as #app_trait<S, L>>::stop_all_tasks(self, #sim_callback_arg)?;
                }
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
        let keyframe_manager_binding = keyframe_logging_enabled
            .then(|| quote! { let kf_manager = &mut runtime.keyframes_manager; });
        let keyframe_reset =
            keyframe_logging_enabled.then(|| quote! { kf_manager.try_reset(clid, clock)?; });
        let keyframe_finish =
            keyframe_logging_enabled.then(|| quote! { kf_manager.end_of_processing(clid)?; });
        let keyframe_bytes = if keyframe_logging_enabled {
            quote! { kf_manager.last_encoded_bytes }
        } else {
            quote! { 0 }
        };
        let dropped_keyframes_total = if keyframe_logging_enabled {
            quote! { kf_manager.dropped_keyframes_total() }
        } else {
            quote! { 0 }
        };
        let prepare_arena_keyframes = parallel_rt_run_supported.then(|| {
            quote! {
                runtime
                    .copperlists_manager
                    .prepare_keyframes(&kf_manager.capture_capacities);
            }
        });
        let keyframe_preallocation = keyframe_logging_enabled.then(|| {
            quote! {
                {
                    let runtime = &mut self.copper_runtime;
                    let tasks = &runtime.tasks;
                    let __cu_bridges = &runtime.bridges;
                    let kf_manager = &mut runtime.keyframes_manager;
                    kf_manager.begin_capture_preallocation();
                    #(#keyframe_preallocation_code)*
                    kf_manager.finish_capture_preallocation()?;
                    #prepare_arena_keyframes
                }
            }
        });
        let keyframe_finish_pending = keyframe_logging_enabled.then(|| {
            quote! { self.copper_runtime.keyframes_manager.finish_pending()?; }
        });

        let live_completed = (sim_mode && logstream_enabled).then(|| {
            quote! {
                let _ = sim_callback(SimStep::CopperListCompleted(culist));
            }
        });
        let live_replay_impl = if sim_mode && logstream_enabled {
            live_twin::runtime_support(
                application_name,
                &mission_mod,
                &culist_plan,
                &culist_exec_entities,
            )
        } else {
            quote! {}
        };
        let pipeline_iteration_keyframe = keyframe_logging_enabled.then(|| {
            quote! {
                if let Some(keyframe) = culist.keyframe_mut() {
                    keyframe.reset_distributed(
                        clid,
                        clock.now(),
                        runtime.keyframes_manager.capture_interval(),
                    );
                }
                let (keyframe_ptr, keyframe_len) = culist.keyframe_capture();
            }
        });
        let pipeline_iteration_keyframe_field =
            keyframe_logging_enabled.then(|| quote! { keyframe_ptr, keyframe_len, });
        let iteration_body = if parallel_rt_run_supported {
            quote! {
                // Explicit stepping uses one lease from the same arena and the
                // same generated component stages as the Pipeline run loop.
                let runtime = &mut self.copper_runtime;
                let clock = runtime.clock();
                let instance_id = runtime.instance_id();
                let subsystem_code = runtime.subsystem_code();
                let task_ptrs: #mission_mod::ParallelTaskPtrs = #parallel_task_ptr_values;
                let bridge_ptrs: #mission_mod::ParallelBridgePtrs = #parallel_bridge_ptr_values;
                let monitor = &runtime.monitor;
                let cl_manager = &mut runtime.copperlists_manager;
                let clid = cl_manager.next_cl_id();
                let mut culist = cl_manager.acquire_iteration_slot()?;
                cl_manager.note_admission(clid + 1);
                culist.reset(clid);
                culist.change_state(cu29::copperlist::CopperListState::Processing);
                #pipeline_iteration_keyframe
                let mut step_rt = #mission_mod::ParallelProcessStepRuntime {
                    clock: &clock,
                    execution_probe: runtime.execution_probe.as_ref(),
                    monitor,
                    task_ptrs: &task_ptrs,
                    bridge_ptrs: &bridge_ptrs,
                    #pipeline_iteration_keyframe_field
                    culist: culist.as_ptr(),
                    clid,
                    ctx: cu29::context::CuContext::from_runtime_metadata(
                        clock.clone(), clid, instance_id, subsystem_code,
                        #mission_mod::TASK_IDS,
                    ),
                };
                #(
                    match #parallel_process_step_idents(&mut step_rt) {
                        Ok(cu29::curuntime::ProcessStepOutcome::Continue) => {}
                        Ok(cu29::curuntime::ProcessStepOutcome::AbortCopperList) => {
                            if let Some(keyframe) = culist.keyframe_mut() {
                                keyframe.discard_distributed();
                            }
                            step_rt.ctx.clear_current_component();
                            step_rt.ctx.clear_current_task();
                            let monitor_result = monitor.process_copperlist(
                                &step_rt.ctx,
                                #mission_mod::MONITOR_LAYOUT.view(&#mission_mod::collect_metadata(&culist)),
                            );
                            drop(step_rt);
                            if let cu29::curuntime::OwnedCopperListSubmission::Recycled(slot) =
                                cl_manager.submit_slot(culist)?
                            {
                                cl_manager.recycle_slot(slot);
                            }
                            monitor_result?;
                            return Ok(());
                        }
                        Err(error) => {
                            drop(step_rt);
                            cl_manager.recycle_slot(culist);
                            return Err(error);
                        }
                    }
                )*
                step_rt.ctx.clear_current_component();
                step_rt.ctx.clear_current_task();
                let monitor_result = monitor.process_copperlist(
                    &step_rt.ctx,
                    #mission_mod::MONITOR_LAYOUT.view(&#mission_mod::collect_metadata(&culist)),
                );
                drop(step_rt);
                #(#preprocess_logging_calls)*
                if let cu29::curuntime::OwnedCopperListSubmission::Recycled(slot) =
                    cl_manager.submit_slot(culist)?
                {
                    cl_manager.recycle_slot(slot);
                }
                monitor_result?;
                monitor.observe_copperlist_io(cu29::monitoring::CopperListIoStats {
                    raw_culist_bytes: core::mem::size_of::<CuList>() as u64
                        + cl_manager.last_handle_bytes,
                    handle_bytes: cl_manager.last_handle_bytes,
                    encoded_culist_bytes: cl_manager.last_encoded_bytes,
                    keyframe_bytes: cl_manager.last_keyframe_bytes,
                    structured_log_bytes_total: ::cu29::prelude::structured_log_bytes_total(),
                    culistid: clid,
                    dropped_copperlists_total: cl_manager.dropped_copperlists_total(),
                    dropped_keyframes_total: 0,
                });
                Ok(())
            }
        } else {
            quote! {
                // Pre-explode the runtime to avoid complexity with partial borrowing in the generated code.
                let runtime = &mut self.copper_runtime;
                let clock_handle = runtime.clock();
                let clock = &clock_handle;
                let instance_id = runtime.instance_id();
                let subsystem_code = runtime.subsystem_code();
                let execution_probe = &runtime.execution_probe;
                let monitor = &mut runtime.monitor;
                let tasks = &mut runtime.tasks;
                let __cu_bridges = &mut runtime.bridges;
                let cl_manager = &mut runtime.copperlists_manager;
                #keyframe_manager_binding
                let iteration_clid = cl_manager.next_cl_id();
                let mut ctx = cu29::context::CuContext::from_runtime_metadata(
                    clock.clone(), iteration_clid, instance_id, subsystem_code,
                    #mission_mod::TASK_IDS,
                );
                let mut __cu_abort_copperlist = false;
                #(#preprocess_calls)*
                let culist = cl_manager.create()?;
                let clid = culist.id;
                debug_assert_eq!(clid, iteration_clid);
                #keyframe_reset
                culist.change_state(cu29::copperlist::CopperListState::Processing);
                let mut ctx = cu29::context::CuContext::from_runtime_metadata(
                    clock.clone(), iteration_clid, instance_id, subsystem_code,
                    #mission_mod::TASK_IDS,
                );
                {
                    let msgs = &mut culist.msgs.0;
                    #[allow(unused_macros)]
                    macro_rules! __cu_input {
                        ($field:tt) => { &msgs.$field };
                        ($field:tt, $port:tt) => { &msgs.$field.$port };
                    }
                    #[allow(unused_macros)]
                    macro_rules! __cu_output {
                        ($field:tt) => { &mut msgs.$field };
                        ($field:tt, $port:tt) => { &mut msgs.$field.$port };
                    }
                    #(#anytime_job_locals)*
                    '__cu_process_steps: {
                        #(#runtime_plan_code)*
                    }
                }
                if __cu_abort_copperlist {
                    ctx.clear_current_component();
                    ctx.clear_current_task();
                    let monitor_result = monitor.process_copperlist(
                        &ctx,
                        #mission_mod::MONITOR_LAYOUT.view(&#mission_mod::collect_metadata(&culist)),
                    );
                    cl_manager.end_of_processing(clid)?;
                    monitor_result?;
                    return Ok(());
                }
                ctx.clear_current_component();
                ctx.clear_current_task();
                let monitor_result = monitor.process_copperlist(
                    &ctx,
                    #mission_mod::MONITOR_LAYOUT.view(&#mission_mod::collect_metadata(&culist)),
                );
                #(#preprocess_logging_calls)*
                #live_completed
                cl_manager.end_of_processing(clid)?;
                monitor_result?;
                #(#postprocess_calls)*
                #keyframe_finish
                monitor.observe_copperlist_io(cu29::monitoring::CopperListIoStats {
                    raw_culist_bytes: core::mem::size_of::<CuList>() as u64
                        + cl_manager.last_handle_bytes,
                    handle_bytes: cl_manager.last_handle_bytes,
                    encoded_culist_bytes: cl_manager.last_encoded_bytes,
                    keyframe_bytes: #keyframe_bytes,
                    structured_log_bytes_total: ::cu29::prelude::structured_log_bytes_total(),
                    culistid: clid,
                    dropped_copperlists_total: cl_manager.dropped_copperlists_total(),
                    dropped_keyframes_total: #dropped_keyframes_total,
                });
                Ok(())
            }
        };
        let run_methods: proc_macro2::TokenStream = quote! {

            #run_one_iteration {
                #iteration_body
            }

            fn restore_keyframe(&mut self, keyframe: &KeyFrame) -> CuResult<()> {
                let runtime = &mut self.copper_runtime;
                let clock_handle = runtime.clock();
                let clock = &clock_handle;
                let tasks = &mut runtime.tasks;
                let __cu_bridges = &mut runtime.bridges;
                let mut frames = cu29::curuntime::KeyFramePayloadReader::new(keyframe)?;
                #(#keyframe_restore_code)*
                frames.finish()?;
                Ok(())
            }

            #start_all_tasks {
                let _ = self.log_runtime_lifecycle_event(RuntimeLifecycleEvent::MissionStarted {
                    mission: #mission.to_string(),
                });
                let lifecycle_clid = self.copper_runtime.copperlists_manager.last_cl_id();
                let mut ctx = cu29::context::CuContext::from_runtime_metadata(
                    self.copper_runtime.clock(),
                    lifecycle_clid,
                    self.copper_runtime.instance_id(),
                    self.copper_runtime.subsystem_code(),
                    #mission_mod::TASK_IDS,
                );
                #(#start_calls)*
                #keyframe_preallocation
                ctx.clear_current_component();
                ctx.clear_current_task();
                self.copper_runtime.monitor.start(&ctx)?;
                Ok(())
            }

            #stop_all_tasks {
                let lifecycle_clid = self.copper_runtime.copperlists_manager.last_cl_id();
                let mut ctx = cu29::context::CuContext::from_runtime_metadata(
                    self.copper_runtime.clock(),
                    lifecycle_clid,
                    self.copper_runtime.instance_id(),
                    self.copper_runtime.subsystem_code(),
                    #mission_mod::TASK_IDS,
                );
                #(#stop_calls)*
                ctx.clear_current_component();
                ctx.clear_current_task();
                self.copper_runtime.monitor.stop(&ctx)?;
                self.copper_runtime.copperlists_manager.finish_pending()?;
                #keyframe_finish_pending
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

        let app_resources_thread_pools_field = if std {
            quote! { pub thread_pools: Vec<Option<Arc<ThreadPool>>>, }
        } else {
            quote!()
        };

        let app_resources_struct = quote! {
            pub struct AppResources {
                pub config: CuConfig,
                pub config_source: RuntimeLifecycleConfigSource,
                pub resources: ResourceManager,
                #app_resources_thread_pools_field
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
                let mut config = config;
                if #sim_mode { config.log_streaming = None; }
                #constant_override_warning
                #copperlist_count_check
                #planner_config_check
                #keyframe_logging_check
                #logstream_topology_check
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

        let prepare_resources_thread_pools_stmt = if std {
            quote! {
                let thread_pools = #mission_mod::thread_pools_instanciator(&config)?;
            }
        } else {
            quote!()
        };
        let prepare_resources_thread_pools_init = if std {
            quote! { thread_pools, }
        } else {
            quote!()
        };

        let prepare_resources_fn = quote! {
            #prepare_resources_sig {
                let (config, config_source) = #prepare_config_call;

                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp init: building resources");
                let resources = #mission_mod::resources_instanciator(&config)?;
                #prepare_resources_thread_pools_stmt
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp init: resources ready");

                Ok(AppResources {
                    config,
                    config_source,
                    resources,
                    #prepare_resources_thread_pools_init
                })
            }
        };

        let no_logstream_arg = logstream_enabled.then(|| quote! { None, });
        let builder_logstream_arg = logstream_enabled.then(|| quote! { self.logstream, });
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
                        #no_logstream_arg
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
                    Self::build_with_resources(
                        clock,
                        unified_logger,
                        app_resources,
                        instance_id,
                        #no_logstream_arg
                    )
                }
            }
        };

        let build_with_resources_thread_pools_destructure = if std {
            quote! { thread_pools, }
        } else {
            quote!()
        };
        let build_with_resources_thread_pools_call = if std {
            quote! { .with_thread_pools(thread_pools) }
        } else {
            quote!()
        };
        let local_keyframe_sink_init = if local_keyframe_logging_enabled {
            quote! {
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: creating keyframes stream");
                let local_keyframe_sink = stream_write::<KeyFrame, S>(
                    unified_logger.clone(),
                    UnifiedLogType::FrozenTasks,
                    1024 * 1024 * 10, // 10 MiB
                )?;
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: keyframes stream ready");
            }
        } else {
            quote! {
                let local_keyframe_sink = cu29::curuntime::NullWriteStream;
            }
        };
        let configured_logstream_session = (!logstream_resource_specs.is_empty()).then(|| {
            quote! {
                let logstream_session_id = ::cu29::logstream::new_session_id();
                let logstream_schema = <#mission_mod::CuStampedDataSet as ::cu29::logstream::capture::CaptureDataSet>::stream_schema();
            }
        });
        let configured_logstream_initializers = logstream_resource_specs.iter().map(|spec| {
            let index = syn::Index::from(spec.destination_index);
            let bundle_index = spec.stream.bundle_index;
            let provider_path = &spec.stream.provider_path;
            let resource_name = LitStr::new(&spec.stream.resource_name, Span::call_site());
            let transport_type = &spec.stream.transport_type;
            let transport_ident = format_ident!("__cu_logstream_transport_{}", spec.destination_index);
            let continuous_ident = format_ident!("__cu_logstream_continuous_{}", spec.destination_index);
            let recovery_ident = format_ident!("__cu_logstream_recovery_{}", spec.destination_index);
            let scheduled = if let Some(feedback) = &spec.feedback {
                let bundle = feedback.bundle_index;
                let provider = &feedback.provider_path;
                let name = LitStr::new(&feedback.resource_name, Span::call_site());
                let ty = &feedback.transport_type;
                quote! {
                    let feedback_rx: #ty = resources.take(
                        cu29::resource::ResourceKey::<()>::new(cu29::resource::BundleIndex::new(#bundle),
                            cu29::resource::resource_index_by_name::<#provider>(#name)).typed::<#ty>()
                    )?.0;
                    let (mut #continuous_ident, #recovery_ident, sender_monitor) = ::cu29::logstream::scheduled_feedback_sinks::<#mission_mod::CuStampedDataSet, _>(
                        ::cu29::logstream::SeparateFeedback { tx: #transport_ident, feedback_rx }, sender_config,
                        if clock.is_mock() { RobotClock::new() } else { clock.clone() },
                    )?;
                }
            } else {
                quote! {
                    let (mut #continuous_ident, #recovery_ident, sender_monitor) = ::cu29::logstream::scheduled_sinks::<#mission_mod::CuStampedDataSet, _>(
                        #transport_ident, sender_config,
                        if clock.is_mock() { RobotClock::new() } else { clock.clone() },
                    )?;
                }
            };
            quote! {
                let destination = &config
                    .log_streaming
                    .as_ref()
                    .expect("compiled logstream configuration is missing")
                    .destinations[#index];
                let plan = ::cu29::logstream::LogStreamPlan::resolve(destination)
                    .map_err(|error| CuError::from(error.to_string()))?;
                let sender_config = plan
                    .sender_config(
                        ::cu29::logstream::StreamIdentity {
                            session_id: logstream_session_id,
                            sender_id: instance_id,
                        },
                        logstream_schema.clone(),
                    )
                    .map_err(|error| CuError::from(error.to_string()))?;
                let #transport_ident: #transport_type = resources
                    .take(
                        cu29::resource::ResourceKey::<()>::new(
                            cu29::resource::BundleIndex::new(#bundle_index),
                            cu29::resource::resource_index_by_name::<#provider_path>(#resource_name),
                        )
                        .typed::<#transport_type>(),
                    )?
                    .0;
                #scheduled
                let structured_outputs = (#continuous_ident.take_structured_log_sink(), structured_outputs);
                stream_monitors.push(sender_monitor.into_runtime_monitor(&destination.id,
                    destination.link.bitrate_bps, usize::from(destination.fec.continuous.repair_every_source_symbols)));
                let #continuous_ident = if logstream_schema.reconstruction.is_empty() {
                    #continuous_ident
                } else {
                    #continuous_ident.with_encoder(::cu29::logstream::capture::encode_capture_record_into)
                };
            }
        });
        let configured_logstream_fanouts = logstream_resource_specs.iter().map(|spec| {
            let continuous_ident =
                format_ident!("__cu_logstream_continuous_{}", spec.destination_index);
            let recovery_ident =
                format_ident!("__cu_logstream_recovery_{}", spec.destination_index);
            quote! {
                let copperlist_sink = ::cu29::fanout::StaticFanoutSink::new(
                    copperlist_sink,
                    #continuous_ident,
                );
                let keyframe_sink = ::cu29::fanout::StaticFanoutSink::new(
                    keyframe_sink,
                    #recovery_ident,
                );
            }
        });
        let configured_logstream_count = logstream_resource_specs.len();
        let copperlist_output_graph = if logstream_enabled {
            quote! {
                let local_copperlist_output_required = config
                    .logging
                    .as_ref()
                    .is_none_or(|logging| logging.enable_task_logging);
                let local_keyframe_output_required = config
                    .logging
                    .as_ref()
                    .is_none_or(|logging| {
                        logging.enable_task_logging && logging.enable_keyframe_logging
                    });
                let logstream_output_required = #configured_logstream_count != 0 || logstream.is_some();
                #[allow(unused_mut)]
                let mut resources = resources;
                let mut stream_monitors = Vec::with_capacity(#configured_logstream_count + usize::from(logstream.is_some()));
                let structured_outputs = ();
                #configured_logstream_session
                #(#configured_logstream_initializers)*
                let injected_logstream_sinks = logstream
                    .map(|(transport, mut sender_config)| {
                        sender_config.continuous.identity.sender_id = instance_id;
                        sender_config.recovery.finite.identity.sender_id = instance_id;
                        let schema = <#mission_mod::CuStampedDataSet as ::cu29::logstream::capture::CaptureDataSet>::stream_schema();
                        if !schema.reconstruction.is_empty() {
                            let manifest = ::cu29::logstream::SessionManifest::decode_record(&sender_config.recovery.manifest_record)
                                .map_err(|e| CuError::from(e.to_string()))?;
                            if manifest.application_schema != schema {
                                return Err(CuError::from("injected sender must use the generated reconstruction schema"));
                            }
                        }
                        let bitrate = sender_config.pacing.bitrate_bps;
                        let baseline = sender_config.continuous.repair_every_source_symbols;
                        let (mut copperlist, keyframe, sender_monitor) = ::cu29::logstream::scheduled_sinks::<
                            #mission_mod::CuStampedDataSet, _
                        >(transport, sender_config,
                            if clock.is_mock() { RobotClock::new() } else { clock.clone() })?;
                        stream_monitors.push(sender_monitor.into_runtime_monitor("injected", bitrate, baseline));
                        let structured = copperlist.take_structured_log_sink();
                        let copperlist = if schema.reconstruction.is_empty() { copperlist }
                            else { copperlist.with_encoder(::cu29::logstream::capture::encode_capture_record_into) };
                        Ok::<_, CuError>(((copperlist, keyframe), structured))
                    })
                    .transpose()?;
                let stream_monitors: Option<::std::sync::Arc<[cu29::monitoring::LogStreamMonitor]>> =
                    (!stream_monitors.is_empty()).then(|| stream_monitors.into());
                let (injected_logstream_sinks, injected_structured) = injected_logstream_sinks.unzip();
                let structured_outputs = (injected_structured.flatten(), structured_outputs);
                let (injected_logstream_sink, injected_logstream_keyframe_sink) = injected_logstream_sinks.unzip();
                let copperlist_sink = ::cu29::fanout::OptionalWriteStream::new(
                    local_copperlist_output_required.then_some(local_copperlist_sink),
                );
                let keyframe_sink = ::cu29::fanout::OptionalWriteStream::new(
                    local_keyframe_output_required.then_some(local_keyframe_sink),
                );
                #(#configured_logstream_fanouts)*
                let copperlist_sink = ::cu29::fanout::StaticFanoutSink::new(
                    copperlist_sink,
                    ::cu29::fanout::OptionalWriteStream::new(injected_logstream_sink),
                );
                let keyframe_sink = ::cu29::fanout::StaticFanoutSink::new(
                    keyframe_sink,
                    ::cu29::fanout::OptionalWriteStream::new(injected_logstream_keyframe_sink),
                );
                let output_requirements = cu29::curuntime::OutputRequirements::new(
                    local_copperlist_output_required || logstream_output_required,
                    local_keyframe_output_required || logstream_output_required,
                );
            }
        } else {
            quote! {
                // Downstream record demand is independent from local logging.
                let local_copperlist_output_required = config
                    .logging
                    .as_ref()
                    .is_none_or(|logging| logging.enable_task_logging);
                let local_keyframe_output_required = config
                    .logging
                    .as_ref()
                    .is_none_or(|logging| {
                        logging.enable_task_logging && logging.enable_keyframe_logging
                    });
                let output_requirements = cu29::curuntime::OutputRequirements::new(
                    local_copperlist_output_required,
                    local_keyframe_output_required,
                );
                let copperlist_sink = local_copperlist_sink;
                let keyframe_sink = local_keyframe_sink;
            }
        };

        let monitor_instanciator = if logstream_enabled {
            quote! { |config: &CuConfig, metadata: cu29::monitoring::CuMonitoringMetadata, runtime: cu29::monitoring::CuMonitoringRuntime| {
                let runtime = match &stream_monitors {
                    Some(streams) => runtime.with_log_streams(streams.clone()),
                    None => runtime,
                };
                #mission_mod::monitor_instanciator(config, metadata, runtime)
            } }
        } else {
            quote! { #mission_mod::monitor_instanciator }
        };

        let local_structured_logger_init = quote! {
            let local_structured_log_sink = ::cu29::prelude::LogStream::new(
                ::cu29::prelude::UnifiedLogType::StructuredLogLine,
                unified_logger.clone(), 4096 * 10,
            )?;
        };
        let (early_structured_logger_init, late_structured_logger_init) = if logstream_enabled {
            (
                quote! {},
                quote! {
                    #local_structured_logger_init
                    let logger_runtime = if logstream_output_required {
                        ::cu29::prelude::LoggerRuntime::init(
                            clock.clone(),
                            ::cu29::logstream::StructuredLogStream::new(local_structured_log_sink, structured_outputs),
                            None::<::cu29::prelude::NullLog>,
                        )
                    } else {
                        ::cu29::prelude::LoggerRuntime::init(clock.clone(), local_structured_log_sink, None::<::cu29::prelude::NullLog>)
                    };
                },
            )
        } else {
            (
                quote! {
                    #local_structured_logger_init
                    let logger_runtime = ::cu29::prelude::LoggerRuntime::init(
                        clock.clone(), local_structured_log_sink, None::<::cu29::prelude::NullLog>,
                    );
                },
                quote! {},
            )
        };

        let build_with_resources_fn = quote! {
            #build_with_resources_sig {
                let AppResources {
                    config,
                    config_source,
                    resources,
                    #build_with_resources_thread_pools_destructure
                } = app_resources;

                #early_structured_logger_init

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
                let local_copperlist_sink = stream_write::<#mission_mod::CuList, S>(
                    unified_logger.clone(),
                    UnifiedLogType::CopperList,
                    default_section_size,
                    // the 2 sizes are not directly related as we encode the CuList but we can
                    // assume the encoded size is close or lower than the non encoded one
                    // This is to be sure we have the size of at least a Culist and some.
                )?;
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: copperlist stream ready");

                #local_keyframe_sink_init

                #copperlist_output_graph
                #late_structured_logger_init

                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: creating runtime lifecycle stream");
                let effective_config_ron = config
                    .serialize_ron()
                    .unwrap_or_else(|_| "<failed to serialize config>".to_string());
                let mut local_lifecycle_sink = stream_write::<RuntimeLifecycleRecord, S>(
                    unified_logger.clone(),
                    UnifiedLogType::RuntimeLifecycle,
                    effective_config_ron.len().saturating_add(1024 * 64),
                )?;
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
                local_lifecycle_sink.log(&RuntimeLifecycleRecord {
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
                        #monitor_instanciator,
                        #mission_mod::bridges_instanciator,
                    ),
                    copperlist_sink,
                    keyframe_sink,
                    output_requirements,
                )
                .with_subsystem(#application_name::subsystem())
                .with_instance_id(instance_id)
                .with_resources(resources)
                #build_with_resources_thread_pools_call
                .build()?;
                #[cfg(target_os = "none")]
                ::cu29::prelude::info!("CuApp new: runtime built");

                let application = Ok(#application_name {
                    copper_runtime,
                    runtime_lifecycle_sink: Some(Box::new(local_lifecycle_sink)),
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
                    #(#task_debug_state_registration_calls)*
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
                    let timestamp = self.copper_runtime.clock_ref().now();
                    let Some(sink) = self.runtime_lifecycle_sink.as_mut() else {
                        return Err(CuError::from("Runtime lifecycle stream is not initialized"));
                    };
                    sink.log(&RuntimeLifecycleRecord { timestamp, event })
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

                fn debug_state_type_path(task_id: &str) -> Option<&'static str> {
                    match task_id {
                        #(#task_debug_state_type_path_arms)*
                        _ => None,
                    }
                }

                fn with_debug_state<R>(
                    &self,
                    task_id: &str,
                    f: impl FnOnce(&dyn cu29::reflect::Reflect) -> R,
                ) -> Option<R> {
                    match task_id {
                        #(#task_debug_state_read_arms)*
                        _ => None,
                    }
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

                    #[allow(deprecated)] // replays via the deprecated raw iteration on purpose
                    fn replay_recorded_copperlist(
                        &mut self,
                        clock_mock: &RobotClockMock,
                        copperlist: &CopperList<Self::RecordedDataSet>,
                        keyframe: Option<&KeyFrame>,
                    ) -> CuResult<()> {
                        cu29::continuity::validate_replay_continuity(
                            self.copper_runtime_mut().copperlists_manager.next_cl_id(),
                            copperlist.id,
                            keyframe.map(|frame| frame.culistid),
                        )?;
                        let timestamp = if let Some(keyframe) = keyframe {
                            if !self.copper_runtime_mut().captures_keyframe(copperlist.id) {
                                return Err(CuError::from(format!(
                                    "CopperList {} is not configured to capture a keyframe in this runtime",
                                    copperlist.id
                                )));
                            }
                            keyframe.timestamp
                        } else {
                            cu29::simulation::recorded_copperlist_timestamp(copperlist)
                                .ok_or_else(|| CuError::from(format!(
                                    "Recorded copperlist {} has no process_time.start timestamps",
                                    copperlist.id
                                )))?
                        };
                        // Recorded replay runs offline. Drain the previous output
                        // and align allocation only after validating this boundary.
                        self.copper_runtime_mut().copperlists_manager
                            .prepare_recorded_replay(copperlist.id)?;
                        self.copper_runtime_mut().keyframes_manager.finish_pending()?;
                        if let Some(keyframe) = keyframe {
                            self.copper_runtime_mut().set_forced_keyframe_timestamp(timestamp);
                            self.copper_runtime_mut().lock_keyframe(keyframe);
                        }
                        clock_mock.set_value(timestamp.as_nanos());

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
                        builder.with_sim_callback(&mut noop).build_impl()
                    }
                }
            })
        } else {
            None
        };

        let (builder_build_thread_pools_stmt, builder_build_thread_pools_init) = if std {
            (
                quote! {
                    let thread_pools = #mission_mod::thread_pools_instanciator(&config)?;
                },
                quote! { thread_pools, },
            )
        } else {
            (quote!(), quote!())
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

        let builder_logstream_field = logstream_enabled.then(|| {
            quote! {
                logstream: Option<(
                    Box<dyn ::cu29::logstream::CuStreamTx>,
                    ::cu29::logstream::LogStreamSenderConfig,
                )>,
            }
        });
        let builder_logstream_init = logstream_enabled.then(|| quote! { logstream: None, });
        let builder_logstream_copy =
            logstream_enabled.then(|| quote! { logstream: self.logstream, });
        let builder_with_logstream_method = logstream_enabled.then(|| {
            quote! {
                /// Adds a nonblocking CopperList plus keyframe/recovery point packet resource.
                ///
                /// One sender worker owns the endpoint and schedules both semantic lanes
                /// against the same budget. Physical pacing uses a running RobotClock,
                /// even when the application's clock is mocked.
                pub fn with_logstream<T>(
                    mut self,
                    transport: T,
                    config: ::cu29::logstream::LogStreamSenderConfig,
                ) -> Self
                where
                    T: ::cu29::logstream::CuStreamTx + 'static,
                {
                    self.logstream = Some((Box::new(transport), config));
                    self
                }
            }
        });

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
                        #builder_logstream_field
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
                            #builder_logstream_init
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
                        #builder_logstream_field
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
                            #builder_logstream_init
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
                            #[deprecated(
                                since = "1.2.0",
                                note = "use the typed lifecycle handle returned by `build()` instead"
                            )]
                            #[allow(deprecated)] // forwards to the deprecated raw trait method
                            pub fn start_all_tasks(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::start_all_tasks(self, sim_callback)
                            }
                            #[deprecated(
                                since = "1.2.0",
                                note = "use the typed lifecycle handle returned by `build()` instead"
                            )]
                            #[allow(deprecated)] // forwards to the deprecated raw trait method
                            pub fn run_one_iteration(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::run_one_iteration(self, sim_callback)
                            }
                            #[deprecated(
                                since = "1.2.0",
                                note = "use the typed lifecycle handle returned by `build()` instead"
                            )]
                            #[allow(deprecated)] // forwards to the deprecated raw trait method
                            pub fn run(&mut self, sim_callback: &mut impl FnMut(SimStep) -> SimOverride) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::run(self, sim_callback)
                            }
                            #[deprecated(
                                since = "1.2.0",
                                note = "use the typed lifecycle handle returned by `build()` instead"
                            )]
                            #[allow(deprecated)] // forwards to the deprecated raw trait method
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
                            #[deprecated(
                                since = "1.2.0",
                                note = "use the typed lifecycle handle returned by `build()` instead"
                            )]
                            #[allow(deprecated)] // forwards to the deprecated raw trait method
                            pub fn start_all_tasks(&mut self) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::start_all_tasks(self)
                            }
                            #[deprecated(
                                since = "1.2.0",
                                note = "use the typed lifecycle handle returned by `build()` instead"
                            )]
                            #[allow(deprecated)] // forwards to the deprecated raw trait method
                            pub fn run_one_iteration(&mut self) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::run_one_iteration(self)
                            }
                            #[deprecated(
                                since = "1.2.0",
                                note = "use the typed lifecycle handle returned by `build()` instead"
                            )]
                            #[allow(deprecated)] // forwards to the deprecated raw trait method
                            pub fn run(&mut self) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::run(self)
                            }
                            #[deprecated(
                                since = "1.2.0",
                                note = "use the typed lifecycle handle returned by `build()` instead"
                            )]
                            #[allow(deprecated)] // forwards to the deprecated raw trait method
                            pub fn stop_all_tasks(&mut self) -> CuResult<()> {
                                <Self as #app_trait<MmapSectionStorage, UnifiedLoggerWrite>>::stop_all_tasks(self)
                            }
                        }
            })
        } else {
            None // if no-std, let the user figure our the correct logger type they need to provide anyway.
        };

        let (builder_build_app_return, builder_build_app_wrap) = if sim_mode {
            (
                quote! { cu29::prelude::app::CuSimAppLifecycle<S, L, #application_name> },
                quote! { cu29::prelude::app::CuSimAppLifecycle },
            )
        } else {
            (
                quote! { cu29::prelude::app::CuAppLifecycle<S, L, #application_name> },
                quote! { cu29::prelude::app::CuAppLifecycle },
            )
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
                        #builder_logstream_copy
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
                        #builder_logstream_copy
                        _storage: core::marker::PhantomData,
                    }
                }

                #builder_with_config_method
                #builder_with_log_path_method
                #builder_sim_callback_method
                #builder_with_logstream_method

                /// Builds the application wrapped in its compile-time checked
                /// lifecycle, in the `Initialized` state: start it with
                /// `start()` or drive the full cycle with `run_until_shutdown()`.
                /// The pre-typestate lifecycle methods remain callable on the
                /// returned handle (with deprecation warnings) until Copper 2.0.
                #[allow(dead_code)]
                pub fn build(self) -> CuResult<#builder_build_app_return> {
                    Ok(#builder_build_app_wrap::new(self.build_impl()?))
                }

                fn build_impl(self) -> CuResult<#application_name> {
                    let clock = self
                        .clock
                        .ok_or(CuError::from("Clock missing from builder"))?;
                    let (config, config_source) = #builder_prepare_config_call;
                    let resources = (self.resources_factory)(&config)?;
                    #builder_build_thread_pools_stmt
                    let app_resources = AppResources {
                        config,
                        config_source,
                        resources,
                        #builder_build_thread_pools_init
                    };
                    #application_name::build_with_resources(
                        clock,
                        self.unified_logger,
                        app_resources,
                        self.instance_id,
                        #builder_logstream_arg
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
                use cu29::simulation::CuSimSrcTaskPack;
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
                let _ = (resources, thread_pools);
                Ok(())
            }
        } else {
            quote! { Ok(( #(#task_sim_instances_init_code),*, )) }
        };

        let sim_tasks_instanciator = if sim_mode {
            Some(quote! {
                pub fn tasks_instanciator_sim<'c>(
                    all_instances_configs: Vec<Option<&'c ComponentConfig>>,
                    resources: &mut ResourceManager,
                    thread_pools: &[Option<Arc<ThreadPool>>],
                ) -> CuResult<CuSimTasks> {
                    #sim_inst_body
            }})
        } else {
            None
        };

        let tasks_inst_body_std = if task_instances_init_code.is_empty() {
            quote! {
                let _ = (resources, thread_pools);
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
                    thread_pools: &[Option<Arc<ThreadPool>>],
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

        // Build the rayon thread pools declared under `runtime.thread_pools`,
        // indexed positionally to the config's pool order. Reserved pool ids
        // (such as `"rt"`, applied directly to parallel-rt stage workers) leave
        // a `None` slot so background-task pool indices stay aligned.
        let thread_pools_instanciator = if std {
            quote! {
                pub fn thread_pools_instanciator(
                    config: &CuConfig,
                ) -> CuResult<Vec<Option<Arc<ThreadPool>>>> {
                    let Some(runtime) = config.runtime.as_ref() else {
                        return Ok(Vec::new());
                    };
                    let mut pools: Vec<Option<Arc<ThreadPool>>> =
                        Vec::with_capacity(runtime.thread_pools.len());
                    for pool_spec in &runtime.thread_pools {
                        if pool_spec.id == cu29::config::RT_POOL {
                            pools.push(None);
                            continue;
                        }
                        let pool = cu29::thread_pool::build_pool(pool_spec)?;
                        pools.push(Some(Arc::new(pool)));
                    }
                    Ok(pools)
                }
            }
        } else {
            quote! {}
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
                use cu29::__private::sync::{Arc, Mutex};
                use std::fmt::{Debug, Formatter};
                use std::fmt::Result as FmtResult;
                use std::mem::size_of;
                use std::boxed::Box;
                use std::sync::atomic::{AtomicBool, Ordering};
            }
        } else {
            quote! {
                use alloc::boxed::Box;
                use alloc::string::String;
                use alloc::string::ToString;
                use cu29::__private::sync::{Arc, Mutex};
                use core::sync::atomic::{AtomicBool, Ordering};
                use core::fmt::{Debug, Formatter};
                use core::fmt::Result as FmtResult;
                use core::mem::size_of;
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

                #mission_constant_contents

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
                use cu29::cutask::CuStatelessTask;
                // Anytime tasks keep their raw type in the tuple; lifecycle
                // calls resolve through this trait import.
                #[allow(unused_imports)]
                use cu29::cutask_anytime::CuAnytimeTask;
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
                #(#autogenerated_output_warnings)*

                // One ZST per anytime node carrying its RON policy into the
                // type system; unset knobs const-fold away in AnytimeJob::check.
                #(#anytime_policy_defs)*

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
                #thread_pools_instanciator
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
                #live_replay_impl
                #recorded_replay_app_impl
                #distributed_replay_app_impl

                #std_application_impl

                #application_builder
            }

        };
        all_missions_tokens.push(mission_mod_tokens);
    }

    let default_application_tokens = if all_missions
        .iter()
        .any(|(mission_name, _)| mission_name == "default")
    {
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

    let mission_module_names = all_missions
        .iter()
        .map(|(mission, _)| {
            parse_str::<Ident>(mission)
                .expect("Could not make an identifier of the mission name")
                .unraw()
                .to_string()
        })
        .collect::<BTreeSet<_>>();
    let root_constant_modules = constant_modules.root_modules_except(&mission_module_names);

    let result: proc_macro2::TokenStream = quote! {
        #root_constant_modules
        #(#all_missions_tokens)*
        #default_application_tokens
    };

    result.into()
}
