use super::*;

pub(super) fn dataset_support(
    plan: &CuExecutionLoop,
    packs: &[OutputPack],
    helpers: &[Option<Ident>],
    modes: &[HandleContent],
    count: usize,
    codecs: &[Option<SlotCodecBinding>],
) -> proc_macro2::TokenStream {
    let mut capture = vec![true; packs.len()];
    let mut abis = vec![quote! { None }; packs.len()];
    let mut contracts = Vec::new();
    for unit in &plan.steps {
        let CuExecutionUnit::Step(step) = unit else {
            continue;
        };
        let policy = step.node.streaming();
        if policy.replay != cu29_runtime::config::StreamReplay::Reconstruct {
            continue;
        }
        if step.task_type != CuTaskType::Regular
            || step.node.get_flavor() != Flavor::Task
            || step.node.is_background()
            || step.node.is_anytime()
            || !step.node.is_logging_enabled()
        {
            return quote! { compile_error!("reconstruct requires an ordinary, synchronous, logged deterministic task"); };
        }
        let Some(abi) = policy.replay_abi.filter(|abi| *abi != 0) else {
            return quote! { compile_error!("reconstruct requires a nonzero replay_abi"); };
        };
        let task: Type = parse_str(step.node.get_type()).unwrap();
        contracts.push(quote! {
            const _: () = assert!(<#task as ::cu29::CuCrossPlatformDeterministic>::REPLAY_ABI == #abi,
                "task deterministic replay ABI does not match RON");
        });
        let slot = step.output_msg_pack.as_ref().unwrap().culist_index as usize;
        capture[slot] = false;
        abis[slot] = quote! { Some(#abi) };
    }
    let hybrid = capture.contains(&false);
    if hybrid
        && (cfg!(feature = "flat-copperlist-encoding")
            || codecs.iter().any(Option::is_some)
            || modes.iter().any(|mode| *mode != HandleContent::default()))
    {
        return quote! { compile_error!("hybrid streaming currently requires the lossless native compressed codec and full handle capture"); };
    }
    let encode = build_compressed_culist_tuple_encode(packs, helpers, modes, count, Some(&capture));
    let mut presence = Vec::new();
    let mut digests = Vec::new();
    let mut flat_abis = Vec::new();
    let mut metadata = Vec::new();
    for (i, pack) in packs.iter().enumerate() {
        let slot = syn::Index::from(i);
        for port in 0..pack.msg_types.len() {
            let access = if pack.is_multi() {
                let port = syn::Index::from(port);
                quote! { #slot.#port }
            } else {
                quote! { #slot }
            };
            presence.push(quote! { self.0.#access.payload().is_some() });
            digests.push(if capture[i] { quote! { None } } else {
                quote! { ::cu29::logstream::capture::reconstruction_digest(&self.0.#access.payload())? }
            });
            flat_abis.push(abis[i].clone());
            metadata.push(quote! {
                self.0.#access.tov = captured.0.#access.tov;
                self.0.#access.metadata = captured.0.#access.metadata.clone();
            });
        }
    }
    let schema = hybrid.then(|| quote! { schema.reconstruction = Self::RECONSTRUCTION.to_vec(); });
    quote! {
        #(#contracts)*
        #encode
        impl ::cu29::logstream::capture::CaptureDataSet for CuStampedDataSet {
            const RECONSTRUCTION: &'static [Option<u32>] = &[#(#flat_abis),*];
            fn original_presence(&self) -> Vec<bool> { vec![#(#presence),*] }
            fn stream_schema() -> ::cu29::logstream::ApplicationSchema {
                #[allow(unused_mut)]
                let mut schema = ::cu29::logstream::ApplicationSchema::from_output_specs(
                    <Self as MatchingTasks>::get_output_specs());
                #schema
                schema
            }
            fn encode_capture<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                self.__encode_capture(encoder)
            }
            fn capture_proof(&self) -> ::cu29::logstream::Result<::cu29::logstream::capture::CaptureProof> {
                Ok(::cu29::logstream::capture::CaptureProof {
                    version: 1,
                    original_presence: self.original_presence(),
                    reconstructed_digests: vec![#(#digests),*],
                })
            }
            fn restore_sender_metadata(&mut self, captured: &Self) { #(#metadata)* }
        }
    }
}

pub(super) fn runtime_support(
    app: &Ident,
    mission: &Ident,
    plan: &CuExecutionLoop,
    entities: &[ExecutionEntity],
) -> proc_macro2::TokenStream {
    let mut reconstruct_arms = Vec::new();
    let mut lifecycle_arms = Vec::new();
    for unit in &plan.steps {
        let CuExecutionUnit::Step(step) = unit else {
            continue;
        };
        if !matches!(
            entities[step.node_id as usize].kind,
            ExecutionEntityKind::Task { .. }
        ) || step.node.streaming().replay != cu29_runtime::config::StreamReplay::Reconstruct
        {
            continue;
        }
        let variant = Ident::new(&config_id_to_enum(&step.node.get_id()), Span::call_site());
        let slot = syn::Index::from(step.output_msg_pack.as_ref().unwrap().culist_index as usize);
        let ports = step.output_msg_pack.as_ref().unwrap().msg_types.len();
        let metadata: Vec<_> = (0..ports).map(|port| {
            let (output, recorded) = if ports == 1 { (quote! { output }, quote! { captured.msgs.0.#slot }) }
            else { let port = syn::Index::from(port); (quote! { output.#port }, quote! { captured.msgs.0.#slot.#port }) };
            quote! { #output.tov = #recorded.tov; #output.metadata = #recorded.metadata.clone(); }
        }).collect();
        let recorded = if ports == 1 {
            quote! { captured.msgs.0.#slot }
        } else {
            quote! { captured.msgs.0.#slot.0 }
        };
        reconstruct_arms.push(quote! {
            #mission::SimStep::#variant(CuTaskCallbackState::Process(_, _)) => {
                if let Some(start) = Option::<CuTime>::from(#recorded.metadata.process_time.start) {
                    clock_mock.set_value(start.as_nanos());
                }
                SimOverride::ExecuteByRuntime
            }
            #mission::SimStep::#variant(CuTaskCallbackState::ProcessCompleted(output)) => {
                #(#metadata)*
                SimOverride::ExecutedBySim
            }
            #mission::SimStep::#variant(_) => SimOverride::ExecuteByRuntime,
        });
        lifecycle_arms
            .push(quote! { #mission::SimStep::#variant(_) => SimOverride::ExecuteByRuntime, });
    }
    quote! {
        impl ::cu29::logstream::twin::LiveReplay for #app {
            type DataSet = #mission::CuStampedDataSet;
            #[allow(deprecated)]
            fn build_twin() -> CuResult<(Self, RobotClockMock)> {
                let (clock, mock) = RobotClock::mock();
                let mut config = CuConfig::deserialize_ron(&Self::original_config())?;
                if let Some(logging) = &mut config.logging {
                    logging.enable_task_logging = false;
                    logging.enable_keyframe_logging = false;
                }
                let mut app = Self::builder().with_clock(clock).with_config(config)
                    .with_sim_callback(&mut |_| SimOverride::ExecutedBySim).build()?.into_inner();
                <Self as CuSimApplication<NoopSectionStorage, NoopLogger>>::start_all_tasks(&mut app,
                    &mut |step| match step { #(#lifecycle_arms)* _ => SimOverride::ExecutedBySim })?;
                Ok((app, mock))
            }
            #[allow(deprecated)]
            fn replay_capture(&mut self, clock_mock: &RobotClockMock,
                captured: &mut CopperList<Self::DataSet>, keyframe: Option<&KeyFrame>) -> CuResult<()> {
                use ::cu29::logstream::capture::CaptureDataSet;
                cu29::continuity::validate_replay_continuity(
                    self.copper_runtime.copperlists_manager.next_cl_id(), captured.id,
                    keyframe.map(|frame| frame.culistid))?;
                let timestamp = keyframe.map(|frame| frame.timestamp)
                    .or_else(|| cu29::simulation::recorded_copperlist_timestamp(captured))
                    .ok_or_else(|| CuError::from("capture has no sender timestamp"))?;
                self.copper_runtime.copperlists_manager.prepare_recorded_replay(captured.id)?;
                self.copper_runtime.keyframes_manager.finish_pending()?;
                if let Some(frame) = keyframe {
                    <Self as CuSimApplication<NoopSectionStorage, NoopLogger>>::restore_keyframe(self, frame)?;
                    self.copper_runtime.set_forced_keyframe_timestamp(timestamp);
                    self.copper_runtime.lock_keyframe(frame);
                }
                clock_mock.set_value(timestamp.as_nanos());
                let mut completed = false;
                let mut callback = |step: #mission::SimStep<'_>| -> SimOverride { match step {
                    #mission::SimStep::CopperListCompleted(list) => {
                        list.msgs.restore_sender_metadata(&captured.msgs);
                        core::mem::swap(&mut captured.msgs, &mut list.msgs);
                        completed = true;
                        SimOverride::ExecutedBySim
                    }
                    #(#reconstruct_arms)*
                    other => {
                        let result = #mission::recorded_replay_step(other, captured);
                        if result == SimOverride::ExecuteByRuntime { SimOverride::ExecutedBySim } else { result }
                    }
                }};
                <Self as CuSimApplication<NoopSectionStorage, NoopLogger>>::run_one_iteration(self, &mut callback)?;
                if !completed { return Err(CuError::from("live replay aborted before completing the CopperList")); }
                Ok(())
            }
        }
    }
}
