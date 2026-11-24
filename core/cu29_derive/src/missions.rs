//! Typed mission selection for live multi-mission applications.
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use std::collections::BTreeSet;
use syn::{Ident, Visibility};

pub(crate) fn generate(app: &Ident, vis: &Visibility, modules: &[Ident]) -> TokenStream {
    let mission = format_ident!("{app}Mission");
    let active = format_ident!("{app}RunningMission");
    let names: Vec<_> = modules.iter().map(ToString::to_string).collect();
    let mut used = BTreeSet::new();
    let variants: Option<Vec<_>> = names
        .iter()
        .map(|n| {
            let variant = crate::utils::config_id_to_enum(n);
            if !used.insert(variant.clone()) {
                return None;
            }
            syn::parse_str::<Ident>(&variant).ok()
        })
        .collect();
    let variants = variants.as_deref().unwrap_or(modules);
    let api = quote!(::cu29::app);
    let pre = quote!(::cu29::prelude);
    let result = quote!(::cu29::CuResult);
    let runner = format_ident!("{app}Runner");
    quote! {
        /// Runs the missions declared by this app.
        #vis type #runner<'a, S = #pre::NoopSectionStorage, L = #pre::NoopLogger> =
            ::cu29::supervisor::Supervisor<'a, #mission, S, L>;
        #[allow(non_camel_case_types)]
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        #vis enum #mission { #(#variants),* }

        #[allow(non_camel_case_types)]
        #[doc(hidden)]
        #vis enum #active<S, L> {
            #(#variants(#api::CuAppLifecycle<S, L, #modules::#app, #api::Running>)),*
        }
        impl<S: #pre::SectionStorage + 'static, L: #pre::UnifiedLogWrite<S> + 'static> #api::ActiveMission for #active<S,L> {
            fn run_until(&mut self, requested: impl FnMut() -> bool) -> #result<#api::CuRunExit> {
                match self { #(Self::#variants(app) => app.run_until(requested)),* }
            }
            fn wait_until_ready(&mut self, timeout: #pre::CuDuration, ready: impl FnMut() -> bool) -> ::core::result::Result<(), #api::CuWaitError> {
                match self { #(Self::#variants(app) => app.wait_until_ready(timeout, ready)),* }
            }
            fn drain_until(&mut self, timeout: #pre::CuDuration, drained: impl FnMut() -> bool) -> ::core::result::Result<(), #api::CuWaitError> {
                match self { #(Self::#variants(app) => app.drain_until(timeout, drained)),* }
            }
            fn stop(self) -> #result<()> {
                match self { #(Self::#variants(app) => app.stop().map(drop).map_err(|failed| failed.error)),* }
            }
        }
        impl ::cu29::supervisor::ConfiguredMission for #mission {
            type Active<S: #pre::SectionStorage + 'static, L: #pre::UnifiedLogWrite<S> + 'static> = #active<S,L>;
            fn activate<S: #pre::SectionStorage + 'static, L: #pre::UnifiedLogWrite<S> + 'static>(
                self, settings: &mut ::cu29::supervisor::MissionSettings<'_>,
                logger: ::cu29::__private::sync::Arc<::cu29::__private::sync::Mutex<L>>
            ) -> #result<Self::Active<S,L>> {
                match self { #(Self::#variants => Ok(#active::#variants(#modules::start_supervised::<S,L,#pre::LogStream<S,L>>(settings,logger)?))),* }
            }
            fn activate_default(
                self, settings: &mut ::cu29::supervisor::MissionSettings<'_>,
                logger: ::cu29::__private::sync::Arc<::cu29::__private::sync::Mutex<#pre::NoopLogger>>
            ) -> #result<Self::Active<#pre::NoopSectionStorage, #pre::NoopLogger>> {
                match self { #(Self::#variants => Ok(#active::#variants(#modules::start_supervised::<_,_,::cu29::curuntime::NullWriteStream>(settings,logger)?))),* }
            }

        }
    }
}
