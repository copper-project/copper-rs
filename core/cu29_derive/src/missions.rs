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
    let supervisor = format_ident!("{app}Supervisor");
    quote! {
        /// Configures a supervisor for the missions declared by this app.
        #vis type #supervisor<'a, S = #pre::NoopSectionStorage, L = #pre::NoopLogger> =
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
                match self { #(Self::#variants => Ok(#active::#variants(#modules::start_supervised::<S,L>(settings,logger)?))),* }
            }
        }
    }
}
