//! Clock-reference wiring resolved against the compiled resource graph.

use super::*;

fn reference_factory(
    config: &CuConfig,
    mission: &str,
    sim_mode: bool,
) -> CuResult<proc_macro2::TokenStream> {
    let clock = config
        .runtime
        .as_ref()
        .and_then(|runtime| runtime.clock_sync_config());
    if sim_mode {
        return Ok(quote! { Ok(None) });
    }
    let Some(clock) = clock else {
        return Ok(quote! {
            if config.runtime.as_ref().and_then(|runtime| runtime.clock_sync_config()).is_some() {
                return Err(CuError::from("Clock parent differs from compiled configuration"));
            }
            Ok(None)
        });
    };
    let (bundle, slot) = parse_resource_path(&clock.parent)?;
    let bundles = build_bundle_specs(config, mission)?;
    let index = bundles
        .iter()
        .position(|spec| spec.id == bundle)
        .ok_or_else(|| {
            CuError::from(format!(
                "Clock parent bundle '{bundle}' is unavailable in mission '{mission}'"
            ))
        })?;
    let provider = &bundles[index].provider_path;
    let parent = &clock.parent;
    Ok(quote! {
        let cfg = config.runtime.as_ref().and_then(|runtime| runtime.clock_sync_config())
            .ok_or(CuError::from("Compiled clock parent is missing"))?;
        if cfg.parent != #parent { return Err(CuError::from("Clock parent differs from compiled configuration")); }
        cfg.validate()?;
        const KEY: cu29::resource::ResourceKey<()> = cu29::resource::ResourceKey::<()>::new(
            cu29::resource::BundleIndex::new(#index),
            cu29::resource::resource_index_by_name::<#provider>(#slot),
        );
        let key = KEY.typed::<<#provider as cu29::clock_sync::ClockReferenceBundle>::Reference>();
        let reference = resources.take(key)?.0;
        Ok(Some((Box::new(reference), cfg.maintenance())))
    })
}

pub(super) fn factory(
    config: &CuConfig,
    mission: &str,
    sim_mode: bool,
) -> CuResult<proc_macro2::TokenStream> {
    let body = reference_factory(config, mission, sim_mode)?;
    Ok(quote! {
        #[doc(hidden)]
        pub fn clock_reference_instanciator(
            config: &CuConfig, resources: &mut ResourceManager,
        ) -> CuResult<Option<(Box<dyn cu29::clock_sync::ClockReference>, cu29::clock_sync::MaintenanceConfig)>> {
            let _ = (&config, &resources);
            #body
        }
    })
}
