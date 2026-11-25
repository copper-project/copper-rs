//! Emit borrowed parameter tables in the application binary.
#[cfg(feature = "clock-sync")]
use cu29_runtime::config::ClockSyncConfig;
use cu29_runtime::config::{
    Config, DEFAULT_KEYFRAME_INTERVAL, LogStreamAdaptationConfig, LogStreamContinuousFecConfig,
    LogStreamDestinationConfig, LogStreamFecConfig, LogStreamFeedbackConfig, LogStreamLinkConfig,
    LogStreamObjectFecConfig, LogStreamRepairDensity, LogStreamRlcField, LogStreamTransportConfig,
    LogStreamingConfig, LoggingConfig, OnError, OwnedStorage, PlannerConfig, PlannerKind,
    RuntimeConfig, SchedulingPolicy, ThreadPoolConfig,
};
use proc_macro2::TokenStream;
use quote::{ToTokens, quote};
use ron::{Number, Value};

fn value(v: &Value) -> TokenStream {
    let ty = quote!(::cu29::__private::StaticValue);
    match v {
        Value::Bool(x) => quote!(#ty::Bool(#x)),
        Value::Char(x) => quote!(#ty::Char(#x)),
        Value::String(x) => quote!(#ty::String(#x)),
        Value::Bytes(xs) => quote!(#ty::Bytes(&[#(#xs),*])),
        Value::Unit => quote!(#ty::Unit),
        Value::Option(None) => quote!(#ty::Option(None)),
        Value::Option(Some(x)) => {
            let x = value(x);
            quote!(#ty::Option(Some(&#x)))
        }
        Value::Seq(xs) => {
            let xs = xs.iter().map(value);
            quote!(#ty::Seq(&[#(#xs),*]))
        }
        Value::Map(xs) => {
            let xs = xs.iter().map(|(k, v)| {
                let k = value(k);
                let v = value(v);
                quote!((#k,#v))
            });
            quote!(#ty::Map(&[#(#xs),*]))
        }
        Value::Number(n) => match n {
            Number::I8(x) => quote!(#ty::I8(#x)),
            Number::I16(x) => quote!(#ty::I16(#x)),
            Number::I32(x) => quote!(#ty::I32(#x)),
            Number::I64(x) => quote!(#ty::I64(#x)),
            Number::U8(x) => quote!(#ty::U8(#x)),
            Number::U16(x) => quote!(#ty::U16(#x)),
            Number::U32(x) => quote!(#ty::U32(#x)),
            Number::U64(x) => quote!(#ty::U64(#x)),
            // A const block keeps the value promotable, so the borrowed table stays `'static`
            // when it is emitted inline (monitor, bridge, codec and bundle parameters).
            Number::F32(x) => {
                let bits = x.0.to_bits();
                quote!(#ty::F32(const { f32::from_bits(#bits) }))
            }
            Number::F64(x) => {
                let bits = x.0.to_bits();
                quote!(#ty::F64(const { f64::from_bits(#bits) }))
            }
            _ => quote!(compile_error!("unsupported static-config number")),
        },
    }
}

pub(super) fn component(config: Option<&Config<OwnedStorage>>) -> TokenStream {
    let Some(config) = config else {
        return quote!(None);
    };
    let mut entries: Vec<_> = config.entries().collect();
    entries.sort_by_key(|(key, _)| *key);
    let entries = entries.iter().map(|(key, v)| {
        let v = value(v.as_ron());
        quote!((#key,#v))
    });
    quote!(Some(::cu29::config::ComponentConfig::from_static(&[#(#entries),*])))
}

pub(super) fn topology(
    config: &cu29_runtime::config::CuConfig,
    mission: &str,
) -> cu29_traits::CuResult<TokenStream> {
    use cu29_runtime::monitoring::build_monitor_topology;
    let mut topology = build_monitor_topology(config, mission)?;
    topology.nodes.sort_by(|a, b| a.id.cmp(&b.id));
    let m = quote!(::cu29::monitoring);
    let optional = |s: Option<&str>| match s {
        Some(s) => quote!(Some(#s.into())),
        None => quote!(None),
    };
    let nodes = topology.nodes.iter().map(|n| {
        let id = &n.id;
        let name = optional(n.type_name.as_deref());
        let kind = syn::Ident::new(&format!("{:?}", n.kind), proc_macro2::Span::call_site());
        let inputs = &n.inputs;
        let outputs = &n.outputs;
        quote!(#m::MonitorNode { id: #id.into(), type_name: #name, kind: #m::ComponentType::#kind,
            inputs: vec![#(#inputs.into()),*], outputs: vec![#(#outputs.into()),*] })
    });
    let connections = topology.connections.iter().map(|c| {
        let (src,dst,msg) = (&c.src,&c.dst,&c.msg);
        let src_port = optional(c.src_port.as_deref());
        let dst_port = optional(c.dst_port.as_deref());
        quote!(#m::MonitorConnection { src: #src.into(), dst: #dst.into(), msg: #msg.into(), src_port: #src_port, dst_port: #dst_port })
    });
    Ok(
        quote!(#m::MonitorTopology { nodes: vec![#(#nodes),*], connections: vec![#(#connections),*] }),
    )
}

/// Emit an expression of the field's Rust type. Struct patterns and enum
/// matches are exhaustive so schema changes are checked when building the macro.
pub(super) trait StaticExpr {
    fn expr(&self) -> TokenStream;
}

impl<T: StaticExpr> StaticExpr for Option<T> {
    fn expr(&self) -> TokenStream {
        match self {
            Some(value) => {
                let value = value.expr();
                quote!(Some(#value))
            }
            None => quote!(None),
        }
    }
}

impl<T: StaticExpr> StaticExpr for Vec<T> {
    fn expr(&self) -> TokenStream {
        let values = self.iter().map(StaticExpr::expr);
        quote!(vec![#(#values),*])
    }
}

impl StaticExpr for String {
    fn expr(&self) -> TokenStream {
        quote!(#self.into())
    }
}

macro_rules! literals {
    ($($ty:ty),*) => {$(
        impl StaticExpr for $ty {
            fn expr(&self) -> TokenStream { self.to_token_stream() }
        }
    )*};
}
literals!(usize, u16, u32, u64);

macro_rules! config_struct {
    ($name:ident { $($field:ident),* $(,)? }) => {
        impl StaticExpr for $name {
            fn expr(&self) -> TokenStream {
                let Self { $($field),* } = self;
                $(let $field = $field.expr();)*
                quote!(::cu29::config::$name { $($field: #$field),* })
            }
        }
    };
}

config_struct!(LogStreamingConfig { destinations });
config_struct!(LogStreamDestinationConfig {
    id,
    feedback,
    transport,
    link,
    fec,
    recovery_interval,
    metadata_repeat_secs,
    max_record_bytes,
});
config_struct!(LogStreamTransportConfig { type_, resource });
config_struct!(LogStreamFeedbackConfig {
    transport,
    report_interval_ms,
    timeout_ms,
    adaptation
});
config_struct!(LogStreamAdaptationConfig {
    min_repair_every_source_symbols,
    max_repair_every_source_symbols
});
config_struct!(LogStreamLinkConfig {
    mtu_bytes,
    bitrate_bps,
    memory_budget_kib,
    max_latency_ms,
    burst_packets
});
config_struct!(LogStreamFecConfig {
    continuous,
    objects
});
config_struct!(LogStreamContinuousFecConfig {
    field,
    window_symbols,
    repair_every_source_symbols,
    repair_density
});
config_struct!(LogStreamObjectFecConfig {
    max_object_bytes,
    repair_symbols_per_block
});

macro_rules! unit_enum {
    ($name:ident { $($variant:ident),* }) => {
        impl StaticExpr for $name {
            fn expr(&self) -> TokenStream {
                match self { $(Self::$variant => quote!(::cu29::config::$name::$variant)),* }
            }
        }
    };
}

unit_enum!(LogStreamRlcField { Gf2, Gf256 });
unit_enum!(PlannerKind {
    Serial,
    TaskOrder,
    ExplicitSchedule,
    Pipeline
});
impl StaticExpr for LogStreamRepairDensity {
    fn expr(&self) -> TokenStream {
        match self {
            Self::Full => quote!(::cu29::config::LogStreamRepairDensity::Full),
            Self::Threshold(n) => quote!(::cu29::config::LogStreamRepairDensity::Threshold(#n)),
        }
    }
}

impl StaticExpr for PlannerConfig {
    fn expr(&self) -> TokenStream {
        let kind = self.kind().expr();
        let config = component(self.get_config());
        quote!(::cu29::config::PlannerConfig::from_static(#kind, #config))
    }
}

unit_enum!(OnError { Warn, Strict });
impl StaticExpr for SchedulingPolicy {
    fn expr(&self) -> TokenStream {
        super::scheduling_policy_tokens(*self)
    }
}
config_struct!(ThreadPoolConfig {
    id,
    threads,
    affinity,
    policy,
    on_error
});
#[cfg(feature = "clock-sync")]
config_struct!(ClockSyncConfig {
    parent,
    max_error_ns,
    acquisition_timeout_ns,
    sample_interval_ns,
    max_age_ns,
    drift_bound_ppb,
    max_slew_ppb
});

impl StaticExpr for RuntimeConfig {
    fn expr(&self) -> TokenStream {
        let rate_target_hz = self.rate_target_hz.expr();
        let planner = self.planner.expr();
        // The macro reads config with std, even when generating bare-metal code.
        let thread_pools = cfg!(feature = "std").then(|| {
            let pools = self.thread_pools.expr();
            quote!(thread_pools: #pools,)
        });
        #[cfg(feature = "clock-sync")]
        let clock = {
            let clock = self.clock.expr();
            quote!(clock: #clock,)
        };
        #[cfg(not(feature = "clock-sync"))]
        let clock = quote!();
        // A downstream dependency can independently enable another runtime feature.
        quote!({
            #[allow(clippy::needless_update)]
            let runtime = ::cu29::config::RuntimeConfig {
                rate_target_hz: #rate_target_hz,
                planner: #planner,
                #thread_pools
                #clock
                ..Default::default()
            };
            runtime
        })
    }
}

impl StaticExpr for LoggingConfig {
    fn expr(&self) -> TokenStream {
        let Self {
            enable_task_logging,
            enable_keyframe_logging,
            copperlist_count,
            slab_size_mib,
            section_size_mib,
            keyframe_interval,
            keyframe_capacity,
            codecs: _,
        } = self;
        let copperlist_count = copperlist_count.expr();
        let slab_size_mib = slab_size_mib.expr();
        let section_size_mib = section_size_mib.expr();
        let keyframe_capacity = keyframe_capacity.expr();
        // Preserve the static loader's default for an explicitly absent interval.
        let interval = keyframe_interval.unwrap_or(DEFAULT_KEYFRAME_INTERVAL);
        // The macro emits codec registrations separately.
        quote!(::cu29::config::LoggingConfig {
            enable_task_logging: #enable_task_logging, enable_keyframe_logging: #enable_keyframe_logging,
            copperlist_count: #copperlist_count, slab_size_mib: #slab_size_mib, section_size_mib: #section_size_mib,
            keyframe_interval: Some(#interval), keyframe_capacity: #keyframe_capacity, codecs: vec![],
        })
    }
}
