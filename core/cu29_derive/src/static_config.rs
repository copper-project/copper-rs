//! Emit borrowed parameter tables in the application binary.
use cu29_runtime::config::ComponentConfig;
use proc_macro2::TokenStream;
use quote::quote;
use ron::{Number, Value};

fn value(v: &Value) -> TokenStream {
    let ty = quote!(::cu29::config::StaticValue);
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
            Number::F32(x) => {
                let bits = x.0.to_bits();
                quote!(#ty::F32(f32::from_bits(#bits)))
            }
            Number::F64(x) => {
                let bits = x.0.to_bits();
                quote!(#ty::F64(f64::from_bits(#bits)))
            }
            _ => quote!(compile_error!("unsupported static-config number")),
        },
    }
}

pub(super) fn component(config: Option<&ComponentConfig>) -> TokenStream {
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
