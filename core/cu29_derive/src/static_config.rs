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
