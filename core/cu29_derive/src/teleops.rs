use convert_case::{Case, Casing};
use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::{Fields, ItemStruct};

pub fn expand(item: ItemStruct) -> syn::Result<TokenStream> {
    let Fields::Named(fields) = &item.fields else {
        return Err(syn::Error::new_spanned(
            item,
            "Teleops requires named channels",
        ));
    };
    if !item.generics.params.is_empty() || fields.named.len() > u16::MAX as usize {
        return Err(syn::Error::new_spanned(
            item,
            "expected a concrete teleops schema with at most 65535 channels",
        ));
    }
    let name = &item.ident;
    let module = format_ident!("{}", name.to_string().to_case(Case::Snake));
    let vis = &item.vis;
    let ids: Vec<_> = fields
        .named
        .iter()
        .map(|f| f.ident.as_ref().unwrap())
        .collect();
    let types: Vec<_> = fields.named.iter().map(|f| &f.ty).collect();
    let tags: Vec<_> = (0..ids.len() as u16).collect();
    let aliases = ids
        .iter()
        .map(|i| format_ident!("{}Source", i.to_string().to_case(Case::Pascal)));
    let ops = quote!(::cu29::teleops);
    Ok(quote! {
        #vis mod #module {
            use super::*;
            pub struct Operator<S>{pub(super) sender:S,pub(super) observed:Option<u64>}
            impl<S:#ops::Submit> Operator<S>{
                pub fn observing(&mut self,id:u64)->Operator<&mut S>{Operator{sender:&mut self.sender,observed:Some(id)}}
                #(pub fn #ids(&mut self)->#ops::Input<'_,S,#types>{#ops::Input::new(&mut self.sender,#tags,self.observed)})*
            }
            pub struct Publishers {#(pub #ids:#ops::Publisher<#types>),*}
            pub struct Inboxes {#(pub #ids:#ops::Inbox<<#types as #ops::Channel>::Payload>),*}
            #(pub type #aliases=#ops::Source<<#types as #ops::Channel>::Payload>;)*
        }
        impl #ops::Teleops for #name{
            type Operator<S:#ops::Submit>=#module::Operator<S>;
            fn operator<S:#ops::Submit>(sender:S)->Self::Operator<S>{#module::Operator{sender,observed:None}}
        }
        impl #name {
            pub fn inboxes()->::cu29::CuResult<(#module::Publishers,#module::Inboxes)>{
                #(let #ids=#ops::Publisher::<#types>::pair()?;)*
                Ok((#module::Publishers{#(#ids:#ids.0),*},#module::Inboxes{#(#ids:#ids.1),*}))
            }
        }
    })
}
