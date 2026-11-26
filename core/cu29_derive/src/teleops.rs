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
    if !item.generics.params.is_empty() {
        return Err(syn::Error::new_spanned(
            item,
            "expected a concrete teleops schema",
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
    let aliases = ids
        .iter()
        .map(|i| format_ident!("{}Source", i.to_string().to_case(Case::Pascal)));
    let count = ids.len();
    let indices: Vec<_> = (0..count).collect();
    let ops = quote!(::cu29::teleops);
    Ok(quote! {
        #vis mod #module {
            use super::*;
            /// One submission for the displayed robot CopperList. Omitted fields have no new input.
            #[derive(Default)]
            pub struct Update { #(pub #ids: <#types as #ops::Channel>::Batch,)* }
            impl ::cu29::bincode::Encode for Update {
                fn encode<E: ::cu29::bincode::enc::Encoder>(&self,e:&mut E)->Result<(),::cu29::bincode::error::EncodeError>{
                    #(self.#ids.encode(e)?;)* Ok(())
                }
            }
            impl ::cu29::bincode::Decode<()> for Update {
                fn decode<D: ::cu29::bincode::de::Decoder<Context=()>>(d:&mut D)->Result<Self,::cu29::bincode::error::DecodeError>{
                    Ok(Self { #(#ids: ::cu29::bincode::Decode::decode(d)?,)* })
                }
            }
            pub struct Operator<S>{pub sampler:#ops::InputSampler,pub(super) sender:S,pub(super) last_sent:[Option<u64>;#count]}
            impl<S:#ops::Submit> Operator<S>{
                /// Send fresh samples together, suppressing repeated continuous inputs for this frame.
                pub fn submit(&mut self,observed:u64,update:&mut Update)->Result<(),::cu29::logstream::CuStreamTxError>{

                    let mut empty = true;
                    #(if <#types as #ops::Channel>::ONE_PER_CL && self.last_sent[#indices].is_some_and(|last| observed <= last) {
                        update.#ids = Default::default();
                    }
                    if <#types as #ops::Channel>::sample_count(&update.#ids) != 0 {
                        empty = false;

                    })*
                    if !empty {
                        self.sender.submit(observed,update)?;
                        #(if <#types as #ops::Channel>::ONE_PER_CL
                            && <#types as #ops::Channel>::sample_count(&update.#ids) != 0 {
                            self.last_sent[#indices] = Some(observed);
                        })*
                    }
                    Ok(())
                }
            }
            pub struct Publishers {#(pub #ids:#ops::Publisher<#types>),*}
            pub struct Inboxes {#(pub #ids:#ops::Inbox<<#types as #ops::Channel>::Payload>),*}
            #(pub type #aliases=#ops::Source<<#types as #ops::Channel>::Payload>;)*
        }
        impl #ops::Teleops for #name{
            type Operator<S:#ops::Submit>=#module::Operator<S>;
            fn operator<S:#ops::Submit>(sender:S,synced_clock:Option<::cu29::clock::RobotClock>)->Self::Operator<S>{#module::Operator{sender,sampler:#ops::InputSampler::new(synced_clock),last_sent:[None;#count]}}
        }
        impl #name {
            pub fn inboxes()->::cu29::CuResult<(#module::Publishers,#module::Inboxes)>{
                #(let #ids=#ops::Publisher::<#types>::pair()?;)*
                Ok((#module::Publishers{#(#ids:#ids.0),*},#module::Inboxes{#(#ids:#ids.1),*}))
            }
        }
    })
}
