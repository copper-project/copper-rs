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
    let mut digest_member = format_ident!("schema_digest");
    while ids.iter().any(|id| **id == digest_member) {
        digest_member = format_ident!("{digest_member}_");
    }
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
            pub(super) fn schema_id()-> ::cu29::CuResult<[u8;32]> {
                #ops::schema_id(&[#ops::CatalogMission { slots:&[#(#ops::CatalogSlot {
                    task_id:stringify!(#ids), msg_type:stringify!(#types), payload:Some(<#types as #ops::Channel>::WIRE_TYPE)
                }),*] }], &[#((core::any::type_name::<#types>(), <#types as #ops::Channel>::CAPACITY)),*])
            }
            pub struct Operator<S>{pub(super) sender:S,pub(super) schema:[u8;32],pub(super) last_sent:[Option<u64>;#count]}
            impl<S:#ops::Submit> Operator<S>{
                /// Send fresh samples together, suppressing repeated continuous inputs for this frame.
                pub fn submit(&mut self,observed:u64,update:&mut Update)->Result<(),::cu29::logstream::CuStreamTxError>{
                    let mut next = self.last_sent;
                    let mut empty = true;
                    #(if <#types as #ops::Channel>::ONE_PER_CL && next[#indices].is_some_and(|last| observed <= last) {
                        update.#ids = Default::default();
                    }
                    if <#types as #ops::Channel>::sample_count(&update.#ids) != 0 {
                        empty = false;
                        next[#indices] = Some(observed);
                    })*
                    if !empty {
                        self.sender.submit(&self.schema,observed,update)?;
                        self.last_sent = next;
                    }
                    Ok(())
                }
            }
            pub struct Publishers {pub(super) #digest_member:[u8;32],#(pub #ids:#ops::Publisher<#types>),*}
            impl #ops::Dispatch for Publishers {
                fn schema(&self)-> &[u8;32] { &self.#digest_member }
                fn dispatch(&mut self, observed:u64,tick: ::cu29::clock::CuTime,sequence:u64,bytes:&[u8])->Result<(),#ops::Rejected> {
                    let (update,len):(Update,_) = ::cu29::bincode::decode_from_slice(bytes, ::cu29::bincode::config::standard().with_limit::<1200>())
                        .map_err(|_| #ops::Rejected::Framing)?;
                    if len != bytes.len() { return Err(#ops::Rejected::Framing); }
                    #(self.#ids.check_batch(observed, sequence, &update.#ids)?;)*
                    #(self.#ids.apply_batch(observed, tick, sequence, update.#ids);)*
                    Ok(())
                }
            }
            pub struct Inboxes {#(pub #ids:#ops::Inbox<<#types as #ops::Channel>::Payload>),*}
            #(pub type #aliases=#ops::Source<<#types as #ops::Channel>::Payload>;)*
        }
        impl #ops::Teleops for #name{
            type Operator<S:#ops::Submit>=#module::Operator<S>;
            fn operator<S:#ops::Submit>(sender:S)->::cu29::CuResult<Self::Operator<S>>{Ok(#module::Operator{sender,schema:#module::schema_id()?,last_sent:[None;#count]})}
        }
        impl #name {
            pub fn inboxes()->::cu29::CuResult<(#module::Publishers,#module::Inboxes)>{
                #(let #ids=#ops::Publisher::<#types>::pair()?;)*
                Ok((#module::Publishers{#digest_member:#module::schema_id()?,#(#ids:#ids.0),*},#module::Inboxes{#(#ids:#ids.1),*}))
            }
        }
    })
}
