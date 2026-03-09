//! SOME/IP Router — dispatches messages by service/method ID.
//!
//! The router acts as a middleware CuTask: it inspects incoming SOME/IP
//! messages and tags them for dispatch. In a simple setup, it can serve
//! as an echo/handler for known service IDs.

use cu29::prelude::*;
use cu_automotive_payloads::someip::{
    SomeIpMessage, SomeIpReturnCode,
};

/// Maximum number of registered services.
const MAX_SERVICES: usize = 16;

/// A registered service handler entry.
#[derive(Clone, Debug, Default)]
struct ServiceEntry {
    service_id: u16,
    #[allow(dead_code)]
    instance_id: u16,
    active: bool,
}

/// SOME/IP Router task — dispatches incoming requests and generates responses.
///
/// For each incoming request, the router checks if the service/method is
/// registered. If so, it produces a response (echo or configured). Otherwise
/// it produces an error response (UnknownService).
///
/// # Config
/// - `services` (String): Comma-separated list of service IDs to handle, e.g. `"0x0100,0x0200"`.
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct SomeIpRouter {
    services: [ServiceEntry; MAX_SERVICES],
    service_count: usize,
}

impl Freezable for SomeIpRouter {}

impl CuTask for SomeIpRouter {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(SomeIpMessage);
    type Output<'m> = output_msg!(SomeIpMessage);

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let mut services: [ServiceEntry; MAX_SERVICES] =
            core::array::from_fn(|_| ServiceEntry::default());
        let mut count = 0;

        if let Some(cfg) = config {
            if let Ok(Some(svc_str)) = cfg.get::<alloc::string::String>("services") {
                for part in svc_str.split(',') {
                    let trimmed = part.trim();
                    let id = if let Some(hex) = trimmed.strip_prefix("0x") {
                        u16::from_str_radix(hex, 16).unwrap_or(0)
                    } else {
                        trimmed.parse::<u16>().unwrap_or(0)
                    };
                    if id > 0 && count < MAX_SERVICES {
                        services[count] = ServiceEntry {
                            service_id: id,
                            instance_id: 1,
                            active: true,
                        };
                        count += 1;
                    }
                }
            }
        }

        Ok(Self {
            services,
            service_count: count,
        })
    }

    fn process<'i, 'o>(
        &mut self,
        ctx: &CuContext,
        input: &Self::Input<'i>,
        output: &mut Self::Output<'o>,
    ) -> CuResult<()> {
        if let Some(msg) = input.payload() {
            // Only process requests
            if !msg.header.is_request() {
                return Ok(());
            }

            // Check if service is registered
            let known = self.services[..self.service_count]
                .iter()
                .any(|s| s.active && s.service_id == msg.header.service_id);

            let response = if known {
                // Echo back the payload as a response (placeholder behavior)
                SomeIpMessage::response(msg, msg.payload_data())
            } else {
                SomeIpMessage::error(msg, SomeIpReturnCode::UnknownService)
            };

            output.set_payload(response);
            output.tov = Tov::Time(ctx.now());
        }
        Ok(())
    }
}
