//! Remote-debug Zenoh client.

#[cfg(target_os = "linux")]
use super::format_bytes;
use super::{
    API_VERSION, DebugRpcRequest, DebugRpcResponse, RemoteDebugPaths, RemoteDebugShmConfig,
    RemoteDebugShmRole, WireCodec, ZenohSubscriber, cu_error_map, decode_response, decode_value,
    encode_payload, keyexpr, local_client_zenoh_config, preflight_remote_debug_shm,
    validate_client_zenoh_shm_config,
};
use cu29_traits::{CuError, CuResult};
use serde_json::Value;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;
use zenoh::Config as ZenohConfig;

pub struct RemoteDebugZenohClient {
    _paths: RemoteDebugPaths,
    session: zenoh::Session,
    request_pub: zenoh::pubsub::Publisher<'static>,
    reply_sub: ZenohSubscriber,
    reply_topic: String,
    next_request_id: AtomicU64,
    codec: WireCodec,
    #[cfg(target_os = "linux")]
    shm_config: RemoteDebugShmConfig,
}

pub struct RemoteDebugZenohClientBuilder {
    paths: RemoteDebugPaths,
    client_id: String,
    codec: WireCodec,
    zenoh_config: Option<ZenohConfig>,
    shm_config: RemoteDebugShmConfig,
}

impl RemoteDebugZenohClientBuilder {
    pub fn codec(mut self, codec: WireCodec) -> Self {
        self.codec = codec;
        self
    }

    pub fn zenoh_config(mut self, zenoh_config: ZenohConfig) -> Self {
        self.zenoh_config = Some(zenoh_config);
        self
    }

    pub fn shm_config(mut self, shm_config: RemoteDebugShmConfig) -> Self {
        self.shm_config = shm_config;
        self
    }

    pub fn build(self) -> CuResult<RemoteDebugZenohClient> {
        let shm_config = self.shm_config.validate()?;
        let zenoh_config = match self.zenoh_config {
            Some(config) => config,
            None => local_client_zenoh_config(&self.paths, shm_config)?,
        };
        RemoteDebugZenohClient::open(
            zenoh_config,
            self.paths,
            &self.client_id,
            self.codec,
            shm_config,
        )
    }
}

impl RemoteDebugZenohClient {
    pub fn new(paths: RemoteDebugPaths, client_id: &str) -> CuResult<Self> {
        Self::builder(paths, client_id).build()
    }

    pub fn builder(paths: RemoteDebugPaths, client_id: &str) -> RemoteDebugZenohClientBuilder {
        RemoteDebugZenohClientBuilder {
            paths,
            client_id: client_id.to_string(),
            codec: WireCodec::Cbor,
            zenoh_config: None,
            shm_config: RemoteDebugShmConfig::default(),
        }
    }

    fn open(
        zenoh_config: ZenohConfig,
        paths: RemoteDebugPaths,
        client_id: &str,
        codec: WireCodec,
        shm_config: RemoteDebugShmConfig,
    ) -> CuResult<Self> {
        validate_client_zenoh_shm_config(&zenoh_config)?;
        preflight_remote_debug_shm(shm_config, RemoteDebugShmRole::Client)?;
        let session = zenoh::Wait::wait(zenoh::open(zenoh_config)).map_err(cu_error_map(
            "RemoteDebugClient: failed to open Zenoh session",
        ))?;

        let request_pub =
            zenoh::Wait::wait(session.declare_publisher(keyexpr(&paths.rpc_request)?)).map_err(
                cu_error_map("RemoteDebugClient: failed to declare request publisher"),
            )?;

        let reply_topic = format!("{}/rpc/reply/{client_id}", paths.base);
        let reply_sub = zenoh::Wait::wait(session.declare_subscriber(keyexpr(&reply_topic)?))
            .map_err(cu_error_map(
                "RemoteDebugClient: failed to declare reply subscriber",
            ))?;

        Ok(Self {
            _paths: paths,
            session,
            request_pub,
            reply_sub,
            reply_topic,
            next_request_id: AtomicU64::new(1),
            codec,
            #[cfg(target_os = "linux")]
            shm_config,
        })
    }

    pub fn subscribe_events(&self, topic: &str) -> CuResult<ZenohSubscriber> {
        zenoh::Wait::wait(self.session.declare_subscriber(keyexpr(topic)?)).map_err(cu_error_map(
            "RemoteDebugClient: failed to declare events subscriber",
        ))
    }

    pub fn call(
        &self,
        session_id: Option<&str>,
        method: &str,
        params: Value,
    ) -> CuResult<DebugRpcResponse> {
        let request_id = format!("req{}", self.next_request_id.fetch_add(1, Ordering::SeqCst));
        let request = DebugRpcRequest {
            api: API_VERSION.to_string(),
            request_id: request_id.clone(),
            session_id: session_id.map(ToOwned::to_owned),
            method: method.to_string(),
            params,
            reply_to: self.reply_topic.clone(),
        };

        let payload = encode_payload(
            &request,
            self.codec,
            "RemoteDebugClient: request encode failed",
        )?;
        zenoh::Wait::wait(
            self.request_pub
                .put(payload)
                .encoding(self.codec.encoding()),
        )
        .map_err(cu_error_map("RemoteDebugClient: failed to send request"))?;

        loop {
            let sample = self.reply_sub.recv().map_err(|e| {
                CuError::from(format!("RemoteDebugClient: failed receiving reply: {e}"))
            })?;
            #[cfg(target_os = "linux")]
            let payload_len = sample.payload().len();
            #[cfg(target_os = "linux")]
            let payload_is_shm = sample.payload().as_shm().is_some();
            let payload = sample.payload().to_bytes();
            let response = decode_response(payload.as_ref(), self.codec)?;

            if response.request_id == request_id {
                #[cfg(target_os = "linux")]
                if payload_len >= self.shm_config.message_size_threshold_bytes && !payload_is_shm {
                    return Err(CuError::from(format!(
                        "RemoteDebugClient refused a {} reply delivered outside shared memory. \
                         Both endpoints must have SHM enabled and enough RLIMIT_MEMLOCK; \
                         refusing silent Unix-socket fallback.",
                        format_bytes(payload_len),
                    )));
                }
                return Ok(response);
            }
        }
    }

    /// Send one RPC call while receiving events published during the operation.
    pub fn call_with_events(
        &self,
        session_id: Option<&str>,
        method: &str,
        params: Value,
        event_topic: &str,
        mut on_event: impl FnMut(Value),
    ) -> CuResult<DebugRpcResponse> {
        let event_sub = self.subscribe_events(event_topic)?;
        let request_id = format!("req{}", self.next_request_id.fetch_add(1, Ordering::SeqCst));
        let request = DebugRpcRequest {
            api: API_VERSION.to_string(),
            request_id: request_id.clone(),
            session_id: session_id.map(ToOwned::to_owned),
            method: method.to_string(),
            params,
            reply_to: self.reply_topic.clone(),
        };

        let payload = encode_payload(
            &request,
            self.codec,
            "RemoteDebugClient: request encode failed",
        )?;
        zenoh::Wait::wait(
            self.request_pub
                .put(payload)
                .encoding(self.codec.encoding()),
        )
        .map_err(cu_error_map("RemoteDebugClient: failed to send request"))?;

        loop {
            while let Some(sample) = event_sub.try_recv().map_err(|e| {
                CuError::from(format!("RemoteDebugClient: failed receiving event: {e}"))
            })? {
                let payload = sample.payload().to_bytes();
                let event = decode_value(payload.as_ref(), self.codec)?;
                if event.get("request_id").and_then(Value::as_str) == Some(request_id.as_str()) {
                    on_event(event);
                }
            }

            if let Some(sample) = self.reply_sub.try_recv().map_err(|e| {
                CuError::from(format!("RemoteDebugClient: failed receiving reply: {e}"))
            })? {
                #[cfg(target_os = "linux")]
                let payload_len = sample.payload().len();
                #[cfg(target_os = "linux")]
                let payload_is_shm = sample.payload().as_shm().is_some();
                let payload = sample.payload().to_bytes();
                let response = decode_response(payload.as_ref(), self.codec)?;
                if response.request_id != request_id {
                    continue;
                }
                #[cfg(target_os = "linux")]
                if payload_len >= self.shm_config.message_size_threshold_bytes && !payload_is_shm {
                    return Err(CuError::from(format!(
                        "RemoteDebugClient refused a {} reply delivered outside shared memory. \
                         Both endpoints must have SHM enabled and enough RLIMIT_MEMLOCK; \
                         refusing silent Unix-socket fallback.",
                        format_bytes(payload_len),
                    )));
                }
                return Ok(response);
            }

            if let Some(sample) =
                event_sub
                    .recv_timeout(Duration::from_millis(100))
                    .map_err(|e| {
                        CuError::from(format!("RemoteDebugClient: failed receiving event: {e}"))
                    })?
            {
                let payload = sample.payload().to_bytes();
                let event = decode_value(payload.as_ref(), self.codec)?;
                if event.get("request_id").and_then(Value::as_str) == Some(request_id.as_str()) {
                    on_event(event);
                }
            }
        }
    }
}
