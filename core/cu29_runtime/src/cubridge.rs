//! Typed bridge traits and helpers used to connect Copper to external components both as a sink
//! and a source.
//!

use crate::config::ComponentConfig;
use crate::cutask::{CuMsg, CuMsgPayload, Freezable};
use core::marker::PhantomData;
use cu29_clock::RobotClock;
use cu29_traits::CuResult;

#[cfg(not(feature = "std"))]
use alloc::borrow::Cow;

#[cfg(feature = "std")]
use std::borrow::Cow;

#[cfg(not(feature = "std"))]
mod imp {
    pub use alloc::fmt::{Debug, Formatter};
    pub use alloc::string::String;
}

#[cfg(feature = "std")]
mod imp {
    pub use std::fmt::{Debug, Formatter};
}

use imp::*;

/// Compile-time description of a single bridge channel, including the message type carried on it.
///
/// This links its identifier to a payload type enforced at compile time and optionally provides a
/// backend-specific default route/topic/path suggestion. Missions can override that default (or
/// leave it unset) via the bridge configuration file.
#[derive(Copy, Clone)]
pub struct BridgeChannel<Id, Payload> {
    /// Strongly typed identifier used to select this channel.
    pub id: Id,
    /// Backend-specific route/topic/path default the bridge should bind to, if any.
    pub default_route: Option<&'static str>,
    _payload: PhantomData<fn() -> Payload>,
}

impl<Id, Payload> BridgeChannel<Id, Payload> {
    /// Declares a channel that leaves the route unspecified and entirely configuration-driven.
    pub const fn new(id: Id) -> Self {
        Self {
            id,
            default_route: None,
            _payload: PhantomData,
        }
    }

    /// Declares a channel with a default backend route.
    pub const fn with_channel(id: Id, route: &'static str) -> Self {
        Self {
            id,
            default_route: Some(route),
            _payload: PhantomData,
        }
    }

    /// Declares a channel with a default backend route.
    pub const fn with_channel(id: Id, route: &'static str) -> Self {
        Self {
            id,
            default_route: Some(route),
            _payload: PhantomData,
        }
    }

    /// Backward-compatibility helper for older code; prefer [`BridgeChannel::with_channel`].
    #[deprecated(since = "0.11.0", note = "Use BridgeChannel::with_channel instead")]
    pub const fn with_default_route(id: Id, route: &'static str) -> Self {
        Self::with_channel(id, route)
    }
}

impl<Id: Debug, Payload> Debug for BridgeChannel<Id, Payload> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("BridgeChannel")
            .field("id", &self.id)
            .field("default_route", &self.default_route)
            .finish()
    }
}

/// Type-erased metadata exposed for channel enumeration and configuration.
pub trait BridgeChannelInfo<Id: Copy> {
    /// Logical identifier referencing this channel inside the graph.
    fn id(&self) -> Id;
    /// Default backend-specific route/topic/path the bridge recommends binding to.
    fn default_route(&self) -> Option<&'static str>;
}

impl<Id: Copy, Payload> BridgeChannelInfo<Id> for BridgeChannel<Id, Payload> {
    fn id(&self) -> Id {
        self.id
    }

    fn default_route(&self) -> Option<&'static str> {
        self.default_route
    }
}

/// Static metadata describing a channel. Used to pass configuration data at runtime without
/// leaking the channel's payload type.
#[derive(Copy, Clone, Debug)]
pub struct BridgeChannelDescriptor<Id: Copy> {
    /// Strongly typed identifier used to select this channel.
    pub id: Id,
    /// Backend-specific default route/topic/path the bridge suggests binding to.
    pub default_route: Option<&'static str>,
}

impl<Id: Copy> BridgeChannelDescriptor<Id> {
    pub const fn new(id: Id, default_route: Option<&'static str>) -> Self {
        Self { id, default_route }
    }
}

impl<Id: Copy, T> From<&T> for BridgeChannelDescriptor<Id>
where
    T: BridgeChannelInfo<Id> + ?Sized,
{
    fn from(channel: &T) -> Self {
        BridgeChannelDescriptor::new(channel.id(), channel.default_route())
    }
}

/// Runtime descriptor that includes the parsed per-channel configuration.
#[derive(Clone, Debug)]
pub struct BridgeChannelConfig<Id: Copy> {
    /// Static metadata describing this channel.
    pub channel: BridgeChannelDescriptor<Id>,
    /// Optional route override supplied by the mission configuration.
    pub route: Option<String>,
    /// Optional configuration block defined for this channel.
    pub config: Option<ComponentConfig>,
}

impl<Id: Copy> BridgeChannelConfig<Id> {
    /// Creates a descriptor by combining the static metadata and the parsed configuration.
    pub fn from_static<T>(
        channel: &'static T,
        route: Option<String>,
        config: Option<ComponentConfig>,
    ) -> Self
    where
        T: BridgeChannelInfo<Id> + ?Sized,
    {
        Self {
            channel: channel.into(),
            route,
            config,
        }
    }

    /// Returns the route active for this channel (configuration override wins over defaults).
    pub fn effective_route(&self) -> Option<Cow<'_, str>> {
        if let Some(route) = &self.route {
            Some(Cow::Borrowed(route.as_str()))
        } else {
            self.channel.default_route.map(Cow::Borrowed)
        }
    }
}

/// Describes a set of channels for one direction (Tx or Rx) of the bridge.
///
/// This trait is implemented at compile time by Copper from the configuration.
/// Implementations typically expose one `BridgeChannel<Id, Payload>` constant per logical channel and
/// list them through `STATIC_CHANNELS` so the runtime can enumerate the available endpoints.
pub trait BridgeChannelSet {
    /// Enumeration identifying each channel in this set.
    type Id: Copy + Eq + 'static;

    /// Compile-time metadata describing all channels in this set.
    const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>];
}

/// Public trait implemented by every copper bridge.
///
/// A bridge behaves similarly to set of [`crate::cutask::CuSrcTask`] /
/// [`crate::cutask::CuSinkTask`], but it owns the shared transport state and knows how to
/// multiplex multiple channels on a single backend (serial, CAN, middleware, …).
pub trait CuBridge: Freezable {
    /// Outgoing channels (Copper -> external world).
    type Tx: BridgeChannelSet;
    /// Incoming channels (external world -> Copper).
    type Rx: BridgeChannelSet;

    /// Constructs a new bridge.
    ///
    /// The runtime passes the bridge-level configuration plus the per-channel descriptors
    /// so the implementation can cache settings such as QoS, IDs, baud rates, etc.
    fn new(
        config: Option<&ComponentConfig>,
        tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
    ) -> CuResult<Self>
    where
        Self: Sized;

    /// Called before the first send/receive cycle.
    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Gives the bridge a chance to prepare buffers before I/O.
    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Sends a message on the selected outbound channel.
    fn send<'a, Payload>(
        &mut self,
        clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a;

    /// Receives a message from the selected inbound channel.
    ///
    /// Implementations should write into `msg` when data is available.
    fn receive<'a, Payload>(
        &mut self,
        clock: &RobotClock,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a;

    /// Called once the send/receive pair completed.
    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    /// Notifies the bridge that no more I/O will happen until a subsequent [`start`].
    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! __cu29_bridge_channel_ctor {
    ($id:ident, $variant:ident, $payload:ty) => {
        $crate::cubridge::BridgeChannel::<$id, $payload>::new($id::$variant)
    };
    ($id:ident, $variant:ident, $payload:ty, $route:expr) => {
        $crate::cubridge::BridgeChannel::<$id, $payload>::with_channel($id::$variant, $route)
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __cu29_define_bridge_channels {
    (
        @accum
        $vis:vis struct $channels:ident : $id:ident
        [ $($parsed:tt)+ ]
    ) => {
        $crate::__cu29_emit_bridge_channels! {
            $vis struct $channels : $id { $($parsed)+ }
        }
    };
    (
        @accum
        $vis:vis struct $channels:ident : $id:ident
        [ ]
    ) => {
        compile_error!("tx_channels!/rx_channels! require at least one channel");
    };
    (
        @accum
        $vis:vis struct $channels:ident : $id:ident
        [ $($parsed:tt)* ]
        $(#[$chan_meta:meta])* $const_name:ident : $variant:ident => $payload:ty $(= $route:expr)? , $($rest:tt)*
    ) => {
        $crate::__cu29_define_bridge_channels!(
            @accum
            $vis struct $channels : $id
            [
                $($parsed)*
                $(#[$chan_meta])* $const_name : $variant => $payload $(= $route)?,
            ]
            $($rest)*
        );
    };
    (
        @accum
        $vis:vis struct $channels:ident : $id:ident
        [ $($parsed:tt)* ]
        $(#[$chan_meta:meta])* $const_name:ident : $variant:ident => $payload:ty $(= $route:expr)?
    ) => {
        $crate::__cu29_define_bridge_channels!(
            @accum
            $vis struct $channels : $id
            [
                $($parsed)*
                $(#[$chan_meta])* $const_name : $variant => $payload $(= $route)?,
            ]
        );
    };
    (
        @accum
        $vis:vis struct $channels:ident : $id:ident
        [ $($parsed:tt)* ]
        $(#[$chan_meta:meta])* $name:ident => $payload:ty $(= $route:expr)? , $($rest:tt)*
    ) => {
        $crate::__cu29_paste! {
            $crate::__cu29_define_bridge_channels!(
                @accum
                $vis struct $channels : $id
                [
                    $($parsed)*
                    $(#[$chan_meta])* [<$name:snake:upper>] : [<$name:camel>] => $payload $(= $route)?,
                ]
                $($rest)*
            );
        }
    };
    (
        @accum
        $vis:vis struct $channels:ident : $id:ident
        [ $($parsed:tt)* ]
        $(#[$chan_meta:meta])* $name:ident => $payload:ty $(= $route:expr)?
    ) => {
        $crate::__cu29_paste! {
            $crate::__cu29_define_bridge_channels!(
                @accum
                $vis struct $channels : $id
                [
                    $($parsed)*
                    $(#[$chan_meta])* [<$name:snake:upper>] : [<$name:camel>] => $payload $(= $route)?,
                ]
            );
        }
    };
    (
        $vis:vis struct $channels:ident : $id:ident {
            $($body:tt)*
        }
    ) => {
        $crate::__cu29_define_bridge_channels!(
            @accum
            $vis struct $channels : $id
            []
            $($body)*
        );
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __cu29_emit_bridge_channels {
    (
        $vis:vis struct $channels:ident : $id:ident {
            $(
                $(#[$chan_meta:meta])*
                $const_name:ident : $variant:ident => $payload:ty $(= $route:expr)?,
            )+
        }
    ) => {
        #[derive(Copy, Clone, Debug, Eq, PartialEq, ::serde::Serialize, ::serde::Deserialize)]
        #[repr(usize)]
        #[serde(rename_all = "snake_case")]
        $vis enum $id {
            $(
                $variant,
            )+
        }

        impl $id {
            /// Returns the zero-based ordinal for this channel (macro order).
            pub const fn as_index(self) -> usize {
                self as usize
            }
        }

        $vis struct $channels;

        #[allow(non_upper_case_globals)]
        impl $channels {
            $(
                $(#[$chan_meta])*
                $vis const $const_name: $crate::cubridge::BridgeChannel<$id, $payload> =
                    $crate::__cu29_bridge_channel_ctor!(
                        $id, $variant, $payload $(, $route)?
                    );
            )+
        }

        impl $crate::cubridge::BridgeChannelSet for $channels {
            type Id = $id;

            const STATIC_CHANNELS: &'static [&'static dyn $crate::cubridge::BridgeChannelInfo<Self::Id>] =
                &[
                    $(
                        &Self::$const_name,
                    )+
                ];
        }
    };
}

/// Declares the transmit channels of a [`CuBridge`] implementation.
///
/// # Examples
///
/// ```
/// # use cu29_runtime::tx_channels;
/// # struct EscCommand;
/// tx_channels! {
///     esc0 => EscCommand,
///     esc1 => EscCommand = "motor/esc1",
/// }
/// ```
///
/// ```
/// # use cu29_runtime::tx_channels;
/// # struct StateMsg;
/// tx_channels! {
///     pub(crate) struct MyTxChannels : MyTxId {
///         state => StateMsg,
///     }
/// }
/// ```
///
/// Channels declared through the macro gain `#[repr(usize)]` identifiers and an
/// inherent `as_index()` helper that returns the zero-based ordinal (matching
/// declaration order), which is convenient when indexing fixed arrays.
#[macro_export]
macro_rules! tx_channels {
    (
        $vis:vis struct $channels:ident : $id:ident {
            $(
                $(#[$chan_meta:meta])* $entry:tt => $payload:ty $(= $route:expr)?
            ),+ $(,)?
        }
    ) => {
        $crate::__cu29_define_bridge_channels! {
            $vis struct $channels : $id {
                $(
                    $(#[$chan_meta])* $entry => $payload $(= $route)?,
                )+
            }
        }
    };
    ({ $($rest:tt)* }) => {
        $crate::tx_channels! {
            pub struct TxChannels : TxId { $($rest)* }
        }
    };
    ($($rest:tt)+) => {
        $crate::tx_channels!({ $($rest)+ });
    };
}

/// Declares the receive channels of a [`CuBridge`] implementation.
///
/// See [`tx_channels!`](crate::tx_channels!) for details on naming and indexing.
#[macro_export]
macro_rules! rx_channels {
    (
        $vis:vis struct $channels:ident : $id:ident {
            $(
                $(#[$chan_meta:meta])* $entry:tt => $payload:ty $(= $route:expr)?
            ),+ $(,)?
        }
    ) => {
        $crate::__cu29_define_bridge_channels! {
            $vis struct $channels : $id {
                $(
                    $(#[$chan_meta])* $entry => $payload $(= $route)?,
                )+
            }
        }
    };
    ({ $($rest:tt)* }) => {
        $crate::rx_channels! {
            pub struct RxChannels : RxId { $($rest)* }
        }
    };
    ($($rest:tt)+) => {
        $crate::rx_channels!({ $($rest)+ });
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::ComponentConfig;
    use crate::cutask::CuMsg;
    #[cfg(not(feature = "std"))]
    use alloc::vec::Vec;
    use cu29_clock::RobotClock;
    use cu29_traits::CuError;
    use serde::{Deserialize, Serialize};
    #[cfg(feature = "std")]
    use std::vec::Vec;

    // ---- Generated channel payload stubs (Copper build output) ---------------
    #[derive(Clone, Debug, Default, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
    struct ImuMsg {
        accel: i32,
    }

    #[derive(Clone, Debug, Default, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
    struct MotorCmd {
        torque: i16,
    }

    #[derive(Clone, Debug, Default, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
    struct StatusMsg {
        temperature: f32,
    }

    #[derive(Clone, Debug, Default, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
    struct AlertMsg {
        code: u32,
    }

    tx_channels! {
        struct MacroTxChannels : MacroTxId {
            imu_stream => ImuMsg = "telemetry/imu",
            motor_stream => MotorCmd,
        }
    }

    rx_channels! {
        struct MacroRxChannels : MacroRxId {
            status_updates => StatusMsg = "sys/status",
            alert_stream => AlertMsg,
        }
    }

    // ---- Generated channel identifiers --------------------------------------
    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    enum TxId {
        Imu,
        Motor,
    }

    #[derive(Copy, Clone, Debug, Eq, PartialEq)]
    enum RxId {
        Status,
        Alert,
    }

    // ---- Generated channel descriptors & registries -------------------------
    struct TxChannels;

    impl TxChannels {
        pub const IMU: BridgeChannel<TxId, ImuMsg> =
            BridgeChannel::with_channel(TxId::Imu, "telemetry/imu");
        pub const MOTOR: BridgeChannel<TxId, MotorCmd> =
            BridgeChannel::with_channel(TxId::Motor, "motor/cmd");
    }

    impl BridgeChannelSet for TxChannels {
        type Id = TxId;

        const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] =
            &[&Self::IMU, &Self::MOTOR];
    }

    struct RxChannels;

    impl RxChannels {
        pub const STATUS: BridgeChannel<RxId, StatusMsg> =
            BridgeChannel::with_channel(RxId::Status, "sys/status");
        pub const ALERT: BridgeChannel<RxId, AlertMsg> =
            BridgeChannel::with_channel(RxId::Alert, "sys/alert");
    }

    impl BridgeChannelSet for RxChannels {
        type Id = RxId;

        const STATIC_CHANNELS: &'static [&'static dyn BridgeChannelInfo<Self::Id>] =
            &[&Self::STATUS, &Self::ALERT];
    }

    // ---- User-authored bridge implementation --------------------------------
    #[derive(Default)]
    struct ExampleBridge {
        port: String,
        imu_samples: Vec<i32>,
        motor_torques: Vec<i16>,
        status_temps: Vec<f32>,
        alert_codes: Vec<u32>,
    }

    impl Freezable for ExampleBridge {}

    impl CuBridge for ExampleBridge {
        type Tx = TxChannels;
        type Rx = RxChannels;

        fn new(
            config: Option<&ComponentConfig>,
            _tx_channels: &[BridgeChannelConfig<TxId>],
            _rx_channels: &[BridgeChannelConfig<RxId>],
        ) -> CuResult<Self> {
            let mut instance = ExampleBridge::default();
            if let Some(cfg) = config {
                if let Some(port) = cfg.get::<String>("port") {
                    instance.port = port;
                }
            }
            Ok(instance)
        }

        fn send<'a, Payload>(
            &mut self,
            _clock: &RobotClock,
            channel: &'static BridgeChannel<TxId, Payload>,
            msg: &CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            match channel.id {
                TxId::Imu => {
                    let imu_msg = msg.downcast_ref::<ImuMsg>()?;
                    let payload = imu_msg
                        .payload()
                        .ok_or_else(|| CuError::from("imu missing payload"))?;
                    self.imu_samples.push(payload.accel);
                    Ok(())
                }
                TxId::Motor => {
                    let motor_msg = msg.downcast_ref::<MotorCmd>()?;
                    let payload = motor_msg
                        .payload()
                        .ok_or_else(|| CuError::from("motor missing payload"))?;
                    self.motor_torques.push(payload.torque);
                    Ok(())
                }
            }
        }

        fn receive<'a, Payload>(
            &mut self,
            _clock: &RobotClock,
            channel: &'static BridgeChannel<RxId, Payload>,
            msg: &mut CuMsg<Payload>,
        ) -> CuResult<()>
        where
            Payload: CuMsgPayload + 'a,
        {
            match channel.id {
                RxId::Status => {
                    let status_msg = msg.downcast_mut::<StatusMsg>()?;
                    status_msg.set_payload(StatusMsg { temperature: 21.5 });
                    if let Some(payload) = status_msg.payload() {
                        self.status_temps.push(payload.temperature);
                    }
                    Ok(())
                }
                RxId::Alert => {
                    let alert_msg = msg.downcast_mut::<AlertMsg>()?;
                    alert_msg.set_payload(AlertMsg { code: 0xDEAD_BEEF });
                    if let Some(payload) = alert_msg.payload() {
                        self.alert_codes.push(payload.code);
                    }
                    Ok(())
                }
            }
        }
    }

    #[test]
    fn channel_macros_expose_static_metadata() {
        assert_eq!(MacroTxChannels::STATIC_CHANNELS.len(), 2);
        assert_eq!(
            MacroTxChannels::IMU_STREAM.default_route,
            Some("telemetry/imu")
        );
        assert!(MacroTxChannels::MOTOR_STREAM.default_route.is_none());
        assert_eq!(MacroTxId::ImuStream as u8, MacroTxId::ImuStream as u8);
        assert_eq!(MacroTxId::ImuStream.as_index(), 0);
        assert_eq!(MacroTxId::MotorStream.as_index(), 1);

        assert_eq!(MacroRxChannels::STATIC_CHANNELS.len(), 2);
        assert_eq!(
            MacroRxChannels::STATUS_UPDATES.default_route,
            Some("sys/status")
        );
        assert!(MacroRxChannels::ALERT_STREAM.default_route.is_none());
        assert_eq!(MacroRxId::StatusUpdates.as_index(), 0);
        assert_eq!(MacroRxId::AlertStream.as_index(), 1);
    }

    #[test]
    fn bridge_trait_compiles_and_accesses_configs() {
        let mut bridge_cfg = ComponentConfig::default();
        bridge_cfg.set("port", "ttyUSB0".to_string());

        let tx_descriptors = [
            BridgeChannelConfig::from_static(&TxChannels::IMU, None, None),
            BridgeChannelConfig::from_static(&TxChannels::MOTOR, None, None),
        ];
        let rx_descriptors = [
            BridgeChannelConfig::from_static(&RxChannels::STATUS, None, None),
            BridgeChannelConfig::from_static(&RxChannels::ALERT, None, None),
        ];

        assert_eq!(
            tx_descriptors[0]
                .effective_route()
                .map(|route| route.into_owned()),
            Some("telemetry/imu".to_string())
        );
        assert_eq!(
            tx_descriptors[1]
                .effective_route()
                .map(|route| route.into_owned()),
            Some("motor/cmd".to_string())
        );
        let overridden = BridgeChannelConfig::from_static(
            &TxChannels::MOTOR,
            Some("custom/motor".to_string()),
            None,
        );
        assert_eq!(
            overridden.effective_route().map(|route| route.into_owned()),
            Some("custom/motor".to_string())
        );

        let mut bridge = ExampleBridge::new(Some(&bridge_cfg), &tx_descriptors, &rx_descriptors)
            .expect("bridge should build");

        assert_eq!(bridge.port, "ttyUSB0");

        let clock = RobotClock::default();
        let imu_msg = CuMsg::new(Some(ImuMsg { accel: 7 }));
        bridge
            .send(&clock, &TxChannels::IMU, &imu_msg)
            .expect("send should succeed");
        let motor_msg = CuMsg::new(Some(MotorCmd { torque: -3 }));
        bridge
            .send(&clock, &TxChannels::MOTOR, &motor_msg)
            .expect("send should support multiple payload types");
        assert_eq!(bridge.imu_samples, vec![7]);
        assert_eq!(bridge.motor_torques, vec![-3]);

        let mut status_msg = CuMsg::new(None);
        bridge
            .receive(&clock, &RxChannels::STATUS, &mut status_msg)
            .expect("receive should succeed");
        assert!(status_msg.payload().is_some());
        assert_eq!(bridge.status_temps, vec![21.5]);

        let mut alert_msg = CuMsg::new(None);
        bridge
            .receive(&clock, &RxChannels::ALERT, &mut alert_msg)
            .expect("receive should handle other payload types too");
        assert!(alert_msg.payload().is_some());
        assert_eq!(bridge.alert_codes, vec![0xDEAD_BEEF]);
    }
}
