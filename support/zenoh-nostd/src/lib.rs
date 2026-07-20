#![no_std]

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "alloc")]
use zenoh_nostd::broker::ZBrokerConfig;

use zenoh_nostd::session::*;

#[cfg(feature = "std")]
pub use zenoh_std::StdLinkManager as LinkManager;

#[cfg(feature = "esp32s3")]
pub type LinkManager = zenoh_embassy::EmbassyLinkManager<'static, 512, 3>;

#[cfg(feature = "wasm")]
pub use zenoh_wasm::WasmLinkManager as LinkManager;

#[cfg(feature = "esp32s3")]
mod esp32s3_app {
    pub use embassy_net::{DhcpConfig, Runner, StackResources};
    pub use embassy_time::{Duration, Timer};
    pub use esp_hal::{clock::CpuClock, rng::Rng, timer::timg::TimerGroup};
    pub use esp_println as _;
    pub use esp_radio::{
        Controller,
        wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiStaState},
    };

    pub const SSID: Option<&str> = option_env!("WIFI_SSID");
    pub const PASSWORD: &str = env!("WIFI_PASSWORD");
}

#[cfg(feature = "esp32s3")]
use esp32s3_app::*;

pub const ENDPOINT: &str = match option_env!("ENDPOINT") {
    Some(v) => v,
    None => {
        if cfg!(feature = "wasm") {
            "ws/127.0.0.1:7446"
        } else {
            "tcp/127.0.0.1:7447"
        }
    }
};

pub const PAYLOAD: usize = match usize::from_str_radix(
    match option_env!("PAYLOAD") {
        Some(v) => v,
        None => "8",
    },
    10,
) {
    Ok(v) => v,
    Err(_) => 8,
};

pub const LISTEN: bool = matches!(
    usize::from_str_radix(
        match option_env!("LISTEN") {
            Some(v) => v,
            None => "0",
        },
        10,
    ),
    Ok(1)
);

#[cfg(feature = "esp32s3")]
const BUFF_SIZE: u16 = 512u16;
#[cfg(feature = "std")]
const BUFF_SIZE: u16 = u16::MAX;
#[cfg(feature = "wasm")]
const BUFF_SIZE: u16 = u16::MAX / 2;

pub struct ExampleConfig {
    transports: TransportLinkManager<LinkManager>,
}

#[cfg(feature = "alloc")]
impl ZBrokerConfig for ExampleConfig {
    type LinkManager = LinkManager;
    type Buff = alloc::vec::Vec<u8>;

    fn buff(&self) -> Self::Buff {
        alloc::vec![0; BUFF_SIZE as usize]
    }

    fn transports(&self) -> &TransportLinkManager<Self::LinkManager> {
        &self.transports
    }
}

impl ZSessionConfig for ExampleConfig {
    type LinkManager = LinkManager;

    #[cfg(not(feature = "alloc"))]
    type Buff = [u8; BUFF_SIZE as usize];

    #[cfg(feature = "alloc")]
    type Buff = alloc::vec::Vec<u8>;

    #[cfg(not(feature = "alloc"))]
    type SubCallbacks<'res> = FixedCapacitySubCallbacks<
        'res,
        8,
        zenoh::storage::RawOrBox<56>,
        zenoh::storage::RawOrBox<600>,
    >;

    #[cfg(feature = "alloc")]
    type SubCallbacks<'res> = AllocSubCallbacks<'res, zenoh::storage::Box, zenoh::storage::Box>;

    #[cfg(not(feature = "alloc"))]
    type GetCallbacks<'res> = FixedCapacityGetCallbacks<
        'res,
        8,
        zenoh::storage::RawOrBox<1>,
        zenoh::storage::RawOrBox<32>,
    >;

    #[cfg(feature = "alloc")]
    type GetCallbacks<'res> = AllocGetCallbacks<'res, zenoh::storage::Box, zenoh::storage::Box>;

    #[cfg(not(feature = "alloc"))]
    type QueryableCallbacks<'res> = FixedCapacityQueryableCallbacks<
        'res,
        Self,
        8,
        zenoh::storage::RawOrBox<32>,
        zenoh::storage::RawOrBox<952>,
    >;

    #[cfg(feature = "alloc")]
    type QueryableCallbacks<'res> =
        AllocQueryableCallbacks<'res, Self, zenoh::storage::Box, zenoh::storage::Box>;

    fn buff(&self) -> Self::Buff {
        #[cfg(not(feature = "alloc"))]
        {
            [0u8; BUFF_SIZE as usize]
        }
        #[cfg(feature = "alloc")]
        {
            alloc::vec![0; BUFF_SIZE as usize]
        }
    }

    fn transports(&self) -> &TransportLinkManager<Self::LinkManager> {
        &self.transports
    }
}

pub async fn init_broker_example(spawner: &embassy_executor::Spawner) -> ExampleConfig {
    #[cfg(feature = "std")]
    {
        let _ = spawner;
        ExampleConfig {
            transports: TransportLinkManager::from(LinkManager),
        }
    }
    #[cfg(feature = "wasm")]
    {
        let _ = spawner;
        unimplemented!()
    }
    #[cfg(feature = "esp32s3")]
    {
        use static_cell::StaticCell;

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let peripherals = esp_hal::init(config);

        esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0);

        zenoh::info!("Embassy initialized!");

        let rng = Rng::new();

        static RADIO_CTRL: StaticCell<Controller<'static>> = StaticCell::new();
        let radio_ctrl = esp_radio::init().expect("Failed to init radio");

        let (wifi_controller, interfaces) = esp_radio::wifi::new(
            RADIO_CTRL.init(radio_ctrl),
            peripherals.WIFI,
            Default::default(),
        )
        .expect("Failed to initialize WIFI controller");

        let wifi_interface = interfaces.sta;
        let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);
        let dhcp_config = DhcpConfig::default();
        let config = embassy_net::Config::dhcpv4(dhcp_config);

        static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
        let (stack, runner) = embassy_net::new(
            wifi_interface,
            config,
            RESOURCES.init(StackResources::new()),
            net_seed,
        );

        spawner.spawn(connection(wifi_controller)).ok();
        spawner.spawn(net_task(runner)).ok();

        zenoh::info!("Waiting for link to be up");
        loop {
            if stack.is_link_up() {
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        zenoh::info!("Waiting to get IP address...");
        let ip = loop {
            if let Some(config) = stack.config_v4() {
                zenoh::info!("Got IP: {}", config.address);
                break config.address;
            }
            Timer::after(Duration::from_millis(500)).await;
        };
        zenoh::info!("Network initialized with IP: {}", ip);

        ExampleConfig {
            transports: TransportLinkManager::from(LinkManager::new(stack)),
        }
    }
}

pub async fn init_session_example(spawner: &embassy_executor::Spawner) -> ExampleConfig {
    #[cfg(feature = "std")]
    {
        let _ = spawner;
        ExampleConfig {
            transports: TransportLinkManager::from(LinkManager),
        }
    }
    #[cfg(feature = "wasm")]
    {
        let _ = spawner;
        ExampleConfig {
            transports: TransportLinkManager::from(LinkManager),
        }
    }
    #[cfg(feature = "esp32s3")]
    {
        use static_cell::StaticCell;

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let peripherals = esp_hal::init(config);

        esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0);

        zenoh::info!("Embassy initialized!");

        let rng = Rng::new();

        static RADIO_CTRL: StaticCell<Controller<'static>> = StaticCell::new();
        let radio_ctrl = esp_radio::init().expect("Failed to init radio");

        zenoh::info!("Radio initialized");

        let (wifi_controller, interfaces) = esp_radio::wifi::new(
            RADIO_CTRL.init(radio_ctrl),
            peripherals.WIFI,
            Default::default(),
        )
        .expect("Failed to initialize WIFI controller");

        zenoh::info!("Wifi controller created");

        let wifi_interface = interfaces.sta;
        let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);
        let dhcp_config = DhcpConfig::default();
        let config = embassy_net::Config::dhcpv4(dhcp_config);

        static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
        let (stack, runner) = embassy_net::new(
            wifi_interface,
            config,
            RESOURCES.init(StackResources::new()),
            net_seed,
        );

        zenoh::info!("Network stack created");

        spawner.spawn(connection(wifi_controller)).ok();
        spawner.spawn(net_task(runner)).ok();

        zenoh::info!("Waiting for link to be up");
        loop {
            if stack.is_link_up() {
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        zenoh::info!("Waiting to get IP address...");
        let ip = loop {
            if let Some(config) = stack.config_v4() {
                zenoh::info!("Got IP: {}", config.address);
                break config.address;
            }
            Timer::after(Duration::from_millis(500)).await;
        };
        zenoh::info!("Network initialized with IP: {}", ip);

        ExampleConfig {
            transports: TransportLinkManager::from(LinkManager::new(stack)),
        }
    }
}

#[cfg(feature = "esp32s3")]
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    zenoh::info!("start connection task");
    zenoh::info!("Device capabilities: {:?}", controller.capabilities());
    loop {
        match esp_radio::wifi::sta_state() {
            WifiStaState::Connected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(SSID.unwrap_or("Zenoh").into())
                    .with_password(PASSWORD.into()),
            );
            controller.set_config(&client_config).unwrap();
            zenoh::info!("Starting wifi");
            controller.start_async().await.unwrap();
            zenoh::info!("Wifi started!");
        }
        zenoh::info!("About to connect...");

        match controller.connect_async().await {
            Ok(_) => zenoh::info!("Wifi connected!"),
            Err(e) => {
                zenoh::info!("Failed to connect to wifi: {:?}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[cfg(feature = "esp32s3")]
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
