#![cfg_attr(feature = "esp32s3", no_std)]
#![cfg_attr(feature = "esp32s3", no_main)]
#![cfg_attr(feature = "wasm", no_main)]

use embassy_time::{Duration, Instant};
use zenoh_examples::*;
use zenoh_nostd::session::*;

#[embassy_executor::task]
async fn session_task(session: &'static Session<'static, ExampleConfig>) {
    if let Err(e) = session.run().await {
        zenoh::error!("Error in session task: {}", e);
    }
}

async fn entry(spawner: embassy_executor::Spawner) -> zenoh::ZResult<()> {
    #[cfg(feature = "log")]
    env_logger::init();

    zenoh::info!("zenoh-nostd z_ping example");

    // All channels that will be used must outlive `Resources`.
    // **Note**: as a direct implication, here you need to make a static channel.
    static CHANNEL: static_cell::StaticCell<
        embassy_sync::channel::Channel<
            embassy_sync::blocking_mutex::raw::NoopRawMutex,
            FixedCapacitySample<128, 128>,
            8,
        >,
    > = static_cell::StaticCell::new();
    let channel = CHANNEL.init(embassy_sync::channel::Channel::new());

    let config = init_session_example(&spawner).await;
    let session = if LISTEN {
        zenoh::listen!(ExampleConfig: config, Endpoint::try_from(ENDPOINT)?)
    } else {
        zenoh::connect!(ExampleConfig: config, Endpoint::try_from(ENDPOINT)?)
    };

    spawner.spawn(session_task(session)).unwrap();

    let ping = session
        .declare_publisher(zenoh::keyexpr::new("test/ping")?)
        .finish()
        .await?;

    let pong = session
        .declare_subscriber(zenoh::keyexpr::new("test/pong")?)
        .channel(channel.dyn_sender(), channel.dyn_receiver())
        .finish()
        .await?;

    let data: [u8; PAYLOAD] = core::array::from_fn(|i| (i % 10) as u8);
    let mut samples = [0u64; 100];

    zenoh::info!("Warming up for 1s");
    let now = Instant::now();

    while now.elapsed() < Duration::from_secs(1) {
        ping.put(&data).finish().await?;
        pong.recv().await.expect("Channel Closed");
    }

    zenoh::info!("Starting ping-pong measurements");

    for sample in samples.iter_mut() {
        let start = Instant::now();

        ping.put(&data).finish().await?;
        pong.recv().await.expect("Channel Closed");

        *sample = start.elapsed().as_micros();
    }

    for (i, rtt) in samples.iter().enumerate() {
        zenoh::info!(
            "{} bytes: seq={} rtt={:?}µs lat={:?}µs",
            data.len(),
            i,
            rtt,
            rtt / 2
        );
    }

    let avg_rtt: u64 = samples.iter().sum::<u64>() / samples.len() as u64;
    let avg_lat: u64 = avg_rtt / 2;

    zenoh::info!(
        "Average RTT: {:?}µs, Average Latency: {:?}µs",
        avg_rtt,
        avg_lat
    );

    Ok(())
}

#[cfg_attr(feature = "std", embassy_executor::main)]
#[cfg_attr(feature = "wasm", embassy_executor::main)]
#[cfg_attr(feature = "esp32s3", esp_rtos::main)]
async fn main(spawner: embassy_executor::Spawner) {
    if let Err(e) = entry(spawner).await {
        zenoh::error!("Error in main: {}", e);
    }

    zenoh::info!("Exiting main");
}

#[cfg(feature = "esp32s3")]
mod esp32s3_app {
    use esp_hal::rng::Rng;
    pub use esp_println as _;
    use getrandom::{Error, register_custom_getrandom};

    #[panic_handler]
    fn panic(info: &core::panic::PanicInfo) -> ! {
        zenoh_nostd::session::zenoh::error!("Panic: {}", info);

        loop {}
    }

    extern crate alloc;

    esp_bootloader_esp_idf::esp_app_desc!();

    register_custom_getrandom!(getrandom_custom);
    pub fn getrandom_custom(bytes: &mut [u8]) -> Result<(), Error> {
        Rng::new().read(bytes);
        Ok(())
    }
}
