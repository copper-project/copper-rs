#![cfg_attr(feature = "esp32s3", no_std)]
#![cfg_attr(feature = "esp32s3", no_main)]
#![cfg_attr(feature = "wasm", no_main)]

use zenoh_examples::*;
use zenoh_nostd::session::*;

async fn entry(spawner: embassy_executor::Spawner) -> zenoh::ZResult<()> {
    #[cfg(feature = "log")]
    env_logger::init();

    zenoh::info!("zenoh-nostd z_pub_thr example");

    let config = init_session_example(&spawner).await;
    let mut resources = Resources::default();
    let session = if LISTEN {
        zenoh::listen_ignore_invalid_sn(&mut resources, &config, Endpoint::try_from(ENDPOINT)?)
            .await?
    } else {
        zenoh::connect_ignore_invalid_sn(&mut resources, &config, Endpoint::try_from(ENDPOINT)?)
            .await?
    };

    let payload: [u8; PAYLOAD] = core::array::from_fn(|i| (i % 10) as u8);
    let publisher = session
        .declare_publisher(zenoh::keyexpr::new("test/thr")?)
        .finish()
        .await?;

    let mut count: usize = 0;
    let mut start = embassy_time::Instant::now();
    embassy_futures::select::select(session.run(), async {
        loop {
            if let Err(e) = publisher.put(&payload).finish().await {
                zenoh::error!("Error publishing message: {}", e);
                break;
            }

            if count < 100_000 {
                count += 1;
            } else {
                let thpt = count as f64 / (start.elapsed().as_micros() as f64 / 1_000_000.0);
                zenoh::info!("{} msgs/s", thpt);
                count = 0;
                start = embassy_time::Instant::now();
            }
        }

        Ok::<(), Error>(())
    })
    .await;

    Ok(())
}

#[cfg_attr(feature = "std", embassy_executor::main)]
#[cfg_attr(feature = "esp32s3", esp_rtos::main)]
#[cfg_attr(feature = "wasm", embassy_executor::main)]
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
