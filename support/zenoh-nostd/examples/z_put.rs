#![cfg_attr(feature = "esp32s3", no_std)]
#![cfg_attr(feature = "esp32s3", no_main)]
#![cfg_attr(feature = "wasm", no_main)]

use zenoh_examples::*;
use zenoh_nostd::session::*;

async fn entry(spawner: embassy_executor::Spawner) -> zenoh::ZResult<()> {
    #[cfg(feature = "log")]
    env_logger::init();

    zenoh::info!("zenoh-nostd z_put example");

    let config = init_session_example(&spawner).await;
    let session = if LISTEN {
        zenoh::listen!(ExampleConfig: config, Endpoint::try_from(ENDPOINT)?)
    } else {
        zenoh::connect!(ExampleConfig: config, Endpoint::try_from(ENDPOINT)?)
    };

    // In this example we don't care about maintaining the session alive but we do it anyway for demonstration purpose. Know
    // that it's not mandatory to do a `session.run()` if you just need to `put` a value on the network.
    // We then have two choices:
    //  1) Spawn a new task to run the `session.run()` in background, but it requires the `session` to be `'static`.
    //  2) Use `select` or `join` to run both the session and the subscriber in the same task.
    // Here we use the second approach. For a demonstration of the first approach, see the `z_open` example.

    let ke = zenoh::keyexpr::new("demo/example")?;
    let payload = b"Hello, from no-std!";

    embassy_futures::select::select(session.run(), async {
        match session.put(ke, payload).finish().await {
            Ok(_) => {
                zenoh::info!(
                    "[Put] Sent PUT ('{}': '{}')",
                    ke.as_str(),
                    core::str::from_utf8(payload).unwrap()
                );
            }
            Err(e) => {
                zenoh::error!("{}", e)
            }
        };
    })
    .await;

    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;

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
