#![cfg_attr(feature = "esp32s3", no_std)]
#![cfg_attr(feature = "esp32s3", no_main)]
#![cfg_attr(feature = "wasm", no_main)]

use zenoh_examples::*;
use zenoh_nostd::session::*;

async fn response_callback(resp: &GetResponse<'_>) {
    response_callback_sync(resp);
}

fn response_callback_sync(resp: &GetResponse<'_>) {
    match resp {
        GetResponse::Ok(reply) => {
            zenoh::info!(
                "[Get] Received OK Reply ('{}': '{:?}')",
                reply.keyexpr().as_str(),
                core::str::from_utf8(reply.payload()).unwrap()
            );
        }
        GetResponse::Err(reply) => {
            zenoh::error!(
                "[Get] Received ERR Reply ('{}': '{:?}')",
                reply.keyexpr().as_str(),
                core::str::from_utf8(reply.payload()).unwrap()
            );
        }
    }
}

async fn entry(spawner: embassy_executor::Spawner) -> zenoh::ZResult<()> {
    #[cfg(feature = "log")]
    env_logger::init();

    zenoh::info!("zenoh-nostd z_get example");

    let config = init_session_example(&spawner).await;
    let session = if LISTEN {
        zenoh::listen!(ExampleConfig: config, Endpoint::try_from(ENDPOINT)?)
    } else {
        zenoh::connect!(ExampleConfig: config, Endpoint::try_from(ENDPOINT)?)
    };

    // In the following code we use the direct `callback` API. `zenoh` lets you provide either `sync` or `async` callbacks for your convenience.
    // **Be careful**, in both case the callback should resolve almost instantly as it will `stop` the `reading` task of the session!
    //
    // This means that if you want to do high computation you may prefer to use the `channel` API, or to delegate into your own channels.
    // See the `z_sub` example to see how the `channel` API works.

    session
        .get(zenoh::keyexpr::new("demo/example/**")?)
        .callback(async |resp| response_callback(resp).await)
        .finish()
        .await?;

    session
        .get(zenoh::keyexpr::new("demo/example/**")?)
        .callback_sync(response_callback_sync)
        .finish()
        .await?;

    Ok(session.run().await?)
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
