#![cfg_attr(feature = "esp32s3", no_std)]
#![cfg_attr(feature = "esp32s3", no_main)]
#![cfg_attr(feature = "wasm", no_main)]

use zenoh_examples::*;
use zenoh_nostd::session::*;

struct Stats {
    round_count: usize,
    round_size: usize,
    finished_rounds: usize,
    round_start: embassy_time::Instant,
    global_start: Option<embassy_time::Instant>,
}

impl Stats {
    fn increment(&mut self) {
        if self.round_count == 0 {
            self.round_start = embassy_time::Instant::now();
            if self.global_start.is_none() {
                self.global_start = Some(self.round_start)
            }
            self.round_count += 1;
        } else if self.round_count < self.round_size {
            self.round_count += 1;
        } else {
            self.print_round();
            self.finished_rounds += 1;
            self.round_count = 0;
        }
    }

    fn print_round(&self) {
        let elapsed = self.round_start.elapsed().as_micros() as f64 / 1_000_000.0;
        let throughput = (self.round_size as f64) / elapsed;
        zenoh::info!("{} msg/s", throughput);
    }
}

async fn entry(spawner: embassy_executor::Spawner) -> zenoh::ZResult<()> {
    #[cfg(feature = "log")]
    env_logger::init();

    zenoh::info!("zenoh-nostd z_sub_thr example");

    let close =
        embassy_sync::signal::Signal::<embassy_sync::blocking_mutex::raw::NoopRawMutex, ()>::new();

    let config = init_session_example(&spawner).await;
    let mut resources = Resources::default();
    let session = if LISTEN {
        zenoh::listen_ignore_invalid_sn(&mut resources, &config, Endpoint::try_from(ENDPOINT)?)
            .await?
    } else {
        zenoh::connect_ignore_invalid_sn(&mut resources, &config, Endpoint::try_from(ENDPOINT)?)
            .await?
    };

    let mut stats = Stats {
        round_count: 0,
        round_size: 100_000,
        finished_rounds: 0,
        round_start: embassy_time::Instant::now(),
        global_start: None,
    };

    let send = &close;
    session
        .declare_subscriber(zenoh::keyexpr::new("test/thr")?)
        .callback_sync(move |_| {
            if stats.finished_rounds >= 10000 {
                send.signal(());
                return;
            }

            stats.increment();
        })
        .finish()
        .await?;

    embassy_futures::select::select(session.run(), close.wait()).await;

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
