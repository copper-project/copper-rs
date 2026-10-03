use cu_self_describing_logs::Application;
use cu_self_describing_logs::VALUE_DECODE_CATALOG;

fn main() {
    std::fs::create_dir_all("logs").expect("create logs directory");
    let app = Application::builder()
        .with_value_decode_catalog(VALUE_DECODE_CATALOG)
        .with_log_path("logs/wheel.copper", Some(32 * 1024 * 1024))
        .expect("create log")
        .build()
        .expect("build wheel application");
    let mut app = app.start().expect("start tasks");
    for _ in 0..10 {
        app.run_one_iteration().expect("record wheel sample");
    }
    app.stop().expect("stop tasks");
}
