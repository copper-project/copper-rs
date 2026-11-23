use cu29::prelude::*;
use std::env;
use url::Url;

#[copper_runtime(config = "examples/rtsp.ron")]
struct CameraApp {}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut url = Url::parse(&env::var("CAMERA_URL")?)?;
    url.set_username(&env::var("CAMERA_USER")?)
        .map_err(|_| "CAMERA_URL must support a username")?;
    url.set_password(Some(&env::var("CAMERA_PASSWORD")?))
        .map_err(|_| "CAMERA_URL must support a password")?;

    let mut config = read_configuration("examples/rtsp.ron")?;
    let graph = config.get_graph_mut(None)?;
    let camera = graph
        .get_node_id_by_name("camera")
        .ok_or("missing camera")?;
    graph
        .get_node_mut(camera)
        .ok_or("missing camera")?
        .set_param(
            "pipeline",
            format!("rtspsrc location={url} ! decodebin ! appsink name=copper"),
        );

    std::fs::create_dir_all("logs")?;
    CameraApp::builder()
        .with_config(config)
        .save_effective_config(false)
        .with_log_path("logs/rtsp.copper", None)?
        .build()?
        .run_until_shutdown()
        .map_err(|e| e.error)?;
    Ok(())
}
