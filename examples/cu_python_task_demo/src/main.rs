use cu_python_task::PyTaskMode;
use cu_python_task_demo::run_demo;

fn parse_mode() -> PyTaskMode {
    match std::env::args().nth(1).as_deref() {
        Some("embedded") => PyTaskMode::Embedded,
        Some("process_shm") | Some("process-shm") => PyTaskMode::ProcessShm,
        Some("process") | None => PyTaskMode::Process,
        Some(other) => {
            eprintln!(
                "Unsupported mode '{other}', expected 'process', 'process_shm', or 'embedded'"
            );
            std::process::exit(2);
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    run_demo(parse_mode(), 3)?;
    Ok(())
}
