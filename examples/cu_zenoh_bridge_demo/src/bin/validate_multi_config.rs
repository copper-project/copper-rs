use cu29::config::read_multi_configuration;

fn main() {
    if let Err(err) = drive() {
        eprintln!("validate-multi-config failed: {err}");
        std::process::exit(1);
    }
}

fn drive() -> Result<(), Box<dyn std::error::Error>> {
    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| format!("{}/multi_copper.ron", env!("CARGO_MANIFEST_DIR")));
    let config = read_multi_configuration(&config_path)?;

    println!("Validated multi-Copper config: {config_path}");
    println!("Subsystems:");
    for subsystem in &config.subsystems {
        println!(
            "  {} -> subsystem_code={} config={}",
            subsystem.id, subsystem.subsystem_code, subsystem.config_path
        );
    }

    println!("Interconnects:");
    for interconnect in &config.interconnects {
        println!(
            "  {} -> {} [{} via {}]",
            interconnect.from, interconnect.to, interconnect.msg, interconnect.bridge_type
        );
    }

    Ok(())
}
