file::rename("Cargo.toml.template", "Cargo.toml");
if !variable::get("pgs_enabled") {
    file::delete("schedule.ron");
    file::delete("src/pgs_candidate.rs");
    file::delete("src/pgs.rs");
}
