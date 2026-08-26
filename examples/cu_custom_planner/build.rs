fn main() {
    cu29_build::setup();
    cu29::planner::emit_plan::<custom_planner::Alphabetical>("copperconfig.ron").unwrap();
}
