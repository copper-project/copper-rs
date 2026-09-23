# Shared viewer entry points. Path handling and logreader orchestration live in Rust.
VIEWER_WORKSPACE_ROOT := source_directory() + "/../.."
VIEWER_PROJECT_DIR := justfile_directory()

graph-view config="copperconfig.ron" output="graph.svg" mission="" features="":
  cargo run --manifest-path "{{VIEWER_WORKSPACE_ROOT}}/Cargo.toml" -p cu29-graph-view -- "{{config}}" --base-dir "{{VIEWER_PROJECT_DIR}}" --output "{{output}}" --mission "{{mission}}" --features "{{features}}" --open

graph-view-log config output log logreader package="" mission="" features="":
  cargo run --manifest-path "{{VIEWER_WORKSPACE_ROOT}}/Cargo.toml" -p cu29-graph-view -- "{{config}}" --base-dir "{{VIEWER_PROJECT_DIR}}" --output "{{output}}" --log "{{log}}" --logreader "{{logreader}}" --logreader-package "{{package}}" --mission "{{mission}}" --features "{{features}}" --open

schedule-view config="copperconfig.ron" output="schedule.svg" mission="" features="":
  cargo run --manifest-path "{{VIEWER_WORKSPACE_ROOT}}/Cargo.toml" -p cu29-schedule-view -- "{{config}}" --base-dir "{{VIEWER_PROJECT_DIR}}" --output "{{output}}" --mission "{{mission}}" --features "{{features}}" --open

schedule-view-log config output log logreader package="" mission="" features="":
  cargo run --manifest-path "{{VIEWER_WORKSPACE_ROOT}}/Cargo.toml" -p cu29-schedule-view -- "{{config}}" --base-dir "{{VIEWER_PROJECT_DIR}}" --output "{{output}}" --log "{{log}}" --logreader "{{logreader}}" --logreader-package "{{package}}" --mission "{{mission}}" --features "{{features}}" --open
