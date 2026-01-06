file::rename("Cargo.toml.template", "Cargo.toml");
file::rename(
    "apps/{{project-name}}/Cargo.toml.template",
    "apps/{{project-name}}/Cargo.toml",
);
file::rename(
    "libs/{{project-name}}-types/Cargo.toml.template",
    "libs/{{project-name}}-types/Cargo.toml",
);
file::rename(
    "tasks/{{project-name}}-tasks/Cargo.toml.template",
    "tasks/{{project-name}}-tasks/Cargo.toml",
);
