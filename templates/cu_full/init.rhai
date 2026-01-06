file::rename("Cargo.toml.template", "Cargo.toml");
file::rename(
    "apps/{{project-name|kebab_case}}/Cargo.toml.template",
    "apps/{{project-name|kebab_case}}/Cargo.toml",
);
file::rename(
    "libs/{{project-name|kebab_case}}-types/Cargo.toml.template",
    "libs/{{project-name|kebab_case}}-types/Cargo.toml",
);
file::rename(
    "tasks/{{project-name|kebab_case}}-tasks/Cargo.toml.template",
    "tasks/{{project-name|kebab_case}}-tasks/Cargo.toml",
);
