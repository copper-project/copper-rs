file::rename("Cargo.toml.template", "Cargo.toml");

// Resolve the absolute path of a relative directory
// let relative_path = "../../";
//let absolute_path = file::canonicalize(relative_path);
//set_variable("copper_root", absolute_path);
let template_dir = variable::get("CARGO_GENERATE_TEMPLATE_DIRECTORY");
print(`Current directory: ${template_dir}`);
 

