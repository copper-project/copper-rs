use convert_case::{Case, Casing};
use std::path::PathBuf;
use walkdir::WalkDir;

/// Small tool to create a valid enum entry from an identifier.
pub(crate) fn config_id_to_enum(id: &str) -> String {
    let mut candidate = id
        .chars()
        .map(|c| if c.is_alphanumeric() { c } else { '_' })
        .collect::<String>();

    candidate = candidate.to_case(Case::Pascal);

    if candidate.chars().next().is_some_and(|c| c.is_ascii_digit()) {
        candidate.insert(0, '_');
    }

    candidate
}

/// Same as config_id_to_enum but for a struct member name
pub(crate) fn config_id_to_struct_member(id: &str) -> String {
    let mut candidate = id
        .chars()
        .map(|c| if c.is_alphanumeric() { c } else { '_' })
        .collect::<String>();

    candidate = candidate.to_case(Case::Snake);

    if candidate.chars().next().is_some_and(|c| c.is_ascii_digit()) {
        candidate.insert(0, '_');
    }

    candidate
}

// Lifted this HORROR but it works.
pub fn caller_crate_root() -> PathBuf {
    let crate_name =
        std::env::var("CARGO_PKG_NAME").expect("failed to read ENV var `CARGO_PKG_NAME`!");
    let current_dir = std::env::current_dir().expect("failed to unwrap env::current_dir()!");
    let search_entry = format!("name=\"{crate_name}\"");
    for entry in WalkDir::new(&current_dir)
        .into_iter()
        .filter_entry(|e| !e.file_name().eq_ignore_ascii_case("target"))
    {
        let Ok(entry) = entry else {
            continue;
        };
        if !entry.file_type().is_file() {
            continue;
        }
        let Some(file_name) = entry.path().file_name() else {
            continue;
        };
        if !file_name.eq_ignore_ascii_case("Cargo.toml") {
            continue;
        }
        let Ok(cargo_toml) = std::fs::read_to_string(entry.path()) else {
            continue;
        };
        if cargo_toml
            .chars()
            .filter(|&c| !c.is_whitespace())
            .collect::<String>()
            .contains(search_entry.as_str())
        {
            return entry.path().parent().unwrap().to_path_buf();
        }
    }
    current_dir
}

#[cfg(test)]
mod tests {
    use crate::utils::config_id_to_enum;

    fn is_valid_rust_identifier(input: &str) -> bool {
        if input.is_empty() {
            return false;
        }

        // Check if the first character is valid
        let mut chars = input.chars();
        if let Some(first) = chars.next() {
            if !first.is_alphabetic() && first != '_' {
                return false;
            }
        }

        // Check the rest of the characters
        if !chars.all(|c| c.is_alphanumeric() || c == '_') {
            return false;
        }

        // Check if it's a Rust keyword (can use a set of known keywords)
        let keywords = [
            "as", "break", "const", "continue", "crate", "else", "enum", "extern", "false", "fn",
            "for", "if", "impl", "in", "let", "loop", "match", "mod", "move", "mut", "pub", "ref",
            "return", "self", "Self", "static", "struct", "super", "trait", "true", "type",
            "unsafe", "use", "where", "while",
        ];

        !keywords.contains(&input)
    }

    #[test]
    fn test_identifier_to_enum() {
        let test_cases = ["toto", "#id", "!!something", "hey?", "é", "t"];

        test_cases.iter().for_each(|input| {
            let after = config_id_to_enum(input);
            assert!(
                is_valid_rust_identifier(after.as_str()),
                "bf {input} af {after}"
            );
        })
    }

    #[test]
    fn test_identifier_to_struct_member() {
        assert_eq!(crate::utils::config_id_to_struct_member("toto"), "toto");
        assert_eq!(crate::utils::config_id_to_struct_member("#id"), "id");
        assert_eq!(
            crate::utils::config_id_to_struct_member("!!something"),
            "something"
        );
        assert_eq!(crate::utils::config_id_to_struct_member("hey?"), "hey");
        assert_eq!(crate::utils::config_id_to_struct_member("é"), "é");
        assert_eq!(crate::utils::config_id_to_struct_member("T"), "t");
        assert_eq!(
            crate::utils::config_id_to_struct_member("Test_Dunder"),
            "test_dunder"
        );
    }
}
