use virtue::prelude::*;
use virtue::utils::{ParsedAttribute, parse_tagged_attribute};

pub struct ContainerAttributes {
    pub crate_name: String,
    pub bounds: Option<(String, Literal)>,
    pub decode_bounds: Option<(String, Literal)>,
    pub decode_context: Option<(String, Literal)>,
    pub borrow_decode_bounds: Option<(String, Literal)>,
    pub encode_bounds: Option<(String, Literal)>,
}

impl Default for ContainerAttributes {
    fn default() -> Self {
        Self {
            crate_name: "::bincode".to_string(),
            bounds: None,
            decode_bounds: None,
            decode_context: None,
            encode_bounds: None,
            borrow_decode_bounds: None,
        }
    }
}

impl FromAttribute for ContainerAttributes {
    fn parse(group: &Group) -> Result<Option<Self>> {
        let attributes = match parse_tagged_attribute(group, "bincode")? {
            Some(body) => body,
            None => return Ok(None),
        };
        let mut result = Self::default();
        for attribute in attributes {
            match attribute {
                ParsedAttribute::Property(key, val) if key.to_string() == "crate" => {
                    let val_string = val.to_string();
                    if val_string.starts_with('"') && val_string.ends_with('"') {
                        result.crate_name = val_string[1..val_string.len() - 1].to_string();
                    } else {
                        return Err(Error::custom_at("Should be a literal str", val.span()));
                    }
                }
                ParsedAttribute::Property(key, val) if key.to_string() == "bounds" => {
                    let val_string = val.to_string();
                    if val_string.starts_with('"') && val_string.ends_with('"') {
                        result.bounds =
                            Some((val_string[1..val_string.len() - 1].to_string(), val));
                    } else {
                        return Err(Error::custom_at("Should be a literal str", val.span()));
                    }
                }
                ParsedAttribute::Property(key, val) if key.to_string() == "decode_bounds" => {
                    let val_string = val.to_string();
                    if val_string.starts_with('"') && val_string.ends_with('"') {
                        result.decode_bounds =
                            Some((val_string[1..val_string.len() - 1].to_string(), val));
                    } else {
                        return Err(Error::custom_at("Should be a literal str", val.span()));
                    }
                }
                ParsedAttribute::Property(key, val) if key.to_string() == "decode_context" => {
                    let val_string = val.to_string();
                    if val_string.starts_with('"') && val_string.ends_with('"') {
                        result.decode_context =
                            Some((val_string[1..val_string.len() - 1].to_string(), val));
                    } else {
                        return Err(Error::custom_at("Should be a literal str", val.span()));
                    }
                }
                ParsedAttribute::Property(key, val) if key.to_string() == "encode_bounds" => {
                    let val_string = val.to_string();
                    if val_string.starts_with('"') && val_string.ends_with('"') {
                        result.encode_bounds =
                            Some((val_string[1..val_string.len() - 1].to_string(), val));
                    } else {
                        return Err(Error::custom_at("Should be a literal str", val.span()));
                    }
                }
                ParsedAttribute::Property(key, val)
                    if key.to_string() == "borrow_decode_bounds" =>
                {
                    let val_string = val.to_string();
                    if val_string.starts_with('"') && val_string.ends_with('"') {
                        result.borrow_decode_bounds =
                            Some((val_string[1..val_string.len() - 1].to_string(), val));
                    } else {
                        return Err(Error::custom_at("Should be a literal str", val.span()));
                    }
                }
                ParsedAttribute::Tag(i) => {
                    return Err(Error::custom_at("Unknown field attribute", i.span()));
                }
                ParsedAttribute::Property(key, _) => {
                    return Err(Error::custom_at("Unknown field attribute", key.span()));
                }
                _ => {}
            }
        }
        Ok(Some(result))
    }
}

#[derive(Default)]
pub struct FieldAttributes {
    pub with_serde: bool,
    pub skip: bool,
    pub default: Option<String>,
}

impl FromAttribute for FieldAttributes {
    fn parse(group: &Group) -> Result<Option<Self>> {
        let attributes = match parse_tagged_attribute(group, "bincode")? {
            Some(body) => body,
            None => return Ok(None),
        };
        let mut result = Self::default();
        for attribute in attributes {
            match attribute {
                ParsedAttribute::Tag(i) if i.to_string() == "with_serde" => {
                    result.with_serde = true;
                }
                ParsedAttribute::Tag(i) if i.to_string() == "skip" => {
                    result.skip = true;
                }
                ParsedAttribute::Property(key, val) if key.to_string() == "default" => {
                    let value = val.to_string();
                    if value.starts_with('"') && value.ends_with('"') && value.len() > 2 {
                        result.default = Some(value[1..value.len() - 1].to_string());
                    } else {
                        return Err(Error::custom_at(
                            "Expected a nonempty function path string",
                            val.span(),
                        ));
                    }
                }
                ParsedAttribute::Tag(i) => {
                    return Err(Error::custom_at("Unknown field attribute", i.span()));
                }
                ParsedAttribute::Property(key, _) => {
                    return Err(Error::custom_at("Unknown field attribute", key.span()));
                }
                _ => {}
            }
        }
        if result.default.is_some() && !result.skip {
            return Err(Error::custom_at(
                "bincode default requires skip",
                group.span(),
            ));
        }
        if result.skip && result.with_serde {
            return Err(Error::custom_at(
                "bincode skip cannot be combined with with_serde",
                group.span(),
            ));
        }
        Ok(Some(result))
    }
}

impl FieldAttributes {
    pub fn default_expression(&self) -> String {
        self.default.as_ref().map_or_else(
            || "core::default::Default::default()".to_string(),
            |path| format!("{path}()"),
        )
    }
}

/// Preserve the existing generic-bound inference except for runtime-only fields.
#[derive(Default)]
pub struct FieldBounds {
    has_skipped: bool,
    encoded_identifiers: std::collections::HashSet<String>,
    pub defaults: Vec<String>,
}

impl FieldBounds {
    pub fn new<'a>(groups: impl Iterator<Item = &'a Fields>) -> Result<Self> {
        let mut result = Self::default();
        for group in groups {
            let fields: Vec<_> = match group {
                Fields::Tuple(fields) => fields.iter().collect(),
                Fields::Struct(fields) => fields.iter().map(|(_, field)| field).collect(),
            };
            for field in fields {
                let attributes = field
                    .attributes
                    .get_attribute::<FieldAttributes>()?
                    .unwrap_or_default();
                if attributes.skip {
                    result.has_skipped = true;
                    if attributes.default.is_none() {
                        let ty = field
                            .r#type
                            .iter()
                            .map(ToString::to_string)
                            .collect::<Vec<_>>()
                            .join(" ");
                        result
                            .defaults
                            .push(format!("{ty}: core::default::Default"));
                    }
                } else {
                    collect_identifiers(&field.r#type, &mut result.encoded_identifiers);
                }
            }
        }
        Ok(result)
    }

    pub fn encodes(&self, ident: &Ident) -> bool {
        !self.has_skipped || self.encoded_identifiers.contains(&ident.to_string())
    }
}

fn collect_identifiers(tokens: &[TokenTree], result: &mut std::collections::HashSet<String>) {
    for token in tokens {
        match token {
            TokenTree::Ident(ident) => {
                result.insert(ident.to_string());
            }
            TokenTree::Group(group) => {
                collect_identifiers(&group.stream().into_iter().collect::<Vec<_>>(), result)
            }
            _ => {}
        }
    }
}
