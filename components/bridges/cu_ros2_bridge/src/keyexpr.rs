#[macro_export]
macro_rules! format_keyexpr_raw {
    ($first:expr, $($arg:expr),*) => {{
        use $crate::error::cu_error_map;

        let mut result = KeyExpr::new($first.to_string());
        $(
            if let Ok(r) = result {
              result = r.join(&$arg.to_string());
            }
        )*
        result.map_err(cu_error_map("Bad keyexpr argument"))
    }};
}

#[macro_export]
macro_rules! format_keyexpr {
    ($first:expr, $($arg:expr),*) => {{

        use $crate::keyexpr::normalize_chunk;
        $crate::format_keyexpr_raw!(normalize_chunk($first), $(normalize_chunk($arg)), *)
    }};
}

/// Liveliness keyexpr mangles key parts in order to escape slashes
#[macro_export]
macro_rules! format_liveliness_keyexpr {
    ($first:expr, $($arg:expr),*) => {{

        use $crate::keyexpr::mangle_chunk;
        $crate::format_keyexpr_raw!(mangle_chunk($first), $(mangle_chunk($arg)), *)
    }};
}

pub(crate) fn normalize_chunk<T>(arg: T) -> String
where
    T: ToString,
{
    let str = arg.to_string();
    let str_trimmed = str.trim_matches('/');
    // Empty chunks are forbidden
    if str_trimmed.is_empty() {
        return "_".into();
    }

    str_trimmed.into()
}

pub(crate) fn mangle_chunk<T>(arg: T) -> String
where
    T: ToString,
{
    let str = arg.to_string();
    // Empty chunks are forbidden
    if str.is_empty() {
        return "%".into();
    }

    str.replace("/", "%")
}

#[cfg(test)]
mod tests {
    use zenoh::key_expr::KeyExpr;

    #[test]
    fn test_basic_keyexpr() {
        {
            let keyexpr = format_keyexpr!("test", "copper").unwrap();
            assert_eq!(keyexpr.to_string(), "test/copper")
        }
        {
            let keyexpr = format_liveliness_keyexpr!("test", "copper").unwrap();
            assert_eq!(keyexpr.to_string(), "test/copper")
        }
    }

    #[test]
    fn test_special_keyexpr() {
        {
            let keyexpr = format_keyexpr!("@prefix", "/topic/a/", 1).unwrap();
            assert_eq!(keyexpr.to_string(), "@prefix/topic/a/1")
        }

        {
            let keyexpr = format_liveliness_keyexpr!("@prefix", "", "/topic/a", 1).unwrap();
            assert_eq!(keyexpr.to_string(), "@prefix/%/%topic%a/1")
        }
    }
}
