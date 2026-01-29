use std::collections::BTreeMap;

pub fn flatten_json(
    prefix: &str,
    value: &serde_json::Value,
) -> BTreeMap<String, serde_json::Value> {
    let mut out = BTreeMap::new();
    match value {
        serde_json::Value::Object(map) => {
            for (k, v) in map {
                let key = format!("{}/{}", prefix, k);
                out.extend(flatten_json(&key, v));
            }
        }
        serde_json::Value::Array(arr) => {
            for (idx, v) in arr.iter().enumerate() {
                let key = format!("{}/{}", prefix, idx);
                out.extend(flatten_json(&key, v));
            }
        }
        _ => {
            out.insert(prefix.to_string(), value.clone());
        }
    }
    out
}

pub fn extract_scalars(flat: &BTreeMap<String, serde_json::Value>) -> Vec<(String, f64)> {
    flat.iter()
        .filter_map(|(k, v)| match v {
            serde_json::Value::Number(n) => n.as_f64().map(|f| (k.clone(), f)),
            serde_json::Value::Bool(b) => Some((k.clone(), if *b { 1.0 } else { 0.0 })),
            _ => None,
        })
        .collect()
}
