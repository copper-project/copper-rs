use std::sync::Mutex;
use crate::config::ConfigNode;
use crate::config::Value;

lazy_static! {
    pub static ref REGISTRY: Mutex<Vec<Box<dyn Plugin>>> = Mutex::new(Vec::new());
}

pub trait CuTask {}

// pub trait CuTaskConfig {
//     fn get<T>(&self, key: &str) -> T;
// }

pub struct CuTaskConfig(ConfigNode);

impl CuTaskConfig {
    pub fn new(node: ConfigNode) -> Self {
        CuTaskConfig(node)
    }

    pub fn get<T: std::convert::From<Value>>(&self, key: &str) -> Option<T> {
        self.0.get_param(key)
    }
}

type CuTaskCreator = dyn Fn(&CuTaskConfig) -> dyn CuTask;

pub trait CuSource {}
pub trait CuSink {}

pub type CuError = Box<dyn std::error::Error>;

pub trait Plugin : Send + Sync {
    fn initialize(&self, cfg: CuTaskConfig) -> Result<(), CuError> {
        Ok(())
    }
}

#[macro_export]
macro_rules! register_plugin {
    ($plugin_ty:ty) => {
        pub fn _register_plugin() {
            let plugin: $plugin_ty = Default::default();
            $crate::core::REGISTRY.lock().unwrap().push(Box::new(plugin));
        }
    };
}

#[cfg(test)]
mod tests {
    use crate::core::Plugin;

    #[derive(Default)]
    struct TestPlugin {}

    impl Plugin for TestPlugin { }

    register_plugin!(TestPlugin);

    #[test]
    fn test_register_plugin() {
        _register_plugin();
        let registry = crate::core::REGISTRY.lock().unwrap();
        assert_eq!(registry.len(), 1);
    }

    #[test]
    fn test_plugin_config_accessor() {
        let mut config = crate::config::ConfigNode::new("instanceName", "instanceType");
        config.set_param("test", 42);
        let task_config = crate::core::CuTaskConfig::new(config);
        let value: i32 = task_config.get("test").unwrap();
        assert_eq!(value, 42);
    }
}

