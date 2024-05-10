use std::sync::Mutex;

lazy_static! {
    pub static ref PLUGIN_REGISTRY: Mutex<Vec<Box<dyn Plugin>>> = Mutex::new(Vec::new());
}

pub enum PluginType {
    Driver,
    Algorithm,
    GPU
}

pub trait Plugin : Send + Sync {
    fn name(&self) -> &'static str;
    fn plugin_type(&self) -> PluginType;
    fn initialize(&self);
}

#[macro_export]
macro_rules! register_plugin {
    ($plugin_ty:ty) => {
        pub fn _register_plugin() {
            let plugin: $plugin_ty = Default::default();
            $crate::core::PLUGIN_REGISTRY.lock().unwrap().push(Box::new(plugin));
        }
    };
}

#[cfg(test)]
mod tests {
    use crate::core::Plugin;
    use crate::core::PluginType;

    #[derive(Default)]
    struct TestPlugin {}

    impl Plugin for TestPlugin {
        fn name(&self) -> &'static str {
            "TestPlugin"
        }

        fn plugin_type(&self) -> PluginType {
            PluginType::Driver
        }

        fn initialize(&self) {
            println!("TestPlugin initialized");
        }
    }

    register_plugin!(TestPlugin);

    #[test]
    fn test_register_plugin() {
        _register_plugin();
        let registry = crate::core::PLUGIN_REGISTRY.lock().unwrap();
        assert_eq!(registry.len(), 1);
        assert_eq!(registry[0].name(), "TestPlugin");
    }
}

