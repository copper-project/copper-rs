use cu_iceoryx2_sink::IceoryxSink;
use cu_iceoryx2_src::IceoryxSrc;
use cu_rp_gpio::RPGpioPayload;

// Type aliases for the config.
pub type MyIceoryxSink = IceoryxSink<RPGpioPayload>;
pub type MyIceoryxSrc = IceoryxSrc<RPGpioPayload>;
