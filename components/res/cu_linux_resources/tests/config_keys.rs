use cu_linux_resources::{
    I2C0_DEV_KEY, I2C1_DEV_KEY, I2C2_DEV_KEY, SERIAL0_BAUDRATE_KEY, SERIAL0_DEV_KEY,
    SERIAL0_PARITY_KEY, SERIAL0_STOPBITS_KEY, SERIAL0_TIMEOUT_MS_KEY, SERIAL1_DEV_KEY,
    SERIAL2_DEV_KEY, SERIAL3_DEV_KEY, SERIAL4_DEV_KEY, SERIAL5_DEV_KEY,
};

#[test]
fn linux_resources_uses_dev_key_naming_for_serial_and_i2c() {
    assert_eq!(SERIAL0_DEV_KEY, "serial0_dev");
    assert_eq!(SERIAL1_DEV_KEY, "serial1_dev");
    assert_eq!(SERIAL2_DEV_KEY, "serial2_dev");
    assert_eq!(SERIAL3_DEV_KEY, "serial3_dev");
    assert_eq!(SERIAL4_DEV_KEY, "serial4_dev");
    assert_eq!(SERIAL5_DEV_KEY, "serial5_dev");
    assert_eq!(I2C0_DEV_KEY, "i2c0_dev");
    assert_eq!(I2C1_DEV_KEY, "i2c1_dev");
    assert_eq!(I2C2_DEV_KEY, "i2c2_dev");
}

#[test]
fn serial_slot_zero_exposes_full_setting_keys() {
    assert_eq!(SERIAL0_BAUDRATE_KEY, "serial0_baudrate");
    assert_eq!(SERIAL0_PARITY_KEY, "serial0_parity");
    assert_eq!(SERIAL0_STOPBITS_KEY, "serial0_stopbits");
    assert_eq!(SERIAL0_TIMEOUT_MS_KEY, "serial0_timeout_ms");
}
