use cu_linux_resources::LINUX_RESOURCE_SLOT_NAMES;

#[test]
fn linux_resources_exposes_fixed_serial_and_i2c_slots() {
    assert_eq!(
        LINUX_RESOURCE_SLOT_NAMES,
        &[
            "serial_acm0",
            "serial_acm1",
            "serial_acm2",
            "serial_usb0",
            "serial_usb1",
            "serial_usb2",
            "i2c0",
            "i2c1",
            "i2c2",
            "gpio_out0",
            "gpio_out1",
            "gpio_out2",
            "gpio_in0",
            "gpio_in1",
            "gpio_in2",
        ]
    );
}
