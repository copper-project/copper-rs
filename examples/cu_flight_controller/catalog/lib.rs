//! Host-side catalog generation for the flight-controller graphs.
extern crate cu29 as bevy;

#[path = "../src/messages.rs"]
mod messages;

mod mcu {
    cu29::prelude::gen_cumsgs!("../mcu_config.ron");

    pub fn write() {
        cu29_build::catalog::write_value_decode_catalog(
            "mcu_catalog.rs",
            &cumsgs::default::value_decode_catalog_with(|builder| {
                builder.register::<cu_msp_lib::structs::MspRequest>();
            })
            .expect("describe MCU payloads"),
        )
        .expect("package MCU catalog");
    }
}

#[cfg(feature = "compute")]
mod compute {
    cu29::prelude::gen_cumsgs!("../compute_config.ron");

    pub fn write() {
        cu29_build::catalog::write_value_decode_catalog(
            "compute_catalog.rs",
            &cumsgs::value_decode_catalog_with(|builder| {
                builder.register::<cu_zed::ZedCalibrationBundle>();
                builder.register::<cu_zed::ZedRigTransforms>();
            })
            .expect("describe compute payloads"),
        )
        .expect("package compute catalog");
    }
}

/// Package the selected graphs into the application's build output directory.
pub fn write() {
    mcu::write();
    #[cfg(feature = "compute")]
    compute::write();
}
