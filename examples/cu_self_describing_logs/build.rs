#[cfg(feature = "self-describing-logs")]
use cu_self_describing_payloads as payloads;

#[cfg(feature = "self-describing-logs")]
cu29::prelude::gen_cumsgs!("copperconfig.ron");

fn main() {
    #[cfg(feature = "self-describing-logs")]
    {
        cu29_build::setup();
        println!("cargo::rerun-if-changed=copperconfig.ron");
        let catalog = cumsgs::value_decode_catalog().expect("describe recorded payloads");
        cu29_build::catalog::write_value_decode_catalog("catalog.rs", &catalog)
            .expect("package recorded payloads");
    }
}
