fn main() {
    cu29_build::setup();
    {% if pgs_enabled %}
    println!("cargo:rerun-if-changed=target/pgs/selected.config.ron");
    {% endif %}
}
