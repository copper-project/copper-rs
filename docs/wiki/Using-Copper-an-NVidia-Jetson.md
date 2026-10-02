Installing Copper in the Jetson (of any type) is basically a matter of installing Rust.

After the standard flashing and installation you just have to install rust itself:
```
$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
$ source ~/.bashrc
```

Then some tooling depending of what you need, all of them are optional:

```bash
$ sudo apt install git
$ cargo install just                                           # to use the justfiles in the repo
$ cargo install cargo-cunew                                    # if you want the Copper project/workspace scaffolder
```

Clone the repo and test drive Copper with an example
```bash
$ git clone https://github.com/copper-project/copper-rs
$ cd copper-rs/examples/cu_caterpillar
$ cargo run -r
```

If you want to cross compile you will need to install the target for the jetson on you workstation, use musl so you are sure the executable is fully static:

```
## one your *workstation*
$ rustup target add aarch64-unknown-linux-musl
                                                     # the musl toolchain
$ sudo apt install musl-tools gcc-aarch64-linux-gnu  # for debian/ubuntu
                                                     # be sure you don't have the python dependency in the cu29-log
                                                     # extract in your Cargo.toml
$ cargo build --release --target aarch64-unknown-linux-musl
$ scp ../../target/aarch64-unknown-linux-musl/release/cu-caterpillar jetson:
$ ssh jetson
$ ./cu-caterpillar
```
