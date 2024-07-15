You can leapfrog your Copper development here by generating a copper project from a template.

Prerequisites:

Rust / Cargo of course:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Then install cargo-generate:

```bash
cargo install cargo-generate
```

Now you can generate a new Copper project, for example execute this from this directory to build an end to end Copper project:

```bash
cargo cunew [destination folder]
```

It should ask you what is your project name and it should generate a full project for you.