# ControlLab

An interactive control-systems & robotics simulator written in Rust — a
cart-pole (inverted pendulum) balanced by real controllers (PID, LQR, and an
energy-based swing-up), with live tuning and telemetry. It runs natively and,
later, in the browser via WebAssembly.

🚧 **Status:** early days — this is being built as a learning project, one
phase at a time.

## Project layout

```
control-lab/
├── Cargo.toml          # workspace root (ties the crates together)
└── crates/
    └── sim-core/       # pure simulation logic (no UI, no web) — fully testable
        ├── src/lib.rs          # the library (mostly empty for now)
        └── examples/playground.rs   # a scratch file to experiment in
```

More crates (the egui/eframe desktop+web app) get added in later phases.

## Running it

You'll need Rust installed: <https://rustup.rs>

```bash
# Run the scratch example and see some output
cargo run -p sim-core --example playground

# Run the tests
cargo test
```
