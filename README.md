# ControlLab

An interactive control-systems & robotics simulator written in Rust — a
cart-pole (inverted pendulum) balanced by real controllers (PID, LQR, and an
energy-based swing-up), with live tuning and telemetry. It runs natively and,
later, in the browser via WebAssembly.

🚧 **Status:** early days — this is being built as a learning project, one
phase at a time.

## 📋 The spec

**[SPEC.md](SPEC.md)** is the source of truth: requirements, architecture, the
control-theory math, per-component interfaces, tests, and phased milestones. The
`sim-core` modules are scaffolded as stubs with TODO checklists that point back
into the spec — implement the bodies against it.

## Project layout

```
control-lab/
├── Cargo.toml          # workspace root (ties the crates together)
├── SPEC.md             # the technical specification (build against this)
└── crates/
    └── sim-core/       # pure simulation logic (no UI, no web) — fully testable
        ├── src/
        │   ├── lib.rs          # module wiring
        │   ├── state.rs        # CartPoleState, CartPoleParams        (stub)
        │   ├── dynamics.rs     # equations of motion                  (stub)
        │   ├── integrator.rs   # RK4                                  (stub)
        │   ├── simulation.rs   # ties it together, step()             (stub)
        │   └── control/        # Controller trait + PID / LQR / swing-up (stubs)
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
