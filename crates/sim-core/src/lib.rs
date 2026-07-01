//! sim-core — the pure simulation logic for ControlLab.
//!
//! No UI, no web (see SPEC.md §3 NFR-1: this crate must never depend on egui/
//! eframe/wasm). Everything here is testable headless with `cargo test -p
//! sim-core`.
//!
//! Build order and requirements live in SPEC.md. Each module below is a stub
//! with a TODO checklist — you implement the bodies.

pub mod control;
pub mod dynamics;
pub mod integrator;
pub mod simulation;
pub mod state;
