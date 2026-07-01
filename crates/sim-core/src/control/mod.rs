//! Controllers: the `Controller` trait and its implementations.
//!
//! Spec: SPEC.md §6.4. The trait is the abstraction every controller
//! implements, so they're interchangeable behind a `Box<dyn Controller>`.

// TODO:
//   - [ ] Define the Controller trait:
//           fn control(&mut self, state: &CartPoleState, dt: f64) -> f64;
//           fn reset(&mut self) {}
//           fn name(&self) -> &'static str;
//   - [ ] Implement Pid (pid.rs) first; add Lqr / SwingUp in Phase 3.

pub mod lqr;
pub mod pid;
pub mod swingup;
