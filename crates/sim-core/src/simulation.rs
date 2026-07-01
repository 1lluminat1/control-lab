//! The simulation: owns state + params + the active controller, and advances
//! everything one timestep.
//!
//! Spec: SPEC.md §6.5.

// TODO:
//   - [ ] pub struct Simulation holding the state, params, and a
//         Box<dyn Controller>.
//   - [ ] pub fn step(&mut self, dt: f64): ask the controller for a force,
//         clamp it to +/- force_limit, take one rk4 step, store the result.
//   - [ ] Constructor, reset, disturb (apply an impulse), and getters.
//   - [ ] Test T-5: a PID sim started near upright keeps |theta| bounded.
