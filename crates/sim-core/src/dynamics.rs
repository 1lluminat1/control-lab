//! Equations of motion — the physics core.
//!
//! Spec: SPEC.md §6.2 (component) and §5.2 (the equations).

// TODO:
//   - [ ] pub fn derivative(s: &CartPoleState, p: &CartPoleParams, force: f64)
//         -> CartPoleState — a direct transcription of §5.2, returning the
//         derivative packed as [x_dot, x_ddot, theta_dot, theta_ddot].
//   - [ ] Test: at theta = PI (hanging), zero velocity, zero force, the angular
//         acceleration is ~0 (it's at rest), and the sign matches "gravity
//         pulls the pole toward theta = PI".

use crate::state::{CartPoleParams, CartPoleState};

pub fn derivative(s: &CartPoleState, p: &CartPoleParams, force: f64) -> CartPoleState {
    CartPoleState { x: (), x_dot: (), theta: (), theta_dot: () }
}