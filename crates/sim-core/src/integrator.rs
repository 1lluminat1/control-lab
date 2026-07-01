//! Numerical integration: 4th-order Runge-Kutta (RK4).
//!
//! Spec: SPEC.md §6.3 (component) and §5.3 (the RK4 formula).

// TODO:
//   - [ ] pub fn rk4(s: &CartPoleState, p: &CartPoleParams, force: f64,
//         dt: f64) -> CartPoleState using the k1..k4 scheme from §5.3.
//         (Force is held constant across the step.)
//   - [ ] Test T-1: one step matches a hand-computed value
//         (approx::assert_relative_eq!).
