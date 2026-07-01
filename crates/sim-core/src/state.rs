//! Data model: the cart-pole's state and physical parameters.
//!
//! Spec: SPEC.md §6.1 (component) and §5.1 (meaning of each field).

// TODO:
//   - [ ] CartPoleState { x, x_dot, theta, theta_dot: f64 }, deriving
//         Debug, Clone, Copy, PartialEq.
//   - [ ] CartPoleParams (cart_mass, pole_mass, pole_half_len, gravity,
//         force_limit) with a Default or constructor.
//   - [ ] impl std::ops::Add for CartPoleState — field-wise add.
//   - [ ] impl std::ops::Mul<f64> for CartPoleState — field-wise scale.
//         (These two make RK4 read like the math: `s + k1 * (dt / 2.0)`.)
//   - [ ] Unit test: a + b and s * 2.0 behave field-wise.
