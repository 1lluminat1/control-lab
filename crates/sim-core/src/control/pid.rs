//! PID controller — holds the pole upright once it's near upright.
//!
//! Spec: SPEC.md §6.4 + §5.4. Phase 1.

// TODO:
//   - [ ] struct Pid with gains (kp, ki, kd) plus integral + prev_error state.
//   - [ ] impl Controller for Pid: error e = 0 - theta; output
//         F = -(kp*e + ki*integral + kd*e_dot), clamped to +/- force_limit.
//   - [ ] reset() clears the integral and prev_error.
//   - [ ] Signs depend on your angle convention — verify by experiment.
