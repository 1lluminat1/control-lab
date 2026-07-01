//! LQR controller — optimal balance about upright.
//!
//! Spec: SPEC.md §6.4 + §5.5. Phase 3. Uses `nalgebra` (add it as a dependency
//! when you start this).

// TODO:
//   - [ ] Linearize §5.2 about upright to get A (4x4), B (4x1). Derive and
//         cross-check against Tedrake / SciPy.
//   - [ ] struct Lqr holding the gain K; compute K once at construction by
//         iterating the discrete Riccati equation to convergence.
//   - [ ] impl Controller for Lqr: u = -K*x, clamped to +/- force_limit.
//   - [ ] Tests T-6 (gain vs. reference) and T-7 (settling).
