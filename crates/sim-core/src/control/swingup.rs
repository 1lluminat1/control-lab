//! Energy swing-up — pumps the pole up from hanging, then hands off to LQR.
//!
//! Spec: SPEC.md §6.4 + §5.6. Phase 3. The showstopper (~40 lines).

// TODO:
//   - [ ] struct SwingUp with the energy gain k_e, a basin threshold, and an
//         inner Lqr.
//   - [ ] impl Controller for SwingUp: while |theta| >= basin, pump energy
//           u = k_e * (E - E_upright) * sign(theta_dot * cos theta)  (clamped);
//         once |theta| < basin, delegate to the inner LQR.
//   - [ ] Demo target: start hanging, swing up, balance.
