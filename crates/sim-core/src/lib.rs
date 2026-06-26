//! sim-core — the pure simulation logic for ControlLab.
//!
//! Right now this is almost empty on purpose. You'll fill it in as you learn,
//! piece by piece: the cart-pole state, the physics, the integrator, and the
//! controllers. The golden rule for this crate: it never imports any UI or web
//! code. That's what keeps it portable and easy to test.

// A tiny placeholder test, just so `cargo test` has something green to show.
// You'll delete this once you write your first real test.
#[cfg(test)]
mod tests {
    #[test]
    fn it_compiles_and_tests_run() {
        assert_eq!(2 + 2, 4);
    }
}
