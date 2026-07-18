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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartPoleState {
    pub x: f64,
    pub x_dot: f64,
    pub theta: f64,
    pub theta_dot: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartPoleParams {
    pub cart_mass: f64,
    pub pole_mass: f64,
    pub pole_half_len: f64,
    pub gravity: f64,
    pub force_limit: f64,
}

impl Default for CartPoleParams {
    fn default() -> Self {
        Self {
            cart_mass: 1.0,
            pole_mass: 0.1,
            pole_half_len: 0.5,
            gravity: 9.81,
            force_limit: 10.0,
        }
    }
}

impl std::ops::Add for CartPoleState {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            x_dot: self.x_dot + rhs.x_dot,
            theta: self.theta + rhs.theta,
            theta_dot: self.theta_dot + rhs.theta_dot,
        }
    }
}

impl std::ops::Mul<f64> for CartPoleState {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            x_dot: self.x_dot * rhs,
            theta: self.theta * rhs,
            theta_dot: self.theta_dot * rhs,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ops() {
        let s1 = CartPoleState {
            x: 1.0,
            x_dot: 2.0,
            theta: 3.0,
            theta_dot: 4.0,
        };
        let s2 = CartPoleState {
            x: 0.5,
            x_dot: 0.5,
            theta: 0.5,
            theta_dot: 0.5,
        };

        // Addition
        let sum = s1 + s2;
        assert_eq!(
            sum,
            CartPoleState {
                x: 1.5,
                x_dot: 2.5,
                theta: 3.5,
                theta_dot: 4.5
            }
        );

        // Multiplication
        let scaled = s1 * 2.0;
        assert_eq!(
            scaled,
            CartPoleState {
                x: 2.0,
                x_dot: 4.0,
                theta: 6.0,
                theta_dot: 8.0
            }
        );
    }
}
