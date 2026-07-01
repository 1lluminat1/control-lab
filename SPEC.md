# ControlLab — Technical Specification

> An interactive control-systems & robotics simulator in Rust: a cart-pole
> (inverted pendulum) balanced by real controllers — PID, LQR, and an
> energy-based swing-up — with live tuning and telemetry, running natively and
> in the browser via WebAssembly.

**How to use this document.** This is the *what to build*, not the *how to write
it*. Each component lists its required interface (the signatures to implement)
and acceptance criteria (how you know it's correct). You write the bodies. When
you get stuck, bring the specific function + your attempt + the compiler error,
and ask for help.

Section map:
1. Overview & vision
2. Functional requirements
3. Non-functional requirements
4. Architecture
5. Domain & math reference
6. Component specifications
7. Testing requirements
8. Phased milestones & Definition of Done
9. Interface reference (all signatures in one place)
10. Getting unstuck

---

## 1. Overview & vision

**The demo that proves it works:** open the app, see the pole hanging straight
down, click *Swing-up*, and watch it pump itself upright and balance —
rejecting pokes you throw at it — all tunable with sliders, all plotted live.

**Definition of "done" (core project, Phases 0–4):**
- A `sim-core` crate that simulates the cart-pole and runs PID, LQR, and
  swing-up controllers — fully unit- and property-tested, with zero UI/web
  dependencies.
- An `app` crate (egui/eframe) that renders the cart-pole live, exposes sliders
  and buttons, and plots telemetry — building for **both** native desktop and
  web (WASM).
- A public GitHub Pages URL anyone can click, green CI, and a README with a GIF.

---

## 2. Functional requirements

| ID | Requirement |
|----|-------------|
| FR-1 | Represent the cart-pole state as `[x, ẋ, θ, θ̇]` and its physical parameters (masses, pole length, gravity, force limit). |
| FR-2 | Compute the state derivative from the equations of motion (§5.2) given the current state, parameters, and an applied force. |
| FR-3 | Advance the simulation one timestep using **4th-order Runge–Kutta** (RK4), not Euler. |
| FR-4 | Define a `Controller` abstraction that maps a state to a control force, so controllers are interchangeable. |
| FR-5 | Implement a **PID** controller that holds the pole upright when started near upright. |
| FR-6 | Implement an **LQR** controller: linearize about upright, solve the discrete Riccati equation for the gain `K`, apply `u = −K·x`. |
| FR-7 | Implement an **energy swing-up** controller that pumps the pole up from hanging, then **switches to LQR** once inside the basin (`|θ|` small). |
| FR-8 | Clamp every controller's output to the configured force limit. |
| FR-9 | Provide a `Simulation` type that owns state + params + active controller and exposes `step(dt)`. |
| FR-10 | Render the cart, pole, bob, and track in a native window, mapping physics meters → screen pixels. |
| FR-11 | Provide interactive controls: sliders (PID gains, masses, pole length, gravity, LQR Q/R weights) and buttons (Reset, Pause, Disturb). |
| FR-12 | Let the user select the active controller at runtime (PID / LQR / Swing-up→LQR). |
| FR-13 | Plot rolling telemetry (θ, x, control force vs. time) live. |
| FR-14 | Decouple physics from frame rate via a fixed-timestep accumulator. |
| FR-15 | Build to WASM and run the same app in the browser; deploy to GitHub Pages. |

---

## 3. Non-functional requirements

| ID | Requirement |
|----|-------------|
| NFR-1 | **`sim-core` purity.** `sim-core` must never depend on `egui`, `eframe`, or any UI/web crate. All its tests run headless with `cargo test -p sim-core`. |
| NFR-2 | **Determinism.** Given the same `(state, force, dt)`, a step always produces the same next state. |
| NFR-3 | **Precision.** Use `f64` throughout `sim-core`. |
| NFR-4 | **Numerical correctness.** With zero force and no friction, total mechanical energy is conserved (within tolerance) over thousands of steps. |
| NFR-5 | **Quality gates.** `cargo fmt --check`, `cargo clippy -- -D warnings`, and `cargo test` all pass in CI on every push. |
| NFR-6 | **Portability.** Keep `sim-core` `std`-minimal so it could later target an MCU (stretch goal). No I/O in the hot loop. |
| NFR-7 | **Single-threaded hot loop** (WASM has no threads by default). |

---

## 4. Architecture

Cargo workspace, two crates. The split enforces a clean boundary between pure
simulation logic and I/O.

```
control-lab/
├── Cargo.toml                 # [workspace]
├── SPEC.md                    # this document
├── crates/
│   ├── sim-core/              # PURE logic — no UI, no web. Fully tested.
│   │   ├── src/
│   │   │   ├── lib.rs         # module wiring + crate docs
│   │   │   ├── state.rs       # CartPoleState, CartPoleParams (+ vector ops)
│   │   │   ├── dynamics.rs    # derivative(): equations of motion
│   │   │   ├── integrator.rs  # rk4()
│   │   │   ├── simulation.rs  # Simulation: state+params+controller, step()
│   │   │   └── control/
│   │   │       ├── mod.rs     # Controller trait
│   │   │       ├── pid.rs
│   │   │       ├── lqr.rs
│   │   │       └── swingup.rs
│   │   └── examples/
│   │       └── playground.rs  # your scratch sandbox
│   └── app/                   # I/O: egui/eframe. Native AND wasm. (Phase 2+)
│       └── src/
│           ├── main.rs        # native entry point
│           ├── lib.rs         # wasm entry point
│           ├── app.rs         # the eframe::App: layout + frame loop
│           ├── scene.rs       # draw the cart-pole with egui Painter
│           └── plots.rs       # egui_plot telemetry panels
```

**Golden rule (NFR-1):** a dependency arrow from `sim-core` to `app` (or to any
UI/web crate) is a bug. `app` depends on `sim-core`, never the reverse.

**Dependencies** (add only when the phase needs them):
- `sim-core`: `nalgebra` (LQR matrices). Dev: `proptest`, `approx`.
- `app`: `sim-core`, `eframe`, `egui`, `egui_plot`. Web: `trunk`,
  `console_error_panic_hook`, `wasm-bindgen`.

---

## 5. Domain & math reference

### 5.1 State & parameters
State vector (4 numbers): `[x, ẋ, θ, θ̇]`
- `x` — cart position (m); `ẋ` — cart velocity (m/s)
- `θ` — pole angle **measured from upright** (rad): `0` = balanced upright,
  `π` = hanging straight down
- `θ̇` — pole angular velocity (rad/s)

Parameters: cart mass `m_c`, pole mass `m`, pole half-length `l`, gravity `g`,
force limit `F_max`.

### 5.2 Equations of motion
Classic Barto/Sutton cart-pole (uniform-rod pole), given applied force `F`:

```
temp = (F + m·l·θ̇²·sin θ) / (m_c + m)
θ̈    = (g·sin θ − cos θ · temp) / ( l · (4/3 − m·cos²θ / (m_c + m)) )
ẍ    = temp − m·l·θ̈·cos θ / (m_c + m)
```

The derivative of the state `[x, ẋ, θ, θ̇]` is therefore `[ẋ, ẍ, θ̇, θ̈]`.

### 5.3 RK4 integration
One step of size `dt`, where `f(s)` = the derivative from §5.2 (force held
constant across the step):

```
k1 = f(s)
k2 = f(s + k1·dt/2)
k3 = f(s + k2·dt/2)
k4 = f(s + k3·dt)
s_next = s + (k1 + 2·k2 + 2·k3 + k4)·(dt/6)
```

Implementing this cleanly is *why* `CartPoleState` needs `+` and `·scalar`
(§6.1).

### 5.4 PID (Phase 1)
Error `e = 0 − θ`. Output `F = −(Kp·e + Ki·∫e dt + Kd·ė)`, clamped to `±F_max`.
- Integrate `∫e dt` by accumulating `e·dt` each call; consider anti-windup
  (clamp the integral) as a refinement.
- `ė` ≈ `(e − e_prev)/dt`, or use `−θ̇` directly.
- **Signs depend on your angle convention — verify by experiment.** This is
  expected; tuning it is part of the exercise.

### 5.5 LQR (Phase 3)
1. **Linearize** §5.2 about upright (`θ≈0`: `sin θ≈θ`, `cos θ≈1`, `θ̇²≈0`) to get
   `ẋ = A·x + B·u`, with `A` 4×4 and `B` 4×1. Derive these yourself and
   cross-check against Tedrake's *Underactuated Robotics* or SciPy.
2. Choose cost matrices `Q` (4×4, penalizes state error) and `R` (1×1,
   penalizes control effort).
3. Iterate the **discrete algebraic Riccati equation** to convergence:
   ```
   P ← Q + Aᵀ·P·A − Aᵀ·P·B·(R + Bᵀ·P·B)⁻¹·Bᵀ·P·A
   K =            (R + Bᵀ·P·B)⁻¹·Bᵀ·P·A
   ```
   (Discretize `A`,`B` first, e.g. `A_d ≈ I + A·dt`, `B_d ≈ B·dt`.)
4. Control law: `u = −K·x`, clamped to `±F_max`. Use `nalgebra` fixed-size
   types (`Matrix4`, `Vector4`, `Matrix4x1`).

### 5.6 Energy swing-up (Phase 3)
While hanging, pump energy toward the upright energy, then hand off to LQR:

```
u = k_e · (E − E_upright) · sign(θ̇ · cos θ)      // clamp to ±F_max
```

Switch to the LQR controller once `|θ| < basin` (e.g. `0.3 rad`). `E` is the
pole's total mechanical energy; `E_upright` is its energy at the upright rest
position. This is the Åström–Furuta law.

---

## 6. Component specifications

> Signatures below are the **contract**. Implement the bodies. Types are
> illustrative of intent — you may adjust names/shapes as long as the behavior
> and acceptance criteria hold.

### 6.1 `state.rs`
**Responsibility:** the data model + vector-space arithmetic that makes RK4 read
like the math.

Required items:
```rust
pub struct CartPoleState { pub x: f64, pub x_dot: f64, pub theta: f64, pub theta_dot: f64 }
pub struct CartPoleParams { /* cart_mass, pole_mass, pole_half_len, gravity, force_limit: f64 */ }

// field-wise, so RK4 can write `s + k1 * (dt/2.0)`
impl std::ops::Add for CartPoleState { /* ... */ }
impl std::ops::Mul<f64> for CartPoleState { /* ... */ }
```
Derive `Debug, Clone, Copy, PartialEq` on `CartPoleState`. Provide a sensible
`Default` or constructor for `CartPoleParams`.

**Acceptance:** `a + b` adds field-wise; `s * 2.0` scales field-wise; state is
`Copy` (pass by value, no borrow-checker friction).

### 6.2 `dynamics.rs`
**Responsibility:** pure transcription of §5.2.
```rust
pub fn derivative(s: &CartPoleState, p: &CartPoleParams, force: f64) -> CartPoleState
```
Returns the derivative packed as a `CartPoleState` (`x_dot`, `x_ddot`,
`theta_dot`, `theta_ddot`).

**Acceptance:** at `θ = π` (hanging) with zero velocity and zero force, angular
acceleration ≈ 0 (it's at rest). Sign of `θ̈` matches "gravity pulls the pole
down toward `θ = π`."

### 6.3 `integrator.rs`
**Responsibility:** §5.3.
```rust
pub fn rk4(s: &CartPoleState, p: &CartPoleParams, force: f64, dt: f64) -> CartPoleState
```
**Acceptance:** one step matches a hand-computed value
(`approx::assert_relative_eq!`); see §7.

### 6.4 `control/` — the `Controller` trait and implementations
`control/mod.rs`:
```rust
pub trait Controller {
    fn control(&mut self, state: &CartPoleState, dt: f64) -> f64;
    fn reset(&mut self) {}
    fn name(&self) -> &'static str;
}
```
- `control/pid.rs` — `struct Pid { /* gains, integral, prev_error */ }` implementing `Controller` (§5.4).
- `control/lqr.rs` — `struct Lqr { /* gain K */ }` implementing `Controller` (§5.5). Compute `K` once at construction.
- `control/swingup.rs` — `struct SwingUp { /* k_e, basin, inner Lqr */ }` implementing `Controller` (§5.6), switching to LQR inside the basin.

**Acceptance:** each clamps output to `±F_max`; `reset()` clears PID integral
state; swapping controllers is a `Box<dyn Controller>` reassignment.

### 6.5 `simulation.rs`
**Responsibility:** tie it together.
```rust
pub struct Simulation { /* state, params, controller: Box<dyn Controller> */ }
impl Simulation {
    pub fn step(&mut self, dt: f64) { /* control → clamp → rk4 → store */ }
    // + constructor, reset, disturb (apply an impulse), getters
}
```
**Acceptance:** `step` calls the controller, clamps the force, integrates one RK4
step, and updates the stored state. A PID sim started near upright stays upright.

### 6.6 `app/` crate (Phase 2 & 4)
Not scaffolded yet — create it when you reach Phase 2 (`cargo new --lib
crates/app`, add to the workspace). Requirements: FR-10..FR-15.
- `app.rs`: an `eframe::App`; `update()` runs a fixed-timestep accumulator
  (target `dt = 1/240 s`), then draws side panel (controls) + central panel
  (scene + plots), then `ctx.request_repaint()`.
- `scene.rs`: draw track/cart/pole/bob with `ui.painter()`; meters→pixels map.
- `plots.rs`: `egui_plot` charts backed by `VecDeque` ring buffers of samples.
- `lib.rs`: `#[wasm_bindgen]` start hook + `console_error_panic_hook` (Phase 4).

---

## 7. Testing requirements

`sim-core` is where you stand out. Required tests (`cargo test -p sim-core`):

| ID | Test | What it asserts |
|----|------|-----------------|
| T-1 | RK4 single step (unit) | One `rk4` step matches a hand-computed next state within tolerance (`approx`). |
| T-2 | Determinism (property, `proptest`) | Same `(state, force, dt)` ⇒ identical next state. |
| T-3 | Energy conservation (property) | `force = 0`, no friction ⇒ total mechanical energy constant (within tolerance) over thousands of steps. |
| T-4 | Stable equilibrium | Released at `θ = π` with zero velocity stays near `θ = π`. **Do not** assert this for upright — upright is unstable by design. |
| T-5 | PID hold | Started within a few degrees of upright, PID keeps `|θ|` bounded below a threshold. |
| T-6 | LQR gain reference (Phase 3) | `K` for a fixed `(A,B,Q,R)` matches a value cross-checked offline (e.g. SciPy `solve_discrete_are`), pinned in the test. |
| T-7 | LQR settling (Phase 3) | From a small perturbation, closed-loop `θ → 0` below a threshold within T seconds. |

Add `examples/headless.rs` that steps a PID sim and prints the state each step —
a graphics-free way to watch it hold the pole.

---

## 8. Phased milestones & Definition of Done

- [ ] **P0 — Fundamentals & env.** Rust Book Ch 1–6 + Ch 10–11; Rustlings
      through `traits`. Codespace runs `cargo run`/`cargo test`. *(Env: done.)*
- [ ] **P1 — sim-core.** state + dynamics + RK4 + `Controller` + PID +
      `Simulation`; tests T-1..T-5 green; `examples/headless.rs` shows PID
      holding near upright.
- [ ] **P2 — Visualization.** `app` crate: native window, live cart-pole,
      sliders, buttons (Reset/Pause/Disturb), plots, fixed-timestep loop.
      Poke the pole and watch PID recover.
- [ ] **P3 — LQR + swing-up.** `lqr.rs`, `swingup.rs`, controller dropdown;
      tests T-6, T-7. Start hanging → *Swing-up→LQR* → it stands and balances.
- [ ] **P4 — WASM + deploy + polish.** `trunk serve` works; CI (fmt/clippy/test)
      green; GitHub Pages live URL; README with GIF + control-theory section.
- [ ] **P5 — Stretch (optional).** Kalman/observer; MCU target; 2nd plant;
      presets & CSV export; Bevy 3D; `criterion` benchmarks.

Each phase ends with a commit.

---

## 9. Interface reference (all signatures in one place)

```rust
// state.rs
pub struct CartPoleState { pub x: f64, pub x_dot: f64, pub theta: f64, pub theta_dot: f64 }
pub struct CartPoleParams { /* masses, pole_half_len, gravity, force_limit: f64 */ }
impl std::ops::Add for CartPoleState { type Output = Self; /* ... */ }
impl std::ops::Mul<f64> for CartPoleState { type Output = Self; /* ... */ }

// dynamics.rs
pub fn derivative(s: &CartPoleState, p: &CartPoleParams, force: f64) -> CartPoleState;

// integrator.rs
pub fn rk4(s: &CartPoleState, p: &CartPoleParams, force: f64, dt: f64) -> CartPoleState;

// control/mod.rs
pub trait Controller {
    fn control(&mut self, state: &CartPoleState, dt: f64) -> f64;
    fn reset(&mut self) {}
    fn name(&self) -> &'static str;
}
// control/pid.rs      -> struct Pid : Controller
// control/lqr.rs      -> struct Lqr : Controller
// control/swingup.rs  -> struct SwingUp : Controller

// simulation.rs
pub struct Simulation { /* state, params, controller: Box<dyn Controller> */ }
impl Simulation { pub fn step(&mut self, dt: f64); /* new, reset, disturb, getters */ }
```

---

## 10. Getting unstuck

When you hit a wall, the fastest help is a focused ask. Bring:
1. **Which component** you're on (e.g. "T-3 energy conservation" or "`rk4`").
2. **Your attempt** (the code you wrote).
3. **The exact compiler error or wrong output** (copy-paste it verbatim —
   Rust's errors are excellent and reading them together teaches the most).

Good first questions to expect: "why won't this borrow?", "how do I write the
`Add` impl?", "my energy test drifts — bug or tolerance?", "how do I store a
`Box<dyn Controller>`?". Ask small, ask often.

**Suggested build order for P1:** `state` (+ its ops & tests) → `dynamics`
(+ hanging-rest test) → `integrator` (+ T-1) → property tests T-2/T-3/T-4 →
`Controller` trait → `Pid` → `Simulation` → `examples/headless.rs`.
