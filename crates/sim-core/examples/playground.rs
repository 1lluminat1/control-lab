//! A scratch file for experimenting — the same snippet you ran in the online
//! Playground, now living on your own machine.
//!
//! Run it from the project root with:
//!
//!     cargo run -p sim-core --example playground
//!
//! Try changing the numbers and re-running. You can't break anything here.

fn main() {
    let mass = 2.0;
    let gravity = 9.81;
    let weight = mass * gravity;
    println!("weight = {weight} newtons");
}
