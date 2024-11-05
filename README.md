# NewBLE
Repo for the Bluetooth server and client code

# Intro to Embedded Rust
- See <https://doc.rust-lang.org/book/> for a tutorial on the language itself.
- See <https://docs.rust-embedded.org/book/intro/index.html> on extra information specific for embedded programming.

# Installation
See
- <https://docs.esp-rs.org/book/introduction.html>
- <https://docs.esp-rs.org/no_std-training/>

for installation instructions.
Choose the `no_std` option for the `exp32` with `Xtensa` architecture.

# Flash and run
After following installation you should have `cargo` and `espflash` installed so `cargo run --release` when the board is plugged in should work. If it hangs on `Connecting...` cancel the command (`Ctrl-c`) and try again or try pressing the `RESET` button of the development board.

# Documentation
`cargo doc --open` should open the documentation for all dependencies and the project in your browser.
