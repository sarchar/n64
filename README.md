# WIP N64 Emulator

This is a work-in-progress N64 emulator written in Rust. I've been in the
emulation scene for decades but mostly only focused on the NES/SNES/GB era of
consoles. It's time I conquered 3D.

# Requirements

You'll need a PIF rom (sorry, can't help you out here. Google is your friend)
in the working directory named "boot.rom".  And you'll need a test rom. I
recommend [n64-systemtest](https://github.com/lemmy-64/n64-systemtest).

*NOTE*: Since the TLB isn't implemented (yet), the n64-systemtest as-is will
restart repeatedly. If you want to see the tests conclude, you'll need to
comment out the tests in `n64-systemtest/src/tests/testlist.rs` relating to the
TLB. That would be the tests starting with super::tlb::WiredRandom up-to
super::traps::delay::TNEDelay2.

# Building

Clone the repository and execute:

```
$ cargo run --release --features gui -- n64-systemtest.z64
```

Or if you want to use the debugger,

```
$ cargo run --release -- n64-systemtest.z64 -D
```

