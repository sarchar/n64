# WIP N64 Emulator

This is a work-in-progress N64 emulator written in Rust.  I've never written a 
large project in Rust before this.

Currently, it is capable of running a few commercial games via HLE.  My goal 
is to create a reasonably fast but accurate emulator.  

# Features

* Fully passes all but the RDP tests in n64-systemtest, using the interpreter core
* JIT dynrec CPU core, passing *almost* all n64-systemtest tests
* avx256/512-based RSP core
* Full PIF emulation (requires boot rom to run)
* Basic HLE support
* WIP graphical debugger
* Audio partially supported via LLE
  
# Requirements

You'll need a PIF rom (sorry, can't help you out here. Google is your friend)
in the ./bios/ directory "pifrom.v64".  And you'll need a program to run. I
recommend [n64-systemtest](https://github.com/lemmy-64/n64-systemtest).

You'll also need to have SDL2 installed.  Follow the instructions at
[https://crates.io/crates/sdl](https://crates.io/crates/sdl2).

# Building

Clone the repository and execute:

```
$ cargo run --release -- n64-systemtest.z64
```

Pass `-h` to see a list of command line arguments.  I recommend enabling docking when
using the debugger with `-D`.

# Screenshots

* Debugger (WIP):
![image](https://github.com/user-attachments/assets/2200ba80-2afa-43ef-b44f-6dca49370b8c)

* Current test rate (failed 4 out of 3555 tests):
![image](https://github.com/sarchar/n64/assets/4928176/568e5671-af3b-4144-b7b9-6386299ac27b)

* Super Mario 64:
![image](https://github.com/sarchar/n64/assets/4928176/a0be3e4c-0321-4854-ba26-d001741f44e5)

* Ocarina of Time:

Latest:
![image](https://github.com/sarchar/n64/assets/4928176/e40b5fde-ed41-4fd2-8f7e-e2e7e12eeb8b)

Old:
![image](https://github.com/sarchar/n64/assets/4928176/bd3a537f-187b-4b5a-a2c7-e6b43ede9cc7)
![image](https://github.com/sarchar/n64/assets/4928176/bf109d74-6892-4980-b4ec-1b30cd87b059)


* "morphdemo.n64" from NuSystems

![image](https://github.com/sarchar/n64/assets/4928176/d135b1d4-824a-4b0a-984d-405da7c4213f)

* "squaredemo.n64" from [n64-sdk-demo](https://github.com/jsdf/n64-sdk-demo)

![image](https://github.com/sarchar/n64/assets/4928176/a22fb087-a33c-444c-8526-a21a2d031a2e)

* "onetri.n64" N64 demo:

![image](https://github.com/sarchar/n64/assets/4928176/132a48b2-2aea-4b1e-9900-5df393774221)

* [Mandelbrot](https://github.com/PeterLemon/N64/tree/master/CP1/Fractal/32BPP/640X480/Mandelbrot/Double)

<img width="640" alt="Screenshot 2023-11-23 123055" src="https://github.com/sarchar/n64/assets/4928176/1a7b5ec1-c1d2-41f6-b131-377f195c0a45">

* First ever graphical output:

![image](https://github.com/sarchar/n64/assets/4928176/01de0e3a-be14-4d40-aa44-223c0f96d9ae)
