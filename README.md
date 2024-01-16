# WIP N64 Emulator

This is a work-in-progress N64 emulator written in Rust. I've been in the
emulation scene for decades but mostly only focused on the NES/SNES/GB era of
consoles. It's time I conquered 3D.

# Requirements

You'll need a PIF rom (sorry, can't help you out here. Google is your friend)
in the ./bios/ directory "pifrom.v64".  And you'll need a program to run. I
recommend [n64-systemtest](https://github.com/lemmy-64/n64-systemtest).

# Building

Clone the repository and execute:

```
$ cargo run --release --features gui -- n64-systemtest.z64
```

Or if you want to use the debugger,

```
$ cargo run --release -- n64-systemtest.z64 -D
```

# Screenshots

* Current test rate:

![image](https://github.com/sarchar/n64/assets/4928176/c6a6b774-afde-40d6-9acd-8f0157002369)

* "squaredemo.n64" from [n64-sdk-demo](https://github.com/jsdf/n64-sdk-demo)

![image](https://github.com/sarchar/n64/assets/4928176/a22fb087-a33c-444c-8526-a21a2d031a2e)

* "onetri.n64" N64 demo:

![image](https://github.com/sarchar/n64/assets/4928176/132a48b2-2aea-4b1e-9900-5df393774221)

* [Mandelbrot](https://github.com/PeterLemon/N64/tree/master/CP1/Fractal/32BPP/640X480/Mandelbrot/Double)

<img width="640" alt="Screenshot 2023-11-23 123055" src="https://github.com/sarchar/n64/assets/4928176/1a7b5ec1-c1d2-41f6-b131-377f195c0a45">

* First ever graphical output:

![image](https://github.com/sarchar/n64/assets/4928176/01de0e3a-be14-4d40-aa44-223c0f96d9ae)
