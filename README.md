# WIP N64 Emulator

This is a work-in-progress N64 emulator written in Rust. I've been in the
emulation scene for decades but mostly only focused on the NES/SNES/GB era of
consoles. It's time I conquered 3D.

# Requirements

You'll need a PIF rom (sorry, can't help you out here. Google is your friend)
in the working directory named "boot.rom".  And you'll need a test rom. I
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

![image](https://github.com/sarchar/n64/assets/4928176/7b206464-8b36-4a8a-838c-7800696358ec)

* [Mandelbrot](https://github.com/PeterLemon/N64/tree/master/CP1/Fractal/32BPP/640X480/Mandelbrot/Double)

<img width="640" alt="Screenshot 2023-11-23 123055" src="https://github.com/sarchar/n64/assets/4928176/1a7b5ec1-c1d2-41f6-b131-377f195c0a45">

* First ever graphical output:

![image](https://github.com/sarchar/n64/assets/4928176/01de0e3a-be14-4d40-aa44-223c0f96d9ae)
