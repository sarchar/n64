// Co-processor 1 is the VR4300's FPU
// This file implements the FPU emulation

struct InstructionDecode {
    v: u32,
    special: u32,
    fmt: u32,

    ft: usize,
    fs: usize,
    fd: usize,
}

pub struct Cop1 {
    cr: [u64; 32],
    gpr: [f64; 32],

    inst: InstructionDecode,
}

impl Cop1 {
    pub fn new() -> Cop1 {
        Cop1 {
            cr: [0u64; 32],
            gpr: [0f64; 32],

            inst: InstructionDecode {
                v: 0, special: 0, fmt: 0, ft: 0, fs: 0, fd: 0,
            },
        }
    }

    //pub fn gpr(&mut self) -> &[f64] {
    //    &self.gpr
    //}

    //pub fn gpr_mut(&mut self) -> &mut [f64] {
    //    &mut self.gpr
    //}

    pub fn ldc(&mut self, ft: usize, value: u64) {
        self.gpr[ft] = f64::from_bits(value);
    }

    pub fn lwc(&mut self, ft: usize, value: u32) {
        // TODO the cast from f32 to f64 may not be correct (might need to stay as f32?)
        self.gpr[ft] = f32::from_bits(value) as f64;
    }

    pub fn sdc(&mut self, ft: usize) -> u64 {
        self.gpr[ft].to_bits()
    }

    pub fn swc(&mut self, ft: usize) -> u32 {
        (self.gpr[ft] as f32).to_bits()
    }

    pub fn cfc(&mut self, rd: usize) -> u64 {
        self.cr[rd]
    }

    // Move Control to Coprocessor
    // store gpr rt into coprocessor control register rd
    pub fn ctc(&mut self, value: u64, rd: usize) {
        self.cr[rd] = value;
    }

    pub fn mtc(&mut self, _value: u32, rd: usize) {
        if (rd & 0x01) == 0 {
            // write value to low word of fpu gpr
        } else {
            // write high word
        }
    }

    pub fn dmfc(&mut self, rd: usize) -> u64 {
        self.gpr[rd].to_bits()
    }

    pub fn dmtc(&mut self, value: u64, rd: usize) {
        self.gpr[rd] = f64::from_bits(value);
    }

    pub fn special(&mut self, inst: u32) {
        self.inst.v       = inst;
        self.inst.special = self.inst.v & 0x3F;
        self.inst.fmt     = (self.inst.v >> 21) & 0x1F;
        self.inst.ft      = ((self.inst.v >> 16) & 0x1F) as usize;
        self.inst.fs      = ((self.inst.v >> 11) & 0x1F) as usize;
        self.inst.fd      = ((self.inst.v >>  6) & 0x1F) as usize;

        match self.inst.special {
            // ADD
            0b000_000 => {
                eprintln!("COP1: add.{:X} f{}, f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs, self.inst.ft);
            },
            0b000_001 => {
                eprintln!("COP1: sub.{:X} f{}, f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs, self.inst.ft);
            },
            0b000_010 => {
                eprintln!("COP1: mul.{:X} f{}, f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs, self.inst.ft);
            },
            0b000_011 => {
                eprintln!("COP1: div.{:X} f{}, f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs, self.inst.ft);
            },
            0b000_100 => {
                eprintln!("COP1: sqrt.{:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
            },
            0b000_101 => {
                eprintln!("COP1: abs.{:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
            },
            0b000_110 => {
                eprintln!("COP1: mov.{:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
            },
            0b000_111 => {
                eprintln!("COP1: neg.{:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
            },
            0b001_000 => {
                eprintln!("COP1: round.l f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b001_001 => {
                eprintln!("COP1: trunc.l f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b001_010 => {
                eprintln!("COP1: ceil.l f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b001_011 => {
                eprintln!("COP1: floor.l f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b001_100 => {
                eprintln!("COP1: round.w f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b001_101 => {
                eprintln!("COP1: trunc.w f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b001_110 => {
                eprintln!("COP1: ceil.w f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b001_111 => {
                eprintln!("COP1: floor.w f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b100_000 => {
                eprintln!("COP1: cvt.s f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b100_001 => {
                eprintln!("COP1: cvt.d f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b100_100 => {
                eprintln!("COP1: cvt.w f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b100_101 => {
                eprintln!("COP1: cvt.l f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b110_001 => {
                eprintln!("COP1: c.un f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b110_010 => {
                eprintln!("COP1: c.eq f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b110_100 => {
                eprintln!("COP1: c.olt f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b110_101 => {
                eprintln!("COP1: c.ult f{}, f{}", self.inst.fd, self.inst.fs);
            },
            0b110_110 => {
                eprintln!("COP1: c.le f{}, f{}", self.inst.fd, self.inst.fs);
            },
            _ => panic!("CPU: unknown cp1 function: 0b{:02b}_{:03b}", self.inst.special >> 3, self.inst.special & 0x07)
        };
    }
}
