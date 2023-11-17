// Co-processor 1 is the VR4300's FPU
// This file implements the FPU emulation
#![allow(non_upper_case_globals)]

#[allow(unused_imports)]
use tracing::{debug, error, info};

use crate::cpu::InstructionFault;

const _Cop1_Revision     : usize = 0;
const _Cop1_ControlStatus: usize = 31;

const Format_Single: u8 = 0b10_000;
const Format_Double: u8 = 0b10_001;
const Format_Word  : u8 = 0b10_100;
const Format_Long  : u8 = 0b10_101;

struct InstructionDecode {
    v: u32,
    special: u32,
    fmt: u8,

    ft: usize,
    fs: usize,
    fd: usize,
}

// FGRs can be used as integer GPRs, single-precision floats, or double-precision floats
// we use a union to allow all options
#[derive(Copy, Clone)]
#[repr(C)]
union Fgr {
    as_u64: u64,
    as_u32: u32,
    as_f32: f32,
    as_f64: f64,
}

pub struct Cop1 {
    // The FPU only has two control registers, 0 (Implementation/Revision) and 31 (Control/Status)
    // and instead of a sparse array, we maintain two separate variables
    fcr_implementation_revision: u64,
    fcr_control_status: u64,

    // 32 FGRs that can be used in various ways
    fgr: [Fgr; 32],

    // FR bit from the cop0 Status register
    // when true, indicates 32 floating point registers vs 16
    fr_bit: bool,

    inst: InstructionDecode,
}

impl Cop1 {
    pub fn new() -> Cop1 {
        Cop1 {
            fcr_implementation_revision: 0xA00,
            fcr_control_status: 0,

            fgr: [Fgr{ as_u64: 0 }; 32],
            fr_bit: false,

            inst: InstructionDecode {
                v: 0, special: 0, fmt: 0, ft: 0, fs: 0, fd: 0,
            },
        }
    }

    // Set the FR bit from the Status register.
    pub fn set_fr(&mut self, fr: bool) {
        self.fr_bit = fr;
    }

    //pub fn fgr32(&mut self, r: usize) -> f32 {
    //    let shift = 32 - ((r & 0x01) << 5);
    //    f32::from_bits(self.fgr[r & !0x01].as_u64 >> shift)
    //}

    pub fn ldc(&mut self, ft: usize, value: u64) -> Result<(), InstructionFault> {
        self.fgr[ft].as_f64 = f64::from_bits(value);
        Ok(())
    }

    pub fn lwc(&mut self, ft: usize, value: u32) -> Result<(), InstructionFault> {
        // TODO the cast from f32 to f64 may not be correct (might need to stay as f32?)
        self.fgr[ft].as_f32 = f32::from_bits(value);
        Ok(())
    }

    pub fn sdc(&mut self, ft: usize) -> Result<u64, InstructionFault> {
        Ok(unsafe {self.fgr[ft].as_u64})
    }

    pub fn swc(&mut self, ft: usize) -> Result<u32, InstructionFault> {
        Ok(unsafe {self.fgr[ft].as_u32})
    }

    // Move Control Word from Coprocessor
    pub fn cfc(&mut self, rd: usize) -> Result<u32, InstructionFault> {
        match rd {
            0 => {
                Ok(self.fcr_implementation_revision as u32)
            },

            31 => {
                Ok(self.fcr_control_status as u32)
            },

            _ => Err(InstructionFault::CoprocessorUnusable),
        }
    }

    // Move Control Word to Coprocessor
    pub fn ctc(&mut self, value: u64, rd: usize) -> Result<(), InstructionFault> {
        match rd {
            0 => {
                // fcr0 is a read-only register
                Ok(())
            },

            31 => {
                self.fcr_control_status = value & 0x0183_FFFF;
                Ok(())
            },

            _ => Err(InstructionFault::CoprocessorUnusable),
        }
    }

    // Move (general purpose value) from Coprocessor
    pub fn mfc(&mut self, rd: usize) -> Result<u32, InstructionFault> {
        if self.fr_bit { 
            Ok(unsafe {self.fgr[rd].as_u32})
        } else {
            // If FR isn't set, even registers are indexed with the odd bit
            let shift = (rd & 0x01) << 5;
            Ok(unsafe {(self.fgr[rd & !0x01].as_u64 >> shift) as u32})
        }
    }

    // Move (general purpose value) to Coprocessor
    pub fn mtc(&mut self, value: u32, rd: usize) -> Result<(), InstructionFault> {
        if self.fr_bit { 
            unsafe { 
                self.fgr[rd].as_u64 = (self.fgr[rd].as_u64 & !0xFFFF_FFFF) | (value as u64);
            }
        } else {
            // If FR isn't set, even registers are indexed with the odd bit
            let shift = (rd & 0x01) << 5;
            let rd = rd & !0x01;
            unsafe {
                self.fgr[rd].as_u64 = (self.fgr[rd].as_u64 & (0xFFFF_FFFF_0000_0000 >> shift)) | ((value as u64) << shift);
            }
        };
        Ok(())
    }

    pub fn dmfc(&mut self, rd: usize) -> Result<u64, InstructionFault> {
        // only even registers available if FR bit is not set
        let rd = if self.fr_bit { rd } else { rd & !0x01 };

        Ok(unsafe {self.fgr[rd].as_u64})
    }

    pub fn dmtc(&mut self, value: u64, rd: usize) -> Result<(), InstructionFault> {
        // only even registers available if FR bit is not set
        if self.fr_bit { 
            self.fgr[rd].as_u64 = value;
        } else {
            self.fgr[rd & !0x01].as_u64 = value;
        };

        Ok(())
    }

    pub fn special(&mut self, inst: u32) -> Result<(), InstructionFault> {
        self.inst.v       = inst;
        self.inst.special = self.inst.v & 0x3F;
        self.inst.fmt     = ((self.inst.v >> 21) & 0x1F) as u8;
        self.inst.ft      = ((self.inst.v >> 16) & 0x1F) as usize;
        self.inst.fs      = ((self.inst.v >> 11) & 0x1F) as usize;
        self.inst.fd      = ((self.inst.v >>  6) & 0x1F) as usize;

        if !self.fr_bit {
            self.inst.fs &= !0x01;
            //self.inst.ft &= !0x01;
            //self.inst.fd &= !0x01;
        }

        match self.inst.special {
            // ADD
            0b000_000 => {
                debug!(target: "COP1", "add.{:X} f{}, f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs, self.inst.ft);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f32 + self.fgr[self.inst.ft].as_f32
                        };

                        //unsafe {
                        //    info!(target: "COP1", "add result: fs=${:08X} ft=${:08X} result=${:16X}", self.fgr[self.inst.fs].as_f32.to_bits(), self.fgr[self.inst.ft].as_f32.to_bits(), result.to_bits() as u64);
                        //}

                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f64 + self.fgr[self.inst.ft].as_f64
                        };

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_001 => {
                debug!(target: "COP1", "sub.{:X} f{}, f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs, self.inst.ft);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f32 - self.fgr[self.inst.ft].as_f32
                        };

                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f64 - self.fgr[self.inst.ft].as_f64
                        };

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_010 => {
                debug!(target: "COP1", "mul.{:X} f{}, f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs, self.inst.ft);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f32 * self.fgr[self.inst.ft].as_f32
                        };

                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f64 * self.fgr[self.inst.ft].as_f64
                        };

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_011 => {
                debug!(target: "COP1", "div.{:X} f{}, f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs, self.inst.ft);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f32 / self.fgr[self.inst.ft].as_f32
                        };

                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f64 / self.fgr[self.inst.ft].as_f64
                        };

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_100 => {
                debug!(target: "COP1", "sqrt.{:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f32.sqrt()
                        };

                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f64.sqrt()
                        };

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_101 => {
                debug!(target: "COP1", "abs.{:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f32.abs()
                        };

                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe {
                            self.fgr[self.inst.fs].as_f64.abs()
                        };

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_110 => {
                debug!(target: "COP1", "mov.{:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        self.fgr[self.inst.fd].as_u64 = unsafe { self.fgr[self.inst.fs].as_u64 };
                    },
                    Format_Double => { // .D
                        self.fgr[self.inst.fd].as_f64 = unsafe { self.fgr[self.inst.fs].as_f64 };
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_111 => {
                debug!(target: "COP1", "neg.{:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { -self.fgr[self.inst.fs].as_f32 };
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .S
                        let result = unsafe { -self.fgr[self.inst.fs].as_f64 };
                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_000 => {
                debug!(target: "COP1", "round.l f{}, f{}", self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32.round() };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64.round() };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_001 => {
                debug!(target: "COP1", "trunc.l f{}, f{}", self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32 as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64 as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_010 => {
                debug!(target: "COP1", "ceil.l f{}, f{}", self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32.ceil() };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_011 => {
                debug!(target: "COP1", "floor.l f{}, f{}", self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32.floor() };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_100 => {
                debug!(target: "COP1", "round.w f{}, f{}", self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32.round() as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64.round() as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_101 => {
                debug!(target: "COP1", "trunc.w f{}, f{}", self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32 as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64 as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_110 => {
                debug!(target: "COP1", "ceil.w f{}, f{}", self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32.ceil() as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64.ceil() as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_111 => {
                debug!(target: "COP1", "floor.w f{}, f{}", self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32.floor() as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64.floor() as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b100_000 => {
                debug!(target: "COP1", "cvt.s.{:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64 as f32 };
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Word => { // .W
                        let result = unsafe { self.fgr[self.inst.fs].as_u32 as f32 };
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Long => { // .L
                        let result = unsafe { self.fgr[self.inst.fs].as_u64 as f32 };
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b100_001 => {
                debug!(target: "COP1", "cvt.d f{}, f{}", self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32 };
                        self.fgr[self.inst.fd].as_f64 = result as f64;
                    },
                    Format_Word => { // .W
                        let result = unsafe { self.fgr[self.inst.fs].as_u32 };
                        self.fgr[self.inst.fd].as_f64 = result as f64;
                    },
                    Format_Long => { // .L
                        let result = unsafe { self.fgr[self.inst.fs].as_u64 };
                        self.fgr[self.inst.fd].as_f64 = result as f64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b100_100 => {
                debug!(target: "COP1", "cvt.w.{:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32 as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b100_101 => {
                debug!(target: "COP1", "cvt.l f{}, f{}", self.inst.fd, self.inst.fs);
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b110_000 => {
                debug!(target: "COP1", "c.f f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b110_001 => {
                debug!(target: "COP1", "c.un f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b110_010 => {
                debug!(target: "COP1", "c.eq f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b110_011 => {
                debug!(target: "COP1", "c.ueq f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b110_100 => {
                debug!(target: "COP1", "c.olt f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b110_101 => {
                debug!(target: "COP1", "c.ult f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b110_110 => {
                debug!(target: "COP1", "c.le f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b110_111 => {
                debug!(target: "COP1", "c.ule f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b111_000 => {
                debug!(target: "COP1", "c.sf f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b111_001 => {
                debug!(target: "COP1", "c.ngle f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b111_010 => {
                debug!(target: "COP1", "c.seq f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b111_011 => {
                debug!(target: "COP1", "c.ngl f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b111_100 => {
                debug!(target: "COP1", "c.lt f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b111_101 => {
                debug!(target: "COP1", "c.nge f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b111_110 => {
                debug!(target: "COP1", "c.le f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            0b111_111 => {
                debug!(target: "COP1", "c.ngt f{}, f{}", self.inst.fd, self.inst.fs);
                Ok(())
            },
            _ => {
                error!(target: "CPU", "COP1: unknown cp1 function: 0b{:02b}_{:03b}", self.inst.special >> 3, self.inst.special & 0x07);
                Ok(())
            },
        }
    }
}
