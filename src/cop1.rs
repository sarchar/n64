// Co-processor 1 is the VR4300's FPU
// This file implements the FPU emulation
#![allow(non_upper_case_globals)]

#[allow(unused_imports)]
use tracing::{debug, error, info};

use num_traits::{Float, Zero};

use crate::cpu::InstructionFault;

const _Cop1_Revision     : usize = 0;
const _Cop1_ControlStatus: usize = 31;

const Format_Single: u8 = 0b10_000;
const Format_Double: u8 = 0b10_001;
const Format_Word  : u8 = 0b10_100;
const Format_Long  : u8 = 0b10_101;

const FpeCause_Inexact      : u64 = 0b000001;
const FpeCause_Underflow    : u64 = 0b000010;
const FpeCause_Overflow     : u64 = 0b000100;
const FpeCause_DivByZero    : u64 = 0b001000;
const FpeCause_Invalid      : u64 = 0b010000;
const FpeCause_Unimplemented: u64 = 0b100000;

#[allow(dead_code)]
extern "C" {
    pub static c_fe_upward: i32;
    pub static c_fe_downward: i32;
    pub static c_fe_tonearest: i32;
    pub static c_fe_towardzero: i32;

    pub static c_fe_divbyzero: i32;
    pub static c_fe_inexact: i32;
    pub static c_fe_invalid: i32;
    pub static c_fe_overflow: i32;
    pub static c_fe_underflow: i32;
    pub static c_fe_all_except: i32;

    fn c_fesetround(round: i32) -> i32;
    fn c_fegetround() -> i32;
    fn c_feclearexcept(excepts: i32) -> i32;
    fn c_fetestexcept(excepts: i32) -> i32;

    fn c_f32_add(a: f32, b: f32) -> f32;
    fn c_f64_add(a: f64, b: f64) -> f64;
    fn c_f32_sub(a: f32, b: f32) -> f32;
    fn c_f64_sub(a: f64, b: f64) -> f64;
    fn c_f32_mul(a: f32, b: f32) -> f32;
    fn c_f64_mul(a: f64, b: f64) -> f64;
    fn c_f32_div(a: f32, b: f32) -> f32;
    fn c_f64_div(a: f64, b: f64) -> f64;
}

static fe_upward    : &i32 = unsafe { &c_fe_upward };
static fe_downward  : &i32 = unsafe { &c_fe_downward };
static fe_tonearest : &i32 = unsafe { &c_fe_tonearest };
static fe_towardzero: &i32 = unsafe { &c_fe_towardzero };
static fe_divbyzero : &i32 = unsafe { &c_fe_divbyzero };
static fe_inexact   : &i32 = unsafe { &c_fe_inexact };
static fe_invalid   : &i32 = unsafe { &c_fe_invalid };
static fe_overflow  : &i32 = unsafe { &c_fe_overflow };
static fe_underflow : &i32 = unsafe { &c_fe_underflow };
static fe_all_except: &i32 = unsafe { &c_fe_all_except };

fn fesetround(round: &i32)      -> i32 { unsafe { c_fesetround(*round) } }
fn fegetround()                 -> i32 { unsafe { c_fegetround() } }
fn feclearexcept(excepts: &i32) -> i32 { unsafe { c_feclearexcept(*excepts) } }
fn fetestexcept(excepts: &i32)  -> i32 { unsafe { c_fetestexcept(*excepts) } }

// f32 and f64 need QUIET_NAN_BIT
trait SignallingNan {
    fn is_signalling_nan(&self) -> bool;
}

impl SignallingNan for f32 {
    fn is_signalling_nan(&self) -> bool {
        static QUIET_NAN_BIT: u32 = 0x0040_0000;
        self.is_nan() && (self.to_bits() & QUIET_NAN_BIT) == 0
    }
}

impl SignallingNan for f64 {
    fn is_signalling_nan(&self) -> bool {
        static QUIET_NAN_BIT: u64 = 0x0008_0000_0000_0000;
        self.is_nan() && (self.to_bits() & QUIET_NAN_BIT) == 0
    }
}

struct InstructionDecode {
    v: u32,
    special: u32,
    fmt: u8,
    cond: u8,

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

    // Set to the result of compare instructions. COC in the datasheet.
    condition_signal: bool,

    inst: InstructionDecode,

    // system rounding mode
    system_rounding_mode: i32,
}

impl Cop1 {
    pub fn new() -> Cop1 {
        Cop1 {
            fcr_implementation_revision: 0xA00,
            fcr_control_status: 0,

            fgr: [Fgr{ as_u64: 0 }; 32],
            fr_bit: false,
            condition_signal: false,

            inst: InstructionDecode {
                v: 0, special: 0, fmt: 0, cond: 0, ft: 0, fs: 0, fd: 0,
            },

            system_rounding_mode: fegetround(),
        }
    }

    // Set the FR bit from the Status register.
    pub fn set_fr(&mut self, fr: bool) {
        self.fr_bit = fr;
    }

    pub fn condition_signal(&self) -> bool {
        self.condition_signal
    }

    //pub fn fgr32(&mut self, r: usize) -> f32 {
    //    let shift = 32 - ((r & 0x01) << 5);
    //    f32::from_bits(self.fgr[r & !0x01].as_u64 >> shift)
    //}

    // Update the cause bits in fcr_control_status and if the corresponding enable bit is set,
    // raise an exception. The unimplemented instruction bit (E) always generates an exception
    fn update_cause(&mut self, cause: u64, update_flag: bool) -> Result<(), InstructionFault> {
        self.fcr_control_status |= cause << 12;
        let unimplemented_instruction = (cause & 0x20) != 0;
        if unimplemented_instruction || ((cause & ((self.fcr_control_status >> 7) & 0x1F)) != 0) {
            Err(InstructionFault::FloatingPointException)
        } else {
            if update_flag {
                // update flags when the exception is not generated
                self.fcr_control_status |= (cause & 0x1F) << 2;
            }
            Ok(())
        }
    }

    pub fn ldc(&mut self, ft: usize, value: u64) -> Result<(), InstructionFault> {
        if !self.fr_bit {
            // Place double word into ft and ft+1. The operation is undefined if bit 0 of ft is not 0
            self.fgr[ft & !0x01].as_u64 = value;
        } else { 
            self.fgr[ft].as_u64 = value;
        }
        Ok(())
    }

    pub fn lwc(&mut self, ft: usize, value: u32) -> Result<(), InstructionFault> {
        if !self.fr_bit {
            // Place value into either the low word or high word based on bit 0 of ft
            let shift = (ft & 0x01) << 5; // 0 or 32
            let old = unsafe { self.fgr[ft & !0x01].as_u64 };
            self.fgr[ft & !0x01].as_u64 = (old & (0xFFFF_FFFF_0000_0000 >> shift)) | ((value as u64) << shift); 
        } else { 
            // The datasheet says that resulting value of the high order 32-bits of the register are undefined
            // but the n64-systemtests program requires us to preserve the value
            let old = unsafe { self.fgr[ft].as_u64 };
            self.fgr[ft].as_u64 = (old & 0xFFFF_FFFF_0000_0000) | (value as u64);
        }
        Ok(())
    }

    pub fn sdc(&mut self, ft: usize) -> Result<u64, InstructionFault> {
        let result = if !self.fr_bit {
            // Get double word from ft and ft+1. The operation is undefined if bit 0 of ft is not 0
            unsafe { self.fgr[ft & !0x01].as_u64 }
        } else { 
            unsafe { self.fgr[ft].as_u64 }
        };

        Ok(result)
    }

    pub fn swc(&mut self, ft: usize) -> Result<u32, InstructionFault> {
        let result = if !self.fr_bit {
            // Get value from either the low word or high word based on bit 0 of ft
            let shift = (ft & 0x01) << 5; // 0 or 32
            let old = unsafe { self.fgr[ft & !0x01].as_u64 };
            (old >> shift) & 0xFFFF_FFFF
        } else { 
            // The datasheet says that resulting value of the high order 32-bits of the register are undefined
            // but the n64-systemtests program requires us to preserve the value
            let old = unsafe { self.fgr[ft].as_u64 };
            old & 0xFFFF_FFFF
        };

        Ok(result as u32)
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
                // Mask out zero bits
                let value = value & 0x0183_FFFF;
                // Remove the Cause bits from value
                let cause = (value >> 12) & 0x3F;
                self.fcr_control_status = value & !0x0003_F000;
                self.update_cause(cause, false)?;
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

    fn begin_fpu_op(&mut self) {
        let round_mode = match self.fcr_control_status & 0x03 {
            0b00 => fe_tonearest,
            0b01 => fe_towardzero,
            0b10 => fe_upward,
            0b11 => fe_downward,
            _ => panic!("not valid"),
        };

        fesetround(round_mode);
        feclearexcept(fe_all_except);

        self.fcr_control_status &= !0x0001F000;
    }

    fn check_input<T: Float + Zero + SignallingNan>(&mut self, value: T) -> Result<T, InstructionFault> {
        if value.is_nan() {
            let signalling = value.is_signalling_nan();
            if signalling { // sNaN is not supported
                self.update_cause(FpeCause_Unimplemented, true)?;
            } else { // qNaNs cause invalid instruction
                self.update_cause(FpeCause_Invalid, true)?;
            }
        } else if value.is_subnormal() {
            self.update_cause(FpeCause_Unimplemented, true)?;
        }

        Ok(value)
    }

    // don't like that this function is duplicated
    fn end_fpu_op_f32(&mut self, result: f32) -> Result<f32, InstructionFault> {
        let excepts = fetestexcept(fe_all_except);
        //error!(target: "CPU", "got excepts={}", excepts);

        let mut retval = Ok(result);
        let mut cause = 0;

        if (excepts & *fe_inexact) != 0 {
            cause |= FpeCause_Inexact;
        }

        if (excepts & *fe_invalid) != 0 {
            cause |= FpeCause_Invalid;
            retval = Ok(f32::from_bits(0x7FBFFFFF));
        }

        if (excepts & *fe_overflow) != 0 {
            cause |= FpeCause_Overflow;
        }

        if (excepts & *fe_divbyzero) != 0 {
            cause |= FpeCause_DivByZero;
        }

        let is_underflow = (excepts & *fe_underflow) != 0;

        if result.is_nan() {
            retval = Ok(f32::from_bits(0x7FBFFFFF));
        } else if result.is_subnormal() || is_underflow {
            let flush_subnormals = ((self.fcr_control_status >> 24) & 0x01) != 0;
            let underflow_enabled = (self.fcr_control_status & (0x02 << 7)) != 0;
            let inexact_enabled = (self.fcr_control_status & (0x01 << 7)) != 0;
            if !flush_subnormals || underflow_enabled || inexact_enabled {
                // overwrite other exceptions on Unimpl
                cause = FpeCause_Unimplemented;
            } else {
                cause |= FpeCause_Inexact | FpeCause_Underflow;
            }

            // set result depending on rounding mode
            let is_neg = (result.to_bits() >> 31) != 0;
            retval = Ok(match self.fcr_control_status & 0x03 {
                0b00 | 0b01 => { if !is_neg { 0.0 } else { -0.0 } }, // tonearest, towardzero
                // these two return the minimum positive normal number
                0b10 => { if !is_neg { f32::from_bits(0x00800000) } else { -0.0 } }, // upward
                0b11 => { if !is_neg { 0.0 } else { -f32::from_bits(0x00800000) } }, // downward
                _ => panic!("not valid"),
            });
        }

        if cause != 0 {
            self.update_cause(cause, true)?;
        }

        fesetround(&self.system_rounding_mode);

        retval
    }

    // don't like that this function is duplicated
    // DO NOT CHANGE THIS FUNCTION WITHOUT CHANGING THE ABOVE ONE AS WELL!!
    // THEY SHOULD BE IDENTICAL EXCEPT FOR F32->F64 CHANGES
    fn end_fpu_op_f64(&mut self, result: f64) -> Result<f64, InstructionFault> {
        let excepts = fetestexcept(fe_all_except);
        //error!(target: "CPU", "got excepts={}", excepts);

        let mut retval = Ok(result);
        let mut cause = 0;

        if (excepts & *fe_inexact) != 0 {
            cause |= FpeCause_Inexact;
        }

        if (excepts & *fe_invalid) != 0 {
            cause |= FpeCause_Invalid;
            retval = Ok(f64::from_bits(0x7FF7_FFFF_FFFF_FFFF));
        }

        if (excepts & *fe_overflow) != 0 {
            cause |= FpeCause_Overflow;
        }

        if (excepts & *fe_divbyzero) != 0 {
            cause |= FpeCause_DivByZero;
        }

        let my_is_subnormal = |v: f64| -> bool {
            let v = v.to_bits();
            (v & 0x7FF0_0000_0000_0000) == 0 && (v & 0x000F_FFFF_FFFF_FFFF) != 0
        };

        let is_underflow = (excepts & *fe_underflow) != 0;

        if result.is_nan() {
            retval = Ok(f64::from_bits(0x7FF7_FFFF_FFFF_FFFF));
        } else if my_is_subnormal(result) || is_underflow {
            let flush_subnormals = ((self.fcr_control_status >> 24) & 0x01) != 0;
            let underflow_enabled = (self.fcr_control_status & (0x02 << 7)) != 0;
            let inexact_enabled = (self.fcr_control_status & (0x01 << 7)) != 0;
            if !flush_subnormals || underflow_enabled || inexact_enabled {
                // overwrite other exceptions on Unimpl
                cause = FpeCause_Unimplemented;
            } else {
                cause |= FpeCause_Inexact | FpeCause_Underflow;
            }

            // set result depending on rounding mode
            retval = Ok(match self.fcr_control_status & 0x03 {
                0b00 | 0b01 => { if result >= 0.0 { 0.0 } else { -0.0 } }, // tonearest, towardzero
                // these two return the minimum positive normal number
                // positive normal number is 1 in the exponent ant the fraction all zeroes
                0b10 => { if result >= 0.0 { f64::from_bits(0x0010_0000_0000_0000) } else { -0.0 } }, // upward
                0b11 => { if result >= 0.0 { 0.0 } else { -f64::from_bits(0x0010_0000_0000_0000) } }, // downward
                _ => panic!("not valid"),
            });
        }

        if cause != 0 {
            self.update_cause(cause, true)?;
        }

        fesetround(&self.system_rounding_mode);

        retval
    }

    pub fn special(&mut self, inst: u32) -> Result<(), InstructionFault> {
        self.inst.v       = inst;
        self.inst.special = self.inst.v & 0x3F;
        self.inst.fmt     = ((self.inst.v >> 21) & 0x1F) as u8;
        self.inst.ft      = ((self.inst.v >> 16) & 0x1F) as usize;
        self.inst.fs      = ((self.inst.v >> 11) & 0x1F) as usize;
        self.inst.fd      = ((self.inst.v >>  6) & 0x1F) as usize;
        self.inst.cond    = (self.inst.v & 0x0F) as u8;

        if !self.fr_bit {
            self.inst.fs &= !0x01;
            //self.inst.ft &= !0x01;
            //self.inst.fd &= !0x01;
        }

        match self.inst.special {
            // ADD
            0b000_000 => { // ADD.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f32 })?;
                        let input_b = self.check_input(unsafe { self.fgr[self.inst.ft].as_f32 })?;

                        let result = self.end_fpu_op_f32(unsafe { c_f32_add(input_a, input_b) })?;

                        // need to clear the upper bits, so we don't use as_f32
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f64 })?;
                        let input_b = self.check_input(unsafe { self.fgr[self.inst.ft].as_f64 })?;

                        let result = self.end_fpu_op_f64(unsafe { c_f64_add(input_a, input_b) })?;

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_001 => { // SUB.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f32 })?;
                        let input_b = self.check_input(unsafe { self.fgr[self.inst.ft].as_f32 })?;

                        let result = self.end_fpu_op_f32(unsafe { c_f32_sub(input_a, input_b) })?;

                        // need to clear the upper bits, so we don't use as_f32
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f64 })?;
                        let input_b = self.check_input(unsafe { self.fgr[self.inst.ft].as_f64 })?;

                        let result = self.end_fpu_op_f64(unsafe { c_f64_sub(input_a, input_b) })?;

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_010 => { // MUL.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f32 })?;
                        let input_b = self.check_input(unsafe { self.fgr[self.inst.ft].as_f32 })?;

                        let result = self.end_fpu_op_f32(unsafe { c_f32_mul(input_a, input_b) })?;

                        // need to clear the upper bits, so we don't use as_f32
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f64 })?;
                        let input_b = self.check_input(unsafe { self.fgr[self.inst.ft].as_f64 })?;

                        let result = self.end_fpu_op_f64(unsafe { c_f64_mul(input_a, input_b) })?;

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_011 => { // DIV.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f32 })?;
                        let input_b = self.check_input(unsafe { self.fgr[self.inst.ft].as_f32 })?;

                        let result = self.end_fpu_op_f32(unsafe { c_f32_div(input_a, input_b) })?;

                        // need to clear the upper bits, so we don't use as_f32
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f64 })?;
                        let input_b = self.check_input(unsafe { self.fgr[self.inst.ft].as_f64 })?;

                        let result = self.end_fpu_op_f64(unsafe { c_f64_div(input_a, input_b) })?;

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_100 => { // SQRT.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f32 })?;

                        let result = self.end_fpu_op_f32(input_a.sqrt())?;

                        // need to clear the upper bits, so we don't use as_f32
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f64 })?;

                        let result = self.end_fpu_op_f64(input_a.sqrt())?;

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_101 => { // ABS.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f32 })?;

                        let result = self.end_fpu_op_f32(input_a.abs())?;

                        // need to clear the upper bits, so we don't use as_f32
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .D
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f64 })?;

                        let result = self.end_fpu_op_f64(input_a.abs())?;

                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b000_110 => { // MOV.fmt -- no call to begin_fpu_op and end_fpu_op
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
            0b000_111 => { // NEG.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        //let result = unsafe { -self.fgr[self.inst.fs].as_f32 };
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f32 })?;

                        let result = self.end_fpu_op_f32(-input_a)?;

                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Double => { // .S
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f64 })?;

                        let result = self.end_fpu_op_f64(-input_a)?;
                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_000 => { // ROUND.L.fmt
                self.begin_fpu_op();
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
            0b001_001 => { // TRUNC.L.fmt
                self.begin_fpu_op();
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
            0b001_010 => { // CEIL.L.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32.ceil() };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_011 => { // FLOOR.L.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32.floor() };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { },
                };
                Ok(())
            },
            0b001_100 => { // ROUND.W.fmt
                self.begin_fpu_op();
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
            0b001_101 => { // TRUNC.W.fmt
                self.begin_fpu_op();
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
            0b001_110 => { // CEIL.W.fmt
                self.begin_fpu_op();
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
            0b001_111 => { // FLOOR.W.fmt
                self.begin_fpu_op();
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
            0b100_000 => { // CVT.S.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Double => { // .D
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f64 })?;

                        let result = self.end_fpu_op_f32(input_a as f32)?;
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Word => { // .W
                        let input_a = unsafe { self.fgr[self.inst.fs].as_u32 as i32 };

                        let result = self.end_fpu_op_f32(input_a as f32)?;
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    Format_Long => { // .L
                        let input_a = unsafe { self.fgr[self.inst.fs].as_u64 as i64 };

                        // From the datasheet, CVT.S/D generate unimplemented operation exceptions
                        // when converting from a 64-bit integer unless all the bits 63 through 55
                        // of a 64-bit integer are 0 or 1
                        let ubits = (input_a as u64) & 0xFF80_0000_0000_0000;
                        if ubits != 0 && ubits != 0xFF80_0000_0000_0000 {
                            let _ = self.update_cause(FpeCause_Unimplemented, true)?;
                        }

                        let result = self.end_fpu_op_f32(input_a as f32)?;
                        self.fgr[self.inst.fd].as_u64 = result.to_bits() as u64;
                    },
                    _ => { 
                        fesetround(&self.system_rounding_mode); // restore rounding changed by begin_fpu_op
                        let _ = self.update_cause(FpeCause_Unimplemented, true)?;
                    },
                };
                Ok(())
            },
            0b100_001 => { // CVT.D.fmt
                info!(target: "COP1", "cvt.d.${:X} f{}, f{}", self.inst.fmt, self.inst.fd, self.inst.fs);
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let input_a = self.check_input(unsafe { self.fgr[self.inst.fs].as_f32 })?;

                        let result = self.end_fpu_op_f64(input_a as f64)?;
                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    Format_Word => { // .W
                        let input_a = unsafe { self.fgr[self.inst.fs].as_u32 as i32 };

                        let result = self.end_fpu_op_f64(input_a as f64)?;
                        self.fgr[self.inst.fd].as_f64 = result as f64;
                    },
                    Format_Long => { // .L
                        let input_a = unsafe { self.fgr[self.inst.fs].as_u64 as i64 };

                        // From the datasheet, CVT.S/D generate unimplemented operation exceptions
                        // when converting from a 64-bit integer unless all the bits 63 through 55
                        // of a 64-bit integer are 0 or 1
                        let ubits = (input_a as u64) & 0xFF80_0000_0000_0000;
                        if ubits != 0 && ubits != 0xFF80_0000_0000_0000 {
                            let _ = self.update_cause(FpeCause_Unimplemented, true)?;
                        }

                        let result = self.end_fpu_op_f64(input_a as f64)?;
                        self.fgr[self.inst.fd].as_f64 = result;
                    },
                    _ => { 
                        fesetround(&self.system_rounding_mode); // restore rounding changed by begin_fpu_op
                        let _ = self.update_cause(FpeCause_Unimplemented, true)?;
                    },
                };
                Ok(())
            },
            0b100_100 => { // CVT.W.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32 as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Word => { // .W
                        return Err(InstructionFault::FloatingPointException);
                    },
                    Format_Long => { // .L
                        let result = unsafe { self.fgr[self.inst.fs].as_u64 as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    _ => { 
                        error!(target: "CPU", "unhandled CVT.W format {:X}", self.inst.fmt);
                        return Err(InstructionFault::Unimplemented);
                    },
                };
                Ok(())
            },
            0b100_101 => { // CVT.L.fmt
                self.begin_fpu_op();
                match self.inst.fmt {
                    Format_Single => { // .S
                        let result = unsafe { self.fgr[self.inst.fs].as_f32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Double => { // .D
                        let result = unsafe { self.fgr[self.inst.fs].as_f64 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Word => { // .W
                        let result = unsafe { self.fgr[self.inst.fs].as_u64 as u32 };
                        self.fgr[self.inst.fd].as_u64 = result as u64;
                    },
                    Format_Long => { // .L
                        return Err(InstructionFault::FloatingPointException);
                    },
                    _ => { 
                        error!(target: "CPU", "unhandled CVT.L format {:X}", self.inst.fmt);
                        return Err(InstructionFault::Unimplemented);
                    },
                };
                Ok(())
            },

            // the FPU compare function is pretty neat, as it's always the same compare but the
            // resulting condition flag depends on a mask in the instruction I wonder if there
            // would be any noticeable improvement in speed if each compare was special cased,
            // rather than the generic compare here... i.e., "C.F.S" should just set condition
            // to False and not actually do any compares.
            0b110_000..=0b111_111 => { // C.cond.fmt
                let (lt, eq, unord) = match self.inst.fmt {
                    Format_Single => { // .S
                        let left  = unsafe { self.fgr[self.inst.fs].as_f32 };
                        let right = unsafe { self.fgr[self.inst.ft].as_f32 };
                        (left < right, left == right, false)
                    },
                    Format_Double => { // .D
                        let left  = unsafe { self.fgr[self.inst.fs].as_f64 };
                        let right = unsafe { self.fgr[self.inst.ft].as_f64 };
                        (left < right, left == right, false)
                    },
                    _ => {
                        return Err(InstructionFault::Unimplemented)
                    },
                };
                let condition = (((self.inst.cond & 0x04) != 0) && lt)
                              | (((self.inst.cond & 0x02) != 0) && eq)
                              | (((self.inst.cond & 0x01) != 0) && unord);

                // Set bit 23 in FCR[31] to the condition result
                self.fcr_control_status = (self.fcr_control_status & !0x800000) | ((condition as u64) << 23);

                // And quicker access
                self.condition_signal = condition;

                Ok(())
            },

            /*
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
            */
            _ => {
                error!(target: "CPU", "COP1: unknown cp1 function: 0b{:02b}_{:03b}", self.inst.special >> 3, self.inst.special & 0x07);
                Ok(())
            },
        }
    }
}
