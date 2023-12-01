#![allow(non_upper_case_globals)]
use tracing::{debug, error, info};

use crate::*;

const IMask_SP: u32 = 0;
const IMask_SI: u32 = 1;
const IMask_AI: u32 = 2;
const IMask_VI: u32 = 3;
const IMask_PI: u32 = 4;
const IMask_DP: u32 = 5;

pub struct MipsInterface {
    interrupt_mask: u32,
}

impl MipsInterface {
    pub fn new() -> MipsInterface {
        MipsInterface { 
            interrupt_mask: 0,
        }
    }
}

impl Addressable for MipsInterface {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        debug!(target: "MI", "read32 offset=${:08X}", offset);

        let result = match offset {
            // MI_VERSION
            // https://n64brew.dev/wiki/MIPS_Interface#0x0430_0004_-_MI_VERSION
            0x0_0004 => {
                debug!(target: "MI", "version read");
                0x0202_0102
            },

            // MI_MASK
            0x0_000C => {
                debug!(target: "MI", "MI_MASK read");
                self.interrupt_mask
            }

            _ => {
                error!(target: "MI", "unimplemented read32 offset=${:08X}", offset);
                0
            },
        };

        Ok(result)
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        debug!(target: "MI", "write32 value=${:08X} offset=${:08X}", value, offset);

        match offset {
            0x0_0000 => { 
                info!(target: "MI", "write MI_MODE value=${:08X}", value);
            },

            0x0_000C => {
                info!(target: "MI", "write MI_MASK value=${:08X}", value);

                for mask in [IMask_DP, IMask_PI, IMask_VI, IMask_AI, IMask_SI, IMask_SP] {
                    // Clear mask
                    if (value & (1 << ((mask * 2) + 1))) != 0 {
                        self.interrupt_mask &= !(1 << mask);
                    }

                    // Set mask
                    if (value & (1 << (mask * 2))) != 0 {
                        self.interrupt_mask |= 1 << mask;
                    }
                }
            },

            _ => panic!("MI: unhandled write32 ${:08X}", offset),
        };

        Ok(WriteReturnSignal::None)
    }
}


