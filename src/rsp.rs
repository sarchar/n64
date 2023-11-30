use tracing::{trace, debug, error, info, warn};

use crate::*;

/// N64 Reality Signal Processor
/// Resides on the die of the RCP.
pub struct Rsp {
    mem: Vec<u32>,

    pc: u32,

    si_status: u32,

    semaphore: bool,
}

impl Rsp {
    pub fn new() -> Rsp {
        Rsp {
            mem: vec![0u32; 2*1024], // 8KiB
            pc: 0x0000_0000,
            si_status: 0b0000_0000_0000_0001, // bit 0 (HALTED) set
            semaphore: false,
        }
    }

    pub fn step(&mut self) {
        // don't step if halted
        if (self.si_status & 0x01) != 0 { return; }

        if let Ok(inst) = self.read_u32(self.pc as usize | 0x1000) {
            //info!(target: "RSP", "${:08X}: ${:08X}", self.pc | 0x1000, inst);
            self.pc = (self.pc + 4) & 0x0FFC;
        }
    }

    fn read_register(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RSP", "read32 register offset=${:08X}", offset);

        let value = match offset {
            // SP_STATUS
            0x4_0010 => {
                info!(target: "RSP", "read SP_STATUS");
                self.si_status
            },

            // SP_DMA_BUSY
            0x4_0018 => {
                debug!(target: "RSP", "read SP_DMA_BUSY");

                // mirror of DMA_BUSY in self.si_status
                (self.si_status & 0x04) >> 2
            },

            // SP_SEMAPHORE
            0x4_001C => {
                if !self.semaphore {
                    self.semaphore = true;
                    0
                } else {
                    1
                }
            },

            // SP_PC
            0x8_0000 => {
                debug!(target: "RSP", "read SP_PC");

                self.pc
            },

            _ => {
                error!(target: "RSP", "unknown register read offset=${:08X}", offset);

                0
            },
        };

        Ok(value)
    }

    fn write_register(&mut self, value: u32, offset: usize) {
        match offset {
            // SP_STATUS 
            0x4_0010 => {
                debug!(target: "RSP", "write SP_STATUS value=${:08X}", value);

                // clear halt flag
                if (value & 0x01) != 0 { 
                    self.si_status &= !0x01; 

                    // sum up the first 44 bytes of imem
                    let mut sum: u32 = 0;
                    for i in 0..44 {
                        if let Ok(v) = self.read_u8(0x1000 + i) {
                            sum = sum.wrapping_add(v as u32);
                        }
                    }
                    info!(target: "RSP", "sum of IMEM at start: ${:08X} (if this value is 0x9E2, it is a bootcode program)", sum);

                    info!(target: "RSP", "starting RSP at PC=${:08X}", self.pc);
                }
            },

            // SP_SEMAPHORE
            0x4_001C => {
                if value == 0 {
                    self.semaphore = false;
                }
            },

            // SP_PC
            0x8_0000 => {
                debug!(target: "RSP", "write SP_PC value=${:08X}", value);

                self.pc = value & 0x0FFC;
            },

            _ => {
                warn!(target: "RSP", "unknown write32 register value=${:08X} offset=${:08X}", value, offset);
            }
        };
    }
}

impl Addressable for Rsp {
    fn print_debug_ipl2(&self) {
        let base = 2017;
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+0] , self.mem[base+1] , self.mem[base+2] , self.mem[base+3]);
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+4] , self.mem[base+5] , self.mem[base+6] , self.mem[base+7]);
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+8] , self.mem[base+9] , self.mem[base+10], self.mem[base+11]);
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+12], self.mem[base+13], self.mem[base+14], self.mem[base+15]);
    }

    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RSP", "read32 offset=${:08X}", offset);

        match offset & 0x000F_0000 {
            0x0000_0000..=0x0003_FFFF => {
                let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
                Ok(self.mem[mem_offset as usize])
            },

            0x0004_0000..=0x000B_FFFF => self.read_register(offset & 0x000F_FFFF),

            _ => panic!("invalid RSP read"),
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RSP", "write32 value=${:08X} offset=${:08X}", value, offset);

        match offset & 0x000F_0000 {
            0x0000_0000..=0x0003_FFFF => {
                let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
                if mem_offset < (0x1000>>2) {
                    const TASK_TYPE: usize = 0xFC0 >> 2;
                    const DL_START: usize = 0xFF0 >> 2;
                    const DL_LEN: usize = 0xFF4 >> 2;
                    match mem_offset {
                        DL_START => {
                            info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (DL start)", value, offset);
                        },
                        DL_LEN => {
                            info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (DL length)", value, offset);
                        },
                        TASK_TYPE => {
                            info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (TaskType)", value, offset);
                        },
                        _ => {},
                    }
                } else {
                    //info!(target: "RSPMEM", "write value=${:08X} offset=${:08X}", value, offset);
                }
                self.mem[mem_offset as usize] = value;
            },

            0x0004_0000..=0x000B_FFFF => self.write_register(value, offset & 0x000F_FFFF),

            _ => panic!("invalid RSP write"),
        };

        Ok(WriteReturnSignal::None)
    }
}


