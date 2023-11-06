use crate::Addressable;

/// N64 Reality Signal Processor
/// Resides on the die of the RCP.
pub struct Rsp {
    mem: Vec<u32>,
    si_status: u32,
}

impl Rsp {
    pub fn new() -> Rsp {
        Rsp {
            mem: vec![0u32; 2*1024], // 8KiB
            si_status: 0b0000_0000_0000_0001, // bit 0 (HALTED) set
        }
    }

    pub fn print_debug_ipl2(&self) {
        let base = 2017;
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+0] , self.mem[base+1] , self.mem[base+2] , self.mem[base+3]);
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+4] , self.mem[base+5] , self.mem[base+6] , self.mem[base+7]);
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+8] , self.mem[base+9] , self.mem[base+10], self.mem[base+11]);
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+12], self.mem[base+13], self.mem[base+14], self.mem[base+15]);
    }

    fn read_register(&mut self, offset: usize) -> u32 {
        match offset {
            // SP_STATUS
            0x4_0010 => {
                println!("RSP: read SP_STATUS");
                self.si_status
            },

            // SP_DMA_BUSY
            0x4_0018 => {
                println!("RSP: read SP_DMA_BUSY");

                // mirror of DMA_BUSY in self.si_status
                (self.si_status & 0x04) >> 2
            },

            _ => panic!("Unknown RSP register read ${:08X}", offset)
        }
    }

    fn write_register(&mut self, _value: u32, offset: usize) {
        match offset {
            // SP_STATUS 
            0x4_0010 => {
                println!("RSP: write SP_STATUS");
            },
            _ => panic!("Unknown RSP register write ${:08X}", offset)
        };
    }
}

impl Addressable for Rsp {
    fn read_u32(&mut self, offset: usize) -> u32 {
        println!("RSP: read32 offset=${:08X}", offset);

        match offset & 0x000F_0000 {
            0x0000_0000..=0x0003_FFFF => {
                let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
                self.mem[mem_offset as usize]
            },

            0x0004_0000..=0x000B_FFFF => self.read_register(offset & 0x000F_FFFF),
            _ => panic!("invalid RSP read"),
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> &mut Self {
        println!("RSP: write32 value=${:08X} offset=${:08X}", value, offset);

        match offset & 0x000F_0000 {
            0x0000_0000..=0x0003_FFFF => {
                let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
                self.mem[mem_offset as usize] = value;
            },

            0x0004_0000..=0x000B_FFFF => self.write_register(value, offset & 0x000F_FFFF),

            _ => panic!("invalid RSP write"),
        };
        self
    }
}


