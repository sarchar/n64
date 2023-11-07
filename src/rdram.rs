use crate::*;

pub struct RdramInterface {
    ram: Vec<u32>,
}

impl RdramInterface {
    pub fn new() -> RdramInterface {
        RdramInterface { 
            ram: vec![0u32; 2*1024*1024], // 8MiB => 8*1024*1024/4 = 2MiB
        }
    }

    fn read_register(&mut self, offset: usize) -> u32 {
        println!("RDRAM: read_register offset=${:08X}", offset);
        0
    }

    fn write_register(&mut self, value: u32, offset: usize, broadcast: bool) -> &mut Self {
        println!("RDRAM: write_register value=${:08X} offset=${:08X} broadcast={}", value, offset, broadcast);

        self
    }
}

impl Addressable for RdramInterface {
    fn read_u32(&mut self, offset: usize) -> u32 {
        println!("RDRAM: read32 offset=${:08X}", offset);

        match offset {
            // RDRAM memory space
            0x0000_0000..=0x03EF_FFFF => {
                let rdram_address = ((offset & 0x03FF_FFFF) >> 2) as usize;
                if rdram_address < self.ram.len() {
                    self.ram[rdram_address]
                } else { 0 }
            },

            // RDRAM registers
            0x03F0_0000..=0x03FF_FFFF => {
                let register = offset & 0x0007_FFFF;
                self.read_register(register)
            },

            // RI_SELECT
            0x0400_000C => {
                // TODO
                println!("RI: read RI_SELECT");
                0
            },

            // RI_REFRESH
            0x0400_0010 => {
                println!("RI: read RI_REFRESH");
                0
            },
            _ => panic!("RDRAM: unhandled read32 ${:08X}", offset),
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> WriteReturnSignal {
        println!("RDRAM: write32 value=${:08X} offset=${:08X}", value, offset);

        match offset {
            // RDRAM memory space
            0x0000_0000..=0x007F_FFFF => {
                let rdram_address = offset & 0x03FF_FFFF;
                //println!("RDRAM: translated RDRAM address=${:016X}", rdram_address);
                self.ram[(rdram_address >> 2) as usize] = value;
            },

            // "broken" RDRAM memory access
            0x0080_0000..=0x03EF_FFFF => {
                panic!("RDRAM: write32 to broken RDRAM memory access");
            },

            // RDRAM registers
            0x03F0_0000..=0x03FF_FFFF => {
                let broadcast = (offset & 0x0008_0000) != 0;
                let register = offset & 0x0007_FFFF;
                self.write_register(value, register, broadcast);
            },

            // RI_MODE
            0x0400_0000 => {
                println!("RI: write RI_MODE value=${:08X}", value);
                assert!(value == 0 || value == 0x0E);
            },

            // RI_CONFIG
            0x0400_0004 => {
                println!("RI: write RI_CONFIG value=${:08X}", value);
                assert!(value == 0x40);
            },

            // RI_CURRENT_LOAD
            0x0400_0008 => { 
                println!("RI: write RI_CURRENT_LOAD value=${:08X}", value);
                assert!(value == 0);
            },


            // RI_SELECT
            0x0400_000C => {
                println!("RI: write RI_SELECT value=${:08X}", value);
                assert!(value == 0x14);
            },

            // RI_REFRESH
            0x0400_0010 => {
                println!("RI: write RI_REFRESH value=${:08X}", value);
                assert!(value == 0x00063634);
            },

            _ => panic!("RDRAM: unhandled write32 ${:08X}", offset),
        };

        WriteReturnSignal::None
    }
}


