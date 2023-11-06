use crate::*;

pub struct MipsInterface {
}

impl MipsInterface {
    pub fn new() -> MipsInterface {
        MipsInterface { }
    }
}

impl Addressable for MipsInterface {
    fn read_u32(&mut self, offset: usize) -> u32 {
        println!("MI: read32 offset=${:08X}", offset);

        match offset {
            // MI_VERSION
            // https://n64brew.dev/wiki/MIPS_Interface#0x0430_0004_-_MI_VERSION
            0x0_0004 => {
                println!("MI: version read");
                0x0202_0102
            },

            _ => panic!("MI: unhandled read32 ${:08X}", offset),
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> WriteReturnSignal {
        println!("MI: write32 value=${:08X} offset=${:08X}", value, offset);

        match offset {
            0x0_0000 => { 
                println!("MI: write MI_MODE value=${:08X}", value);
            },

            _ => panic!("MI: unhandled write32 ${:08X}", offset),
        };

        WriteReturnSignal::None
    }
}


