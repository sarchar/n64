#[allow(unused_imports)]
use tracing::{trace, debug, error, warn, info};

use crate::*;

pub struct RdramInterface {
    ram: Vec<u32>,
    repeat_count: Option<u32>
}

impl RdramInterface {
    pub fn new() -> RdramInterface {
        RdramInterface { 
            ram: vec![0u32; 2*1024*1024], // 8MiB => 8*1024*1024/4 = 2MiB
            repeat_count: None,
        }
    }

    fn read_register(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        debug!(target: "RDRAM", "read_register offset=${:08X}", offset);
        Ok(0)
    }

    fn write_register(&mut self, value: u32, offset: usize, broadcast: bool) -> &mut Self {
        debug!(target: "RDRAM", "write_register value=${:08X} offset=${:08X} broadcast={}", value, offset, broadcast);

        // TODO when writing the Delay register and repeat_count is set, refer to the note here:
        // https://n64brew.dev/wiki/RDRAM#0x02_-_Delay

        self
    }

    pub fn rdram(&self) -> &Vec<u32> {
        &self.ram
    }

    pub fn set_repeat_count(&mut self, repeat_count: Option<u32>) {
        self.repeat_count = repeat_count;
    }
}

impl Addressable for RdramInterface {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RDRAM", "read32 offset=${:08X}", offset);

        match offset {
            // RDRAM memory space
            0x0000_0000..=0x03EF_FFFF => {
                let rdram_address = ((offset & 0x03FF_FFFF) >> 2) as usize;
                if rdram_address < self.ram.len() {
                    Ok(self.ram[rdram_address])
                } else { Ok(0) }
            },

            // RDRAM registers
            0x03F0_0000..=0x03FF_FFFF => {
                let register = offset & 0x0007_FFFF;
                self.read_register(register)
            },

            // RI_SELECT
            0x0400_000C => {
                // TODO
                debug!(target: "RDRAM", "read RI_SELECT");
                Ok(0)
            },

            // RI_REFRESH
            0x0400_0010 => {
                debug!(target: "RDRAM", "read RI_REFRESH");
                Ok(0)
            },
            _ => panic!("RDRAM: unhandled read32 ${:08X}", offset),
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RDRAM", "write32 value=${:08X} offset=${:08X}", value, offset);

        if let Some(_rc) = self.repeat_count {
            if offset != 0x03F8_0008 { // TODO right now we ignore this repeat_count write. See write_register
                todo!("repeat count write to ${:08X} value=${:08X}", offset, value);
            }
            self.repeat_count = None;
        }

        match offset {
            // RDRAM memory space
            0x0000_0000..=0x007F_FFFF => {
                let rdram_address = offset & 0x03FF_FFFF;
                self.ram[(rdram_address >> 2) as usize] = value;
            },

            // "broken" RDRAM memory access
            0x0080_0000..=0x03EF_FFFF => {
                panic!("RDRAM: write32 to broken RDRAM memory access offset=${:08X}", offset);
            },

            // RDRAM registers
            0x03F0_0000..=0x03FF_FFFF => {
                let broadcast = (offset & 0x0008_0000) != 0;
                let register = offset & 0x0007_FFFF;
                self.write_register(value, register, broadcast);
            },

            // RI_MODE
            0x0400_0000 => {
                debug!(target: "RDRAM", "write RI_MODE value=${:08X}", value);
                assert!(value == 0 || value == 0x0E);
            },

            // RI_CONFIG
            0x0400_0004 => {
                debug!(target: "RDRAM", "write RI_CONFIG value=${:08X}", value);
                assert!(value == 0x40);
            },

            // RI_CURRENT_LOAD
            0x0400_0008 => { 
                debug!(target: "RDRAM", "write RI_CURRENT_LOAD value=${:08X}", value);
                assert!(value == 0);
            },


            // RI_SELECT
            0x0400_000C => {
                debug!(target: "RDRAM", "write RI_SELECT value=${:08X}", value);
                assert!(value == 0x14);
            },

            // RI_REFRESH
            0x0400_0010 => {
                debug!(target: "RDRAM", "write RI_REFRESH value=${:08X}", value);
                assert!(value == 0x00063634);
            },

            _ => panic!("RDRAM: unhandled write32 ${:08X}", offset),
        };

        Ok(WriteReturnSignal::None)
    }

    fn read_block(&mut self, offset: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        if offset < 0x0080_0000 {
            // why doesn't std::vec have copy_into(offset, source_slice)?
            let (_, right) = self.ram.split_at_mut(offset >> 2);
            let (left, _) = right.split_at_mut((length >> 2) as usize);
            Ok(left.to_owned())
        } else {
            todo!("not likely");
        }
    }

    fn write_block(&mut self, offset: usize, block: &[u32]) -> Result<WriteReturnSignal, ReadWriteFault> {
        if offset < 0x0080_0000 {
            // why doesn't std::vec have copy_into(offset, source_slice)?
            let (_, right) = self.ram.split_at_mut(offset >> 2);
            let (left, _) = right.split_at_mut(block.len());
            left.copy_from_slice(block);
            Ok(WriteReturnSignal::None)
        } else {
            todo!("not likely");
        }
    }
}


