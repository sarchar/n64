#[allow(unused_imports)]
use tracing::{debug, error, info};

use crate::*;

/// N64 PIF-ROM, where the boot rom is stored
/// boot_rom is big endian data
/// Essentially part of the PeripheralInterface but the PIF-ROM and features abstracted out
pub struct PifRom {
    boot_rom: Vec<u8>,    
    ram: Vec<u32>,
    command_finished: bool,
}

impl PifRom {
    pub fn new(boot_rom: Vec<u8>, pi: &mut peripheral::PeripheralInterface) -> PifRom {
        let mut ram = vec![0u32; 16]; // 64 byte RAM

        // CRC the IPL3 code, which starts after the 64 byte header and goes up to address 0x1000
        let mut crc: u64 = 0;
        for i in (0x40..0x1000).step_by(4) {
            crc += match pi.read_u32((0x1000_0000 | i) as usize) {
                Ok(value) => value as u64,
                Err(_) => panic!("could not read cartridge, this shouldn't happen"),
            }
        }

        // HACK! To simulate the CIC exchange, we need certain seed values at 0x7E4 in PIF ram
        match crc {
            0xD057C85244 => { // CIC 6102, GoldenEye
                info!(target: "PIF", "CIC 6102 detected");
                ram[9] = 0x00003F3F;
            },

            0x11A49F60E96 => { // CIC CIC 6105, Ocarina of Time, Majora's Mask
                info!(target: "PIF", "CIC 6105 detected");
                ram[9] = 0x00029100;
            },

            _ => {
                info!(target: "PIF", "unknown IPL3/CIC checksum ${:08X}. game probably won't run", crc);
            }
        };

        PifRom {
            boot_rom: boot_rom,
            ram: ram,
            command_finished: false,
        }
    }
}

impl Addressable for PifRom {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        debug!(target: "PIF", "read32 offset=${:08X}", offset);

        if offset < 1984 {
            Ok(((self.boot_rom[offset+1] as u32) << 24)
               | ((self.boot_rom[offset+0] as u32) << 16)
               | ((self.boot_rom[offset+3] as u32) << 8)
               | (self.boot_rom[offset+2] as u32))
        } else if offset == 0x7FC {
            // HACK! data is always available (bit 7 set)
            if self.command_finished { 
                self.command_finished = false;
                Ok(0x00000080)
            } else { 
                Ok(0x00000000)
            }
        } else {
            let ram_offset = offset.wrapping_sub(0x7C0) >> 2;
            if ram_offset < 16 {
                Ok(self.ram[ram_offset as usize])
            } else {
                panic!("unhandled PIF read offset=${:08X}", offset)
            }
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        if offset < 0x7C0 {
            error!(target: "PIF", "invalid write value=${:08X} offset=${:08X}", value, offset);
        } else if offset == 0x7FC {
            //panic!("PIF: write command port");
            if (value & 0x10) != 0 {
                debug!(target: "PIF", "disable PIF-ROM access");
            } else if (value & 0x20) != 0 {
                debug!(target: "PIF", "CPU checksum ready");
                self.command_finished = true;
            } else if (value & 0x40) != 0 {
                debug!(target: "PIF", "run checksum");
            } else if (value & 0x08) != 0 {
                debug!(target: "PIF", "Yay! BOOT IS DONE!");
            } else if (value & 0x07) != 0 {
                panic!("PIF: not implemented PIF command ${:08X}", value);
            }
        } else {
            let ram_offset = offset.wrapping_sub(0x7C0) >> 2;
            if ram_offset < 16 {
                //info!(target: "PIF-RAM", "write value=${:08X} offset=${:08X}", value, offset);
                self.ram[ram_offset as usize] = value;
            } else {
                panic!("PIF: invalid write value=${:08X} offset=${:08X}", value, offset);
            }
        }

        Ok(WriteReturnSignal::None)
    }

    fn write_u16(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        // SH incorrectly overwrites lower bytes with zeroes
        let offset = offset & !0x01;
        let shift = 16 - ((offset & 0x02) << 3);
        self.write_u32(value << shift, offset & !0x02)
    }

    fn write_u8(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        // SB incorrectly overwrites lower bytes with zeroes
        let shift = 24 - ((offset & 0x03) << 3);
        self.write_u32(value << shift, offset & !0x03)
    }
}


