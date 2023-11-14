use std::fs;

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
    pub fn new(boot_rom_file: &str) -> PifRom {
        let boot_rom_data = fs::read(boot_rom_file).expect("Boot rom not found");

        // HACK! To simulate the CIC exchange, we need seeds at location 0x7E4 in PIF memory
        let mut ram = vec![0u32; 16]; // 64 byte RAM
        ram[9] = 0x0000_3F3F;

        PifRom {
            boot_rom: boot_rom_data,
            ram: ram,
            command_finished: false,
        }
    }
}

impl Addressable for PifRom {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        println!("PIF: read32 offset=${:08X}", offset);

        if offset < 1984 {
            Ok(((self.boot_rom[offset+0] as u32) << 24)
               | ((self.boot_rom[offset+1] as u32) << 16)
               | ((self.boot_rom[offset+2] as u32) << 8)
               | (self.boot_rom[offset+3] as u32))
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
            eprintln!("PIF: invalid write value=${:08X} offset=${:08X}", value, offset);
        } else if offset == 0x7FC {
            //panic!("PIF: write command port");
            if (value & 0x10) != 0 {
                println!("PIF: disable PIF-ROM access");
            } else if (value & 0x20) != 0 {
                println!("PIF: CPU checksum ready");
                self.command_finished = true;
            } else if (value & 0x40) != 0 {
                println!("PIF: run checksum");
            } else if (value & 0x08) != 0 {
                println!("PIF: Yay! BOOT IS DONE!");
            } else if (value & 0x07) != 0 {
                panic!("PIF: not implemented PIF command ${:08X}", value);
            }
        } else {
            let ram_offset = offset.wrapping_sub(0x7C0) >> 2;
            if ram_offset < 16 {
                self.ram[ram_offset as usize] = value;
            } else {
                panic!("PIF: invalid write value=${:08X} offset=${:08X}", value, offset);
            }
        }

        Ok(WriteReturnSignal::None)
    }
}


