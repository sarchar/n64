use std::fs;

use crate::Addressable;

/// N64 Peripheral Interface
/// Connects EEPROM, cartridge, controllers, and more
pub struct PeripheralInterface {
    cartridge_rom: Vec<u8>,
}

impl PeripheralInterface {
    pub fn new(cartridge_rom_file: &str) -> PeripheralInterface {
        let cartridge_rom = fs::read(cartridge_rom_file).expect("Could not open cartridge ROM file");

        PeripheralInterface {
            cartridge_rom: cartridge_rom,
        }
    }
}

impl Addressable for PeripheralInterface {
    fn read_u32(&mut self, offset: usize) -> u32 {
        println!("PI: read32 offset=${:08X}", offset);

        if offset < 0x0800_0000 {
            panic!("N64DD read")
        } else if offset < 0x1000_0000 {
            panic!("Cartridge SRAM/FlashRAM read")
        } else if offset < 0x1FC0_0000 {
            let cartridge_rom_offset = offset & 0x0FFF_FFFF;
            println!("CART: read32 offset=${:08X}", cartridge_rom_offset);

            if cartridge_rom_offset >= self.cartridge_rom.len() {
                0xFFFFFFFF
            } else {
                ((self.cartridge_rom[cartridge_rom_offset + 0] as u32) << 24)
                | ((self.cartridge_rom[cartridge_rom_offset + 1] as u32) << 16)
                | ((self.cartridge_rom[cartridge_rom_offset + 2] as u32) << 8)
                | (self.cartridge_rom[cartridge_rom_offset + 3] as u32)
            }
        } else {
            panic!("PI: invalid read")
        }
    }

    fn write_u32(&mut self, _value: u32, _offset: usize) {
        panic!("PI: write32");
    }
}


