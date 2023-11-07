use std::cmp;
use std::fs;
use std::str;

use crate::*;

/// N64 Peripheral Interface
/// Connects EEPROM, cartridge, controllers, and more
pub struct PeripheralInterface {
    dram_addr: u32,
    cart_addr: u32,
    dma_status: u32,

    cartridge_rom: Vec<u8>,

    debug_buffer: Vec<u8>,
    debug_newline: bool,
}

impl PeripheralInterface {
    pub fn new(cartridge_rom_file: &str) -> PeripheralInterface {
        let cartridge_rom = fs::read(cartridge_rom_file).expect("Could not open cartridge ROM file");

        PeripheralInterface {
            dram_addr: 0,
            cart_addr: 0,
            dma_status: 0,
            cartridge_rom: cartridge_rom,

            debug_buffer: vec![0; 0x200],
            debug_newline: true,
        }
    }

    fn read_register(&mut self, offset: usize) -> u32 {
        match offset & 0xF_FFFF {
            // PI_STATUS
            0x0_0010 => {
                println!("PI: read PI_STATUS");
                let ret = self.dma_status;
                if self.dma_status == 0x01 {
                    self.dma_status = 0x08;
                }
                ret
            },
            _ => panic!("PI: unhandled register read ${:08X}", offset),
        }
    }

    fn write_register(&mut self, value: u32, offset: usize) -> WriteReturnSignal {
        match offset & 0xF_FFFF {
            // PI_DRAM_ADDR
            0x0_0000 => {
                println!("PI: write PI_DRAM_ADDR value=${:08X}", value);
                self.dram_addr = value;

                WriteReturnSignal::None
            },

            // PI_CART_ADDR
            0x0_0004 => {
                println!("PI: write PI_CART_ADDR value=${:08X}", value);
                self.cart_addr = value;

                WriteReturnSignal::None
            },

            // PI_WR_LEN
            0x0_000C => {
                println!("PI: write PI_WR_LEN value=${:08X}", value);

                assert!((self.cart_addr & 0xF000_0000) == 0x1000_0000); // right now only cartridge rom is valid for dma

                let cart_addr = self.cart_addr & !0xF000_0000;
                let mut end = cart_addr + value + 1;
                if (cart_addr as usize) <= self.cartridge_rom.len() {
                    // truncate dma
                    end = cmp::min(self.cartridge_rom.len() as u32, end);

                    //let cart_data = &self.cartridge_rom[(self.cart_addr as usize)..end];
                    let dma_info = DmaInfo {
                        source_address: cart_addr,
                        dest_address: self.dram_addr,
                        count: end - cart_addr,
                    };

                    self.dma_status = 0x01;
                    WriteReturnSignal::StartDMA(dma_info)
                } else {
                    WriteReturnSignal::None
                }
            },

            // PI_STATUS
            0x0_0010 => {
                println!("PI: write PI_STATUS value=${:08X}", value);
                if (value & 0x01) != 0 { println!("PI: DMA reset"); }
                if (value & 0x02) != 0 { println!("PI: clear interrupt flag"); }

                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_LAT
            0x0_0014 => {
                println!("PI: write PI_BSD_DOM1_LAT");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_PWD
            0x0_0018 => {
                println!("PI: write PI_BSD_DOM1_PWD");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_PGS
            0x0_001C => {
                println!("PI: write PI_BSD_DOM1_PGS");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_RLS
            0x0_0020 => {
                println!("PI: write PI_BSD_DOM1_RLS");
                WriteReturnSignal::None
            },

            _ => panic!("PI: unhandled register write ${:08X}", offset),
        }
    }

    pub fn get_dma_info(&self, dma_info: &DmaInfo) -> &[u8] {
        let end = dma_info.source_address + dma_info.count;
        return &self.cartridge_rom[(dma_info.source_address as usize)..(end as usize)];
    }
}

impl Addressable for PeripheralInterface {
    fn read_u32(&mut self, offset: usize) -> u32 {
        println!("PI: read32 offset=${:08X}", offset);

        if offset < 0x0500_0000 {
            self.read_register(offset)
        } else if offset < 0x0800_0000 {
            panic!("N64DD read")
        } else if offset < 0x1000_0000 {
            panic!("Cartridge SRAM/FlashRAM read")
        } else if offset < 0x1FC0_0000 {
            let cartridge_rom_offset = offset & 0x0FFF_FFFF;
            println!("CART: read32 offset=${:08X}", cartridge_rom_offset);

            if cartridge_rom_offset >= self.cartridge_rom.len() {
                0x00000000
            } else {
                ((self.cartridge_rom[cartridge_rom_offset + 0] as u32) << 24)
                | ((self.cartridge_rom[cartridge_rom_offset + 1] as u32) << 16)
                | ((self.cartridge_rom[cartridge_rom_offset + 2] as u32) << 8)
                | (self.cartridge_rom[cartridge_rom_offset + 3] as u32)
            }
        } else {
            panic!("PI: invalid read at ${:08X}", offset)
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> WriteReturnSignal {
        println!("PI: write32 value=${:08X} offset=${:08X}", value, offset);

        if offset < 0x0500_0000 {
            self.write_register(value, offset)
        } else if offset == 0x13FF_0014 {
            let slice = &self.debug_buffer[0..(value as usize)];
            let msg = str::from_utf8(slice).unwrap();

            for c in msg.chars() {
                if self.debug_newline {
                    eprint!("DEBUG: message: ");
                    self.debug_newline = false;
                }

                eprint!("{}", c);

                if c == '\n' {
                    self.debug_newline = true;
                }
            }

            WriteReturnSignal::None
        } else if offset >= 0x13FF_0020 && offset <= 0x13FF_0220 {
            let buffer_offset = offset - 0x13FF_0020;
            self.debug_buffer[buffer_offset+0] = ((value >> 24) & 0xFF) as u8;
            self.debug_buffer[buffer_offset+1] = ((value >> 16) & 0xFF) as u8;
            self.debug_buffer[buffer_offset+2] = ((value >>  8) & 0xFF) as u8;
            self.debug_buffer[buffer_offset+3] = ((value >>  0) & 0xFF) as u8;
            WriteReturnSignal::None
        } else {
            panic!("PI: unhandled write32 value=${:08X} offset=${:08X}", value, offset);
        }
    }
}


