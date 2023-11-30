use std::cmp;
use std::fs;
use std::str;

#[allow(unused_imports)]
use tracing::{trace,debug,info,warn,error};

use crate::*;

/// N64 Peripheral Interface
/// Connects EEPROM, cartridge, controllers, and more
pub struct PeripheralInterface {
    dram_addr: u32,
    cart_addr: u32,
    dma_status: u32,
    io_busy: u32,

    cartridge_rom: Vec<u8>,
    cartridge_rom_write: Option<u32>,

    debug_buffer: Vec<u8>,
    debug_string: String,
}

impl PeripheralInterface {
    pub fn new(cartridge_rom_file: &str) -> PeripheralInterface {
        let cartridge_rom = fs::read(cartridge_rom_file).expect("Could not open cartridge ROM file");

        PeripheralInterface {
            dram_addr: 0,
            cart_addr: 0,
            dma_status: 0,
            io_busy: 0,

            cartridge_rom: cartridge_rom,
            cartridge_rom_write: None,

            debug_buffer: vec![0; 0x200],
            debug_string: String::new(),
        }
    }

    fn read_register(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        match offset & 0xF_FFFF {
            // PI_STATUS
            0x0_0010 => {
                debug!(target: "PI", "read PI_STATUS");
                let ret = self.dma_status | self.io_busy;
                if self.dma_status == 0x01 {
                    self.dma_status = 0x08;
                }
                if self.io_busy != 0 {
                    self.io_busy = 0;
                }
                Ok(ret)
            },

            // PI_BSD_DOM1_LAT
            0x0_0014 => {
                info!(target: "PI", "read PI_BSD_DOM1_LAT");
                Ok(0)
            },

            // PI_BSD_DOM1_PWD
            0x0_0018 => {
                info!(target: "PI", "read PI_BSD_DOM1_PWD");
                Ok(0)
            },

            // PI_BSD_DOM1_PGS
            0x0_001C => {
                info!(target: "PI", "read PI_BSD_DOM1_PGS");
                Ok(0)
            },

            // PI_BSD_DOM1_RLS
            0x0_0020 => {
                info!(target: "PI", "read PI_BSD_DOM1_RLS");
                Ok(0)
            },

            // PI_BSD_DOM2_LAT
            0x0_0024 => {
                info!(target: "PI", "read PI_BSD_DOM2_LAT");
                Ok(0)
            },

            // PI_BSD_DOM2_PWD
            0x0_0028 => {
                info!(target: "PI", "read PI_BSD_DOM2_PWD");
                Ok(0)
            },

            // PI_BSD_DOM2_PGS
            0x0_002C => {
                info!(target: "PI", "read PI_BSD_DOM2_PGS");
                Ok(0)
            },

            // PI_BSD_DOM2_RLS
            0x0_0030 => {
                info!(target: "PI", "read PI_BSD_DOM2_RLS");
                Ok(0)
            },
            _ => panic!("PI: unhandled register read ${:08X}", offset),
        }
    }

    fn write_register(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        let result = match offset & 0xF_FFFF {
            // PI_DRAM_ADDR
            0x0_0000 => {
                debug!(target: "PI", "write PI_DRAM_ADDR value=${:08X}", value);
                self.dram_addr = value;

                WriteReturnSignal::None
            },

            // PI_CART_ADDR
            0x0_0004 => {
                debug!(target: "PI", "write PI_CART_ADDR value=${:08X}", value);
                self.cart_addr = value;

                WriteReturnSignal::None
            },

            // PI_WR_LEN
            0x0_000C => {
                debug!(target: "PI", "write PI_WR_LEN value=${:08X}", value);

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
                debug!(target: "PI", "write PI_STATUS value=${:08X}", value);
                if (value & 0x01) != 0 { debug!(target: "PI", "DMA reset"); }
                if (value & 0x02) != 0 { debug!(target: "PI", "clear interrupt flag"); }

                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_LAT
            0x0_0014 => {
                debug!(target: "PI", "write PI_BSD_DOM1_LAT");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_PWD
            0x0_0018 => {
                debug!(target: "PI", "write PI_BSD_DOM1_PWD");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_PGS
            0x0_001C => {
                debug!(target: "PI", "write PI_BSD_DOM1_PGS");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_RLS
            0x0_0020 => {
                debug!(target: "PI", "write PI_BSD_DOM1_RLS");
                WriteReturnSignal::None
            },

            _ => panic!("PI: unhandled register write ${:08X}", offset),
        };

        Ok(result)
    }

    pub fn get_dma_info(&self, dma_info: &DmaInfo) -> &[u8] {
        let end = dma_info.source_address + dma_info.count;
        return &self.cartridge_rom[(dma_info.source_address as usize)..(end as usize)];
    }
}

impl Addressable for PeripheralInterface {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "PI", "read32 offset=${:08X}", offset);

        if offset < 0x0500_0000 {
            self.read_register(offset)
        } else if offset < 0x0600_0000 {
            info!(target: "PI", "read32 N64DD registers offset=${:08X}", offset);
            Ok(0)
        } else if offset < 0x0800_0000 {
            info!(target: "PI", "read32 N64DD IPL rom offset=${:08X}", offset);
            Ok(0)
        } else if offset < 0x1000_0000 {
            error!(target: "PI", "unimplemented Cartridge SRAM/FlashRAM read");
            Ok(0)
        } else if offset < 0x1FC0_0000 {
            let cartridge_rom_offset = offset & 0x0FFF_FFFF;
            debug!(target: "CART", "read32 offset=${:08X}", cartridge_rom_offset);

            if let Some(value) = self.cartridge_rom_write {
                self.cartridge_rom_write = None;
                Ok(value)
            } else {
                if cartridge_rom_offset >= self.cartridge_rom.len() {
                    Ok(0x00000000)
                } else {
                    Ok(((self.cartridge_rom[cartridge_rom_offset + 0] as u32) << 24)
                        | ((self.cartridge_rom[cartridge_rom_offset + 1] as u32) << 16)
                        | ((self.cartridge_rom[cartridge_rom_offset + 2] as u32) << 8)
                        | (self.cartridge_rom[cartridge_rom_offset + 3] as u32))
                }
            }
        } else {
            debug!(target: "PI", "open bus read at ${:08X}", offset);
            // lower 16 bits repeated in both halves of the word
            Ok((((offset & 0xFFFF) << 16) | (offset & 0xFFFF)) as u32)
        }
    }

    fn read_u16(&mut self, offset: usize) -> Result<u16, ReadWriteFault> {
        // 16-bit read from CART is buggy
        let i = (offset & 0x02) >> 1;
        let ret = ((self.read_u32((offset & !0x03) + (i << 2))? & 0xFFFF0000) >> 16) as u16;
        debug!(target: "PI", "read16 offset=${:08X} return=${:04X}", offset, ret);
        Ok(ret)
    }

    fn read_u8(&mut self, offset: usize) -> Result<u8, ReadWriteFault> {
        // 8-bit read only has access to every other 16 bits
        let half = self.read_u16(offset & !0x01)?;
        let shift = 8 - ((offset & 0x01) << 3);
        let ret = (half >> shift) & 0xFF;
        debug!(target: "PI", "read8 offset=${:08X} return=${:02X}", offset, ret);
        Ok(ret as u8)
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        debug!(target: "PI", "write32 value=${:08X} offset=${:08X}", value, offset);

        if offset < 0x0500_0000 {
            self.write_register(value, offset)
        } else if offset == 0x13FF_0014 {
            let slice = &self.debug_buffer[0..(value as usize)];
            let msg = str::from_utf8(slice).unwrap();

            for c in msg.chars() {
                if c == '\n' {
                    info!(target: "PI", "message: {}", self.debug_string);
                    self.debug_string = String::new();
                } else {
                    self.debug_string.push(c);
                }
            }

            Ok(WriteReturnSignal::None)
        } else if offset >= 0x13FF_0020 && offset <= 0x13FF_0220 {
            let buffer_offset = offset - 0x13FF_0020;
            self.debug_buffer[buffer_offset+0] = ((value >> 24) & 0xFF) as u8;
            self.debug_buffer[buffer_offset+1] = ((value >> 16) & 0xFF) as u8;
            self.debug_buffer[buffer_offset+2] = ((value >>  8) & 0xFF) as u8;
            self.debug_buffer[buffer_offset+3] = ((value >>  0) & 0xFF) as u8;
            Ok(WriteReturnSignal::None)
        } else if offset >= 0x1000_0000 && offset < 0x1FC0_0000 {
            //debug!(target: "PI", "wrote to ROM value=${:08X} offset=${:08X}", value, offset);
            if let None = self.cartridge_rom_write {
                self.cartridge_rom_write = Some(value);
                // set the IO busy flag of PI_STATUS on cart write
                self.io_busy = 0x02;
            }
            Ok(WriteReturnSignal::None)
        } else {
            error!(target: "PI", "unhandled write32 value=${:08X} offset=${:08X}", value, offset);
            Ok(WriteReturnSignal::None)
        }
    }

    fn write_u16(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
		// writes to CART are padded with zeroes in the lower bytes
        if offset >= 0x1000_0000 && offset < 0x1FC0_0000 {
            let shift = 16 - ((offset & 0x03) << 3);
            self.write_u32(value << shift, offset)
        } else {
            Addressable::write_u16(self, value, offset)
        }
	}

    fn write_u8(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
		// writes to CART are padded with zeroes in the lower bytes
        if offset >= 0x1000_0000 && offset < 0x1FC0_0000 {
            let shift = 24 - ((offset & 0x03) << 3);
            self.write_u32(value << shift, offset)
        } else {
            Addressable::write_u8(self, value, offset)
        }
	}
}


