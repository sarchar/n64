use std::cmp;
use std::str;
use std::sync::mpsc;

#[allow(unused_imports)]
use tracing::{trace,debug,info,warn,error};

use crate::*;

use rcp::DmaInfo;
use mips::{InterruptUpdate, InterruptUpdateMode, IMask_PI};

/// N64 Peripheral Interface
/// Connects EEPROM, cartridge, controllers, and more
pub struct PeripheralInterface {
    comms: SystemCommunication,

    dram_addr: u32,
    cart_addr: u32,
    dma_status: u32,
    io_busy: u32,

    cartridge_rom: Vec<u32>,
    cartridge_rom_write: Option<u32>,
    cartridge_rom_write_time: u64,

    dma_completed_rx: mpsc::Receiver<DmaInfo>,
    dma_completed_tx: mpsc::Sender<DmaInfo>,

    // ISViewer 
    debug_buffer: Vec<u8>,
    debug_string: String,
    is_write_pos: usize,
    //is_read_pos: usize,
    is_magic: u32,
}

impl PeripheralInterface {
    pub fn new(comms: SystemCommunication, cartridge_rom: Vec<u8>) -> PeripheralInterface {
        // convert cartridge_rom to u32
        let mut word_rom = vec![];
        for i in (0..cartridge_rom.len()).step_by(4) {
            word_rom.push(u32::from_be_bytes([cartridge_rom[i+0], cartridge_rom[i+1], cartridge_rom[i+2], cartridge_rom[i+3]]));
        }

        // need some dma completed channels
        let (dma_completed_tx, dma_completed_rx) = mpsc::channel();

        PeripheralInterface {
            comms: comms,

            dram_addr: 0,
            cart_addr: 0,
            dma_status: 0,
            io_busy: 0,

            cartridge_rom: word_rom,
            cartridge_rom_write: None,
            cartridge_rom_write_time: 0,

            dma_completed_rx: dma_completed_rx,
            dma_completed_tx: dma_completed_tx,

            // ISViewer
            debug_buffer: vec![0; 0xFFE0], // ISViewer buffer starts at 0x......20, so buf size is 0x10000-0x20
            debug_string: String::new(),
            is_magic: 0,
            //is_read_pos: 0,
            is_write_pos: 0,
        }
    }

    pub fn reset(&mut self) {
        info!(target: "PI", "reset");
        self.dram_addr    = 0;
        self.cart_addr    = 0;
        self.dma_status   = 0;
        self.io_busy      = 0;
        self.debug_buffer = vec![0; 0xFFE0];
        self.debug_string = String::new();
        self.is_magic     = 0;
        self.is_write_pos = 0;
        self.cartridge_rom_write = None;
        
        while self.dma_completed_rx.try_recv().is_ok() {}
    }

    pub fn step(&mut self) {
        // if dma is running, check for dma completed
        if let Ok(_) = self.dma_completed_rx.try_recv() {
            if (self.dma_status & 0x01) != 0 {
                self.dma_status = 0x08;
                self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_PI, InterruptUpdateMode::SetInterrupt)).unwrap();
            }
        }
    }

    fn read_register(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        match offset & 0xF_FFFF {
            // PI_STATUS
            0x0_0010 => {
                trace!(target: "PI", "read PI_STATUS");

                let ret = self.dma_status | self.io_busy;

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
                trace!(target: "PI", "write PI_DRAM_ADDR value=${:08X}", value);
                self.dram_addr = value & 0x00FF_FFFF;

                WriteReturnSignal::None
            },

            // PI_CART_ADDR
            0x0_0004 => {
                trace!(target: "PI", "write PI_CART_ADDR value=${:08X}", value);
                self.cart_addr = value;

                WriteReturnSignal::None
            },

            // PI_RD_LEN
            0x0_0008 => {
                trace!(target: "PI", "write PI_RD_LEN value=${:08X}", value);

                //info!(target: "PI", "DMA INTO PI from DRAM=${:08X} DEST=${:08X}", self.dram_addr, self.cart_addr);

                // see if previous dma has completed
                if (self.dma_status & 0x01) != 0 {
                    if let Ok(_) = self.dma_completed_rx.try_recv() {
                        self.dma_status = 0x08;
                    }
                }

                if (self.dma_status & 0x01) == 0 { // don't initiate another DMA if one is in progress
                    if self.cart_addr >= 0x0800_0000 && self.cart_addr < 0x1000_0000 {
                        let start = self.cart_addr & !0xF000_0000;
                        let end = start + value + 1;

                        let dma_info = DmaInfo {
                            initiator     : "PI-SRAM",
                            source_address: self.dram_addr,
                            dest_address  : self.cart_addr,
                            count         : 1,
                            length        : ((end - start) + 7) & !7,
                            completed     : Some(self.dma_completed_tx.clone()),
                            ..Default::default()
                        };

                        self.dma_status |= 0x01;
                        self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                        self.comms.break_cpu();
                    }
                }

                WriteReturnSignal::None
            },

            // PI_WR_LEN
            0x0_000C => {
                trace!(target: "PI", "write PI_WR_LEN value=${:08X}", value);

                assert!((self.cart_addr & 0xF000_0000) == 0x1000_0000 ||
                        (self.cart_addr & 0xF800_0000) == 0x0800_0000, "PI DMA initiated from ${:08X}", self.cart_addr); // right now only cartridge rom is valid for dma

                // TODO the logic determining DMA completions might not be correct, but it's fine for now.

                // see if previous dma has completed
                if (self.dma_status & 0x01) != 0 {
                    if let Ok(_) = self.dma_completed_rx.try_recv() {
                        self.dma_status = 0x08;
                    }
                }

                if (self.dma_status & 0x01) == 0 { // don't initiate another DMA if one is in progress
                    if self.cart_addr < 0x1000_0000 {
                        let start = self.cart_addr & !0xF000_0000;
                        let end = start + value + 1;

                        let dma_info = DmaInfo {
                            initiator     : "PI-SRAM",
                            source_address: self.cart_addr,
                            dest_address  : self.dram_addr,
                            count         : 1,
                            length        : end - start,
                            completed     : Some(self.dma_completed_tx.clone()),
                            ..Default::default()
                        };

                        self.dma_status |= 0x01;
                        self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                        self.comms.break_cpu();
                        WriteReturnSignal::None
                    } else {
                        let start = self.cart_addr & !0xF000_0000;
                        let mut end = start + value + 1;
                        if (start as usize) <= self.cartridge_rom.len() * 4 {
                            // truncate dma
                            end = cmp::min((self.cartridge_rom.len() * 4) as u32, end);

                            let dma_info = DmaInfo {
                                initiator     : "PI-CART",
                                source_address: self.cart_addr,
                                dest_address  : self.dram_addr,
                                count         : 1,
                                length        : end - start,
                                completed     : Some(self.dma_completed_tx.clone()),
                                ..Default::default()
                            };

                            self.dma_status |= 0x01;
                            self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                            self.comms.break_cpu();
                            WriteReturnSignal::None
                        } else {
                            WriteReturnSignal::None
                        }
                    }
                } else {
                    panic!("might need to handle this case at some point");
                }
            },

            // PI_STATUS
            0x0_0010 => {
                trace!(target: "PI", "write PI_STATUS value=${:08X}", value);

                if (value & 0x01) != 0 { // DMA stop/reset
                }

                if (value & 0x02) != 0 { // clear INT flag 
                    self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_PI, InterruptUpdateMode::ClearInterrupt)).unwrap();
                    self.dma_status &= !0x08;
                }

                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_LAT
            0x0_0014 => {
                trace!(target: "PI", "write PI_BSD_DOM1_LAT");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_PWD
            0x0_0018 => {
                trace!(target: "PI", "write PI_BSD_DOM1_PWD");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_PGS
            0x0_001C => {
                trace!(target: "PI", "write PI_BSD_DOM1_PGS");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM1_RLS
            0x0_0020 => {
                trace!(target: "PI", "write PI_BSD_DOM1_RLS");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM2_LAT
            0x0_0024 => {
                trace!(target: "PI", "write PI_BSD_DOM2_LAT");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM2_PWD
            0x0_0028 => {
                trace!(target: "PI", "write PI_BSD_DOM2_PWD");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM2_PGS
            0x0_002C => {
                trace!(target: "PI", "write PI_BSD_DOM2_PGS");
                WriteReturnSignal::None
            },

            // PI_BSD_DOM2_RLS
            0x0_0030 => {
                trace!(target: "PI", "write PI_BSD_DOM2_RLS");
                WriteReturnSignal::None
            },

            _ => panic!("PI: unhandled register write ${:08X}", offset),
        };

        Ok(result)
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
        } else if offset == 0x13FF_0000 { // ISViewer magic
            Ok(self.is_magic) // usually 'IS64'
        } else if offset == 0x13FF_0004 { // ISViewer get - get read position
            Ok(0) //self.is_read_pos as u32) 
        } else if offset == 0x13FF_0014 { // ISViewer write - get write position
            Ok(self.is_write_pos as u32) 
        } else if offset >= 0x13FF_0020 && offset < 0x13FF_FFE0 { // ISViewer data
            let buffer_offset = offset - 0x13FF_0020;
            let v = ((self.debug_buffer[buffer_offset+0] as u32) << 24)
                      | ((self.debug_buffer[buffer_offset+1] as u32) << 16)
                      | ((self.debug_buffer[buffer_offset+2] as u32) <<  8)
                      | ((self.debug_buffer[buffer_offset+3] as u32) <<  0);
            Ok(v)
        } else if offset < 0x1FC0_0000 { // 
            let cartridge_rom_offset = offset & 0x0FFF_FFFF;
            //debug!(target: "CART", "read32 offset=${:08X}", cartridge_rom_offset);

            match self.cartridge_rom_write {
                Some(value) => {
                    let delta = (self.comms.total_cpu_steps.get() as u64) - self.cartridge_rom_write_time;
                    self.cartridge_rom_write = None;
                    if delta < 50 { // TODO I don't konw the correct value for this write decay, but 
                                    // 50 is working on n64-systemtests
                        return Ok(value)
                    }
                },
                _ => {},
            }

            if cartridge_rom_offset >= self.cartridge_rom.len() * 4 {
                Ok(0x00000000)
            } else {
                Ok(self.cartridge_rom[(cartridge_rom_offset >> 2) as usize])
            }
        } else if offset >= 0x1FFF_0000 && offset < 0x2000_0000 {
            info!(target: "PI", "read from SC64 register ${:02X}", offset & 0xFF);
            Ok(0)
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
        trace!(target: "PI", "read16 offset=${:08X} return=${:04X}", offset, ret);
        Ok(ret)
    }

    fn read_u8(&mut self, offset: usize) -> Result<u8, ReadWriteFault> {
        // 8-bit read only has access to every other 16 bits
        let half = self.read_u16(offset & !0x01)?;
        let shift = 8 - ((offset & 0x01) << 3);
        let ret = (half >> shift) & 0xFF;
        trace!(target: "PI", "read8 offset=${:08X} return=${:02X}", offset, ret);
        Ok(ret as u8)
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "PI", "write32 value=${:08X} offset=${:08X}", value, offset);

        if offset < 0x0500_0000 {
            self.write_register(value, offset)
        } else if offset == 0x13FF_0000 { // ISViewer magic
            self.is_magic = value;
            Ok(WriteReturnSignal::None)
        } else if offset == 0x13FF_0014 { // ISViewer put - set write position
            let end = value as usize;

            let slice = if end < self.is_write_pos { 
                todo!();
                //// need to concatenate two slices, is_write_pos..END and START..end
                //let slice_a = &self.debug_buffer[self.is_write_pos..];
                //let slice_b = &self.debug_buffer[..end];
                //[slice_a, slice_b].concat()
            } else {
                (&self.debug_buffer[self.is_write_pos..end]).to_vec()
            };

            let (res, _enc, errors) = encoding_rs::EUC_JP.decode(&slice);
            let msg = match str::from_utf8(&slice) {
                Err(_) => {
                    // try japanese!
                    if errors {
                        println!("couldn't convert {:?} using EUC-JP", slice);
                        ""
                    } else {
                    //    let new_slice = &self.debug_buffer[self.is_write_pos..(self.is_write_pos + valid)];
                    //    self.is_read_pos = (self.is_write_pos + valid) % self.debug_buffer.len();
                    //    str::from_utf8(new_slice).unwrap()
                        &*res
                    }
                },

                Ok(msg) => {
                    //self.is_read_pos = end;
                    msg
                }
            };

            for c in msg.chars() {
                if c == '\n' {
                    info!(target: "PI", "message: {}", self.debug_string);
                    self.debug_string = String::new();
                } else {
                    if self.debug_string.len() < 0x10000 {
                        self.debug_string.push(c);
                    }
                }
            }

            self.is_write_pos = 0;//end % self.debug_buffer.len();

            Ok(WriteReturnSignal::None)
        } else if offset >= 0x13FF_0020 && offset < 0x13FF_FFE0 { // ISViewer data
            let buffer_offset = offset - 0x13FF_0020;
            self.debug_buffer[buffer_offset+0] = ((value >> 24) & 0xFF) as u8;
            self.debug_buffer[buffer_offset+1] = ((value >> 16) & 0xFF) as u8;
            self.debug_buffer[buffer_offset+2] = ((value >>  8) & 0xFF) as u8;
            self.debug_buffer[buffer_offset+3] = ((value >>  0) & 0xFF) as u8;
            Ok(WriteReturnSignal::None)
        } else if offset >= 0x1000_0000 && offset < 0x1FC0_0000 {
            //debug!(target: "PI", "wrote to ROM value=${:08X} offset=${:08X}", value, offset);
            match self.cartridge_rom_write {
                // if cartridge_rom_write isn't set, note the first write to cart
                None => {
                    self.cartridge_rom_write = Some(value);
                    self.cartridge_rom_write_time = self.comms.total_cpu_steps.get() as u64;
                    // set the IO busy flag of PI_STATUS on cart write
                    self.io_busy = 0x02;
                },
                _ => {},
            }
            Ok(WriteReturnSignal::None)
        } else if offset >= 0x1FFF_0000 && offset < 0x2000_0000 {
            info!(target: "PI", "write to SC64 register ${:02X}", offset & 0xFF);
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

    fn read_block(&mut self, offset: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        if offset >= 0x0800_0000 && offset < 0x1000_0000 { // SRAM
            info!(target: "SRAM", "read_block {} bytes from ${:08X}", length, offset);
            let length = if length > (256*1024) { 256 * 1024 } else { length };
            Ok(vec![0u32; (length >> 2) as usize])
        } else if offset >= 0x1000_0000 && offset < 0x1FC0_0000 { // CART memory
            let length = (length + 3) & !3; // round length up to a multiple of 4 for the read
            if (offset & 0x02) != 0 { // 16-bit aligned DMA, slow for now but I think not too common
                let start = (offset & !0x1000_0000) as usize;
                let mut r = vec![];
                for i in (0..(length >> 1) as usize).step_by(2) { // loop over 16-bit reads two at a time
                    let address = start + (i << 1);
                    let shift = 16 - ((address & 0x02) << 3);
                    let h0 = (self.cartridge_rom[address >> 2] >> shift) & 0xFFFF;
                    
                    let address = start + ((i + 1) << 1);
                    let shift = 16 - ((address & 0x02) << 3);
                    let h1 = (self.cartridge_rom[address >> 2] >> shift) & 0xFFFF;

                    r.push((h0 << 16) | h1);
                }
                Ok(r)
            } else { // 32-bit aligned can do memcpy
                assert!((length & 0x03) == 0); // length needs to be 32-bit (and really 64-bit, since this is going to DRAM)
                let start = (offset & !0x1000_0000) >> 2;
                let end = start + (length as usize >> 2);
                Ok((&self.cartridge_rom[start..end]).to_vec())
            }
        } else {
            todo!("probably not used ever");
        }
    }

    fn write_block(&mut self, offset: usize, block: &[u32], length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
        if (block.len() * 4) as u32 != length { todo!(); }
        if offset >= 0x0800_0000 && offset < 0x1000_0000 { // SRAM
            info!(target: "SRAM", "write_block {} bytes to ${:08X}", block.len(), offset);
        }
        Ok(WriteReturnSignal::None)
    }
}


