use std::any::Any;
use std::cmp;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::str;
use std::sync::mpsc;

#[allow(unused_imports)]
use tracing::{trace,debug,info,warn,error};

use crate::*;

use rcp::DmaInfo;
use mips::{InterruptUpdate, InterruptUpdateMode, IMask_PI};

use pifrom::Flash;

#[derive(Copy, Clone, Debug, PartialEq)]
enum FlashMode {
    Idle,
    Status,
    Erase,
    Read,
    Write
}

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

    // Timing
    test_start: Option<std::time::Instant>,

    // SRAM
    sram_file: PathBuf,
    sram: Option<Vec<u32>>,
    sram_dirty: Option<std::time::Instant>,

    // FlashRAM
    flash_file: PathBuf,
    flash: Option<Vec<u32>>,
    flash_dirty: Option<std::time::Instant>,
    flash_mode: FlashMode,
    flash_status: u64,
    flash_offset: u32,
    flash_buffer: Vec<u32>,
}

struct DmaInfoUserData  {
    incomplete     : bool,
    actual_rom_size: u32,
}

impl rcp::DmaInfoUserData for DmaInfoUserData {
    fn as_any(&self) -> &dyn Any { self }
}

impl PeripheralInterface {
    pub fn new<P: AsRef<Path>>(comms: SystemCommunication, rom_filename: P, cartridge_rom: Vec<u8>) -> PeripheralInterface {
        // convert cartridge_rom to u32
        let mut word_rom = vec![];
        for i in (0..((cartridge_rom.len() / 4) * 4)).step_by(4) {
            word_rom.push(u32::from_be_bytes([cartridge_rom[i+0], cartridge_rom[i+1], cartridge_rom[i+2], cartridge_rom[i+3]]));
        }

        // need some dma completed channels
        let (dma_completed_tx, dma_completed_rx) = mpsc::channel();

        // try loading SRAM data if it exists
        let savedata_dir = get_savedata_dir();
        let mut sram_file = savedata_dir.join(rom_filename);
        sram_file.set_extension("sram");

        let sram = match fs::read(&sram_file) {
            Ok(data) => { // convert to words
                assert!((data.len() % 4) == 0);
                let mut words = Vec::new();
                for chunk in data.chunks(4) {
                    //words.push(((chunk[0] as u32) << 24) | ((chunk[1] as u32) << 16) | ((chunk[2] as u32) << 8) | (chunk[3] as u32));
                    words.push(u32::from_be_bytes(chunk.try_into().unwrap()));
                }
                info!(target: "SRAM", "loaded {} bytes from {:?}", words.len()*4, sram_file);
                Some(words)
            },
            Err(_) => None,
        };

        let mut flash_file = sram_file.clone();
        flash_file.set_extension("flash");

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

            test_start: None,

            sram_file: sram_file.into(),
            sram: sram,
            sram_dirty: None,

            flash_file: flash_file.into(),
            flash: None,
            flash_dirty: None,
            flash_mode: FlashMode::Idle,
            flash_status: 0,
            flash_offset: 0,
            flash_buffer: vec![0; 128 >> 2],
        }
    }

    pub fn get_sram_file(&self) -> &PathBuf {
        &self.sram_file
    }

    pub fn set_flash(&mut self, flash: Flash) {
        let flash_size = match flash {
            Flash::None => { return; },
            Flash::_128KiB => 128 * 1024,
        };

        self.flash = match fs::read(&self.flash_file) {
            Ok(data) => { // convert to words
                assert!((data.len() % 4) == 0);
                let mut words = Vec::new();
                for chunk in data.chunks(4) {
                    words.push(u32::from_be_bytes(chunk.try_into().unwrap()));
                }
                info!(target: "FLASH", "loaded {} bytes from {:?}", words.len()*4, self.flash_file);

                if (words.len() * 4) != flash_size {
                    warn!(target: "FLASH", "save file size was incorrect, resizing flash");
                    words.resize(flash_size >> 2, 0);
                }
                Some(words)
            },

            Err(_) => {
                let mut flash = Vec::new();            
                flash.resize(flash_size >> 2, 0);
                Some(flash)
            },
        };
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
        self.flash_mode   = FlashMode::Idle;
        self.flash_status = 0;
        self.flash_offset = 0;
        self.flash_buffer.clear();
        
        while self.dma_completed_rx.try_recv().is_ok() {}
    }

    pub fn step(&mut self) {
        // if dma is running, check for dma completed
        while let Ok(dma_info) = self.dma_completed_rx.try_recv() {
            if (self.dma_status & 0x01) != 0 {
                // update cart_addr
                let actual_rom_size = if dma_info.user_data.is_some() {
                    let user_info_ref = dma_info.user_data.as_ref().unwrap();
                    let user_info = user_info_ref.as_any().downcast_ref::<DmaInfoUserData>().unwrap();

                    // if this is an incomplete DMA, skip all field updates
                    if user_info.incomplete { continue; }

                    user_info.actual_rom_size
                } else {
                    dma_info.length
                };
                self.cart_addr = (self.cart_addr + ((actual_rom_size + 1) & !1)) & 0xFFFF_FFFE;

                // update dram_addr
                let dram_misalignment = self.dram_addr & 0x07;
                //let old_dram_addr = self.dram_addr;
                let dram_increment = ((dma_info.length + dram_misalignment + 7) & !7).saturating_sub(dram_misalignment);
                self.dram_addr = (self.dram_addr + dram_increment & !7) & 0x00FF_FFFE;
                //if dram_misalignment != 0 { println!("misalignment = {}, dram_addr before={:08X} after={:08X}", dram_misalignment, old_dram_addr, self.dram_addr); }

                // set state and trigger PI interrupt
                self.dma_status = 0x08;
                self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_PI, InterruptUpdateMode::SetInterrupt)).unwrap();
            }
        }

        if let Some(dirty_time) = self.sram_dirty {
            let elapsed = dirty_time.elapsed().as_secs_f64();
            if elapsed >= 1.0 {
                self.save_sram();
                self.sram_dirty = None;
            }
        }

        if let Some(dirty_time) = self.flash_dirty {
            let elapsed = dirty_time.elapsed().as_secs_f64();
            if elapsed >= 1.0 {
                self.save_flash();
                self.flash_dirty = None;
            }
        }
    }

    pub fn save_sram(&mut self) {
        {
            let mut fp = std::fs::File::create(&self.sram_file).expect("could not save SRAM data");
            for word in self.sram.as_ref().unwrap() {
                let _ = fp.write(&word.to_be_bytes()).expect("could not save SRAM data");
            }
        }

        info!(target: "SRAM", "saved to {:?}", self.sram_file);
    }

    pub fn save_flash(&mut self) {
        {
            let mut fp = std::fs::File::create(&self.flash_file).expect("could not save Flash RAM data");
            for word in self.flash.as_ref().unwrap() {
                let _ = fp.write(&word.to_be_bytes()).expect("could not save Flash RAM data");
            }
        }

        info!(target: "FLASH", "saved to {:?}", self.sram_file);
    }

    fn read_register(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        match offset & 0xF_FFFF {
            // PI_DRAM_ADDR
            0x0_0000 => {
                Ok(self.dram_addr)
            },

            // PI_CART_ADDR
            0x0_0004 => {
                Ok(self.cart_addr)
            },

            // PI_RD_LEN
            // reading this register appears to always return 0x7F
            0x0_0008 => {
                Ok(0x7F)
            }

            // PI_WR_LEN
            // reading this register usually returns 0x7F
            0x0_000C => {
                Ok(0x7F)
            }

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
                self.dram_addr = value & 0x00FF_FFFE;

                WriteReturnSignal::None
            },

            // PI_CART_ADDR
            0x0_0004 => {
                trace!(target: "PI", "write PI_CART_ADDR value=${:08X}", value);
                self.cart_addr = value & 0xFFFF_FFFE;

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
                        let requested_size = value + 1;

                        // determine the usage of the internal based on rdram page size (with 64-bit aligned access)
                        let internal_buffer_size = std::cmp::min(128, 0x800 - (self.dram_addr & 0x7F8)); 

                        // PI internal buffer is weird. I guess misaligned first blocks always have
                        // that last byte written, but less than that works as expected.
                        let dram_misalignment = self.dram_addr & 0x07;

                        let actual_rom_size = if requested_size >= (std::cmp::max(8, internal_buffer_size - 2) - dram_misalignment) {
                            (requested_size + 1) & !1
                        } else {
                            requested_size
                        };

                        let actual_transfer_size = if actual_rom_size < (internal_buffer_size - dram_misalignment) {
                            actual_rom_size.saturating_sub(dram_misalignment)
                        } else {
                            actual_rom_size
                        };

                        //if dram_misalignment == 2 && actual_transfer_size >= 134 {
                        //    actual_transfer_size += 2;
                        //} else if dram_misalignment == 6 && actual_transfer_size >= 124 {
                        //    actual_transfer_size += 6;
                        //}

                        let mut end = start + actual_transfer_size;
                        //if dram_misalignment != 0 { println!("starting dma with dram misalignment = {}, length = {} value={}", dram_misalignment, actual_transfer_size, value); }
                        if (start as usize) <= self.cartridge_rom.len() * 4 {
                            // truncate dma
                            end = cmp::min((self.cartridge_rom.len() * 4) as u32, end);

                            // there's a "gap" of writing in the area internal_buffer_size-2*dram_misalignment..internal_buffer_size-1*dram_misalignment
                            // simulated here with two DMAs...
                            if dram_misalignment == 0 || actual_transfer_size < internal_buffer_size - 2*dram_misalignment {
                                let dma_info = DmaInfo {
                                    initiator     : "PI-CART",
                                    source_address: self.cart_addr,
                                    dest_address  : self.dram_addr,
                                    count         : 1,
                                    length        : end - start,
                                    completed     : Some(self.dma_completed_tx.clone()),
                                    user_data     : Some(Box::new(DmaInfoUserData {
                                        incomplete     : false,
                                        actual_rom_size: actual_rom_size,
                                    })),
                                    ..Default::default()
                                };
                                self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                            } else {
                                let dma_info = DmaInfo {
                                    initiator     : "PI-CART(1/2)",
                                    source_address: self.cart_addr,
                                    dest_address  : self.dram_addr,
                                    count         : 1,
                                    length        : internal_buffer_size - 2*dram_misalignment,
                                    completed     : Some(self.dma_completed_tx.clone()),
                                    user_data     : Some(Box::new(DmaInfoUserData {
                                        incomplete     : true,
                                        actual_rom_size: 0,
                                    })),
                                    ..Default::default()
                                };
                                self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();

                                self.dram_addr += internal_buffer_size - 1*dram_misalignment;
                                let dma_info = DmaInfo {
                                    initiator     : "PI-CART(2/2)",
                                    source_address: self.cart_addr + internal_buffer_size - 1*dram_misalignment,
                                    dest_address  : self.dram_addr,
                                    count         : 1,
                                    length        : (end - start).saturating_sub(internal_buffer_size - 1*dram_misalignment),
                                    completed     : Some(self.dma_completed_tx.clone()),
                                    user_data     : Some(Box::new(DmaInfoUserData {
                                        incomplete     : false,
                                        actual_rom_size: actual_rom_size,
                                    })),
                                    ..Default::default()
                                };
                                self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                            }

                            self.dma_status |= 0x01;
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

    fn read_sram_block(&mut self, offset: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        trace!(target: "SRAM", "read_block {} bytes from ${:08X}", length, offset);

        let length = if length > (256*1024) { 256 * 1024 } else { length };
        match self.sram {
            None => { // return all 0 while self.sram is None
                Ok(vec![0u32; (length >> 2) as usize])
            },

            // if the sram exists, make sure it's the right size and return a chunk
            Some(ref mut sram_data) => {
                let end = (offset + length as usize) >> 2;
                if end > sram_data.len() {
                    debug!(target: "SRAM", "increasing SRAM to {} bytes", end * 4);
                    sram_data.resize(end, 0);
                }
                Ok(sram_data[offset >> 2..][..(length >> 2) as usize].to_owned())
            },
        }
    }

    fn write_sram_block(&mut self, offset: usize, block: &[u32], length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "SRAM", "write_block {} bytes to ${:08X}", block.len() * 4, offset);

        if self.sram.is_none() {
            self.sram = Some(Vec::new());
        }

        let end = (offset + length as usize) >> 2;
        if end > self.sram.as_ref().unwrap().len() {
            debug!(target: "SRAM", "increasing SRAM to {} bytes", end * 4);
            self.sram.as_mut().unwrap().resize(end, 0);
        }

        // copy block[:length] into sram
        self.sram.as_mut().unwrap()[offset >> 2..][..(length >> 2) as usize].copy_from_slice(&block[..(length >> 2) as usize]);
        self.sram_dirty = Some(std::time::Instant::now());

        Ok(WriteReturnSignal::None)
    }

    fn write_sram_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "SRAM", "write32 value=${:08X} offset=${:08X}", value, offset);

        if self.sram.is_none() {
            self.sram = Some(Vec::new());
        }

        let end = (offset + 4) >> 2;
        if end > self.sram.as_ref().unwrap().len() {
            debug!(target: "SRAM", "increasing SRAM to {} bytes", end * 4);
            self.sram.as_mut().unwrap().resize(end, 0);
        }

        self.sram.as_mut().unwrap()[offset >> 2] = value;
        self.sram_dirty = Some(std::time::Instant::now());

        Ok(WriteReturnSignal::None)
    }

    fn read_flash_block(&mut self, offset: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        trace!(target: "FLASH", "read_block {} bytes from ${:08X}", length, offset);

        match self.flash_mode {
            FlashMode::Idle => {
                let mut data = Vec::new();
                data.resize(std::cmp::min(((length + 3) & !3) as usize, 1*1024*1024), 0);
                Ok(data)
            },

            FlashMode::Status => {
                let status_bytes = [(self.flash_status >> 32) as u32, self.flash_status as u32];
                let length = std::cmp::min(((length + 3) & !3) as usize, status_bytes.len() * 4);

                Ok((&status_bytes[0..length >> 2]).to_owned())
            },

            FlashMode::Read => {
                let flash_data = self.flash.as_ref().unwrap();
                Ok(flash_data[offset >> 2..][..(length >> 2) as usize].to_owned())
            },

            FlashMode::Erase | FlashMode::Write => unimplemented!(),
        }
    }

    fn write_flash_block(&mut self, offset: usize, block: &[u32], length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "FLASH", "write_block {} bytes to ${:08X}", block.len() * 4, offset);

        if self.flash_mode == FlashMode::Write { // DMA up to 128 bytes into flash internal storage
            let remaining = (self.flash_buffer.len() * 4) - offset;
            let write_size = std::cmp::min(remaining, length as usize);

            // copy block[:write_size] into flash buffer at given offset
            self.flash_buffer[offset >> 2..][..write_size >> 2].copy_from_slice(&block[..write_size >> 2]);
        } else {
            unimplemented!("unimplemented flash write");
        }

        Ok(WriteReturnSignal::None)
    }

    fn write_flash_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "FLASH", "write32 value=${:08X} offset=${:08X}", value, offset);

        // ignore writes to offset 0
        if offset == 0 { 
            debug!(target: "FLASH", "ignoring write ${:08X} to offset 0", value);
            return Ok(WriteReturnSignal::None); 
        }

        match value >> 24 {
            0x4B => { // Set Erase Offset
                self.flash_offset = (value & 0x0000_FFFF) * 128;
            },

            0x78 => { // Set Mode to Erase
                self.flash_mode = FlashMode::Erase;
                self.flash_status = 0x1111_8008_00C2_001D;
            },

            0xB4 => { // Set Mode to Write
                self.flash_mode = FlashMode::Write;
                self.flash_status = 0x1111_8004_00C2_001D;
            },

            0xA5 => { // Set Write Offset
                self.flash_offset = (value & 0x0000_FFFF) * 128;
                self.flash_status = 0x1111_8004_00C2_001D;
            },

            0xD2 => { // Execute
                debug!(target: "FLASH", "Execute mode = {:?}", self.flash_mode);
                match self.flash_mode {
                    FlashMode::Idle | FlashMode::Status => {},

                    // Perform erase - set 128 bytes starting at self.flash_offset to 0
                    FlashMode::Erase => {
                        let flash_data = self.flash.as_mut().unwrap();
                        flash_data[(self.flash_offset >> 2) as usize..][..128 >> 2].fill(0xFFFF_FFFF);
                    },

                    // Perform write - copy 128 bytes from the flash buffer into flash
                    FlashMode::Write => {
                        let flash_data = self.flash.as_mut().unwrap();
                        flash_data[(self.flash_offset >> 2) as usize..][..128 >> 2].copy_from_slice(&self.flash_buffer);
                        self.flash_dirty = Some(std::time::Instant::now());
                    },

                    _ => todo!("mode = {:?}", self.flash_mode),
                }
            },

            0xE1 => { // Set Mode to Status
                self.flash_mode = FlashMode::Status;
                self.flash_status = 0x1111_8001_00C2_001D;
            },

            0xF0 => { // Set Mode to Read
                self.flash_mode = FlashMode::Read;
                self.flash_status = 0x1111_8004_F000_001D;
            },

            cmd @ _ => {
                unimplemented!("FLASH command ${:02X} not implemented", cmd);
            }
        }

        //self.flash.as_mut().unwrap()[offset >> 2] = value;
        //self.sram_dirty = Some(std::time::Instant::now());

        Ok(WriteReturnSignal::None)
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
            if self.flash.is_some() { // read_u32 is only available to read the Status register
                Ok(match offset & 4 {
                    0 => (self.flash_status >> 32) as u32,
                    4 => self.flash_status as u32,
                    _ => panic!(),
                })
            } else {
                match self.sram {
                    None => Ok(0),
                    Some(ref mut sram_data) => {
                        let offset = offset - 0x0800_0000;
                        let end = (offset + 4) >> 2;
                        if end > sram_data.len() {
                            debug!(target: "SRAM", "increasing SRAM to {} bytes", end * 4);
                            sram_data.resize(end, 0);
                        }
                        Ok(sram_data[offset])
                    },
                }
            }
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
        // 16-bit reads from PI devices are buggy
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
        } else if offset < 0x0800_0000 { // ...?
            panic!("invalid");
        } else if offset < 0x1000_0000 { // FlashRAM/SRAM
            let offset = offset - 0x0800_0000;

            if self.flash.is_some() {
                self.write_flash_u32(value, offset)
            } else {
                self.write_sram_u32(value, offset)
            }
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

                    if self.debug_string.starts_with("Heap range") {
                        self.test_start = Some(std::time::Instant::now());
                    } else if self.debug_string.starts_with("Slowest tests") {
                        self.test_start.and_then(|test_start| -> Option<()> {
                            info!(target: "PI", "total test time: {:.6?} (message)", test_start.elapsed());
                            None
                        });
                    }

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
        trace!(target: "PI", "read_block offset=${:08X} length=${:08X}", offset, length);
        if offset >= 0x0800_0000 && offset < 0x1000_0000 { // SRAM
            let offset = offset & 0x07FF_FFFF;
            if self.flash.is_some() {
                self.read_flash_block(offset, length)
            } else {
                self.read_sram_block(offset, length)
            }
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
                if start < self.cartridge_rom.len() {
                    // read some memory, padded to the full length
                    let remaining_size = self.cartridge_rom.len() - start;
                    let read_size = std::cmp::min(remaining_size, (length as usize) >> 2);
                    let end = start + read_size;
                    let mut res = self.cartridge_rom[start..end].to_owned();
                    let padding_size = ((length as usize) >> 2) - read_size;
                    if padding_size > 0 {
                        res.extend(vec![0; padding_size]);
                    }
                    Ok(res)
                } else {
                    // println!("about to fail reading from start=${:08X}", start);
                    Err(ReadWriteFault::Invalid)
                }
            }
        } else {
            todo!("probably not used ever");
        }
    }

    fn write_block(&mut self, offset: usize, block: &[u32], length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
        if (block.len() * 4) as u32 != length { todo!(); }
        let length = if length > (256*1024) { 256 * 1024 } else { length };
        if offset >= 0x0800_0000 && offset < 0x1000_0000 { // SRAM
            let offset = offset & 0x07FF_FFFF;
            if self.flash.is_some() {
                self.write_flash_block(offset, block, length)
            } else {
                self.write_sram_block(offset, block, length)
            }
        } else {
            unimplemented!("PI: unknown DMA to ${:08X}", offset);
        }
    }
}

impl Drop for PeripheralInterface {
    fn drop(&mut self) {
        //todo!();
        //if self.sram_dirty.is_some() {
        //    self.save_sram();
        //}
    }
}
