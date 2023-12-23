use std::fmt;
use std::mem;
use std::sync::mpsc;

#[allow(unused_imports)]
use tracing::{debug, error, trace, info};

use crate::*;

use crate::mips::MipsInterface;
use crate::peripheral::PeripheralInterface;
use crate::pifrom::PifRom;
use crate::rdp::Rdp;
use crate::rdram::RdramInterface;
use crate::rsp::Rsp;
use crate::serial::SerialInterface;
use crate::video::VideoInterface;

#[derive(Default)]
pub struct DmaInfo {
    pub initiator     : &'static str, // for debugging
    pub source_address: u32, // RCP physical address
    pub dest_address  : u32, // RCP physical address
    pub count         : u32, // number of lines
    pub length        : u32, // length of a line
    pub source_stride : u32, // bytes to skip on the source after copy
    pub dest_stride   : u32, // bytes to skip on the dest after a copy 

    // callback function to let the initiator know the DMA has completed
    pub completed     : Option<mpsc::Sender<DmaInfo>>,
}

impl fmt::Debug for DmaInfo {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "DmaInfo {{ initiator: {:?}, source_address: ${:08X}, dest_address: ${:08X}, count: {}, length: {}, source_stride: ${:X}, dest_stride: ${:X} }}",
                    self.initiator, self.source_address, self.dest_address, self.count, self.length, self.source_stride, self.dest_stride)
    }
}

/// N64 Reality Control Processor
/// Contains the on-board RSP, RDP and manages the system bus
pub struct Rcp {
    // bus objects
    pub mi: MipsInterface,
    pi: PeripheralInterface,
    rdp: Rdp,
    pub ri: RdramInterface,
    rsp: Rsp,
    si: SerialInterface,
    pub vi: VideoInterface,

    // state

    // communication channels
    start_dma_rx: mpsc::Receiver<DmaInfo>,
}

impl Rcp {
    pub fn new(boot_rom: Vec<u8>, cartridge_rom: Vec<u8>) -> Rcp {
        // create the start dma channel
        let (start_dma_tx, start_dma_rx) = mpsc::channel();

        // create MI first so we can create the interrupt interconnect
        let mut mi = MipsInterface::new();

        // create the PI first
        let mut pi = PeripheralInterface::new(cartridge_rom, start_dma_tx.clone(), mi.get_update_channel());

        // the PIF-ROM needs to know what CIC chip the cartridge is using, so we pass it along
        let pif = PifRom::new(boot_rom, &mut pi);

        Rcp {
            pi : pi,
            rdp: Rdp::new(),
            ri : RdramInterface::new(),
            rsp: Rsp::new(start_dma_tx.clone(), mi.get_update_channel()),
            si : SerialInterface::new(pif, start_dma_tx.clone(), mi.get_update_channel()),
            vi : VideoInterface::new(mi.get_update_channel()),

            start_dma_rx: start_dma_rx,
            mi : mi,
        }
    }

    pub fn start(&mut self) {
        self.rsp.start();
    }

    pub fn step(&mut self, cpu_cycles_elapsed: u64) {
        self.rsp.step();
        self.pi.step();
        self.si.step();
        self.vi.step(cpu_cycles_elapsed);

        // MI should be last to process any incoming interrupts
        self.mi.step();

        // run all the DMAs for the cycle
        loop {
            if let Some(mut dma_info) = self.should_dma() {
                info!(target: "RCP", "starting dma, DmaInfo = {:?}", dma_info);

                if let Err(_) = self.do_dma(&dma_info) {
                    todo!("handle dma error");
                }

                let cb_maybe = mem::replace(&mut dma_info.completed, None);
                if let Some(cb) = cb_maybe {
                    let _ = cb.send(dma_info).unwrap();
                }
                continue;
            }
            break;
        }
    }

    pub fn should_dma(&mut self) -> Option<DmaInfo> {
        if let Ok(dma_info) = self.start_dma_rx.try_recv() {
            return Some(dma_info);
        }
        None
    }


    pub fn should_interrupt(&mut self) -> u32 {
        self.mi.should_interrupt()
    }

    fn rcp_read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RCP", "read32 offset=${:08X}", offset);

        if let (Some(addressable), offset) = self.match_addressable(offset, "read32") {
            addressable.read_u32(offset)
        } else {
            Ok(0)
        }
    }

    fn rcp_read_u16(&mut self, offset: usize) -> Result<u16, ReadWriteFault> {
        trace!(target: "RCP", "read16 offset=${:08X}", offset);

        if let (Some(addressable), offset) = self.match_addressable(offset, "read16") {
            addressable.read_u16(offset)
        } else {
            Ok(0)
        }
    }

    fn rcp_read_u8(&mut self, offset: usize) -> Result<u8, ReadWriteFault> {
        trace!(target: "RCP", "read8 offset=${:08X}", offset);

        if let (Some(addressable), offset) = self.match_addressable(offset, "read8") {
            addressable.read_u8(offset)
        } else {
            Ok(0)
        }
    }

    fn rcp_read_block(&mut self, offset: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        trace!(target: "RCP", "read_block offset=${:08X}", offset);

        if let (Some(addressable), offset) = self.match_addressable(offset, "read_block") {
            addressable.read_block(offset, length)
        } else {
            Ok(vec![])
        }
    }

    fn rcp_write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RCP", "write32 value=${:08X} offset=${:08X}", value, offset);

        if let (Some(addressable), offset) = self.match_addressable(offset, "write32") {
            addressable.write_u32(value, offset)
        } else {
            Ok(WriteReturnSignal::None)
        }
    }

    fn rcp_write_u16(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RCP", "write16 value=${:08X} offset=${:08X}", value, offset);

        if let (Some(addressable), offset) = self.match_addressable(offset, "write16") {
            addressable.write_u16(value, offset)
        } else {
            Ok(WriteReturnSignal::None)
        }
    }

    fn rcp_write_u8(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RCP", "write8 value=${:08X} offset=${:08X}", value, offset);

        if let (Some(addressable), offset) = self.match_addressable(offset, "write8") {
            addressable.write_u8(value, offset)
        } else {
            Ok(WriteReturnSignal::None)
        }
    }

    fn rcp_write_block(&mut self, offset: usize, block: &[u32]) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RCP", "write_block offset=${:08X}", offset);

        if let (Some(addressable), offset) = self.match_addressable(offset, "read_block") {
            addressable.write_block(offset, block)
        } else {
            Ok(WriteReturnSignal::None)
        }
    }

    // given an RCP bus address, return another addressable object on the bus with the given offset
    // adjusted to the addressable
    fn match_addressable(&mut self, offset: usize, mode: &str) -> (Option<&mut dyn Addressable>, usize) {
        match (offset & 0x00F0_0000) >> 20 {
            // RSP range 0x0400_0000-0x040F_FFFF 
            0 => (Some(&mut self.rsp), offset & 0x000F_FFFF),

            // RDP 0x0410_0000-0x041F_FFFF
            1..=2 => (Some(&mut self.rdp), offset & 0x003F_FFFF),

            // MI 0x0430_0000-0x043F_FFFF
            3 => (Some(&mut self.mi), offset & 0x000F_FFFF),

            // VI 0x0440_0000-0x044F_FFFF
            4 => (Some(&mut self.vi), offset & 0x000F_FFFF),

            // AI 0x0450_0000-0x045F_FFFF
            5 => {
                error!(target: "RCP", "unimplemented AI {mode}");
                (None, 0)
            },
            
            // PI 0x0460_0000-0x046F_FFFF
            6 => (Some(&mut self.pi), offset & 0x7FFF_FFFF),

            // RDRAM 0x0470_0000-0x047F_FFFF
            // we pass bit 26 along to indicate the RdramInterface vs RDRAM access
            7 => {
                let repeat_count = self.mi.get_repeat_count();
                self.ri.set_repeat_count(repeat_count);
                (Some(&mut self.ri), 0x0400_0000 | (offset & 0x000F_FFFF))
            },

            // SI 0x0480_0000-0x048F_FFFF
            8 => (Some(&mut self.si), offset & 0x000F_FFFF),

            // 0x0409_0000-0x04FF_FFFF unmapped
            _ => panic!("invalid RCP write"),
        }
    }

    fn get_physical_address(&self, address: usize) -> (MemorySegment, usize) {
        // N64 memory is split into segments that are either cached or uncached, 
        // memory mapped or not, and user or kernel protected. 
        if (address & 0x8000_0000) == 0 {
            (MemorySegment::UserSpace, address & 0x7FFF_FFFF)
        } else {
            let s = match (address & 0x6000_0000) >> 29 {
                0 => MemorySegment::KSeg0,
                1 => MemorySegment::KSeg1,
                2 => MemorySegment::KSSeg,
                3 => MemorySegment::KSeg3,
                _ => panic!("not valid")
            };
            (s, address & 0x1FFF_FFFF)
        }
    }

    // Slow, maybe at some point we can do more of a direct memory copy
    fn do_dma(&mut self, dma_info: &DmaInfo) -> Result<(), ReadWriteFault> {
        //debug!(target: "RCP", "performing DMA!");

        if (dma_info.length % 4) != 0 {
            return Err(ReadWriteFault::Invalid);
        } 

        if (dma_info.count * dma_info.length) == 0 {
            return Ok(());
        }

        let mut source_address = dma_info.source_address & !0x07;
        let mut dest_address = dma_info.dest_address & !0x07;

        // copy dma_info.length bytes dma_info.count times, from dest to source, skipping _stride bytes between
        for _ in 0..dma_info.count {
            let block = self.read_block(source_address as usize, dma_info.length)?;
            self.write_block(dest_address as usize, &block)?;
            source_address += dma_info.source_stride;
            dest_address += dma_info.dest_stride;
        }

        Ok(())

        //for i in 0..(dma_info.count / 4) {
        //    let j = i as usize;
        //    let source_data = self.pi.get_dma_info(&dma_info);
        //    let word = ((source_data[j*4 + 0] as u32) << 24)
        //                | ((source_data[j*4 + 1] as u32) << 16)
        //                | ((source_data[j*4 + 2] as u32) << 8)
        //                | ((source_data[j*4 + 3] as u32));

        //    let _ = self.write_u32(word, (dma_info.dest_address + i * 4) as usize);
        //}
    }
}

impl Addressable for Rcp {
    // The RCP handles all bus arbitration, so that's why the primary bus access is
    // part of the Rcp module
    fn read_u32(&mut self, address: usize) -> Result<u32, ReadWriteFault> {
        let (_segment, physical_address) = self.get_physical_address(address);
        trace!(target: "RCP", "bus read32 address=${:08X} physical=${:08X}", address, physical_address);

        match physical_address & 0xFC00_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000 => self.ri.read_u32(physical_address & 0x03FF_FFFF),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000 => self.rcp_read_u32(physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7C00_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    self.si.read_u32(physical_address & 0x7FFF_FFFF)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.read_u32(physical_address & 0x7FFF_FFFF)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }

    fn read_u16(&mut self, address: usize) -> Result<u16, ReadWriteFault> {
        let (_segment, physical_address) = self.get_physical_address(address);
        trace!(target: "RCP", "bus read16 address=${:08X} physical=${:08X}", address, physical_address);

        match physical_address & 0xFC00_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000 => self.ri.read_u16(physical_address & 0x03FF_FFFF),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000 => self.rcp_read_u16(physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7C00_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    self.si.read_u16(physical_address & 0x7FFF_FFFF)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.read_u16(physical_address & 0x7FFF_FFFF)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }

    fn read_u8(&mut self, address: usize) -> Result<u8, ReadWriteFault> {
        let (_segment, physical_address) = self.get_physical_address(address);
        trace!(target: "RCP", "bus read8 address=${:08X} physical=${:08X}", address, physical_address);

        match physical_address & 0xFC00_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000 => self.ri.read_u8(physical_address & 0x03FF_FFFF),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000 => self.rcp_read_u8(physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7C00_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    // the SI external bus only has the PIF, so forward all access to it
                    self.si.read_u8(physical_address & 0x7FFF_FFFF)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.read_u8(physical_address & 0x7FFF_FFFF)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }


    fn write_u32(&mut self, value: u32, address: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        let (_segment, physical_address) = self.get_physical_address(address);
        trace!(target: "RCP", "bus write32 value=${:08X} address=${:08X} physical=${:08X}", value, address, physical_address);

        match physical_address & 0xFFF0_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000..=0x03F0_0000 => {
                let repeat_count = self.mi.get_repeat_count();
                self.ri.set_repeat_count(repeat_count);
                self.ri.write_u32(value, physical_address & 0x03FF_FFFF)
            },

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000..=0x04F0_0000 => self.rcp_write_u32(value, physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7FF0_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    self.si.write_u32(value, physical_address & 0x7FFF_FFFF)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.write_u32(value, physical_address & 0x7FFF_FFFF)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }

    fn write_u16(&mut self, value: u32, address: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        let (_segment, physical_address) = self.get_physical_address(address);
        trace!(target: "RCP", "bus write16 value=${:08X} address=${:08X} physical=${:08X}", value, address, physical_address);

        match physical_address & 0xFFF0_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000..=0x03F0_0000 => self.ri.write_u16(value, physical_address & 0x03FF_FFFF),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000..=0x04F0_0000 => self.rcp_write_u16(value, physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7FF0_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    self.si.write_u16(value, physical_address & 0x7FFF_FFFF)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.write_u16(value, physical_address & 0x7FFF_FFFF)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }

    fn write_u8(&mut self, value: u32, address: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        let (_segment, physical_address) = self.get_physical_address(address);
        trace!(target: "RCP", "bus write16 value=${:08X} address=${:08X} physical=${:08X}", value, address, physical_address);

        match physical_address & 0xFFF0_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000..=0x03F0_0000 => self.ri.write_u8(value, physical_address & 0x03FF_FFFF),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000..=0x04F0_0000 => self.rcp_write_u8(value, physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7FF0_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    // the SI external bus only has the PIF, so forward all access to it
                    self.si.write_u8(value, physical_address & 0x7FFF_FFFF)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.write_u8(value, physical_address & 0x7FFF_FFFF)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }

    fn read_block(&mut self, address: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        let (_segment, physical_address) = self.get_physical_address(address);
        trace!(target: "RCP", "bus read_block length={} address=${:08X} physical=${:08X}", length, address, physical_address);

        match physical_address & 0xFFF0_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000..=0x03F0_0000 => self.ri.read_block(physical_address & 0x03FF_FFFF, length),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000..=0x04F0_0000 => self.rcp_read_block(physical_address & 0x00FF_FFFF, length),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7FF0_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    self.si.read_block(physical_address & 0x7FFF_FFFF, length)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.read_block(physical_address & 0x7FFF_FFFF, length)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }

    fn write_block(&mut self, address: usize, block: &[u32]) -> Result<WriteReturnSignal, ReadWriteFault> {
        let (_segment, physical_address) = self.get_physical_address(address);
        trace!(target: "RCP", "bus write_block length={} address=${:08X} physical=${:08X}", block.len(), address, physical_address);

        match physical_address & 0xFFF0_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000..=0x03F0_0000 => self.ri.write_block(physical_address & 0x03FF_FFFF, block),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000..=0x04F0_0000 => self.rcp_write_block(physical_address & 0x00FF_FFFF, block),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7FF0_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    self.si.write_block(physical_address & 0x7FFF_FFFF, block)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.write_block(physical_address & 0x7FFF_FFFF, block)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }
}


