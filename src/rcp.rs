use std::fmt;
use std::mem;
use std::sync::{mpsc, Arc, Mutex};

#[allow(unused_imports)]
use tracing::{debug, error, trace, info};

use crate::*;

use crate::audio::AudioInterface;
use crate::mips::MipsInterface;
use crate::peripheral::PeripheralInterface;
use crate::pifrom::PifRom;
use crate::rdp::Rdp;
use crate::rdram::RdramInterface;
use crate::rsp::Rsp;
use crate::serial::SerialInterface;
use crate::video::VideoInterface;

pub struct DmaInfo {
    pub initiator     : &'static str, // for debugging
    pub source_address: u32, // RCP physical address
    pub dest_address  : u32, // RCP physical address
    pub count         : u32, // number of lines
    pub length        : u32, // length of a line
    pub source_stride : u32, // bytes to skip on the source after copy
    pub dest_stride   : u32, // bytes to skip on the dest after a copy 
    pub source_mask   : u32, // address mask after incrementing
    pub dest_mask     : u32, // address mask after incrementing
    pub source_bits   : u32, // address bits to always OR in after incrementing source
    pub dest_bits     : u32, // address bits to always OR in after incrementing dest

    // callback function to let the initiator know the DMA has completed
    pub completed     : Option<mpsc::Sender<DmaInfo>>,
}

impl Default for DmaInfo {
    fn default() -> Self {
        Self {
            initiator     : "default",
            source_address: 0,
            dest_address  : 0,
            count         : 0,
            length        : 0,
            source_stride : 0,
            dest_stride   : 0,
            source_mask   : 0xFFFF_FFFF,
            dest_mask     : 0xFFFF_FFFF,
            source_bits   : 0,
            dest_bits     : 0,
            completed     : None,
        }
    }
}

impl fmt::Debug for DmaInfo {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "DmaInfo {{ initiator: {:?}, source_address: ${:08X}, dest_address: ${:08X}, count: {}, length: {}, source_stride: ${:X}, dest_stride: ${:X}, source_mask: ${:08X}, dest_mask: ${:08X}, source_bits: ${:X}, dest_bits: ${:X} }}",
                    self.initiator, self.source_address, self.dest_address, self.count, self.length, self.source_stride, self.dest_stride, self.source_mask, self.dest_mask, self.source_bits, self.dest_bits)
    }
}

/// N64 Reality Control Processor
/// Contains the on-board RSP, RDP and manages the system bus
pub struct Rcp {
    // bus objects
    ai: AudioInterface,
    pub mi: MipsInterface,
    pi: PeripheralInterface,
    pub ri: RdramInterface,
    rsp: Rsp,
    si: SerialInterface,
    pub vi: VideoInterface,

    // The RDP needs to be accessible from both the CPU and the RSP, so we use a 
    // thread safe rwlock to access it
    rdp: LockedAddressable<Rdp>,

    // state

    // communication channels
    start_dma_rx: mpsc::Receiver<DmaInfo>,
}

impl Rcp {
    pub fn new(mut comms: SystemCommunication, boot_rom: Vec<u8>, cartridge_rom: Vec<u8>) -> Rcp {
        // create the start dma channel
        let (start_dma_tx, start_dma_rx) = mpsc::channel();
        comms.start_dma_tx = Some(start_dma_tx.clone());

        // create MI first so we can create the interrupt interconnect
        let mut mi = MipsInterface::new(comms.clone());
        comms.mi_interrupts_tx = Some(mi.get_update_channel());

        // create the PI first
        let mut pi = PeripheralInterface::new(comms.clone(), cartridge_rom);

        // the PIF-ROM needs to know what CIC chip the cartridge is using, so we pass it along
        let pif = PifRom::new(comms.clone(), boot_rom, &mut pi);

        // create the RDP
        let rdp = Arc::new(Mutex::new(Rdp::new(comms.clone())));

        // create the RSP
        let rsp = Rsp::new(comms.clone(), rdp.clone());

        Rcp {
            ai : AudioInterface::new(comms.clone()),
            pi : pi,
            rdp: LockedAddressable::new(rdp), // wrap rdp in a LockedAddressable so that match_addressable can return the rdp
            ri : RdramInterface::new(comms.clone()),
            rsp: rsp,
            si : SerialInterface::new(comms.clone(), pif),
            vi : VideoInterface::new(comms.clone()),

            start_dma_rx: start_dma_rx,
            mi : mi,
        }
    }

    pub fn start(&mut self) {
        self.rsp.start();
    }

    pub fn stop(&mut self) {
        info!(target: "RCP", "stop");
        // stop rsp and rdp
        self.rsp.stop();
        //self.rdp.stop();
    }

    pub fn reset(&mut self) {
        info!(target: "RCP", "reset");
        self.rsp.reset(); // still stop RSP
        self.rdp.lock().unwrap().reset();
        self.pi.reset();
        self.ai.reset();
        self.ri.reset();
        self.si.reset();
        self.vi.reset();
        self.mi.reset();

        while self.start_dma_rx.try_recv().is_ok() { }
    }

    pub fn calculate_free_cycles(&self) -> u64 {
        // return the min cycles available to all the modules
        let mut cycles = u64::MAX;
        cycles = std::cmp::min(cycles, self.vi.calculate_free_cycles());
        cycles
    }

    pub fn step(&mut self, cpu_cycles_elapsed: u64) {
        // run DMAs before stepping modules, as they often check for dma completion
        // and this way we can trigger interrupts asap
        while let Some(mut dma_info) = self.should_dma() {
            if !dma_info.initiator.starts_with("PI-RD64B") && !dma_info.initiator.starts_with("PI-WR64B") { // don't display PI-RD64B and PI-WR64B
                trace!(target: "DMA", "performing dma: DmaInfo = {:?}", dma_info);
            }

            if let Err(_) = self.do_dma(&mut dma_info) {
                todo!("handle dma error");
            }

            let cb_maybe = mem::replace(&mut dma_info.completed, None);
            if let Some(cb) = cb_maybe {
                let _ = cb.send(dma_info).unwrap();
            }
        }

        // the order here is somewhat important, but MI must be last
        self.rsp.step();
        self.pi.step();
        self.si.step();
        self.vi.step(cpu_cycles_elapsed);
        //{
        //    let mut rdp = self.rdp.lock().unwrap();
        //    rdp.step();
        //}

        // MI is last to process any incoming interrupts generated by the other modules
        self.mi.step();
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

    // given an RCP bus address, return an addressable object on the bus with the given offset
    // adjusted relative to the addressable
    fn match_addressable(&mut self, physical_address: usize, mode: &str) -> (Option<&mut dyn Addressable>, usize) {
        let physical_address = physical_address & !0x8000_0000;

        match physical_address & 0xFC00_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000 => {
                let repeat_count = self.mi.get_repeat_count(); // set the repeat count on RDRAM writes
                self.ri.set_repeat_count(repeat_count);
                (Some(&mut self.ri), physical_address & 0x03FF_FFFF)
            },

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000 => {
                let offset = physical_address & 0x00FF_FFFF;
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
                    5 => (Some(&mut self.ai), offset & 0x000F_FFFF),
                    
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
                    _ => panic!("invalid RCP {mode}"),
                }
            },

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7C00_0000 => {
                if (physical_address & 0xFFE0_0000) == 0x1FC0_0000 {
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    (Some(&mut self.si), physical_address & 0x7FFF_FFFF)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    (Some(&mut self.pi), physical_address & 0x7FFF_FFFF)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }

    // Slow, maybe at some point we can do more of a direct memory copy
    fn do_dma(&mut self, dma_info: &mut DmaInfo) -> Result<(), ReadWriteFault> {
        let transfer_size = dma_info.count * dma_info.length;
        if transfer_size == 0 {
            return Ok(());
        }

        let mut source_address = dma_info.source_address;
        let mut dest_address = dma_info.dest_address;

        // copy dma_info.length bytes dma_info.count times, from dest to source, skipping _stride bytes between
        for _i in 0..dma_info.count {
            //info!(target: "DMA", "(_i={}) moving source_address=${:08X} length=${:08X} to dest_address=${:08X}", _i, source_address, dma_info.length, dest_address);
            let block = self.read_block(source_address as usize, dma_info.length)?;
            source_address += dma_info.length + dma_info.source_stride;
            source_address  = (source_address & dma_info.source_mask) | dma_info.source_bits;

            self.write_block(dest_address as usize, &block, dma_info.length)?;
            dest_address += dma_info.length + dma_info.dest_stride;
            dest_address  = (dest_address & dma_info.dest_mask) | dma_info.dest_bits;
        }

        Ok(())
    }
}

impl Addressable for Rcp {
    fn read_u64(&mut self, address: usize) -> Result<u64, ReadWriteFault> {
        trace!(target: "RCP", "read64 address=${:08X}", address);

        if let (Some(addressable), offset) = self.match_addressable(address, "read64") {
            addressable.read_u64(offset)
        } else {
            Ok(0)
        }
    }

    fn read_u32(&mut self, address: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RCP", "read32 address=${:08X}", address);

        if let (Some(addressable), offset) = self.match_addressable(address, "read32") {
            addressable.read_u32(offset)
        } else {
            Ok(0)
        }
    }

    fn read_u16(&mut self, address: usize) -> Result<u16, ReadWriteFault> {
        trace!(target: "RCP", "read16 address=${:08X}", address);

        if let (Some(addressable), offset) = self.match_addressable(address, "read16") {
            addressable.read_u16(offset)
        } else {
            Ok(0)
        }
    }

    fn read_u8(&mut self, address: usize) -> Result<u8, ReadWriteFault> {
        trace!(target: "RCP", "read8 address=${:08X}", address);

        if let (Some(addressable), offset) = self.match_addressable(address, "read8") {
            addressable.read_u8(offset)
        } else {
            Ok(0)
        }
    }

    fn write_u64(&mut self, value: u64, address: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RCP", "write64 value=${:08X} address=${:08X}", value, address);

        if let (Some(addressable), offset) = self.match_addressable(address, "write64") {
            addressable.write_u64(value, offset)
        } else {
            Ok(WriteReturnSignal::None)
        }
    }

    fn write_u32(&mut self, value: u32, address: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RCP", "write32 value=${:08X} address=${:08X}", value, address);

        if let (Some(addressable), offset) = self.match_addressable(address, "write32") {
            addressable.write_u32(value, offset)
        } else {
            Ok(WriteReturnSignal::None)
        }
    }

    fn write_u16(&mut self, value: u32, address: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RCP", "write16 value=${:08X} address=${:08X}", value, address);

        if let (Some(addressable), offset) = self.match_addressable(address, "write16") {
            addressable.write_u16(value, offset)
        } else {
            Ok(WriteReturnSignal::None)
        }
    }

    fn write_u8(&mut self, value: u32, address: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RCP", "write8 value=${:08X} address=${:08X}", value, address);

        if let (Some(addressable), offset) = self.match_addressable(address, "write8") {
            addressable.write_u8(value, offset)
        } else {
            Ok(WriteReturnSignal::None)
        }
    }

    fn read_block(&mut self, address: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        trace!(target: "RCP", "read_block length={} address=${:08X}", length, address);

        if let (Some(addressable), offset) = self.match_addressable(address, "read_block") {
            addressable.read_block(offset, length)
        } else {
            Err(ReadWriteFault::Invalid)
        }
    }

    fn write_block(&mut self, address: usize, block: &[u32], length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RCP", "write_block length={} address=${:08X}", block.len(), address);

        if let (Some(addressable), offset) = self.match_addressable(address, "write_block") {
            addressable.write_block(offset, block, length)
        } else {
            Err(ReadWriteFault::Invalid)
        }
    }
}


