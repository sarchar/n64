use std::fmt;
use std::mem;
use std::sync::{mpsc, Arc, Mutex};

#[allow(unused_imports)]
use tracing::{debug, error, trace, info};

use crate::*;

use crate::hle;
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

    // direct read or write to memory (when source or dest address == 0xFFFF_FFFF)
    pub internal_buffer: Option<Vec<u32>>,
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
    pub fn new(boot_rom: Vec<u8>, cartridge_rom: Vec<u8>, hle_command_buffer: Option<Arc<hle::HleCommandBuffer>>) -> Rcp {
        // create the start dma channel
        let (start_dma_tx, start_dma_rx) = mpsc::channel();

        // create MI first so we can create the interrupt interconnect
        let mut mi = MipsInterface::new();

        // create the PI first
        let mut pi = PeripheralInterface::new(cartridge_rom, start_dma_tx.clone(), mi.get_update_channel());

        // the PIF-ROM needs to know what CIC chip the cartridge is using, so we pass it along
        let pif = PifRom::new(boot_rom, &mut pi);

        // create the RDP
        let rdp = Arc::new(Mutex::new(Rdp::new(mi.get_update_channel())));

        // create the RSP
        let rsp = Rsp::new(rdp.clone(), start_dma_tx.clone(), mi.get_update_channel(), hle_command_buffer);

        Rcp {
            pi : pi,
            rdp: LockedAddressable::new(rdp), // wrap rdp in a LockedAddressable so that match_addressable can return the rdp
            ri : RdramInterface::new(),
            rsp: rsp,
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
        //{
        //    let mut rdp = self.rdp.lock().unwrap();
        //    rdp.step();
        //}

        // MI should be last to process any incoming interrupts
        self.mi.step();

        // run all the DMAs for the cycle
        loop {
            if let Some(mut dma_info) = self.should_dma() {
                if !dma_info.initiator.starts_with("PI-") { // don't display PI-RD64B and PI-WR64B
                    trace!(target: "DMA", "performing dma: DmaInfo = {:?}", dma_info);
                }

                if let Err(_) = self.do_dma(&mut dma_info) {
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
                    _ => panic!("invalid RCP {mode}"),
                }
            },

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7C00_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
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
        //debug!(target: "RCP", "performing DMA!");

        if (dma_info.length % 4) != 0 {
            return Err(ReadWriteFault::Invalid);
        } 

        let transfer_size = dma_info.count * dma_info.length;
        if transfer_size == 0 {
            return Ok(());
        }

        let mut source_address = dma_info.source_address & !0x07;
        let mut dest_address = dma_info.dest_address & !0x07;

        let mut ret_buffer = if dma_info.dest_address == 0xFFFF_FFFF {
            assert!(dma_info.dest_stride == 0); // internal buffer doesn't support stride
            assert!(dma_info.count == 1);       // we could support count != 1, but hasn't been necessary yet
            Some(Vec::with_capacity((transfer_size >> 2) as usize))
        } else {
            None
        };

        // copy dma_info.length bytes dma_info.count times, from dest to source, skipping _stride bytes between
        for _ in 0..dma_info.count {
            let block = self.read_block(source_address as usize, dma_info.length)?;
            source_address += dma_info.source_stride;

            if let Some(ref mut dest) = ret_buffer {
                dest.extend_from_slice(&block);
            } else {
                self.write_block(dest_address as usize, &block)?;
                dest_address += dma_info.dest_stride;
            }
        }

        dma_info.internal_buffer = ret_buffer;

        Ok(())
    }
}

impl Addressable for Rcp {
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

    fn write_block(&mut self, address: usize, block: &[u32]) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RCP", "write_block length={} address=${:08X}", block.len(), address);

        if let (Some(addressable), offset) = self.match_addressable(address, "write_block") {
            addressable.write_block(offset, block)
        } else {
            Err(ReadWriteFault::Invalid)
        }
    }
}


