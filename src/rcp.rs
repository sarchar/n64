use tracing::debug;

use crate::*;

use crate::rdp::Rdp;
use crate::rsp::Rsp;
use crate::pifrom::PifRom;
use crate::peripheral::PeripheralInterface;
use crate::rdram::RdramInterface;
use crate::mips::MipsInterface;

/// N64 Reality Control Processor
/// Contains the on-board RSP, RDP and manages the system bus
pub struct Rcp {
    rdp: Rdp,
    rsp: Rsp,

    // bus objects
    pif: PifRom,
    pi: PeripheralInterface,
    ri: RdramInterface,
    mi: MipsInterface,
}

impl Rcp {
    pub fn new(pif: PifRom, pi: PeripheralInterface) -> impl Addressable {
        Rcp {
            rdp: Rdp::new(),
            rsp: Rsp::new(),
            ri: RdramInterface::new(),
            mi: MipsInterface::new(),
            pif: pif,
            pi: pi,
        }
    }

    fn rcp_read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        debug!(target: "RCP", "read32 offset=${:08X}", offset);

        match (offset & 0x00F0_0000) >> 20 {
            // RSP range 0x0400_0000-0x040F_FFFF 
            0 => self.rsp.read_u32(offset & 0x000F_FFFF),

            // RDP 0x0410_0000-0x042F_FFFF
            1..=2 => self.rdp.read_u32(offset & 0x003F_FFFF),

            // MI 0x0430_0000-0x043F_FFFF
            3 => self.mi.read_u32(offset & 0x000F_FFFF),

            // VI 0x0440_0000-0x044F_FFFF
            4 => panic!("Video interface (VI) read"),

            // AI 0x0450_0000-0x045F_FFFF
            5 => panic!("Audio interface (AI) read"),
            
            // PI 0x0460_0000-0x046F_FFFF
            6 => self.pi.read_u32(offset & 0x7FFF_FFFF),

            // RDRAM 0x0470_0000-0x047F_FFFF
            // we pass bit 26 along to indicate the RDRAM interface vs RDRAM access
            7 => self.ri.read_u32(0x0400_0000 | (offset & 0x000F_FFFF)),

            // SI 0x0480_0000-0x048F_FFFF
            8 => {
                debug!(target: "RCP", "unimplemented serial interface (SI) read");
                Ok(0)
            }

            // 0x0409_0000-0x04FF_FFFF unmapped
            _ => panic!("invalid RCP read32"),
        }
    }

    fn rcp_read_u16(&mut self, offset: usize) -> Result<u16, ReadWriteFault> {
        debug!(target: "RCP", "read16 offset=${:08X}", offset);

        match (offset & 0x00F0_0000) >> 20 {
            // RSP range 0x0400_0000-0x040F_FFFF 
            0 => self.rsp.read_u16(offset & 0x000F_FFFF),

            // RDP 0x0410_0000-0x042F_FFFF
            1..=2 => self.rdp.read_u16(offset & 0x003F_FFFF),

            // MI 0x0430_0000-0x043F_FFFF
            3 => self.mi.read_u16(offset & 0x000F_FFFF),

            // VI 0x0440_0000-0x044F_FFFF
            4 => panic!("Video interface (VI) read16"),

            // AI 0x0450_0000-0x045F_FFFF
            5 => panic!("Audio interface (AI) read16"),
            
            // PI 0x0460_0000-0x046F_FFFF
            6 => self.pi.read_u16(offset & 0x7FFF_FFFF),

            // RDRAM 0x0470_0000-0x047F_FFFF
            // we pass bit 26 along to indicate the RDRAM interface vs RDRAM access
            7 => self.ri.read_u16(0x0400_0000 | (offset & 0x000F_FFFF)),

            // SI 0x0480_0000-0x048F_FFFF
            8 => {
                debug!(target: "RCP", "unimplemented serial interface (SI) read16");
                Ok(0)
            }

            // 0x0409_0000-0x04FF_FFFF unmapped
            _ => panic!("invalid RCP read16"),
        }
    }

    fn rcp_write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        debug!(target: "RCP", "write32 value=${:08X} offset=${:08X}", value, offset);

        let ret = match (offset & 0x00F0_0000) >> 20 {
            // RSP range 0x0400_0000-0x040F_FFFF 
            0 => self.rsp.write_u32(value, offset & 0x000F_FFFF),

            // RDP 0x0410_0000-0x041F_FFFF
            1..=2 => self.rdp.write_u32(value, offset & 0x003F_FFFF),

            // MI 0x0430_0000-0x043F_FFFF
            3 => self.mi.write_u32(value, offset & 0x000F_FFFF),

            // VI 0x0440_0000-0x044F_FFFF
            4 => {
                debug!(target: "RCP", "unimplemented VI: write32");
                Ok(WriteReturnSignal::None)
            },

            // AI 0x0450_0000-0x045F_FFFF
            5 => {
                debug!(target: "RCP", "unimplemented AI: write32");
                Ok(WriteReturnSignal::None)
            },
            
            // PI 0x0460_0000-0x046F_FFFF
            6 => self.pi.write_u32(value, offset & 0x7FFF_FFFF),

            // RDRAM 0x0470_0000-0x047F_FFFF
            // we pass bit 26 along to indicate the RDRAM interface vs RDRAM access
            7 => self.ri.write_u32(value, 0x0400_0000 | (offset & 0x000F_FFFF)),

            // SI 0x0480_0000-0x048F_FFFF
            8 => {
                debug!(target: "RCP", "unimplemented SI: write32");
                Ok(WriteReturnSignal::None)
            },

            // 0x0409_0000-0x04FF_FFFF unmapped
            _ => panic!("invalid RCP write"),
        };

        if let Ok(ref v) = ret {
            if let WriteReturnSignal::StartDMA(dma_info) = v {
                self.start_dma(dma_info)
            }
        }

        ret
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
    fn start_dma(&mut self, dma_info: &DmaInfo) {
        debug!(target: "RCP::DMA", "performing DMA!");

        //assert!(source_data.len() == dma_info.count as usize);
        assert!((dma_info.count % 4) == 0);

        for i in 0..(dma_info.count / 4) {
            let j = i as usize;
            let source_data = self.pi.get_dma_info(&dma_info);
            let word = ((source_data[j*4 + 0] as u32) << 24)
                        | ((source_data[j*4 + 1] as u32) << 16)
                        | ((source_data[j*4 + 2] as u32) << 8)
                        | ((source_data[j*4 + 3] as u32));

            let _ = self.write_u32(word, (dma_info.dest_address + i * 4) as usize);
        }
    }
}

impl Addressable for Rcp {
    /// TEMP debug
    fn print_debug_ipl2(&self) {
        self.rsp.print_debug_ipl2();
    }

    // The RCP handles all bus arbitration, so that's why the primary bus access is
    // part of the Rcp module
    fn read_u32(&mut self, address: usize) -> Result<u32, ReadWriteFault> {
        let (_segment, physical_address) = self.get_physical_address(address);
        debug!(target: "RCP", "bus read32 address=${:08X} physical=${:08X}", address, physical_address);

        match physical_address & 0xFC00_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000 => self.ri.read_u32(physical_address & 0x03FF_FFFF),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000 => self.rcp_read_u32(physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7C00_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    // the SI external bus only has the PIF, so forward all access to it
                    self.pif.read_u32(physical_address & 0x000FFFFF)
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
        debug!(target: "RCP", "bus read16 address=${:08X} physical=${:08X}", address, physical_address);

        match physical_address & 0xFC00_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000 => self.ri.read_u16(physical_address & 0x03FF_FFFF),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000 => self.rcp_read_u16(physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7C00_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    // the SI external bus only has the PIF, so forward all access to it
                    self.pif.read_u16(physical_address & 0x000FFFFF)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.read_u16(physical_address & 0x7FFF_FFFF)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }

    fn write_u32(&mut self, value: u32, address: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        let (_segment, physical_address) = self.get_physical_address(address);
        debug!(target: "RCP", "bus write32 value=${:08X} address=${:08X} physical=${:08X}", value, address, physical_address);

        match physical_address & 0xFFF0_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000..=0x03F0_0000 => self.ri.write_u32(value, physical_address & 0x03FF_FFFF),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000..=0x04F0_0000 => self.rcp_write_u32(value, physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7FF0_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    // the SI external bus only has the PIF, so forward all access to it
                    self.pif.write_u32(value, physical_address & 0x000FFFFF)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.write_u32(value, physical_address & 0x7FFF_FFFF)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }
}


