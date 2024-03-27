#[allow(unused_imports)]
use tracing::{trace,debug,info,warn,error};
use std::sync::mpsc;

use crate::*;

use rcp::DmaInfo;
use pifrom::PifRom;
use mips::{InterruptUpdate, InterruptUpdateMode, IMask_SI};

pub struct SerialInterface {
    comms: SystemCommunication,

    dma_completed_tx: mpsc::Sender<DmaInfo>,
    dma_completed_rx: mpsc::Receiver<DmaInfo>,

    interrupt_flag: bool,
    pif: PifRom,

    dram_address: u32,
}

impl SerialInterface {
    pub fn new(comms: SystemCommunication, pif: PifRom) -> Self {
        let (tx, rx) = mpsc::channel();

        SerialInterface {
            comms: comms,

            dma_completed_tx: tx,
            dma_completed_rx: rx,

            interrupt_flag: false,
            pif: pif,

            dram_address: 0,
        }
    }

    pub fn reset(&mut self) {
        info!(target: "SI", "reset");
        while self.dma_completed_rx.try_recv().is_ok() {}
        self.interrupt_flag = false;
        self.dram_address = 0;
        self.pif.reset();
    }

    pub fn step(&mut self) {
        if let Ok(_) = self.dma_completed_rx.try_recv() {
            //println!("SerialInterface::step generating SI");
            self.interrupt_flag = true;
            self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_SI, InterruptUpdateMode::SetInterrupt)).unwrap();
        }
    }

    pub fn read_register(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "SI", "read32 register offset=${:08X}", offset);

        let result = match offset {
            // SI_STATUS
            0x0_0018 => {
                (self.interrupt_flag as u32) << 12
            },

            _ => {
                warn!(target: "SI", "unimplemented SI register read offset=${:08X}", offset);
                0
            },
        };

        Ok(result)
    }

    pub fn write_register(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "SI", "write32 register value=${:08X} offset=${:08X}", value, offset);

        match offset {
            // SI_DRAM_ADDR
            0x0_0000 => {
                self.dram_address = value & 0x00FF_FFFF;
            },

            // SI_PIF_AD_RD64B - DMA 64 bytes from PIF-RAM to RDRAM
            0x0_0004 => {
                // Sanity check for now
                if (value & 0xFFF) != 0x7C0 {
                    todo!("code starts PIF-RAM dma at an address not 0x7C0 in PIF-RAM");
                }

                let dma_info = DmaInfo {
                    initiator     : "PI-RD64B",
                    source_address: 0x1FC0_07C0, // start of PIF-RAM
                    dest_address  : self.dram_address & !0x07,
                    count         : 1,  // one loop
                    length        : 64, // 64 bytes
                    completed     : Some(self.dma_completed_tx.clone()),
                    ..Default::default()
                };

                // start DMA
                self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                self.comms.break_cpu();
            },

            // SI_PIF_AD_WR64B - DMA 64 bytes from RDRAM to PIF-RAM
            0x0_0010 => {
                // Sanity check for now
                if (value & 0xFFF) != 0x7C0 {
                    todo!("code starts PIF-RAM dma at an address not 0x7C0 in PIF-RAM");
                }

                let dma_info = DmaInfo {
                    initiator     : "PI-WR64B",
                    source_address: self.dram_address & !0x07,
                    dest_address  : 0x1FC0_07C0, // start of PIF-RAM
                    count         : 1,  // one loop
                    length        : 64, // 64 bytes
                    completed     : Some(self.dma_completed_tx.clone()),
                    ..Default::default()
                };

                // start DMA
                self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                self.comms.break_cpu();
            },

            // SI_STATUS
            0x0_0018 => {
                self.interrupt_flag = false;
                self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_SI, InterruptUpdateMode::ClearInterrupt)).unwrap();
            },

            _ => {
                warn!(target: "SI", "unimplemented SI register write value=${:08X} offset=${:08X}", value, offset);
                return Err(ReadWriteFault::Invalid);
            }
        }
        Ok(WriteReturnSignal::None)
    }
}

impl Addressable for SerialInterface {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        //trace!(target: "SI", "read32 offset=${:08X}", offset);

        match offset & 0x7FE0_0000 {
            0x1FC0_0000 => self.pif.read_u32(offset & 0x000F_FFFF),
            _ => self.read_register(offset),
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "SI", "write32 value=${:08X}, offset=${:08X}", value, offset);

        match offset & 0x7FE0_0000 {
            0x1FC0_0000 => {
                //println!("SI write32 generating interrupt");
                // all writes into PIFROM/RAM generate SI interrupt
                //self.interrupt_flag = true;
                //self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_SI, InterruptUpdateMode::SetInterrupt)).unwrap();
                self.pif.write_u32(value, offset & 0x000F_FFFF)
            },

            _ => self.write_register(value, offset),
        }
    }

    fn write_u16(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        //trace!(target: "SI", "write32 value=${:08X}, offset=${:08X}", value, offset);

        match offset & 0x7FE0_0000 {
            0x1FC0_0000 => {
                //println!("SI write16 generating interrupt");
                //self.interrupt_flag = true;
                //self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_SI, InterruptUpdateMode::SetInterrupt)).unwrap();
                self.pif.write_u16(value, offset & 0x000F_FFFF)
            },
            _ => todo!(),
        }
    }

    fn write_u8(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        //trace!(target: "SI", "write32 value=${:08X}, offset=${:08X}", value, offset);

        match offset & 0x7FE0_0000 {
            0x1FC0_0000 => {
                //println!("SI write8 generating interrupt");
                //self.interrupt_flag = true;
                //self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_SI, InterruptUpdateMode::SetInterrupt)).unwrap();
                self.pif.write_u8(value, offset & 0x000F_FFFF)
            },
            _ => todo!(),
        }
    }

    fn write_block(&mut self, address: usize, block: &[u32], length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
        if (block.len() * 4) as u32 != length { todo!(); }
        match address & 0x7FE0_0000 {
            0x1FC0_0000 => {
                self.pif.write_block(address & 0x001F_FFFF, block, length)
            },
            _ => todo!(),
        }
    }

    fn read_block(&mut self, offset: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        match offset & 0x7FE0_0000 {
            0x1FC0_0000 => {
                self.pif.read_block(offset & 0x001F_FFFF, length)
            },
            _ => todo!(),
        }
    }
}
