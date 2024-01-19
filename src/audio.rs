#[allow(unused_imports)]
use tracing::{debug, error, trace, info, warn};

use crate::*;

use mips::{InterruptUpdate, InterruptUpdateMode, IMask_AI};

#[derive()]
pub struct AudioInterface {
    comms: SystemCommunication,

    dram_address: u32,
    transfer_length: u32,
    dma_enable: bool,

    // TEMP just for now
    first_transfer: bool,
}

impl AudioInterface {
    pub fn new(comms: SystemCommunication) -> Self {
        Self {
            comms: comms,

            dram_address: 0,
            transfer_length: 0,
            dma_enable: false,

            first_transfer: true,
        }
    }

    pub fn step(&mut self) {
        if self.first_transfer && self.transfer_length > 0 {
            self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_AI, InterruptUpdateMode::SetInterrupt)).unwrap();
            self.first_transfer = false;
        }
    }
}

impl Addressable for AudioInterface {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "AI", "read32 address=${:08X}", offset);
        todo!("address=${:08X}", offset);

        //Ok(0)
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "AI", "write32 value=${:08X} address=${:08X}", value, offset);

        match offset {
            // AI_DRAM_ADDR
            0x0_0000 => {
                trace!(target: "AI", "write32 AI_DRAM_ADDR value=${:08X}", value);
                self.dram_address = value & 0x00FFFFF8;
            },

            // AI_LENGTH
            0x0_0004 => {
                trace!(target: "AI", "write32 AI_LENGTH value=${:08X}", value);
                self.transfer_length = value & 0x0003FFF8;
            },

            // AI_CONTROL
            0x0_0008 => {
                trace!(target: "AI", "write32 AI_CONTROL value=${:08X}", value);
                self.dma_enable = (offset & 0x01) == 0x01;
                if self.dma_enable {
                    warn!(target: "AI", "audio enabled. what do?");
                }
            },

            // AI_STATUS
            0x0_000C => {
                // this register is read only, and writes ACK interrupts
                trace!(target: "AI", "write32 AI_STATUS value=${:08X}", value);
                self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_AI, InterruptUpdateMode::ClearInterrupt)).unwrap();
            },

            // AI_DACRATE
            0x0_0010 => {
                trace!(target: "AI", "write32 AI_DACRATE value=${:08X}", value);
                info!(target: "AI", "setting DAC sample rate to {}", 48726144 / (value + 1));
            },

            // AI_BITRATE
            0x0_0014 => {
                trace!(target: "AI", "write32 AI_BITRATE value=${:08X}", value);
            },

            _ => {
                todo!();
            },
        }

        Ok(WriteReturnSignal::None)
    }
}
