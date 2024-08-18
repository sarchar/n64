use std::sync::atomic::Ordering;

#[allow(unused_imports)]
use tracing::{debug, error, info, trace, warn};

use crate::*;
//use mips::{InterruptUpdate, InterruptUpdateMode, IMask_DP};

pub struct Rdp {
    comms: SystemCommunication,

    start: u32,
    start_latch: u32,
    current: u32,
    intermediate_end: u32,
    end: u32,
    status: u32,

    clock: u32,
    last_clock_update: u64,
}

impl Rdp {
    pub fn new(comms: SystemCommunication) -> Rdp {
        Rdp {
            comms: comms,

            start: 0,
            start_latch: 0,
            current: 0,
            intermediate_end: 0,
            end: 0,
            status: 0,

            clock: 0,
            last_clock_update: 0,
        }
    }

    pub fn reset(&mut self) {
        info!(target: "RDP", "reset");
        self.start = 0;
        self.start_latch = 0;
        self.end = 0;
        self.status = 0;
    }

    pub fn update_clock(&mut self) {
        let cur = self.comms.total_cpu_steps.get() as u64;
        let delta = cur - self.last_clock_update;
        self.clock = self.clock.wrapping_add(delta as u32) & 0x00FF_FFFF;
        self.last_clock_update = cur * 2 / 3; // 62.5MHz on the cpu's 93.75MHz TODO
    }
}

impl Addressable for Rdp {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RDP", "read32 offset=${:08X}", offset);

        match offset {
            // DP_START 
            0x0010_0000 => {
                debug!(target: "RDP", "read DP_START");
                Ok(self.start_latch)
            },

            // DP_END 
            0x0010_0004 => {
                debug!(target: "RDP", "read DP_END");
                Ok(self.end)
            },

            // DP_CURRENT 
            0x0010_0008 => {
                debug!(target: "RDP", "read DP_CURRENT");
                // TODO for now assume all DMAs have completed and current just points to end
                Ok(self.current)
            },

            // DP_STATUS 
            0x0010_000C => {
                debug!(target: "RDP", "read DP_STATUS");

                let full_sync = self.comms.rdp_full_sync.load(Ordering::SeqCst);
                if full_sync != 0 {
                    self.status &= !0x40;
                }

                let ret = self.status;

                // if DMA is pending, pretend it happened
                if (self.status & 0x100) != 0 {
                    if (self.status & 0x200) != 0 { // END_PENDING means another DMA
                        self.status &= !0x200;
                        self.current = self.intermediate_end; // update for the next dma (intermediate_end..end)
                        // leave DMA busy set
                    } else {
                        // dma is finished
                        self.status &= !0x100;
                        self.current = self.end;
                    }
                }

                Ok(ret)
            },

            // DP_CLOCK
            0x0010_0010 => {
                self.update_clock();
                // trace!(target: "DP_CLOCK", "clock=${:08X}", self.clock);
                // TODO this doesn't seem to be the cause of OoT crashes...panic!();
                Ok(self.clock)
            },

            _ => {
                //error!(target: "RDP", "invalid or unimplemented RDP read from offset=${:08X}", offset);
                Ok(0)
            },
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RDP", "write32 value=${:08X} offset=${:08X}", value, offset);

        match offset {
            // DP_START 
            0x0010_0000 => {
                debug!(target: "RDP", "write DP_START");
                if (self.status & 0x400) == 0 {
                    self.start_latch = value & 0x00FF_FFF8;
                }

                // set START_PENDING
                self.status |= 0x400;
            },

            // DP_END 
            0x0010_0004 => {
                debug!(target: "RDP", "write DP_END");

                self.end = value & 0x00FF_FFF8;

                if (self.status & 0x400) != 0 { // if START_PENDING is set
                    if (self.status & 0x100) != 0 { // if DMA BUSY (dma is in progress) is set
                        let _ = self.read_u32(0x0010_000C)?;
                        // TODO self.status |= 0x200; // set END PENDING
                    } 

                    if (self.status & 0x100) == 0 {
                        info!(target: "RDP", "new RDP command list ready!");
                        self.start = self.start_latch;
                        self.status &= !0x400;

                        // set DMA BUSY and current to be the start of the commands
                        self.current = self.start;
                        self.status |= 0x100;
                        // BUSY goes to 1 until FullSync is found
                        self.status |= 0x40;
                        // note the current end in an intermediate value
                        self.intermediate_end = self.end
                    }
                } else {
                    // incremental transfer
                }
            },

            // DP_CURRENT 
            0x0010_0008 => {
                debug!(target: "RDP", "write DP_CURRENT");
            },

            // DP_STATUS 
            0x0010_000C => {
                debug!(target: "RDP", "write DP_STATUS");

                // CLR or SET SOURCE (XBUS/RDRAM)
                if (value & 0x03) != 0x03 {
                    if (value & 0x01) == 0x01 {
                        self.status &= !0x01; // use XBUS
                    } else if(value & 0x02) == 0x02 { 
                        self.status |= 0x01;  // use DBUS
                    }
                }

                // if CLR_FREEZE is set then we should have some code to run
                if (value & 0x04) == 0x04 {
                    self.status &= !0x02; // clear FREEZE bit
                }

                if (value & 0x08) == 0x08 {
                    self.status |= 0x02; // set FREEZE bit
                    // writing freeze clears pending DMAs
                    self.status &= !0x700;
                }
            },

            _ => {
                error!(target: "RDP", "invalid or unimplemented RDP write to offset=${:08X}", offset);
            },
        };

        Ok(WriteReturnSignal::None)
    }
}


