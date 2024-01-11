#[allow(unused_imports)]
use tracing::{debug, error, info, trace, warn};

use crate::*;
//use mips::{InterruptUpdate, InterruptUpdateMode, IMask_DP};

pub struct Rdp {
    _comms: SystemCommunication,

    start: u32,
    start_latch: u32,
    end: u32,
    status: u32,
}

impl Rdp {
    pub fn new(comms: SystemCommunication) -> Rdp {
        Rdp {
            _comms: comms,

            start: 0,
            start_latch: 0,
            end: 0,
            status: 0,
        }
    }
}

impl Addressable for Rdp {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RDP", "read32 offset=${:08X}", offset);

        match offset {
            // DP_START 
            0x0010_0000 => {
                debug!(target: "RDP", "read DP_START");
                Ok(0)
            },

            // DP_END 
            0x0010_0004 => {
                debug!(target: "RDP", "read DP_END");
                Ok(0)
            },

            // DP_CURRENT 
            0x0010_0008 => {
                debug!(target: "RDP", "read DP_CURRENT");
                Ok(0)
            },

            // DP_STATUS 
            0x0010_000C => {
                debug!(target: "RDP", "read DP_STATUS");
                Ok(self.status)
            },

            _ => {
                error!(target: "RDP", "invalid or unimplemented RDP read from offset=${:08X}", offset);
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
                self.start_latch = value;

                // set START_PENDING
                self.status |= 0x400;
            },

            // DP_END 
            0x0010_0004 => {
                debug!(target: "RDP", "write DP_END");

                if (self.status & 0x400) != 0 { // if START_PENDING is set
                    info!(target: "RDP", "new RDP command list ready!");
                    self.start = self.start_latch;
                    self.status &= !0x400;
                } else {
                    error!(target: "RDP", "incremental RDP command list!");
                }

                self.end = value;
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
                    // set freeze bit and trigger interrupt
                    self.status |= 0x02;
                }
            },

            _ => {
                error!(target: "RDP", "invalid or unimplemented RDP write to offset=${:08X}", offset);
            },
        };

        Ok(WriteReturnSignal::None)
    }
}


