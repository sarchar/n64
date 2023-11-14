use crate::*;

use tracing::{debug, error};

pub struct Rdp {
}

impl Rdp {
    pub fn new() -> Rdp {
        Rdp {}
    }
}

impl Addressable for Rdp {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        debug!(target: "RDP", "read32 offset=${:08X}", offset);
        match offset {
            // DP_STATUS 
            0x0010_000C => Ok(0),
            _ => {
                error!(target: "RDP", "invalid or unimplemented RDP read from offset=${:08X}", offset);
                Ok(0)
            },
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        debug!(target: "RDP", "write32 value=${:08X} offset=${:08X}", value, offset);
        Ok(WriteReturnSignal::None)
    }
}


