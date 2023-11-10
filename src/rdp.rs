use crate::*;

pub struct Rdp {
}

impl Rdp {
    pub fn new() -> Rdp {
        Rdp {}
    }
}

impl Addressable for Rdp {
    fn read_u32(&mut self, offset: usize) -> u32 {
        println!("RDP: read32 offset=${:08X}", offset);
        match offset {
            // DP_STATUS 
            0x0010_000C => 0,
            _ => panic!("invalid RDP read"),
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> WriteReturnSignal {
        println!("RDP: write32 value=${:08X} offset=${:08X}", value, offset);
        WriteReturnSignal::None
    }
}


