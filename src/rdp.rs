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

    fn write_u32(&mut self, _value: u32, _offset: usize) -> WriteReturnSignal {
        panic!("RDP: write32");
    }
}


