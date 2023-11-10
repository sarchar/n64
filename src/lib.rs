
pub enum MemorySegment {
    UserSpace,
    KSeg0,
    KSeg1,
    KSSeg,
    KSeg3,
}

pub struct DmaInfo {
    source_address: u32,
    dest_address: u32,
    count: u32
}

pub enum WriteReturnSignal {
    None,
    StartDMA(DmaInfo),
}

pub trait Addressable {
    fn read_u32(&mut self, offset: usize) -> u32;
    fn write_u32(&mut self, value: u32, offset: usize) -> WriteReturnSignal;

    /// not every device needs to implement these, so defaults are provided
    fn read_u16(&mut self, offset: usize) -> u16 {
        assert!((offset & 0x01) == 0);
        let word = self.read_u32(offset & !0x02);
        let shift = 16 - ((offset & 0x02) << 3);
        ((word >> shift) & 0xFFFF) as u16
    }

    fn read_u8(&mut self, offset: usize) -> u8 {
        let word = self.read_u32(offset & !0x03);
        let shift = 24 - ((offset & 0x03) << 3);
        ((word >> shift) & 0xFF) as u8
    }

    fn write_u16(&mut self, value: u16, offset: usize) -> WriteReturnSignal {
        assert!((offset & 0x01) == 0);
        let word = self.read_u32(offset & !0x02);
        let shift = 16 - ((offset & 0x03) << 3);
        self.write_u32(((value as u32) << shift) | (word & (0xFFFF0000 >> shift)), offset & !0x02)
    }

    fn write_u8(&mut self, value: u8, offset: usize) -> WriteReturnSignal {
        let word = self.read_u32(offset & !0x03);
        let shift = 24 - ((offset & 0x03) << 3);
        self.write_u32(((value as u32) << shift) | (word & !(0xFFu32 << shift)), offset & !0x03)
    }

    /// TEMP 
    fn print_debug_ipl2(&self) {}
}

pub mod cop1;
pub mod cpu;
pub mod debugger;
pub mod mips;
pub mod peripheral;
pub mod pifrom;
pub mod rcp;
pub mod rdp;
pub mod rdram;
pub mod rsp;

