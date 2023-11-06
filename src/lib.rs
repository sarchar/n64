
pub enum MemorySegment {
    UserSpace,
    KSeg0,
    KSeg1,
    KSSeg,
    KSeg3,
}

pub trait Addressable {
    fn read_u32(&mut self, offset: usize) -> u32;
    fn write_u32(&mut self, value: u32, offset: usize) -> &mut Self;
}

pub mod pifrom;
pub mod rcp;
pub mod rdp;
pub mod rsp;
pub mod peripheral;
pub mod rdram;
pub mod mips;

