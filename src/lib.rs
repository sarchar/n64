use std::cell::RefCell;
use std::rc::Rc;

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

pub enum ReadWriteFault {
    Invalid,
    Break
}

pub trait Addressable {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault>;
    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault>;

    /// not every device needs to implement these, so defaults are provided
    fn read_u16(&mut self, offset: usize) -> Result<u16, ReadWriteFault> {
        assert!((offset & 0x01) == 0);
        let word = self.read_u32(offset & !0x02)?;
        let shift = 16 - ((offset & 0x02) << 3);
        Ok(((word >> shift) & 0xFFFF) as u16)
    }

    fn read_u8(&mut self, offset: usize) -> Result<u8, ReadWriteFault> {
        let word = self.read_u32(offset & !0x03)?;
        let shift = 24 - ((offset & 0x03) << 3);
        Ok(((word >> shift) & 0xFF) as u8)
    }

    fn write_u16(&mut self, value: u16, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        assert!((offset & 0x01) == 0);
        let word = self.read_u32(offset & !0x02)?;
        let shift = 16 - ((offset & 0x03) << 3);
        self.write_u32(((value as u32) << shift) | (word & (0xFFFF0000 >> shift)), offset & !0x02)
    }

    fn write_u8(&mut self, value: u8, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        let word = self.read_u32(offset & !0x03)?;
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
pub mod video;

pub struct System {
    pub rcp: Rc<RefCell<rcp::Rcp>>,
    pub cpu: cpu::Cpu,
}

impl System {
    pub fn new(boot_rom_file_name: &str, cartridge_file_name: &str) -> System {
        // load system rom
        let pif = pifrom::PifRom::new(boot_rom_file_name);

        // load cartridge
        let pi = peripheral::PeripheralInterface::new(cartridge_file_name);

        // create the RCP
        let rcp = Rc::new(RefCell::new(rcp::Rcp::new(pif, pi)));

        // create the CPU with reference to the bus
        let cpu = cpu::Cpu::new(rcp.clone());

        System {
            rcp: rcp,
            cpu: cpu,
        }
    }

    pub fn reset(&mut self) {
        //self.rcp.reset();
        let _ = self.cpu.reset();
    }

    pub fn run(&mut self) {
        loop { let _ = self.step(); }
    }

    pub fn step(&mut self) -> Result<(), cpu::InstructionFault> {
        self.cpu.step()
    }
}

