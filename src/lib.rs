#![feature(stdsimd)]
#![feature(portable_simd)]

use std::cell::RefCell;
use std::fs;
use std::rc::Rc;
use std::sync::{Mutex, Arc};

pub mod avx512f_wrapper;
pub mod cop1;
pub mod cpu;
pub mod debugger;
pub mod hle;
pub mod mips;
pub mod peripheral;
pub mod pifrom;
pub mod rcp;
pub mod rdp;
pub mod rdram;
pub mod rsp;
pub mod serial;
pub mod video;

pub enum WriteReturnSignal {
    None
}

#[derive(Debug)]
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
        let shift = 16 - ((offset & 0x03) << 3);
        Ok(((word >> shift) & 0xFFFF) as u16)
    }

    fn read_u8(&mut self, offset: usize) -> Result<u8, ReadWriteFault> {
        let word = self.read_u32(offset & !0x03)?;
        let shift = 24 - ((offset & 0x03) << 3);
        Ok(((word >> shift) & 0xFF) as u8)
    }

    // The VR4300 has pins selecting the size of the write (byte, halfword, word) but still places
    // the full register on the data bus. So these functions still take u32 as the value. Some
    // modules depend on incorrect behavior that needs the full register
    fn write_u16(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        assert!((offset & 0x01) == 0);
        let word = self.read_u32(offset & !0x02)?;
        let shift = 16 - ((offset & 0x03) << 3);
        self.write_u32(((value & 0xFFFF) << shift) | (word & (0xFFFF0000 >> shift)), offset & !0x02)
    }

    fn write_u8(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        let word = self.read_u32(offset & !0x03)?;
        let shift = 24 - ((offset & 0x03) << 3);
        self.write_u32(((value & 0xFF) << shift) | (word & !(0xFFu32 << shift)), offset & !0x03)
    }

    // block read/write functions. takes/returns a slice of block data to/from a given offset
    fn read_block(&mut self, offset: usize, _length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        todo!("read_block not implemented for address offset ${offset:08X}");
    }

    fn write_block(&mut self, offset: usize, _block: &[u32]) -> Result<WriteReturnSignal, ReadWriteFault> {
        todo!("write_block not implemented for address offset ${offset:08X}");
    }
}

pub struct LockedAddressable<T> {
    addressable: Arc<Mutex<T>>,
}

impl<T> LockedAddressable<T> {
    fn new(v: Arc<Mutex<T>>) -> Self {
        Self {
            addressable: v,
        }
    }
}

impl<T: Addressable> Addressable for LockedAddressable<T> {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        self.addressable.lock().unwrap().read_u32(offset)
    }

    /// not every device needs to implement these, so defaults are provided
    fn read_u16(&mut self, offset: usize) -> Result<u16, ReadWriteFault> {
        self.addressable.lock().unwrap().read_u16(offset)
    }

    fn read_u8(&mut self, offset: usize) -> Result<u8, ReadWriteFault> {
        self.addressable.lock().unwrap().read_u8(offset)
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.addressable.lock().unwrap().write_u32(value, offset)
    }

    fn write_u16(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.addressable.lock().unwrap().write_u16(value, offset)
    }

    fn write_u8(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.addressable.lock().unwrap().write_u8(value, offset)
    }

    fn read_block(&mut self, offset: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        self.addressable.lock().unwrap().read_block(offset, length)
    }

    fn write_block(&mut self, offset: usize, block: &[u32]) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.addressable.lock().unwrap().write_block(offset, block)
    }

}

pub struct System {
    pub rcp: Rc<RefCell<rcp::Rcp>>,
    pub cpu: cpu::Cpu,
}

impl System {
    pub fn new(boot_rom_file_name: &str, cartridge_file_name: &str) -> System {
        // load cartridge into memory
        let cartridge_rom = fs::read(cartridge_file_name).expect("Could not open cartridge ROM file");

        // load system rom. the pifrom needs to know what CIC chip the cart is using
        let boot_rom = fs::read(boot_rom_file_name).expect("Boot rom not found");

        // create the RCP and start it
        let rcp = Rc::new(RefCell::new(rcp::Rcp::new(boot_rom, cartridge_rom)));
        rcp.borrow_mut().start();

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
        loop { let _ = self.step(1000); }
    }

    pub fn step(&mut self, cpu_cycles: u64) -> Result<(), cpu::InstructionFault> {
        let mut i = 0;
        while i < cpu_cycles {
            self.cpu.step()?; // TODO should probably still call rcp.step() if this returns an error?
            i += 1;
        }

        let trigger_int = { // scope rcp borrow_mut()
            let mut rcp = self.rcp.borrow_mut();
            rcp.step(cpu_cycles);
            rcp.should_interrupt()
        };

        if trigger_int != 0 {
            let _ = self.cpu.rcp_interrupt();
        }

        Ok(())
    }
}

