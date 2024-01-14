#![feature(stdsimd)]
#![feature(portable_simd)]

use std::cell::RefCell;
use std::fs;
use std::rc::Rc;
use std::sync::{Arc, Mutex, RwLock};
use std::sync::mpsc;
use std::sync::atomic::{AtomicU32, Ordering};

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
// Collection of thread-safe channels for the front end to communicate with the emulating system
#[derive(Clone)]
pub struct SystemCommunication {
    pub hle_command_buffer: Option<Arc<hle::HleCommandBuffer>>,

    // current render framebuffer
    pub vi_origin: Arc<AtomicU32>,
    // framebuffer width
    pub vi_width: Arc<AtomicU32>,
    // framebuffer format
    pub vi_format: Arc<AtomicU32>, // 2 = 5/5/5/3 (16bpp), 3 = 8/8/8/8 (32bpp)

    // interrupt signal
    pub mi_interrupts_tx: Option<mpsc::Sender<mips::InterruptUpdate>>,
    pub check_interrupts: Arc<AtomicU32>,

    // start DMA transfer signal
    pub start_dma_tx: Option<mpsc::Sender<rcp::DmaInfo>>,

    // direct access to RDRAM as a speed optimization (rather than going through all the RCP code)
    pub rdram: Arc<RwLock<Option<Vec<u32>>>>,
}

impl SystemCommunication {
    pub fn new(hle_command_buffer: Option<hle::HleCommandBuffer>) -> Self {
        Self {
            hle_command_buffer: hle_command_buffer.map_or(None, |v| Some(Arc::new(v))),
            vi_origin         : Arc::new(AtomicU32::new(0)),
            vi_width          : Arc::new(AtomicU32::new(0)),
            vi_format         : Arc::new(AtomicU32::new(0)),
            mi_interrupts_tx  : None,
            check_interrupts  : Arc::new(AtomicU32::new(0)),
            start_dma_tx      : None,
            rdram             : Arc::new(RwLock::new(None)),
        }
    }
}

pub struct System {
    comms: SystemCommunication,

    pub rcp: Rc<RefCell<rcp::Rcp>>,
    pub cpu: cpu::Cpu,
}

impl System {
    pub fn new(comms: SystemCommunication, boot_rom_file_name: &str, cartridge_file_name: &str) -> System {
        // load cartridge into memory
        let cartridge_rom = fs::read(cartridge_file_name).expect("Could not open cartridge ROM file");

        // load system rom. the pifrom needs to know what CIC chip the cart is using
        let boot_rom = fs::read(boot_rom_file_name).expect("Boot rom not found");

        // create the RCP and start it
        let rcp = Rc::new(RefCell::new(rcp::Rcp::new(comms.clone(), boot_rom, cartridge_rom)));
        rcp.borrow_mut().start();

        // create the CPU with reference to the bus
        let cpu = cpu::Cpu::new(rcp.clone());

        System {
            comms: comms,

            rcp: rcp,
            cpu: cpu,
        }
    }

    pub fn reset(&mut self) {
        //self.rcp.reset();
        let _ = self.cpu.reset();
    }

    #[inline(always)]
    pub fn step(&mut self, cpu_cycles: u64) -> Result<(), cpu::InstructionFault> {
        let mut i = 0;
        while i < cpu_cycles && self.comms.check_interrupts.load(Ordering::SeqCst) == 0 {
            self.cpu.step()?; // TODO should probably still call rcp.step() if this returns an error?
            i += 1;
        }

        self.comms.check_interrupts.store(0, Ordering::SeqCst);

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

    pub fn run(&mut self) {
        loop { let _ = self.step(1000); }
    }

}


