#![feature(stdsimd)]
#![feature(portable_simd)]

use std::cell::RefCell;
use std::collections::HashSet;
use std::fs;
use std::path::PathBuf;
use std::pin::Pin;
use std::rc::Rc;
use std::sync::{Arc, Mutex, RwLock};
use std::sync::mpsc;
use std::sync::atomic::{AtomicU32, AtomicBool, Ordering};

use atomic_counter::{AtomicCounter, RelaxedCounter};
use directories_next::ProjectDirs;

pub mod audio;
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

pub const APP_QUALIFIER   : &'static str = "org";
pub const APP_ORGANIZATION: &'static str = "sarcharsoftware";
pub const APP_NAME        : &'static str = "Sarchars n64 Emulator";

pub enum WriteReturnSignal {
    InvalidateBlockCache { physical_address: u64, length: usize },
    None
}

#[derive(Debug)]
pub enum ReadWriteFault {
    Invalid,
    Break
}

pub trait Addressable {
    fn read_u64(&mut self, offset: usize) -> Result<u64, ReadWriteFault> {
        Ok(((self.read_u32(offset)? as u64) << 32) | self.read_u32(offset + 4)? as u64)
    }

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

    fn write_u64(&mut self, value: u64, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.write_u32((value >> 32) as u32, offset)?;
        self.write_u32(value as u32, offset + 4)
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

    // need a length because we may write less than a multiple of 4
    fn write_block(&mut self, offset: usize, _block: &[u32], _length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
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

    fn lock(&mut self) -> std::sync::LockResult<std::sync::MutexGuard<'_, T>> {
        self.addressable.lock()
    }
}

impl<T: Addressable> Addressable for LockedAddressable<T> {
    fn read_u64(&mut self, offset: usize) -> Result<u64, ReadWriteFault> {
        self.addressable.lock().unwrap().read_u64(offset)
    }

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

    fn write_u64(&mut self, value: u64, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.addressable.lock().unwrap().write_u64(value, offset)
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

    fn write_block(&mut self, offset: usize, block: &[u32], length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.addressable.lock().unwrap().write_block(offset, block, length)
    }
}

// Tweakables -- things developers and nerds might want to play with
#[derive(Debug, Clone, Copy, Default)]
pub struct Tweakables {
    pub disable_textures: bool,
    pub disable_lighting: bool,
    pub disable_fog     : bool,
    //shift fog by viewport
    //pub decal_shift     : f32,
    //pub decal_times_w   : bool,
}

// Settings -- normal things people may want to configure (like antialiasing, audio playback rate, etc.)
#[derive(Debug, Clone, Copy, Default)]
pub struct Settings {
    pub cpu_interpreter_only: bool,
}

// Collection of thread-safe channels for the front end to communicate with the emulating system
#[derive(Clone)]
pub struct SystemCommunication {
    pub hle_command_buffer: Option<Arc<hle::HleCommandBuffer>>,

    // reset signal
    pub reset_signal: Arc<AtomicU32>,
    
    // total cpu cycle count
    pub total_cpu_steps: Arc<RelaxedCounter>,
    pub prefetch_counter: Arc<RelaxedCounter>,

    // current render framebuffer
    pub vi_origin: Arc<AtomicU32>,
    // framebuffer width
    pub vi_width: Arc<AtomicU32>,
    // framebuffer format
    pub vi_format: Arc<AtomicU32>, // 2 = 5/5/5/3 (16bpp), 3 = 8/8/8/8 (32bpp)

    // interrupt signal
    pub mi_interrupts_tx: Option<mpsc::Sender<mips::InterruptUpdate>>,
    pub break_cpu_cycles: Pin<Arc<AtomicBool>>,

    // full sync
    pub rdp_full_sync: Arc<AtomicU32>,

    // start DMA transfer signal
    pub start_dma_tx: Option<mpsc::Sender<rcp::DmaInfo>>,

    // direct access to RDRAM as a speed optimization (rather than going through all the RCP code)
    pub rdram: Arc<RwLock<Option<Vec<u32>>>>,

    // current controller states
    pub controllers: Arc<RwLock<Vec<ControllerState>>>,

    // emulation flags that change the way emulation behaves
    pub settings: Arc<RwLock<Settings>>,

    // disable CPU speed throttling
    // 0 - disable throttling
    // 1 - enable throttling
    // 2 - disable for a single frame
    pub cpu_throttle: Arc<AtomicU32>, // atomic enum would be nicer

    // communication with the debugger
    // TODO don't like this RwLock here
    pub debugger: Arc<RwLock<Option<crossbeam::channel::Sender<debugger::DebuggerCommand>>>>,
    pub debugger_windows: Arc<AtomicU32>,

    // tweakables -- fun for geeks
    pub tweakables: Arc<RwLock<Tweakables>>,
}

impl SystemCommunication {
    pub fn new(hle_command_buffer: Option<hle::HleCommandBuffer>) -> Self {
        Self {
            hle_command_buffer: hle_command_buffer.map_or(None, |v| Some(Arc::new(v))),
            reset_signal      : Arc::new(AtomicU32::new(0)),
            total_cpu_steps   : Arc::new(RelaxedCounter::new(0)),
            prefetch_counter  : Arc::new(RelaxedCounter::new(0)),
            vi_origin         : Arc::new(AtomicU32::new(0)),
            vi_width          : Arc::new(AtomicU32::new(0)),
            vi_format         : Arc::new(AtomicU32::new(0)),
            mi_interrupts_tx  : None,
            break_cpu_cycles  : Arc::pin(AtomicBool::new(false)),
            rdp_full_sync     : Arc::new(AtomicU32::new(0)),
            start_dma_tx      : None,
            rdram             : Arc::new(RwLock::new(None)),
            controllers       : Arc::new(RwLock::new(vec![ControllerState::default(); 4])),
            settings          : Arc::new(RwLock::new(Settings::default())),
            cpu_throttle      : Arc::new(AtomicU32::new(1)),
            debugger          : Arc::new(RwLock::new(None)),
            debugger_windows  : Arc::new(AtomicU32::new(0)),
            tweakables        : Arc::new(RwLock::new(Tweakables::default())),
        }
    }

    pub fn break_cpu(&mut self) {
        self.break_cpu_cycles.store(true, Ordering::Relaxed);
    }

    pub fn increment_prefetch_counter(&mut self) {
        self.prefetch_counter.inc();
    }

    pub fn get_prefetch_counter(&mut self) -> usize {
        self.prefetch_counter.get()
    }

    pub fn reset_prefetch_counter(&mut self) -> usize {
        self.prefetch_counter.reset()
    }

    pub fn increment_debugger_windows(&mut self) -> u32 {
        loop {
            let value = self.debugger_windows.load(Ordering::Relaxed);
            if self.debugger_windows.compare_exchange(value, value + 1, Ordering::Relaxed, Ordering::Relaxed).is_ok_and(|v| v == value) {
                return value;
            }
        }
    }

    pub fn decrement_debugger_windows(&mut self) -> u32 {
        loop {
            let value = self.debugger_windows.load(Ordering::Relaxed);
            if self.debugger_windows.compare_exchange(value, value.saturating_sub(1), Ordering::Relaxed, Ordering::Relaxed).is_ok_and(|v| v == value) {
                return value;
            }
        }
    }
}

pub struct System {
    comms: SystemCommunication,

    pub rcp: Rc<RefCell<rcp::Rcp>>,
    pub cpu: RefCell<cpu::Cpu>,

    start_time: std::time::Instant,
    last_cpu_steps: usize,
}

impl System {
    pub fn new(comms: SystemCommunication, boot_rom_file_name: &str, cartridge_file_name: &str) -> System {
        // load cartridge into memory
        let cartridge_rom = fs::read(cartridge_file_name).expect("Could not open cartridge ROM file");

        // load system rom. the pifrom needs to know what CIC chip the cart is using
        let boot_rom = fs::read(boot_rom_file_name).expect("Boot rom not found");

        // create the RCP and start it
        let rcp = Rc::new(RefCell::new(rcp::Rcp::new(comms.clone(), boot_rom, PathBuf::from(cartridge_file_name).file_name().unwrap(), cartridge_rom)));
        rcp.borrow_mut().start();

        // create the CPU with reference to the bus
        let cpu = RefCell::new(cpu::Cpu::new(comms.clone(), rcp.clone()));

        System {
            comms, rcp, cpu,
            start_time: std::time::Instant::now(),
            last_cpu_steps: 0,
        }
    }

    pub fn reset(&mut self) {
        self.rcp.borrow_mut().stop(); // stop the RCP

        // reset everything
        let _ = self.cpu.borrow_mut().reset(false);
        self.rcp.borrow_mut().reset();

        // restart the RCP
        self.rcp.borrow_mut().start();
    }

    pub fn soft_reset(&mut self) {
        let _ = self.cpu.borrow_mut().reset(true);
    }

    #[inline(always)]
    pub fn run_for(&mut self, cpu_cycles: u64, execution_breakpoints: &HashSet<u64>) -> Result<u64, cpu::InstructionFault> {
        let mut cycles_ran = 0;

        let run_result = if self.comms.settings.read().unwrap().cpu_interpreter_only {
            let mut cpu = self.cpu.borrow_mut();
            let mut result = Ok(());

            // to avoid the `if` check after every single cycle, the cpu loop is duplicated in the following if arms
            // TODO: do something better
            if execution_breakpoints.len() > 0 {
                while cycles_ran < cpu_cycles && !self.comms.break_cpu_cycles.load(Ordering::Relaxed) {
                    match cpu.step_interpreter() {
                        Ok(_) => {},
                        err @ Err(_) => {
                            result = err;
                            break;
                        },
                    }

                    cycles_ran += 1;
                    self.comms.total_cpu_steps.inc();

                    // this if check is the only difference between this cycle loop and the one below
                    if execution_breakpoints.contains(&cpu.next_instruction_pc()) {
                        result = Err(cpu::InstructionFault::Break);
                        break;
                    }
                }
            } else {
                while cycles_ran < cpu_cycles && !self.comms.break_cpu_cycles.load(Ordering::Relaxed) {
                    match cpu.step_interpreter() {
                        Ok(_) => {},
                        err @ Err(_) => {
                            result = err;
                            break;
                        },
                    }

                    cycles_ran += 1;
                    self.comms.total_cpu_steps.inc();
                }
            }

            result
        } else {
            // delay...
            match self.comms.cpu_throttle.load(Ordering::Relaxed) {
                i @ 0 | i @ 2 => {
                    self.last_cpu_steps = self.comms.total_cpu_steps.get();
                    self.start_time = std::time::Instant::now();
                    if i == 2 {
                        self.comms.cpu_throttle.store(1, Ordering::Relaxed);
                    }
                },

                1 => {
                    let mut cur_ips = f64::MAX;
                    while cur_ips > 93_750_000.0 {
                        cur_ips = ((self.comms.total_cpu_steps.get() - self.last_cpu_steps) as f64) / self.start_time.elapsed().as_secs_f64();
                    }
                },

                _ => {},
            }

            let mut result = Ok(());
            if !self.comms.break_cpu_cycles.load(Ordering::Relaxed) {
                let mut cpu = self.cpu.borrow_mut();
                let start_steps = cpu.num_steps();
                result = cpu.run_for(cpu_cycles, execution_breakpoints);
                // accumulate total cycles ran
                cycles_ran = cpu.num_steps() - start_steps;
                self.comms.total_cpu_steps.add(cycles_ran as usize);
            }

            result
        }.map(|_| 0); // convert Result<(), InstructionFault> to Result<u64, InstructionFault>

        // set here, since rcp.step() could re-set it, e.g., another dma needs to happen
        self.comms.break_cpu_cycles.store(false, Ordering::Relaxed);

        // handle reset
        match self.comms.reset_signal.load(Ordering::SeqCst) {
            1 => {
                self.comms.reset_signal.store(0, Ordering::SeqCst);
                self.reset();
                return Ok(0);
            },
            2 => {
                self.comms.reset_signal.store(0, Ordering::SeqCst);
                self.soft_reset();
                return Ok(0);
            },
            _ => {},
        };

        let (trigger_int, next_cycle_count) = { // scope rcp borrow_mut()
            let mut rcp = self.rcp.borrow_mut();
            rcp.step(cycles_ran, &mut *self.cpu.borrow_mut());
            (rcp.should_interrupt(), rcp.calculate_free_cycles())
        };

        if trigger_int != 0 {
            let _ = self.cpu.borrow_mut().rcp_interrupt();
        }

        if run_result.is_err() {
            return run_result;
        }

        Ok(next_cycle_count)
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct ButtonState {
    pub held    : bool,
    pub pressed : bool,
    pub released: bool,
}

impl ButtonState {
    fn is_down(&self) -> bool {
        self.pressed || self.held
    }
}

#[derive(Default, Debug, Copy, Clone)]
pub struct ControllerState {
    pub b        : ButtonState,
    pub a        : ButtonState,
    pub z        : ButtonState,
    pub start    : ButtonState,
    pub d_up     : ButtonState,
    pub d_down   : ButtonState,
    pub d_left   : ButtonState,
    pub d_right  : ButtonState,
    pub c_up     : ButtonState,
    pub c_down   : ButtonState,
    pub c_left   : ButtonState,
    pub c_right  : ButtonState,
    pub l_trigger: ButtonState,
    pub r_trigger: ButtonState,
    pub x_axis   : f32,
    pub y_axis   : f32,
}

pub fn get_savedata_dir() -> std::path::PathBuf {
    let project_dir = ProjectDirs::from(APP_QUALIFIER, APP_ORGANIZATION, APP_NAME).expect("could not get project directory");
    let savedata_dir = project_dir.data_dir().join("savedata");
    fs::create_dir_all(&savedata_dir).expect("could not create savedata directory");
    savedata_dir.into()
}

pub fn get_config_dir() -> std::path::PathBuf {
    let project_dirs = ProjectDirs::from(APP_QUALIFIER, APP_ORGANIZATION, APP_NAME).expect("could not get project directory");
    let config_dir = project_dirs.config_dir();
    fs::create_dir_all(&config_dir).expect("could not create config directory");
    config_dir.into()
}

