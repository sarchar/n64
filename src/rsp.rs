#![allow(non_upper_case_globals)]
use std::mem;
use std::sync::{mpsc, Arc, Mutex, RwLock};
use std::thread;

#[cfg(all(target_arch = "x86_64", target_feature = "sse2"))]
use core::arch::x86_64::*;

#[allow(unused_imports)]
use tracing::{trace, debug, error, info, warn};

use crate::*;
use cpu::{InstructionDecode, InstructionFault};

use rcp::DmaInfo;

/// COP0 "registers"
const Cop0_DmaCache      : usize = 0; // IMEM or DMEM address for a DMA transfer
const Cop0_DmaDram       : usize = 1; // RDRAM address for a DMA
const Cop0_DmaReadLength : usize = 2;
const Cop0_DmaWriteLength: usize = 3;
const Cop0_Status        : usize = 4;
const Cop0_DmaFull       : usize = 5;
const Cop0_DmaBusy       : usize = 6;
const Cop0_Semaphore     : usize = 7;
const _Cop0_CmdStart      : usize = 8;
const _Cop0_CmdEnd        : usize = 9;
const _Cop0_CmdCurrent    : usize = 10;
const _Cop0_CmdStatus     : usize = 11;
const _Cop0_CmdClock      : usize = 12;
const _Cop0_CmdBusy       : usize = 13;
const _Cop0_CmdPipeBusy   : usize = 14;
const _Cop0_CmdTMemBusy   : usize = 15;

/// N64 Reality Signal Processor
/// Resides on the die of the RCP.
pub struct Rsp {
    should_interrupt: bool,

    core: Arc<Mutex<RspCpuCore>>,
    shared_state: Arc<RwLock<RspSharedState>>,

    wakeup_tx: Option<mpsc::Sender<()>>,
    broke_rx: Option<mpsc::Receiver<bool>>,

    // access to RspCpuCore memory
    mem: Arc<RwLock<Vec<u32>>>,

    start_dma_tx: mpsc::Sender<DmaInfo>,
    dma_completed_rx: mpsc::Receiver<DmaInfo>,

    // these are copies of state in RspCpuCore so we don't need a lock
    halted: bool,
    broke: bool,
}

#[derive(Debug, Default)]
pub struct RspSharedState {
    intbreak: bool,

    dma_cache: u32,
    dma_dram: u32,
    dma_read_length: u32,
    dma_write_length: u32,
    dma_busy: bool,
    dma_full: Option<DmaInfo>,
    semaphore: bool,

    // current or most recent DMA, and the data that is readable in the DMA registers
    dma_current_cache: u32,
    dma_current_dram: u32,
    dma_current_read_length: u32,
    dma_current_write_length: u32,

    // signals
    signals: u32,
}

#[derive(Debug)]
struct RspCpuCore {
    // registers
    pc: u32,
    current_instruction_pc: u32, // actual PC of the currently executing instruction
                                 // only valid inside step()
    next_instruction: u32,       // emulates delay slot (prefetch, next instruction)
    next_instruction_pc: u32,    // for printing correct delay slot addresses
    is_delay_slot: bool,         // true if the currently executing instruction is in a delay slot
    next_is_delay_slot: bool,    // set to true on branching instructions

    gpr: [u32; 32],

    v: [__m128i; 32],            // 32 128-bit VU registers
    rotate_base: __m128i,

    inst: InstructionDecode,

    instruction_table: [CpuInstruction; 64],
    special_table: [CpuInstruction; 64],
    regimm_table: [CpuInstruction; 32],
    cop2_table: [CpuInstruction; 64],

    // multiple reader single writer memory
    mem: Arc<RwLock<Vec<u32>>>,

    shared_state: Arc<RwLock<RspSharedState>>,

    broke_tx: Option<mpsc::Sender<bool>>,
    start_dma_tx: mpsc::Sender<DmaInfo>,
    dma_completed_tx: mpsc::Sender<DmaInfo>,

    // running state
    halted: bool,
    broke: bool,
}

type CpuInstruction = fn(&mut RspCpuCore) -> Result<(), InstructionFault>;

impl Rsp {
    pub fn new(start_dma_tx: mpsc::Sender<DmaInfo>) -> Rsp {
        let mem = Arc::new(RwLock::new(vec![0u32; 2*1024]));

        let shared_state = Arc::new(RwLock::new(RspSharedState::default()));
    
        // dma_completed channel is created here and sent with every DmaInfo message
        let (dma_completed_tx, dma_completed_rx) = mpsc::channel();

        let core = Arc::new(Mutex::new(RspCpuCore::new(mem.clone(), shared_state.clone(), start_dma_tx.clone(), dma_completed_tx)));

        Rsp {
            should_interrupt: false,

            core: core,

            mem: mem,

            wakeup_tx: None,
            broke_rx: None,

            start_dma_tx: start_dma_tx,
            dma_completed_rx: dma_completed_rx,

            shared_state: shared_state,

            halted: true,
            broke: false,
        }
    }

    pub fn start(&mut self) {
        // create the awake channel
        let (wakeup_tx, wakeup_rx) = mpsc::channel();
        self.wakeup_tx = Some(wakeup_tx);

        // create the broke signal channel
        let (broke_tx, broke_rx) = mpsc::channel();
        self.broke_rx = Some(broke_rx);
    
        let core = Arc::clone(&self.core);
        {
            let mut c = core.lock().unwrap();
            c.broke_tx = Some(broke_tx);
        }

        thread::spawn(move || {
            loop {
                let mut c = core.lock().unwrap();

                // halted can only be set while we don't have a lock, so check here
                if c.halted {
                    //.info!(target: "RSP", "RSP core halted (broke={}), waiting for wakeup", c.broke);

                    // drop the lock on the core and wait for a wakeup signal
                    drop(c);

                    // wait forever for a signal
                    let _ = wakeup_rx.recv().unwrap();

                    // running!
                    let mut c = core.lock().unwrap();
                    c.halted = false;
                    c.broke = false;
                } else {
                    // run for some cycles or until break
                    for _ in 0..100 {
                        let _ = c.step(); // TODO handle errors

                        if c.broke { 
                            c.halted = true; // signal the core is halted before dropping the lock
                            break; 
                        }
                    }
                }
            }
        });
    }

    pub fn step(&mut self) {
        if let Ok(_) = self.dma_completed_rx.try_recv() {
            // we got a completed DMA, so if one is pending start it immediately
            let mut shared_state = self.shared_state.write().unwrap();
            shared_state.dma_busy = false;
            if let Some(dma_info) = mem::replace(&mut shared_state.dma_full, None) {
                shared_state.dma_current_cache = shared_state.dma_cache;
                shared_state.dma_current_dram = shared_state.dma_dram;
                shared_state.dma_current_read_length = shared_state.dma_read_length;
                shared_state.dma_current_write_length = shared_state.dma_write_length;
                shared_state.dma_busy = true;
                self.start_dma_tx.send(dma_info).unwrap(); 
            }
        }
    }

    pub fn is_broke(&mut self) -> bool {
        // check if BREAK happened
        if let Some(broke_rx) = &self.broke_rx {
            if let Ok(set) = broke_rx.try_recv() {
                self.broke = false;
                if set {
                    self.broke = true;
                    self.halted = true;
                    let shared_state = self.shared_state.read().unwrap();
                    if shared_state.intbreak {
                        self.should_interrupt = true;
                    }
                }
            }
        }
        self.broke
    }

    pub fn should_interrupt(&mut self) -> bool {
        let _ = self.is_broke();
        let r = self.should_interrupt;
        self.should_interrupt = false;
        r
    }
    
    fn read_register(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RSP", "read32 register offset=${:08X}", offset);

        let value = match offset {
            // SP_STATUS
            0x4_0010 => {
                //debug!(target: "RSP", "read SP_STATUS");

                // update self.broke
                let _ = self.is_broke();

                let shared_state = self.shared_state.read().unwrap();

                ((self.halted as u32) << 0)
                    | ((self.broke as u32) << 1)
                    | ((shared_state.dma_busy as u32) << 2)
                    | ((shared_state.dma_full.is_some() as u32) << 3)
                    | ((shared_state.intbreak as u32) << 6)
                    | (shared_state.signals << 7)
            },

            // SP_DMA_BUSY
            0x4_0018 => {
                //debug!(target: "RSP", "read SP_DMA_BUSY");

                let shared_state = self.shared_state.read().unwrap();
                shared_state.dma_busy as u32
            },

            // SP_SEMAPHORE
            0x4_001C => {
                let mut shared_state = self.shared_state.write().unwrap();
                if !shared_state.semaphore {
                    shared_state.semaphore = true;
                    0
                } else {
                    1
                }
            },

            // SP_PC
            0x8_0000 => {
                //debug!(target: "RSP", "read SP_PC");

                let core = self.core.lock().unwrap();
                core.next_instruction_pc
            },

            _ => {
                error!(target: "RSP", "unknown register read offset=${:08X}", offset);

                0
            },
        };

        Ok(value)
    }

    fn write_register(&mut self, mut value: u32, offset: usize) {
        match offset {
            // SP_STATUS 
            0x4_0010 => {
                //debug!(target: "RSP", "write SP_STATUS value=${:08X}", value);

                // CLR_BROKE: clear the broke flag
                if (value & 0x04) != 0 {
                    self.broke = false; // set local copy

                    if let Some(broke_rx) = &self.broke_rx {
                        // clear the broke_rx channel (which should never have more than 1 in it
                        // because a break halts the cpu). I imagine if you set set CLR_BROKE
                        // while the CPU is running (you wrote CLR_HALT previously), this might
                        // cause a break signal to get lost. hopefully that's not a problem.
                        loop {
                            if let Err(_) = broke_rx.try_recv() { 
                                break;
                            }
                        }
                    }
                }

                // CLR_HALT: clear halt flag and start RSP
                if (value & 0x01) != 0 { 
                    // sum up the first 44 bytes of imem
                    let mut sum: u32 = 0;
                    for i in 0..44 {
                        if let Ok(v) = self.read_u8(0x1000 + i) {
                            sum = sum.wrapping_add(v as u32);
                        }
                    }

                    //.info!(target: "RSP", "sum of IMEM at start: ${:08X} (if this value is 0x9E2, it is a bootcode program)", sum);
                    //.{
                    //.    let core = self.core.lock().unwrap();
                    //.    info!(target: "RSP", "starting RSP at PC=${:08X}", core.next_instruction_pc);
                    //.}

                    // set local copy
                    self.halted = false;

                    // send wakeup signal
                    if let Some(wakeup_tx) = &self.wakeup_tx {
                        wakeup_tx.send(()).unwrap();
                    }
                }

                // SET_HALT: halt the RSP
                if (value & 0x02) != 0 {
                    // TODO it might be worth converting this to a mpsc channel as well,
                    // depending on if anything uses SET_HALT often enough to causes stalls
                    info!(target: "RSP", "CPU has halted the RSP");
                    let mut c = self.core.lock().unwrap();
                    c.halted = true;
                    self.halted = true;
                }

                // SET_INTBREAK: enable the interrupt on BREAK signal
                if (value & 0x100) != 0 {
                    let mut shared_state = self.shared_state.write().unwrap();
                    shared_state.intbreak = true;
                }

                // CLR_INTBREAK: disable the interrupt on break signal
                if (value & 0x80) != 0 {
                    let mut shared_state = self.shared_state.write().unwrap();
                    shared_state.intbreak = false;
                }

                // loop over the signals
                value >>= 9; // signals start at bit 9
                if value != 0 {
                    let mut shared_state = self.shared_state.write().unwrap();
                    for i in 0..16 {
                        // i loops over 8 signals each CLEAR then SET bits
                        // so signal number is i/2, and SET when i is odd, otherwise clear
                        if (value & (1 << i)) == 0 { continue; }

                        let signo = i >> 1;

                        if (i & 0x01) == 0 { // CLEAR signal
                            shared_state.signals &= !(1 << signo);
                        } else { // SET signal
                            shared_state.signals |= 1 << signo;
                        }
                    }
                }
            },

            // SP_SEMAPHORE
            0x4_001C => {
                if value == 0 {
                    let mut shared_state = self.shared_state.write().unwrap();
                    shared_state.semaphore = false;
                }
            },

            // SP_PC
            0x8_0000 => {
                //debug!(target: "RSP", "write SP_PC value=${:08X}", value);

                let mut core = self.core.lock().unwrap();
                core.pc = value & 0x0FFC;
                let _ = core.prefetch(); // ignore errors here, since core.pc should never be invalid
            },

            _ => {
                warn!(target: "RSP", "unknown write32 register value=${:08X} offset=${:08X}", value, offset);
            }
        };
    }
}

impl Addressable for Rsp {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RSP", "read32 offset=${:08X}", offset);

        match offset & 0x000F_0000 {
            0x0000_0000..=0x0003_FFFF => {
                let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
                let mem = self.mem.read().unwrap();
                Ok(mem[mem_offset as usize])
            },

            0x0004_0000..=0x000B_FFFF => self.read_register(offset & 0x000F_FFFF),

            _ => panic!("invalid RSP read"),
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "RSP", "write32 value=${:08X} offset=${:08X}", value, offset);

        match offset & 0x000F_0000 {
            0x0000_0000..=0x0003_FFFF => {
                let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
                if mem_offset < (0x1000>>2) {
                    //const TASK_TYPE: usize = 0xFC0 >> 2;
                    //const DL_START: usize = 0xFF0 >> 2;
                    //const DL_LEN: usize = 0xFF4 >> 2;
                    //.match mem_offset {
                    //.    DL_START => {
                    //.        info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (DL start)", value, offset);
                    //.    },
                    //.    DL_LEN => {
                    //.        info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (DL length)", value, offset);
                    //.    },
                    //.    TASK_TYPE => {
                    //.        info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (TaskType)", value, offset);
                    //.    },
                    //.    _ => {},
                    //.}
                } else {
                    //info!(target: "RSPMEM", "write value=${:08X} offset=${:08X}", value, offset);
                }

                let mut mem = self.mem.write().unwrap();
                mem[mem_offset as usize] = value;
            },

            0x0004_0000..=0x000B_FFFF => self.write_register(value, offset & 0x000F_FFFF),

            _ => panic!("invalid RSP write"),
        };

        Ok(WriteReturnSignal::None)
    }

    fn write_block(&mut self, offset: usize, block: &[u32]) -> Result<WriteReturnSignal, ReadWriteFault> {
        if offset < 0x2000 { // I/DRAM
            let mut mem = self.mem.write().unwrap();
            let (_, right) = mem.split_at_mut(offset >> 2);
            let (left, _) = right.split_at_mut(block.len());
            left.copy_from_slice(block);
            Ok(WriteReturnSignal::None)
        } else {
            todo!("DMA into ${offset:8X}");
        }
    }
}

use RspCpuCore as Cpu; // shorthand so I can copy code from cpu.rs :)
impl RspCpuCore {
    fn new(mem: Arc<RwLock<Vec<u32>>>, shared_state: Arc<RwLock<RspSharedState>>, start_dma_tx: mpsc::Sender<DmaInfo>, dma_completed_tx: mpsc::Sender<DmaInfo>) -> Self {
        let mut core = RspCpuCore {
            pc: 0,
            current_instruction_pc: 0,
            next_instruction: 0,
            next_instruction_pc: 0,
            is_delay_slot: false,
            next_is_delay_slot: false,

            gpr: [0u32; 32],

            v: [unsafe { _mm_setzero_si128() }; 32],
            rotate_base: unsafe { _mm_set_epi8(31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16) },

            inst: InstructionDecode::default(),

            mem: mem,
            shared_state: shared_state,
            broke_tx: None,
            start_dma_tx: start_dma_tx,
            dma_completed_tx: dma_completed_tx,

            halted: false,
            broke: false,

            instruction_table: [
                //  _000                _001                _010                _011                _100                _101                _110                _111
   /* 000_ */   Cpu::inst_special , Cpu::inst_regimm  , Cpu::inst_j       , Cpu::inst_jal     , Cpu::inst_beq     , Cpu::inst_bne     , Cpu::inst_blez    , Cpu::inst_bgtz    ,
   /* 001_ */   Cpu::inst_addi    , Cpu::inst_addiu   , Cpu::inst_slti    , Cpu::inst_sltiu   , Cpu::inst_andi    , Cpu::inst_ori     , Cpu::inst_xori    , Cpu::inst_lui     ,
   /* 010_ */   Cpu::inst_cop0    , Cpu::inst_reserved, Cpu::inst_cop2    , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 011_ */   Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 100_ */   Cpu::inst_lb      , Cpu::inst_lh      , Cpu::inst_reserved, Cpu::inst_lw      , Cpu::inst_lbu     , Cpu::inst_lhu     , Cpu::inst_reserved, Cpu::inst_lw      , // LWU = LW on RSP
   /* 101_ */   Cpu::inst_sb      , Cpu::inst_sh      , Cpu::inst_reserved, Cpu::inst_sw      , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 110_ */   Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_lwc2    , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_unknown , Cpu::inst_reserved,
   /* 111_ */   Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_swc2    , Cpu::inst_reserved, Cpu::inst_unknown , Cpu::inst_reserved, Cpu::inst_unknown , Cpu::inst_reserved
            ],

            special_table: [
               //   _000                _001                _010                _011                _100                _101                _110                _111
   /* 000_ */   Cpu::special_sll  , Cpu::inst_reserved, Cpu::special_srl  , Cpu::special_sra  , Cpu::special_sllv , Cpu::inst_reserved, Cpu::special_srlv , Cpu::special_srav ,
   /* 001_ */   Cpu::special_jr   , Cpu::special_jalr , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::special_break, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 010_ */   Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 011_ */   Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 100_ */   Cpu::special_add  , Cpu::special_addu , Cpu::special_sub  , Cpu::special_subu , Cpu::special_and  , Cpu::special_or   , Cpu::special_xor  , Cpu::special_nor  ,
   /* 101_ */   Cpu::inst_reserved, Cpu::inst_reserved, Cpu::special_slt  , Cpu::special_sltu , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 110_ */   Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 111_ */   Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved
            ],

            regimm_table: [
               //   _000                 _001                 _010                 _011                 _100               _101                _110                _111
   /* 00_ */    Cpu::regimm_bltz   , Cpu::regimm_bgez   , Cpu::regimm_unknown, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 01_ */    Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 10_ */    Cpu::regimm_bltzal , Cpu::regimm_bgezal , Cpu::regimm_unknown, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 11_ */    Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved
            ],

            cop2_table: [
               //   _000                _001                _010                _011                _100                _101                _110                _111
   /* 000_ */   Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown ,
   /* 001_ */   Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown ,
   /* 010_ */   Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown ,
   /* 011_ */   Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown ,
   /* 100_ */   Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown ,
   /* 101_ */   Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown ,
   /* 110_ */   Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown ,
   /* 111_ */   Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown , Cpu::cop2_unknown 
            ],
        };

        let _ = core.reset();
        core
    }

    fn reset(&mut self) -> Result<(), ReadWriteFault> {
        self.pc     = 0;
        self.halted = true;

        self.prefetch()
    }

    pub fn prefetch(&mut self) -> Result<(), ReadWriteFault> {
        self.next_instruction = self.read_u32(self.pc as usize | 0x1000)?; 
        self.next_instruction_pc = self.pc;
        self.pc = (self.pc + 4) & 0xFFC;

        Ok(())
    }

    // The RSP cpu supports unaligned reads, so we may have to do two fetches if on a word boundary
    fn read_u32_wrapped(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        if (offset & 0x03) != 0 { // any offset other than 0 means two read_32s
            let upper_bits = offset & 0xFFFF_F000;
            let shift = (offset & 0x03) << 3;
            Ok((self.read_u32(offset & !0x03)? << shift)
                | (self.read_u32((((offset & !0x03) + 4) & 0xFFF) | upper_bits)? >> (32 - shift)))
        } else {
            self.read_u32(offset)
        }
    }

    fn write_u32_wrapped(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        if (offset & 0x03) != 0 { // any offset other than 0 means two writes
            let upper_bits = offset & 0xFFFF_F000;
            let shift = (offset & 0x03) << 3;

            let word = self.read_u32(offset & !0x03)?;
            self.write_u32((word & !(0xFFFF_FFFF >> shift)) | (value >> shift), offset & !0x03)?;

            let offset = (((offset & !0x03) + 4) & 0x0FFF) | upper_bits; // recompute offset to next byte, wrapping
            let word = self.read_u32(offset)?;
            self.write_u32((word & (0xFFFF_FFFF >> shift)) | (value << (32 - shift)), offset)
        } else {
            self.write_u32(value, offset)
        }
    }

    fn read_u16_wrapped(&mut self, offset: usize) -> Result<u16, ReadWriteFault> {
        if (offset & 0x03) == 3 { // only wraps to the next word with this one offset
            let upper_bits = offset & 0xFFFF_F000;
            Ok((((self.read_u32(offset & !0x03)? & 0x0000_00FF) << 8)
                | (self.read_u32(((offset + 1) & 0xFFF) | upper_bits)? >> 24)) as u16)
        } else {
            let word = self.read_u32(offset & !0x03)?;
            let shift = 16 - ((offset & 0x03) << 3);
            Ok(((word >> shift) & 0xFFFF) as u16)
        }
    }

    fn write_u16_wrapped(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        if (offset & 0x03) == 3 { // only wraps to the next word with this one offset
            let upper_bits = offset & 0xFFFF_F000;
            self.write_u8((value >> 8) & 0xFF, offset)?;
            self.write_u8(value & 0xFF, ((offset + 1) & 0x0FFF) | upper_bits)
        } else {
            let word = self.read_u32(offset & !0x03)?;
            let shift = (offset & 0x03) << 3;
            self.write_u32((word & !(0xFFFF_0000 >> shift)) | ((value & 0xFFFF) << (16 - shift)), offset & !0x03)
        }
    }

    fn step(&mut self) -> Result<(), InstructionFault> {
        self.current_instruction_pc = self.pc;

        // check for invalid PC addresses
        if (self.pc & 0x03) != 0 {
            panic!("invalid!");
            //return self.address_exception(self.pc, false);
        }

        // current instruction
        let inst = self.next_instruction;
        self.inst.v = inst;

        // instruction decode
        self.inst.op         = inst >> 26;
        self.inst.rs         = ((inst >> 21) & 0x1F) as usize;
        self.inst.rt         = ((inst >> 16) & 0x1F) as usize;
        self.inst.rd         = ((inst >> 11) & 0x1F) as usize;
        self.inst.imm        = (inst & 0xFFFF) as u64;
        self.inst.signed_imm = (self.inst.imm as i16) as u64;
        self.inst.target     = inst & 0x3FFFFFF;
        self.inst.sa         = (inst >> 6) & 0x1F;

        // next instruction prefetch
        self.next_instruction = self.read_u32(self.pc as usize | 0x1000)?;

        // update and increment PC
        self.current_instruction_pc = self.next_instruction_pc;
        self.next_instruction_pc = self.pc;
        self.pc = (self.pc + 4) & 0xFFC;

        // in delay slot flag
        self.is_delay_slot = self.next_is_delay_slot;
        self.next_is_delay_slot = false;

        //info!(target: "RSP", "${:08X}: inst ${:08X} (op=0b{:06b}", self.current_instruction_pc | 0x1000, inst, self.inst.op);

        let result = match self.instruction_table[self.inst.op as usize](self) {
            // faults like Break, Unimplemented actually stop processing
            Err(msg) => {
                // on error, restore the previous instruction since it didn't complete
                self.pc -= 4;
                self.next_instruction_pc = self.current_instruction_pc;
                self.next_instruction = inst;
                Err(msg)
            },

            // Everything else
            Ok(v) => { Ok(v) },
        };

        // r0 must always be zero
        self.gpr[0] = 0;

        result
    }

    #[inline(always)]
    fn branch(&mut self, condition: bool) {
        if condition {
            // target is the sum of the address of the delay slot instruction
            // plus the sign extended and left-shifted immediate offset
            self.pc = (self.pc - 4).wrapping_add((self.inst.signed_imm as u32) << 2) & 0xFFC;
        }

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
    }

    fn inst_reserved(&mut self) -> Result<(), InstructionFault> {
        warn!(target: "RSP", "reserved instruction ${:03b}_{:03b} at pc=${:08X}", self.inst.op >> 3, self.inst.op & 0x07, self.current_instruction_pc);
        panic!("reserved instruction");
    }

    fn inst_unknown(&mut self) -> Result<(), InstructionFault> {
        error!(target: "RSP", "unimplemented function ${:03b}_{:03b}", self.inst.op >> 3, self.inst.op & 0x07);
        Err(InstructionFault::Unimplemented)
    }

    fn regimm_unknown(&mut self) -> Result<(), InstructionFault> {
        error!(target: "RSP", "unimplemented regimm op: 0b{:02b}_{:03b}", self.inst.regimm >> 3, self.inst.regimm & 0x07);
        Err(InstructionFault::Unimplemented)
    }

    fn inst_special(&mut self) -> Result<(), InstructionFault> {
        self.inst.special = self.inst.v & 0x3F;
        self.special_table[self.inst.special as usize](self)
    }

    fn inst_regimm(&mut self) -> Result<(), InstructionFault> {
        self.inst.regimm = (self.inst.v >> 16) & 0x1F;
        self.regimm_table[self.inst.regimm as usize](self)
    }

    fn inst_j(&mut self) -> Result<(), InstructionFault> {
        self.pc = (self.inst.target << 2) & 0xFFC;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
        Ok(())
    }

    fn inst_jal(&mut self) -> Result<(), InstructionFault> {
        self.gpr[31] = self.pc;
        self.pc = (self.inst.target << 2) & 0xFFC;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
        Ok(())
    }

    fn inst_beq(&mut self) -> Result<(), InstructionFault> {
        let condition = self.gpr[self.inst.rs] == self.gpr[self.inst.rt];
        self.branch(condition);
        Ok(())
    }

    fn inst_bne(&mut self) -> Result<(), InstructionFault> {
        let condition = self.gpr[self.inst.rs] != self.gpr[self.inst.rt];
        self.branch(condition);
        Ok(())
    }

    fn inst_blez(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i32) <= 0;
        self.branch(condition);
        Ok(())
    }

    fn inst_bgtz(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i32) > 0;
        self.branch(condition);
        Ok(())
    }

    fn inst_addi(&mut self) -> Result<(), InstructionFault> {
        let rs = self.gpr[self.inst.rs] as i32;
        let rt = self.inst.signed_imm as i32;
        self.gpr[self.inst.rt] = rs.wrapping_add(rt) as u32;
        Ok(())
    }

    fn inst_addiu(&mut self) -> Result<(), InstructionFault> {
        let rs = self.gpr[self.inst.rs];
        let rt = self.inst.signed_imm as u32;
        self.gpr[self.inst.rt] = rs.wrapping_add(rt);
        Ok(())
    }

    fn inst_slti(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i32) < (self.inst.signed_imm as i32) {
            self.gpr[self.inst.rt] = 1;
        } else {
            self.gpr[self.inst.rt] = 0;
        }

        Ok(())
    }

    fn inst_sltiu(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = (self.gpr[self.inst.rs] < (self.inst.signed_imm as u32)) as u32;
        Ok(())
    }

    fn inst_andi(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] & (self.inst.imm as u32);
        Ok(())
    }

    fn inst_ori(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] | (self.inst.imm as u32);
        Ok(())
    }

    fn inst_xori(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] ^ (self.inst.imm as u32);
        Ok(())
    }

    fn inst_lui(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = (self.inst.signed_imm as u32) << 16;
        Ok(())
    }

    fn inst_lb(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.gpr[self.inst.rt] = (self.read_u8(address as usize & 0x0FFF)? as i8) as u32;
        Ok(())
    }

    fn inst_lh(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.gpr[self.inst.rt] = (self.read_u16_wrapped(address as usize & 0x0FFF)? as i16) as u32;
        Ok(())
    }

    fn inst_lw(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.gpr[self.inst.rt] = self.read_u32_wrapped(address as usize & 0x0FFF)?;
        Ok(())
    }

    fn inst_lbu(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.gpr[self.inst.rt] = self.read_u8(address as usize & 0x0FFF)? as u32;
        Ok(())
    }

    fn inst_lhu(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.gpr[self.inst.rt] = self.read_u16_wrapped(address as usize & 0x0FFF)? as u32;
        Ok(())
    }

    fn inst_sb(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.write_u8(self.gpr[self.inst.rt] as u32, address as usize & 0x0FFF)?;
        Ok(())
    }

    fn inst_sh(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.write_u16_wrapped(self.gpr[self.inst.rt], address as usize & 0x0FFF)?;
        Ok(())
    }

    fn inst_sw(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.write_u32_wrapped(self.gpr[self.inst.rt], address as usize & 0x0FFF)?;
        Ok(())
    }

    fn special_sll(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << self.inst.sa;
        Ok(())
    }

    fn special_srl(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> self.inst.sa;
        Ok(())
    }

    fn special_sra(&mut self) -> Result<(), InstructionFault> {
        // shift right with sign
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i32) >> self.inst.sa) as u32;
        Ok(())
    }

    fn special_sllv(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << (self.gpr[self.inst.rs] & 0x3F);
        Ok(())
    }

    fn special_srlv(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> (self.gpr[self.inst.rs] & 0x3F);
        Ok(())
    }

    fn special_srav(&mut self) -> Result<(), InstructionFault> {
        // shift right with sign
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i32) >> (self.gpr[self.inst.rs] & 0x3F)) as u32;
        Ok(())
    }

    fn special_jr(&mut self) -> Result<(), InstructionFault> {
        self.pc = self.gpr[self.inst.rs] & 0xFFC;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;

        Ok(())
    }

    fn special_jalr(&mut self) -> Result<(), InstructionFault> {
        let dest = self.gpr[self.inst.rs] & 0xFFC; // get dest before changing RD, as RS could be == RD
        self.gpr[self.inst.rd] = self.pc; // pc pointing to after the delay slot already
        self.pc = dest;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;

        Ok(())
    }

    fn special_break(&mut self) -> Result<(), InstructionFault> {
        self.broke = true;
                            
        // write break signal to channel
        if let Some(broke_tx) = &self.broke_tx {
            broke_tx.send(true).unwrap();
        }

        Ok(())
    }

    fn special_add(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs].wrapping_add(self.gpr[self.inst.rt]);
        Ok(())
    }

    fn special_addu(&mut self) -> Result<(), InstructionFault> {
        // since the RSP does not signal exceptions, this instruction behaves identically to ADD
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs].wrapping_add(self.gpr[self.inst.rt]);
        Ok(())
    }

    fn special_sub(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs].wrapping_sub(self.gpr[self.inst.rt]);
        Ok(())
    }

    fn special_subu(&mut self) -> Result<(), InstructionFault> {
        // identical to SUB
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs].wrapping_sub(self.gpr[self.inst.rt]);
        Ok(())
    }

    fn special_and(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] & self.gpr[self.inst.rt];
        Ok(())
    }

    fn special_or(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] | self.gpr[self.inst.rt];
        Ok(())
    }

    fn special_xor(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] ^ self.gpr[self.inst.rt];
        Ok(())
    }

    fn special_nor(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = !(self.gpr[self.inst.rs] | self.gpr[self.inst.rt]);
        Ok(())
    }

    fn special_slt(&mut self) -> Result<(), InstructionFault> {
        // set rd to 1 if rs < rt, otherwise 0
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rs] as i32) < (self.gpr[self.inst.rt] as i32)) as u32;
        Ok(())
    }

    fn special_sltu(&mut self) -> Result<(), InstructionFault> {
        // unsigned
        self.gpr[self.inst.rd] = (self.gpr[self.inst.rs] < self.gpr[self.inst.rt]) as u32;
        Ok(())
    }

    fn regimm_bltz(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i32) < 0;
        self.branch(condition);
        Ok(())
    }

    fn regimm_bltzal(&mut self) -> Result<(), InstructionFault> {
        // compute condition before changing gpr31, as rs can be 31...
        let condition = (self.gpr[self.inst.rs] as i32) < 0;
        self.gpr[31] = self.pc; // unconditionally, the address after the delay slot is stored in the link register
        self.branch(condition);
        Ok(())
    }

    fn regimm_bgez(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i32) >= 0;
        self.branch(condition);
        Ok(())
    }

    fn regimm_bgezal(&mut self) -> Result<(), InstructionFault> {
        // compute condition before changing gpr31, as rs can be 31...
        let condition = (self.gpr[self.inst.rs] as i32) >= 0;
        self.gpr[31] = self.pc; // unconditionally, the address after the delay slot is stored in the link register
        self.branch(condition);
        Ok(())
    }

    fn inst_cop0(&mut self) -> Result<(), InstructionFault> {
        let cop0_op = (self.inst.v >> 21) & 0x1F;
        match cop0_op {
            0b00_000 => { // MFC
                self.gpr[self.inst.rt] = match self.inst.rd {
                    Cop0_DmaDram => {
                        let shared_state = self.shared_state.read().unwrap();
                        shared_state.dma_current_dram
                    },

                    Cop0_DmaFull => {
                        let shared_state = self.shared_state.read().unwrap();
                        shared_state.dma_full.is_some() as u32
                    },

                    Cop0_DmaBusy => {
                        let shared_state = self.shared_state.read().unwrap();
                        shared_state.dma_busy as u32
                    },

                    Cop0_Status => {
                        let shared_state = self.shared_state.read().unwrap();

                        ((self.halted as u32) << 0) // never true if we get here
                            | ((self.broke as u32) << 1)  // might be true if broke flag isn't cleared?
                            | ((shared_state.dma_busy as u32) << 2)
                            | ((shared_state.dma_full.is_some() as u32) << 3)
                            | ((shared_state.intbreak as u32) << 6)
                            | (shared_state.signals << 7)
                    },

                    Cop0_Semaphore => {
                        let mut shared_state = self.shared_state.write().unwrap();
                        if !shared_state.semaphore {
                            shared_state.semaphore = true;
                            0
                        } else {
                            1
                        }
                    },

                    _ => todo!("unhandled cop0 register read $c{}", self.inst.rd),
                };
                Ok(())
            },

            0b00_100 => { // MTC
                let mut val = self.gpr[self.inst.rt];

                // fix bits of the various registers
                match self.inst.rd {
                    Cop0_DmaCache => {
                        info!(target: "RSP", "DMA RSP address set to ${:04X}", val & 0x1FFF);
                        let mut shared_state = self.shared_state.write().unwrap();
                        shared_state.dma_cache = val & 0x1FFF;
                        Ok(())
                    },

                    Cop0_DmaDram => {
                        info!(target: "RSP", "DMA DRAM address set to ${:04X}", val & 0x00FF_FFFF);
                        let mut shared_state = self.shared_state.write().unwrap();
                        shared_state.dma_dram = val & 0x00FF_FFFF;
                        Ok(())
                    },

                    Cop0_DmaReadLength => { // RDRAM -> I/DRAM
                        info!(target: "RSP", "DMA read length set to ${:04X}", val);
                        let mut shared_state = self.shared_state.write().unwrap();
                        shared_state.dma_read_length = val;

                        let length = ((val & 0x0FFF) | 0x07) + 1;
                        let count  = ((val >> 12) & 0xFF) + 1;
                        let skip   = (val >> 20) & 0xFFF;

                        let dma_info = DmaInfo {
                            initiator     : "RSP-READ",
                            source_address: shared_state.dma_dram,
                            dest_address  : shared_state.dma_cache | 0x0400_0000,
                            count         : count,
                            length        : length,
                            source_stride : skip,
                            dest_stride   : 0,
                            completed     : Some(self.dma_completed_tx.clone()),
                        };

                        if shared_state.dma_busy {
                            if shared_state.dma_full.is_none() {
                                shared_state.dma_full = Some(dma_info);
                            }
                        } else {
                            shared_state.dma_current_cache = shared_state.dma_cache;
                            shared_state.dma_current_dram = shared_state.dma_dram;
                            shared_state.dma_current_read_length = shared_state.dma_read_length;
                            shared_state.dma_current_write_length = shared_state.dma_write_length;
                            shared_state.dma_busy = true;
                            self.start_dma_tx.send(dma_info).unwrap();
                        }

                        Ok(())
                    },

                    Cop0_DmaWriteLength => {
                        info!(target: "RSP", "DMA write length set to ${:04X}", val);
                        let mut shared_state = self.shared_state.write().unwrap();
                        shared_state.dma_write_length = val;

                        todo!();
                        //let dma_info = DmaInfo::default();
                        //self.start_dma_tx.send(dma_info).unwrap();

                        //Ok(())
                    },

                    Cop0_Status => {
                        info!(target: "RSP", "Cop0_Status write value ${:08X}", val);

                        // CLR_BROKE
                        if (val & 0x04) != 0 {
                            self.broke = false;
                            if let Some(broke_tx) = &self.broke_tx {
                                broke_tx.send(false).unwrap();
                            }

                            val &= !0x04;
                        }

                        if (val & 0x1FF) != 0 { // TODO
                            todo!("wrote status bits ${val:08X}");
                        }

                        // loop over the signals
                        val >>= 9; // signals start at bit 9
                        if val != 0 {
                            let mut shared_state = self.shared_state.write().unwrap();
                            for i in 0..16 {
                                // i loops over 8 signals each CLEAR then SET bits
                                // so signal number is i/2, and SET when i is odd, otherwise clear
                                if (val & (1 << i)) == 0 { continue; }

                                let signo = i >> 1;

                                if (i & 0x01) == 0 { // CLEAR signal
                                    info!(target: "RSP", "clearing signal {}", signo);
                                    shared_state.signals &= !(1 << signo);
                                } else { // SET signal
                                    info!(target: "RSP", "setting signal {}", signo);
                                    shared_state.signals |= 1 << signo;
                                }
                            }
                        }

                        Ok(())
                    },

                    _ => todo!("unhandled cop0 register write $c{}", self.inst.rd),
                }
            },

            _ => {
                error!(target: "RSP", "unimplemented RSP COP0 instruction 0b{:02b}_{:03b}", cop0_op >> 3, cop0_op & 0x07);
                todo!();
                //Ok(())
            }
        }
    }

    #[inline(always)]
    fn v_byte(src: &__m128i, element: u8) -> u8 {
        // that the second parameter to _mm_extract_epi8 is const is unfortunate
        (unsafe { match element & 0x0F {
             0 => _mm_extract_epi8(*src, 15),  1 => _mm_extract_epi8(*src, 14),  2 => _mm_extract_epi8(*src, 13),
             3 => _mm_extract_epi8(*src, 12),  4 => _mm_extract_epi8(*src, 11),  5 => _mm_extract_epi8(*src, 10),
             6 => _mm_extract_epi8(*src,  9),  7 => _mm_extract_epi8(*src,  8),  8 => _mm_extract_epi8(*src,  7),
             9 => _mm_extract_epi8(*src,  6), 10 => _mm_extract_epi8(*src,  5), 11 => _mm_extract_epi8(*src,  4),
            12 => _mm_extract_epi8(*src,  3), 13 => _mm_extract_epi8(*src,  2), 14 => _mm_extract_epi8(*src,  1),
            15 => _mm_extract_epi8(*src,  0),  _ => 0,
        }}) as u8
    }

    #[inline(always)]
    fn v_short(src: &__m128i, element: u8) -> u16 {
        // that the second parameter to _mm_extract_epi8 is const is unfortunate
        (unsafe { match element & 0x07 {
            0 => _mm_extract_epi16(*src, 7), 1 => _mm_extract_epi16(*src, 6), 2 => _mm_extract_epi16(*src, 5),
            3 => _mm_extract_epi16(*src, 4), 4 => _mm_extract_epi16(*src, 3), 5 => _mm_extract_epi16(*src, 2),
            6 => _mm_extract_epi16(*src, 1), 7 => _mm_extract_epi16(*src, 0), _ => 0,
        }}) as u16
    }

    #[inline(always)]
    fn v_long(src: &__m128i, element: u8) -> u32 {
        // that the second parameter to _mm_extract_epi8 is const is unfortunate
        (unsafe { match element & 0x03 {
            0 => _mm_extract_epi32(*src, 3), 1 => _mm_extract_epi32(*src, 2), 
            2 => _mm_extract_epi32(*src, 1), 3 => _mm_extract_epi32(*src, 0), 
            _ => 0,
        }}) as u32
    }

    #[inline(always)]
    fn v_double(src: &__m128i, element: u8) -> u64 {
        // that the second parameter to _mm_extract_epi8 is const is unfortunate
        (unsafe { 
            if (element & 0x01) != 0 { 
                _mm_extract_epi64(*src, 0) 
            } else { 
                _mm_extract_epi64(*src, 1)  
            }
        }) as u64
    }

    // byte_element essentially causes a left rotate
    #[inline(always)]
    fn v_quad(&mut self, vt: usize, byte_element: u8) -> u128 {
        let src = &self.v[vt];
        let d = ((Self::v_double(src, 0) as u128) << 64) | (Self::v_double(src, 1) as u128);

        // rotate d by `element` bytes
        if byte_element != 0 {
            let shift = byte_element * 8;
            (d << shift) | (d >> (128 - shift))
        } else {
            d
        }
    }

    // get u128 from __m128i
    #[inline(always)]
    fn v_as_u128(src: &__m128i) -> u128 {
        unsafe {
            ((_mm_extract_epi64(*src, 0) as u64) as u128)
                | (((_mm_extract_epi64(*src, 1) as u64) as u128) << 64)
        }
    }

    #[inline(always)]
    fn v_reversed(src: &__m128i) -> __m128i {
        unsafe {
            _mm_shuffle_epi8(*src, _mm_set_epi8(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15))
        }
    }

    // returns __m128i that's been left-rotated
    // TODO I haven't found a way to do a full variable bit shift on __m128i yet
    // _mm_bslli_si128 exists, but requires a constant value shift
    // and _mm_sllv_ doesn't have an si128 version
    fn v_left_rotated(&self, v: &__m128i, byte_shift: u8) -> __m128i {
        // rotate_base is the array 15..0 with 16 added to all positions, because
        // byte_shift will be a value 0-15 and is subtracted from the array
        // since _mm_shuffle_epi8 wraps the byte indices
        unsafe {
            let shuffle = _mm_sub_epi8(self.rotate_base, _mm_set1_epi8((byte_shift & 0x0F) as i8));
            _mm_shuffle_epi8(*v, shuffle)
        }
    }

    fn inst_lwc2(&mut self) -> Result<(), InstructionFault> {
        let lwc_op = self.inst.rd; // the Rd field contains a function code for which store operation is being done
        let offset = (((self.inst.v & 0x7F) | ((self.inst.v & 0x40) << 1)) as i8) as u32; // signed 7-bit offset
        let _e = (self.inst.v >> 7) & 0x0F;

        match lwc_op {
            0b00_100 => { // LQV vt, offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                let b0 = self.read_u32_wrapped((address +  0) & 0x0FFF)? as i32;
                let b1 = self.read_u32_wrapped((address +  4) & 0x0FFF)? as i32;
                let b2 = self.read_u32_wrapped((address +  8) & 0x0FFF)? as i32;
                let b3 = self.read_u32_wrapped((address + 12) & 0x0FFF)? as i32;
                self.v[self.inst.rt as usize] = unsafe { _mm_set_epi32(b0, b1, b2, b3) };
                //.let rt = self.inst.rt;
                //.let rs = self.inst.rs;
                //.info!(target: "RSP", "LQV v{}, ${:04X}(r{}) // v{} = ${:04X}_{:04X}_{:04X}_{:04X}_{:04X}_{:04X}_{:04X}_{:04X}", rt, offset, rs, rt,
                //.    self.v_short(rt as usize, 0), self.v_short(rt as usize, 1), self.v_short(rt as usize, 2), self.v_short(rt as usize, 3),
                //.    self.v_short(rt as usize, 4), self.v_short(rt as usize, 5), self.v_short(rt as usize, 6), self.v_short(rt as usize, 7));
            },

            _ => {
                error!(target: "RSP", "unknown LWC2 operation 0b{:02b}_{:03b}", lwc_op >> 3, lwc_op & 0x07);
                todo!();
            }
        }

        Ok(())
    }

    fn inst_swc2(&mut self) -> Result<(), InstructionFault> {
        let swc_op = self.inst.rd; // the Rd field contains a function code for which store operation is being done
        let offset = (((self.inst.v & 0x7F) | ((self.inst.v & 0x40) << 1)) as i8) as u32; // signed 7-bit offset
        let element = ((self.inst.v >> 7) & 0x0F) as u8;

        match swc_op {
            0b00_000 => { // SBV vt[element], offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset) as usize;
                let byte = Self::v_byte(&self.v[self.inst.rt], element);
                self.write_u8(byte as u32, address & 0x0FFF)?;
            },

            0b00_001 => { // SSV vt[element], offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 1) as usize;
                let copy = self.v[self.inst.rt];
                let short = Self::v_short(&self.v_left_rotated(&copy, element), 0);
                self.write_u16_wrapped(short as u32, address & 0x0FFF)?;
            },

            0b00_010 => { // SLV vt[element], offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 2) as usize;
                let copy = self.v[self.inst.rt];
                let long = Self::v_long(&self.v_left_rotated(&copy, element), 0);
                self.write_u32_wrapped(long, address & 0x0FFF)?;
            },

            0b00_011 => { // SDV vt[element], offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 3) as usize;
                let copy = self.v[self.inst.rt];
                let double = Self::v_double(&self.v_left_rotated(&copy, element), 0);
                self.write_u32_wrapped((double >> 32) as u32, address & 0x0FFF)?;
                self.write_u32_wrapped(double as u32, (address + 4) & 0x0FFF)?;
            },

            0b00_100 => { // SQV vt, offset(base)
                let mut address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                let mut d = self.v_quad(self.inst.rt as usize, element);

                let mut byte_count = ((address & !0x0F) + 15) - address  + 1;
                while byte_count >= 4 {
                    self.write_u32_wrapped((d >> 96) as u32, address & 0x0FFF)?;
                    d <<= 32;
                    address += 4;
                    byte_count -= 4;
                }

                match byte_count {
                    3 => {
                        self.write_u16_wrapped(((d >> 112) as u16) as u32, address & 0x0FFF)?;
                        self.write_u8(((d >> 104) as u8) as u32, (address + 2) & 0x0FFF)?;
                    },
                    2 => {
                        self.write_u16_wrapped(((d >> 112) as u16) as u32, address & 0x0FFF)?;
                    },
                    1 => {
                        self.write_u8(((d >> 120) as u8) as u32, address & 0x0FFF)?;
                    },
                    _ => {},
                };
            },

            0b00_101 => { // SRV vt[element], offset(base)
                let mut address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                let d = self.v_quad(self.inst.rt as usize, element);

                let mut byte_count = address & 0x0F;
                let start_byte = 16 - byte_count;

                // rotate d again, and then perform the same as SQV
                let mut d = if start_byte != 0 {
                    let shift = start_byte * 8;
                    (d << shift) | (d >> (128 - shift))
                } else {
                    d
                };

                // write starts at the quad word aligned address and writes byte_count bytes
                address &= !0x0F;

                while byte_count >= 4 {
                    self.write_u32_wrapped((d >> 96) as u32, address & 0x0FFF)?;
                    d <<= 32;
                    address += 4;
                    byte_count -= 4;
                }

                match byte_count {
                    3 => {
                        self.write_u16_wrapped(((d >> 112) as u16) as u32, address & 0x0FFF)?;
                        self.write_u8(((d >> 104) as u8) as u32, (address + 2) & 0x0FFF)?;
                    },
                    2 => {
                        self.write_u16_wrapped(((d >> 112) as u16) as u32, address & 0x0FFF)?;
                    },
                    1 => {
                        self.write_u8(((d >> 120) as u8) as u32, address & 0x0FFF)?;
                    },
                    _ => {},
                };
            },

            // Store packed vector
            0b00_110 => { // SPV vt[e], offset(base)
                //info!(target: "RSP", "spv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 3) as usize;

                // we have to shift right by 8 (and also by 7 if element is not 0)
                // the datasheet says element should be zero, but it doesn't have to be and there's weird behavior when its not
                let src = &self.v[self.inst.rt as usize];

                let shift8 = unsafe { _mm_srli_epi16(*src, 8) };
                let shift7 = unsafe { _mm_srli_epi16(*src, 7) };

                // we need to extract the low byte of each of the 8 elements into the lower 64-bits of a new quadword
                let shift8 = unsafe {
                    let shuffled = _mm_shuffle_epi8(shift8, _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 14, 12, 10, 8, 6, 4, 2, 0));
                    _mm_extract_epi64(shuffled, 0) as u64
                };

                let shift7 = unsafe {
                    let shuffled = _mm_shuffle_epi8(shift7, _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 14, 12, 10, 8, 6, 4, 2, 0));
                    _mm_extract_epi64(shuffled, 0) as u64
                };

                // combine shift8 and shift7 into one 128-bit word, and rotate it
                let d = (shift8 as u128) << 64 | (shift7 as u128);
                let d = if element != 0 {
                    (d << (element * 8)) | (d >> (128 - (element * 8)))
                } else {
                    d
                };

                // write the upper packed 8 bytes to memory
                self.write_u32_wrapped((d >> 96) as u32, (address + 0) & 0x0FFF)?;
                self.write_u32_wrapped((d >> 64) as u32, (address + 4) & 0x0FFF)?;
            },

            0b00_111 => { // suv vt[element], offset(base)
                //info!(target: "RSP", "spv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 3) as usize;

                // we have to shift right by 8 (and also by 7 if element is not 0)
                // the datasheet says element should be zero, but it doesn't have to be and there's weird behavior when its not
                let src = &self.v[self.inst.rt as usize];

                let shift8 = unsafe { _mm_srli_epi16(*src, 8) };
                let shift7 = unsafe { _mm_srli_epi16(*src, 7) };

                // we need to extract the low byte of each of the 8 elements into the lower 64-bits of a new quadword
                let shift8 = unsafe {
                    let shuffled = _mm_shuffle_epi8(shift8, _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 14, 12, 10, 8, 6, 4, 2, 0));
                    _mm_extract_epi64(shuffled, 0) as u64
                };

                let shift7 = unsafe {
                    let shuffled = _mm_shuffle_epi8(shift7, _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 14, 12, 10, 8, 6, 4, 2, 0));
                    _mm_extract_epi64(shuffled, 0) as u64
                };

                // combine shift8 and shift7 into one 128-bit word, and rotate it
                // for SUV, shift7 is stored first
                let d = (shift7 as u128) << 64 | (shift8 as u128);
                let d = if element != 0 {
                    (d << (element * 8)) | (d >> (128 - (element * 8)))
                } else {
                    d
                };

                // write the upper packed 8 bytes to memory
                self.write_u32_wrapped((d >> 96) as u32, (address + 0) & 0x0FFF)?;
                self.write_u32_wrapped((d >> 64) as u32, (address + 4) & 0x0FFF)?;
            },

            0b01_000 => { // SHV vt[element], offset(base)
                //info!(target: "RSP", "shv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                let offset = address & 0x07;
                let addr_base = address & !0x07;

                // shift all elements right 7, take the lower bytes of each elements
                // then write those bytes to every 2nd byte of memory
                let src = &self.v[self.inst.rt as usize];
                let rotated = self.v_left_rotated(src, element);
                let shifted = unsafe { _mm_srli_epi16(rotated, 7) };
                let shuffled = unsafe {
                    // i'm extracting the bytes in reverse, so that shifting them and extracting them is simpler
                    _mm_shuffle_epi8(shifted, _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 4, 6, 8, 10, 12, 14))
                };

                let mut d = unsafe { 
                    _mm_extract_epi64(shuffled, 0) as u64 
                };

                for i in 0..8 {
                    let addr = addr_base + ((offset + i * 2) & 0x0F);
                    self.write_u8((d as u8) as u32, addr & 0x0FFF)?;
                    d >>= 8;
                }
            },

            0b01_001 => { // SFV vt[element], offset(base)
                //info!(target: "RSP", "sfv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                let offset = address & 0x07;
                let addr_base = address & !0x07;

                // only certain element values work here
                const VALID: [bool; 16] = [true, true, false, false, true, true, false, false, true, false, false, true, true, false, false, true];

                // shift all elements right 7, take a certain group of four elements
                // then write those bytes to every 4th byte of memory
                let mut d = if VALID[element as usize] {
                    let src = &self.v[self.inst.rt as usize];
                    let shifted = unsafe { _mm_srli_epi16(*src, 7) };
                    let shuffled = unsafe {
                        // i'm extracting the bytes in reverse, so that shifting them and extracting them is simpler
                        _mm_shuffle_epi8(shifted, match element {
                                // the element bytes we want are the low bytes of the word
                                // element 0th, 1st, 2nd, ...
                                // byte     14,  12,  10,  8,  6,  4,  2,  0
                                // notice they wrap within the half-vector
                                 0 | 15 => _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  8, 10, 12, 14), // 0, 1, 2, 3
                                 1      => _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  4,  6,  0,  2), // 6, 7, 4, 5
                                 4      => _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 14,  8, 10, 12), // 1, 2, 3, 0
                                 5      => _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  2,  4,  6,  0), // 7, 4, 5, 6
                                 8      => _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  2,  4,  6), // 4, 5, 6, 7
                                11      => _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 12, 14,  8), // 3, 0, 1, 2
                                12      => _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  6,  0,  2,  4), // 5, 6, 7, 4
                                _ => _mm_setzero_si128(),
                            }
                        )
                    };

                    (unsafe { _mm_extract_epi32(shuffled, 0) }) as u32
                } else {
                    0
                };

                for i in 0..4 {
                    let addr = addr_base + ((offset + i * 4) & 0x0F);
                    self.write_u8((d as u8) as u32, addr & 0x0FFF)?;
                    d >>= 8;
                }
            },

            0b01_010 => { // SWV vt[element], offset(base)
                //info!(target: "RSP", "swv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                let offset = address & 0x07;
                let addr_base = address & !0x07;

                // get vt rotated by `element` amount, and then rotate and store 
                let src = &self.v[self.inst.rt];
                let mut d = Self::v_as_u128(&Self::v_reversed(&self.v_left_rotated(src, element)));

                for i in 0..16 {
                    self.write_u8((d as u8) as u32, (addr_base + ((offset + i) & 0x0F)) & 0x0FFF)?;
                    d >>= 8;

                    // want element `element + i` in rightmost byte, and sure would be nice to have a non-const extract
                    //.let src = &self.v[self.inst.rt];
                    //.let mut rotated = self.v_left_rotated(src, element + (i as u8) + 1);
                    //.let e = unsafe { _mm_extract_epi8(rotated, 0) } as u8;
                    //.self.write_u8((e as u8) as u32, (addr_base + ((offset + i) & 0x0F)) & 0x0FFF)?;
                }
            },

            0b01_011 => { // STV vt[element], offset(base)
                //info!(target: "RSP", "stv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                let addr_offset = address & 0x07;
                let addr_base = address & !0x07;
                let v_base = self.inst.rt & !0x07;

                for i in 0..8 {
                    // src register is current slice (i) shifted by selected element, wrapping within group of 8 VU regs
                    let src = &self.v[v_base | ((i + (element >> 1)) & 0x07) as usize];

                    // grab the i'th element into the left-most index
                    let rotated = self.v_left_rotated(src, i * 2);

                    // extract it
                    let e = unsafe { _mm_extract_epi16(rotated, 7) } as u16;

                    // can't use write_u16_wrapped because we need to wrap within a 16 byte block, 
                    // so write upper and lower bytes individually
                    let offset = (addr_offset + (i * 2) as usize) & 0x0F;
                    self.write_u8(((e >> 8) as u8) as u32, (addr_base + ((offset    )       )) & 0x0FFF)?;
                    self.write_u8(((e >> 0) as u8) as u32, (addr_base + ((offset + 1) & 0x0F)) & 0x0FFF)?;
                }
            },

            _ => {
                error!(target: "RSP", "unknown SWC2 operation 0b{:02b}_{:03b}", swc_op >> 3, swc_op & 0x07);
                todo!();
            }
        };

        Ok(())
    }

    fn inst_cop2(&mut self) -> Result<(), InstructionFault> {
        if (self.inst.v & (1 << 25)) != 0 {
            let func = self.inst.v & 0x3F;
            self.cop2_table[func as usize](self)
        } else {
            let cop2_op = (self.inst.v >> 21) & 0x1F;
            match cop2_op {
                _ => {
                    error!(target: "RSP", "unimplemented RSP COP2 instruction 0b{:02b}_{:03b}", cop2_op >> 3, cop2_op & 0x07);
                    todo!();
                    //Ok(())
                }
            }
        }
    }

    fn cop2_unknown(&mut self) -> Result<(), InstructionFault> {
        let func = self.inst.v & 0x3F;
        error!(target: "RSP", "unimplemented COP2 function 0b{:03b}_{:03b}", func >> 3, func & 0x07);
        todo!();
    }
}

impl Addressable for RspCpuCore {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        if offset < 0x4_0000 {
            let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
            let mem = self.mem.read().unwrap();
            Ok(mem[mem_offset as usize])
        } else {
            todo!("TODO");
            //.Err(ReadWriteFault::Invalid)
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        if offset < 0x4_0000 {
            let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
            let mut mem = self.mem.write().unwrap();
            mem[mem_offset as usize] = value;
            Ok(WriteReturnSignal::None)
        } else {
            todo!("TODO");
            //.Err(ReadWriteFault::Invalid)
        }
    }

}
