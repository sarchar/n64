#![allow(non_upper_case_globals)]
use std::mem;
use std::sync::{mpsc, Arc, Mutex, RwLock};
use std::thread;

#[cfg(all(target_arch="x86_64", target_feature="sse2"))]
use core::arch::x86_64::*;
use avx512f_wrapper::*;

#[allow(unused_imports)]
use tracing::{trace, debug, error, info, warn};

use crate::*;
use cpu::{InstructionDecode, InstructionFault};
use hle::Hle;
use mips::{InterruptUpdate, InterruptUpdateMode, IMask_SP};

use rcp::DmaInfo;
use rdp::Rdp;

/// COP0 "registers"
const Cop0_DmaCache      : usize = 0; // IMEM or DMEM address for a DMA transfer
const Cop0_DmaDram       : usize = 1; // RDRAM address for a DMA
const Cop0_DmaReadLength : usize = 2;
const Cop0_DmaWriteLength: usize = 3;
const Cop0_Status        : usize = 4;
const Cop0_DmaFull       : usize = 5;
const Cop0_DmaBusy       : usize = 6;
const Cop0_Semaphore     : usize = 7;
const Cop0_CmdStart      : usize = 8;
const Cop0_CmdEnd        : usize = 9;
const Cop0_CmdCurrent    : usize = 10;
const Cop0_CmdStatus     : usize = 11;
const Cop0_CmdClock      : usize = 12;
const _Cop0_CmdBusy       : usize = 13;
const _Cop0_CmdPipeBusy   : usize = 14;
const _Cop0_CmdTMemBusy   : usize = 15;

const Cop2_VCO: usize = 0;
const Cop2_VCC: usize = 1;
const Cop2_VCE: usize = 2;

/// N64 Reality Signal Processor
/// Resides on the die of the RCP.
pub struct Rsp {
    comms: SystemCommunication,

    core: Arc<Mutex<RspCpuCore>>,
    shared_state: Arc<RwLock<RspSharedState>>,

    wakeup_tx: Option<mpsc::Sender<u32>>,
    broke_rx: Option<mpsc::Receiver<bool>>,

    // access to RspCpuCore memory
    mem: Arc<RwLock<Vec<u32>>>,

    dma_completed_rx: mpsc::Receiver<DmaInfo>,
    dma_completed_tx: mpsc::Sender<DmaInfo>,

    // these are copies of state in RspCpuCore so we don't need a lock
    halted: bool,
    broke: bool,
}

#[derive(Debug, Default)]
pub struct RspSharedState {
    exited: bool,
    intbreak: bool,
    halted_self: bool,

    dma_cache: u32,
    dma_dram: u32,
    dma_read_length: u32,
    dma_write_length: u32,
    dma_busy: bool,
    dma_full: Option<DmaInfo>,
    dma_total_size: u32,
    semaphore: bool,

    // current or most recent DMA, and the data that is readable in the DMA registers
    dma_current_cache: u32,
    dma_current_dram: u32,
    dma_current_read_length: u32,
    dma_current_write_length: u32,

    // signals
    signals: u32,
}

struct RspCpuCore {
    comms: SystemCommunication,

    num_steps: u64,

    // registers
    pc: u32,
    current_instruction_pc: u32, // actual PC of the currently executing instruction
                                 // only valid inside step()
    next_instruction: u32,       // emulates delay slot (prefetch, next instruction)
    next_instruction_pc: u32,    // for printing correct delay slot addresses
    is_delay_slot: bool,         // true if the currently executing instruction is in a delay slot
    next_is_delay_slot: bool,    // set to true on branching instructions

    gpr: [u32; 32],
    ccr: [u32; 4],

    v: [__m128i; 32],            // 32 128-bit VU registers
    vacc: __wm512i,              // 8x48bit (in 64-bit lanes) accumulators
    vacc_mask: __wm512i,         // always 0xFFFF_FFFF_FFFF_FFFF
    v256_ones: __m256i,          // 8x32bit 0x0000_0001
    rotate_base: __m128i,

    inst: InstructionDecode,
    inst_e: u8, // used by COP2 math instructions, not SWC/LWC
    inst_vt: usize,
    inst_vs: usize,
    inst_vd: usize,

    instruction_table: [CpuInstruction; 64],
    special_table: [CpuInstruction; 64],
    regimm_table: [CpuInstruction; 32],
    cop2_table: [CpuInstruction; 64],

    // rcp and rsq tables
    rcp_high: bool,
    rcp_input: u16,
    rcp_table: [u16; 512],
    rsq_table: [u16; 512],
    div_result: u32,

    // multiple reader single writer memory
    mem: Arc<RwLock<Vec<u32>>>,

    shared_state: Arc<RwLock<RspSharedState>>,

    broke_tx: Option<mpsc::Sender<bool>>,
    dma_completed_tx: mpsc::Sender<DmaInfo>,

    // running state
    halted: bool,
    halted_self: bool,
    broke: bool,

    // access to the RDP
    rdp: Arc<Mutex<Rdp>>,

    // HLE
    process_task: bool,
}

type CpuInstruction = fn(&mut RspCpuCore) -> Result<(), InstructionFault>;

impl Rsp {
    pub fn new(comms: SystemCommunication, rdp: Arc<Mutex<Rdp>>) -> Rsp {
        let mem = Arc::new(RwLock::new(vec![0u32; 2*1024]));

        let shared_state = Arc::new(RwLock::new(RspSharedState::default()));
    
        // dma_completed channel is created here and sent with every DmaInfo message
        let (dma_completed_tx, dma_completed_rx) = mpsc::channel();

        let core = Arc::new(Mutex::new(RspCpuCore::new(comms.clone(), mem.clone(), shared_state.clone(), rdp, dma_completed_tx.clone())));

        Rsp {
            comms: comms,

            core: core,

            mem: mem,

            wakeup_tx: None,
            broke_rx: None,

            dma_completed_rx: dma_completed_rx,
            dma_completed_tx: dma_completed_tx,

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

        let mut hle: Option<Hle> = if let Some(ref hle_command_buffer) = self.comms.hle_command_buffer {
            Some(Hle::new(self.comms.clone(), hle_command_buffer.clone()))
        } else {
            None
        };

        self.shared_state.write().unwrap().exited = false;

        thread::spawn(move || {
            let mut _started_time = std::time::Instant::now();
            'main_loop: loop {
                let mut c = core.lock().unwrap();

                // halted can only be set while we don't have a lock, so check here
                if c.halted {
                    //.info!(target: "RSP", "RSP core halted (broke={}), waiting for wakeup", c.broke);

                    // drop the lock on the core and wait for a wakeup signal
                    drop(c);

                    // wait forever for a signal
                    match wakeup_rx.recv().unwrap() {
                        0 => {}, // normal wakeup
                        1 => {   // exit thread
                            break 'main_loop;
                        }
                        _ => panic!("invalid"),
                    }

                    // running!
                    let mut c = core.lock().unwrap();
                    _started_time = std::time::Instant::now();
                    c.num_steps = 0;
                    c.halted = false;
                    c.broke = false;

                    // prefetch the next instruction, as the CPU may have (probably!) changed IMEM
                    c.pc = c.next_instruction_pc;
                    let _ = c.prefetch();

                    // OSTasks are only set when the RSP is halted, so whenever we wake up we can check and perform tasks
                    if c.process_task {
                        c.process_task = false;

                        // Task type is the first element of the OSTask structure, which is at 0xFC0 
                        let task_type = c.read_u32(0x0FC0).unwrap();
                        debug!(target: "RSP", "processing task type {}", task_type);
                        match task_type {
                            1 => { // M_GFXTASK
                                let dl_start = c.read_u32(0x0FF0).unwrap(); // OSTask->data_ptr
                                if dl_start != 0 { // for gfx tasks, the DL list start pointer must be valid
                                    let dl_length = c.read_u32(0x0FF4).unwrap(); // OSTask->data_size

                                    // pass along the microcode address so that the HLE can detect which software is running
                                    let ucode_address = c.read_u32(0x0FD0).unwrap(); // OSTask->ucode;

                                    // free the lock on core while running the DL
                                    drop(c);
                                    if let Some(ref mut h) = hle {
                                        h.process_display_list(dl_start, dl_length, ucode_address);
                                    }

                                    // reclaim lock
                                    let mut c = core.lock().unwrap();

                                    // set SIG2 (SP_STATUS_TASKDONE)
                                    {
                                        let mut shared_state = c.shared_state.write().unwrap();
                                        shared_state.signals |= 1 << 2;
                                    }
                                    // and break, which usually triggers SP interrupt
                                    let _ = c.special_break().unwrap();

                                    // RSP isn't running
                                    c.halted = true;

                                    {
                                        let mut rdp = c.rdp.lock().unwrap();
                                        rdp.write_u32(0x04, 0x0010_000C as usize).unwrap(); // write CLR_FREEZE bit
                                    }
                                } else {
                                    todo!();
                                }
                            },

                            2 => { // M_AUDTASK
                                // run on RSP for now
                                debug!(target: "RSP", "stubbed audio tasks for now, as they crash");
                                    // set SIG2 (SP_STATUS_TASKDONE)
                                    {
                                        let mut shared_state = c.shared_state.write().unwrap();
                                        shared_state.signals |= 1 << 2;
                                    }

                                    // and break, which usually triggers SP interrupt
                                    let _ = c.special_break().unwrap();

                                    // RSP isn't running
                                    c.halted = true;
                            },

                            // All other tasks types are LLE'd
                            _ => {
                                if task_type < 8 {
                                    debug!(target: "RSP", "found task type {}", task_type);
                                }
                            },
                        }
                    }
                } else {
                    // run for some cycles or until break
                    for _ in 0..40 {
                        let _ = c.step(); // TODO handle errors

                        if c.broke || c.halted_self { 
                            c.halted = true; // signal the core is halted before dropping the lock
                            break; 
                        }
                    }
                }
            }

            // main loop has exited, mark exit on the shared state
            let c = core.lock().unwrap();
            let mut shared_state = c.shared_state.write().unwrap();
            shared_state.exited = true;
        });
    }

    pub fn stop(&mut self) {
        info!(target: "RSP", "stop");

        // send the exit signal to the running thread
        self.wakeup_tx.as_mut().unwrap().send(1).unwrap(); // 1 = exit code

        loop {
            // repeatedly halt the CPU since it could be receiving wakeups
            // so run and check until the thread has exited
            let mut c = self.core.lock().unwrap();
            if !c.halted {
                c.halted = true;
                c.halted_self = false;
            }

            let shared_state = self.shared_state.read().unwrap();
            if shared_state.exited {
                break;
            }
        }
    }

    pub fn reset(&mut self) {
        info!(target: "RSP", "reset");

        if !self.shared_state.read().unwrap().exited {
            warn!(target: "RSP", "reset without calling stop()");
        }

        // clear all messages
        if let Some(broke_rx) = &self.broke_rx {
            while broke_rx.try_recv().is_ok() {}
        }
        while self.dma_completed_rx.try_recv().is_ok() {}

        // set current state to halted
        self.halted = true;
        self.broke = false;

        // reset shared state
        *self.shared_state.write().unwrap() = RspSharedState::default();

        // reset rsp core
        let _ = self.core.lock().unwrap().reset();
    }

    pub fn step(&mut self) {
        let _ = self.is_broke(); // update interrupts

        if let Ok(_) = self.dma_completed_rx.try_recv() {
            // we got a completed DMA, so if one is pending start it immediately
            let mut shared_state = self.shared_state.write().unwrap();
            shared_state.dma_busy = false;
            if let Some(dma_info) = mem::replace(&mut shared_state.dma_full, None) {
                shared_state.dma_current_cache        = (shared_state.dma_cache & 0x1000) | (((shared_state.dma_cache & !0x07) + shared_state.dma_total_size) & 0x0FFF);
                shared_state.dma_current_dram         = (shared_state.dma_dram & !0x07) + shared_state.dma_total_size;
                shared_state.dma_current_read_length  = 0xFF8;
                shared_state.dma_current_write_length = 0xFF8;
                shared_state.dma_busy = true;
                self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap(); 
                self.comms.break_cpu();
            }
        }
    }

    pub fn is_broke(&mut self) -> bool {
        // update the local BREAK status
        if let Some(broke_rx) = &self.broke_rx {
            while let Ok(set) = broke_rx.try_recv() {
                self.broke = false;
                if set {
                    self.broke = true;
                    self.halted = true;
                }
            }
        }
        self.broke
    }

    fn read_register(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        //info!(target: "RSP", "read32 register offset=${:08X}", offset);

        let value = match offset {
            // SP_DMA_CACHE
            0x4_0000 => {
                let shared_state = self.shared_state.read().unwrap();
                shared_state.dma_current_cache
            },
            
            // SP_DMA_DRAM
            0x4_0004 => {
                let shared_state = self.shared_state.read().unwrap();
                shared_state.dma_current_dram
            },

            // SP_DMA_READ_LENGTH
            0x4_0008 => {
                let shared_state = self.shared_state.read().unwrap();
                shared_state.dma_current_read_length
            },

            // SP_DMA_WRITE_LENGTH
            0x4_000C => {
                let shared_state = self.shared_state.read().unwrap();
                shared_state.dma_current_write_length
            },

            // SP_STATUS
            0x4_0010 => {
                trace!(target: "RSP", "read SP_STATUS at {}", self.comms.total_cpu_steps.get() as u64);

                // update self.broke
                let _ = self.is_broke();

                // update self.halted on self-halting RSP
                if !self.halted {
                    let shared_state = self.shared_state.read().unwrap();
                    if shared_state.halted_self {
                        drop(shared_state);
                        self.halted = true;
                        // clear both shared_state and core states
                        {
                            let mut shared_state = self.shared_state.write().unwrap();
                            shared_state.halted_self = false;
                        }
                        {
                            let mut core = self.core.lock().unwrap();
                            core.halted_self = false;
                        }
                    }
                }

                let shared_state = self.shared_state.read().unwrap();

                ((self.halted as u32) << 0)
                    | ((self.broke as u32) << 1)
                    | ((shared_state.dma_busy as u32) << 2)
                    | ((shared_state.dma_full.is_some() as u32) << 3)
                    | ((shared_state.intbreak as u32) << 6)
                    | (shared_state.signals << 7)
            },

            // SP_DMA_FULL
            0x4_0014 => {
                //debug!(target: "RSP", "read SP_DMA_FULL");

                let shared_state = self.shared_state.read().unwrap();
                shared_state.dma_full.is_some() as u32
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
            // SP_DMA_CACHE
            0x4_0000 => {
                //debug!(target: "RSP", "DMA RSP address set to ${:04X} via CPU", value & 0x1FFF);
                let mut shared_state = self.shared_state.write().unwrap();
                shared_state.dma_cache = value & 0x1FFF;
            },

            // SP_DMA_DRAM
            0x4_0004 => {
                //debug!(target: "RSP", "DMA DRAM address set to ${:04X} via CPU", value & 0x00FF_FFFF);
                let mut shared_state = self.shared_state.write().unwrap();
                shared_state.dma_dram = value & 0x00FF_FFFF;
            },

            // SP_DMA_READ_LENGTH
            0x4_0008 => {
                //debug!(target: "RSP", "DMA read length set to ${:04X} via CPU", value);
                let mut shared_state = self.shared_state.write().unwrap();
                shared_state.dma_read_length = value;

                let length = ((value & 0x0FFF) | 0x07) + 1;
                let count  = ((value >> 12) & 0xFF) + 1;
                let skip   = (value >> 20) & 0xFFF;
                shared_state.dma_total_size = count * length + (count - 1) * skip;

                let dma_info = DmaInfo {
                    initiator     : "RSP-READ(CPU)",
                    source_address: (shared_state.dma_dram) & !0x07,
                    dest_address  : (shared_state.dma_cache | 0x0400_0000) & !0x07,
                    count         : count,
                    length        : length,
                    source_stride : skip,
                    dest_mask     : 0x0000_0FFF,
                    dest_bits     : 0x0400_0000 | (shared_state.dma_cache & 0x1000),
                    completed     : Some(self.dma_completed_tx.clone()),
                    ..Default::default()
                };

                if shared_state.dma_busy {
                    if shared_state.dma_full.is_none() {
                        shared_state.dma_full = Some(dma_info);
                    }
                } else {
                    // update the addresses as if the dma has been completed
                    shared_state.dma_current_cache        = (shared_state.dma_cache & 0x1000) | (((shared_state.dma_cache & !0x07) + shared_state.dma_total_size) & 0x0FFF);
                    shared_state.dma_current_dram         = (shared_state.dma_dram & !0x07) + shared_state.dma_total_size;
                    shared_state.dma_current_read_length  = 0xFF8;
                    shared_state.dma_current_write_length = 0xFF8;
                    shared_state.dma_busy = true;
                    self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                    self.comms.break_cpu();
                }
            },

            // SP_DMA_WRITE_LENGTH
            0x4_000C => {
                //debug!(target: "RSP", "DMA write length set to ${:04X}", value);
                let mut shared_state = self.shared_state.write().unwrap();
                shared_state.dma_write_length = value;

                let length = ((value & 0x0FFF) | 0x07) + 1;
                let count  = ((value >> 12) & 0xFF) + 1;
                let skip   = (value >> 20) & 0xFFF;
                shared_state.dma_total_size = count * length + (count - 1) * skip;

                let dma_info = DmaInfo {
                    initiator     : "RSP-WRITE(CPU)",
                    source_address: (shared_state.dma_cache | 0x0400_0000) & !0x07,
                    dest_address  : (shared_state.dma_dram) & !0x07,
                    count         : count,
                    length        : length,
                    dest_stride   : skip,
                    source_mask   : 0x0000_0FFF,
                    source_bits   : 0x0400_0000 | (shared_state.dma_cache & 0x1000),
                    completed     : Some(self.dma_completed_tx.clone()),
                    ..Default::default()
                };

                if shared_state.dma_busy {
                    if shared_state.dma_full.is_none() {
                        shared_state.dma_full = Some(dma_info);
                    }
                } else {
                    // update the addresses as if the dma has been completed
                    shared_state.dma_current_cache        = (shared_state.dma_cache & 0x1000) | (((shared_state.dma_cache & !0x07) + shared_state.dma_total_size) & 0x0FFF);
                    shared_state.dma_current_dram         = (shared_state.dma_dram & !0x07) + shared_state.dma_total_size;
                    shared_state.dma_current_read_length  = 0xFF8;
                    shared_state.dma_current_write_length = 0xFF8;
                    shared_state.dma_busy = true;
                    self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                    self.comms.break_cpu();
                }
            },

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
                
                // if CLR_HALT and SET_HALT are both set, neither happen
                if (value & 0x03) == 0x03 { value &= !0x03; }

                // CLR_HALT: clear halt flag and start RSP
                if (value & 0x01) != 0 { 
                    // sum up the first 44 bytes of imem
                    //.let mut sum: u32 = 0;
                    //.for i in 0..44 {
                    //.    if let Ok(v) = self.read_u8(0x1000 + i) {
                    //.        sum = sum.wrapping_add(v as u32);
                    //.    }
                    //.}

                    //.info!(target: "RSP", "sum of IMEM at start: ${:08X} (if this value is 0x9E2, it is a bootcode program)", sum);
                    //.{
                    //.    let core = self.core.lock().unwrap();
                    //.    info!(target: "RSP", "starting RSP at PC=${:08X}", core.next_instruction_pc);
                    //.}

                    // set local copy
                    self.halted = false;

                    // send wakeup signal
                    if let Some(wakeup_tx) = &self.wakeup_tx {
                        wakeup_tx.send(0).unwrap(); // 0 = wakeup and execute
                    }
                }

                // SET_HALT: halt the RSP
                if (value & 0x02) != 0 {
                    // TODO it might be worth converting this to a mpsc channel as well,
                    // depending on if anything uses SET_HALT often enough to causes stalls
                    info!(target: "RSP", "CPU has halted the RSP");
                    let mut c = self.core.lock().unwrap();
                    c.halted = true;
                    c.halted_self = false;
                    let mut shared_state = self.shared_state.write().unwrap();
                    shared_state.halted_self = false;
                    self.halted = true;
                }

                // if both SET_INTBREAK and CLR_INTBREAK are set, do nothing
                if (value & 0x180) == 0x180 { value &= !0x180; }

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

                // If both SET and CLR_INTR are set, do nothing
                if (value & 0x18) == 0x18 {
                    value &= !0x18;
                }

                // SET_INTR: manually trigger SP interrupt
                if (value & 0x10) != 0 {
                    self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_SP, InterruptUpdateMode::SetInterrupt)).unwrap();
                    self.comms.break_cpu();
                }

                // CLR_INTR: ack SP interrupt
                if (value & 0x08) != 0 {
                    self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_SP, InterruptUpdateMode::ClearInterrupt)).unwrap();
                }

                // loop over the signals
                value >>= 9; // signals start at bit 9
                if value != 0 {
                    let mut shared_state = self.shared_state.write().unwrap();
                    for i in 0..16 {
                        // i loops over 8 signals each CLEAR then SET bits
                        // so signal number is i/2, and SET when i is odd, otherwise clear
                        if (value & (1 << i)) == 0 { continue; }

                        // toggle the odd bit and if it is also set, do nothing
                        // (you cannot clear and set a signal at the same time)
                        if (value & (1 << (i ^ 0x01))) != 0 { continue; }

                        let signo = i >> 1;

                        // if the signal is set and clear, do nothing
                        if (i & 0x01) == 0 { // CLEAR signal
                            //info!(target: "RSP", "CPU clearing signal {}", signo);
                            shared_state.signals &= !(1 << signo);
                        } else { // SET signal
                            //info!(target: "RSP", "CPU setting signal {}", signo);
                            shared_state.signals |= 1 << signo;
                        }
                    }
                }
            },

            // SP_SEMAPHORE
            0x4_001C => {
                let mut shared_state = self.shared_state.write().unwrap();
                shared_state.semaphore = false;
            },

            // SP_PC
            0x8_0000 => {
                trace!(target: "RSP", "write SP_PC value=${:08X}", value);

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

    // SD on RSP memory is broken, only write the upper word
    fn write_u64(&mut self, value: u64, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        if (offset & 0xF_0000) < 0x4_0000 {
            let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
            let mut mem = self.mem.write().unwrap();
            mem[mem_offset as usize] = (value >> 32) as u32;
            Ok(WriteReturnSignal::None)
        } else {
            todo!("TODO");
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
                    //match mem_offset {
                    //    DL_START => {
                    //        info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (DL start)", value, offset);
                    //    },
                    //    DL_LEN => {
                    //        info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (DL length)", value, offset);
                    //    },
                    //    TASK_TYPE => {
                    //        info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (TaskType)", value, offset);
                    //    },
                    //    _ => {},
                    //}
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

    fn write_u16(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        // SH incorrectly overwrites other bytes with zeroes
        let offset = offset & !0x01;
        let shift = 16 - ((offset & 0x02) << 3);
        self.write_u32(value << shift, offset & !0x02)
    }

    fn write_u8(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        // SB incorrectly overwrites other bytes with zeroes
        let shift = 24 - ((offset & 0x03) << 3);
        self.write_u32(value << shift, offset & !0x03)
    }

    fn read_block(&mut self, offset: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        // wrap offset into local memory
        let mut offset = offset & 0x1FF8;

        if offset < 0x2000 { // I/DRAM
            let mem = self.mem.read().unwrap();
            let (dram, iram) = mem.split_at(0x1000 >> 2);
            let which_mem = if (offset & 0x1000) != 0 { iram } else { dram };
            offset &= 0xFF8; // drop the high bit

            let (_, right) = which_mem.split_at(offset >> 2);
            if (length >> 2) as usize <= right.len() {
                let (left, _) = right.split_at((length >> 2) as usize);
                Ok(left.to_owned())
            } else {
                // start with all the data from the right chunk
                let mut ret = right.to_owned();
                // and read the rest from the beginning of ram
                let mut length = length - ret.len() as u32 * 4;
                while length > 0 {
                    let amount_to_copy = std::cmp::min(0x1000, length);
                    let (left, _) = which_mem.split_at((amount_to_copy >> 2) as usize);
                    ret.extend_from_slice(&left);
                    length -= amount_to_copy;
                }
                Ok(ret)
            }
        } else {
            todo!("DMA out of ${offset:8X}");
        }
    }

    fn write_block(&mut self, offset: usize, block: &[u32], length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
        if (block.len() * 4) as u32 != length { todo!(); }

        // wrap offset into local memory
        let mut offset = offset & 0x1FF8;

        if offset < 0x2000 { // I/DRAM
            {
                let mut mem = self.mem.write().unwrap();
                let (dram, iram) = mem.split_at_mut(0x1000 >> 2);
                let which_mem = if (offset & 0x1000) != 0 { iram } else { dram };
                offset &= 0xFF8; // drop the high bit

                let (_, right) = which_mem.split_at_mut(offset >> 2);
                if block.len() <= right.len() {
                    let (left, _) = right.split_at_mut(block.len());
                    left.copy_from_slice(block);
                } else {
                    right.copy_from_slice(&block[0..right.len()]);
                    let leftover_words = block.len() - right.len();
                    let (left, _) = which_mem.split_at_mut(leftover_words);
                    left.copy_from_slice(&block[(block.len()-leftover_words)..block.len()]);
                }
            }

            // if the TaskType (0x0FC0) value has changed, inform the core
            // TODO: we should check what ucode is loaded before doing this
            // TODO: if hle_enabled...
            if 0x0FC0 >= offset && 0x0FC0 < (offset + block.len() << 2) {
                let mut c = self.core.lock().unwrap();
                c.process_task = true;
            }

            Ok(WriteReturnSignal::None)
        } else {
            todo!("DMA into ${offset:8X}");
        }
    }
}

use RspCpuCore as Cpu; // shorthand so I can copy code from cpu.rs :)
impl RspCpuCore {
    fn new(comms: SystemCommunication, mem: Arc<RwLock<Vec<u32>>>, shared_state: Arc<RwLock<RspSharedState>>, rdp: Arc<Mutex<Rdp>>, dma_completed_tx: mpsc::Sender<DmaInfo>) -> Self {
        // initialize the reciprocal table.. the algorithm is widely available but I'll include the Ares license here as well, since it's used in n64-systemtest
        // The generation of the RCP and RSP tables was ported from Ares: https://github.com/ares-emulator/ares/blob/acd2130a4d4c9e7208f61e0ff762895f7c9b8dc6/ares/n64/rsp/rsp.cpp#L102
        // which uses the following license:
        
        // Copyright (c) 2004-2021 ares team, Near et al
        //
        // Permission to use, copy, modify, and/or distribute this software for any
        // purpose with or without fee is hereby granted, provided that the above
        // copyright notice and this permission notice appear in all copies.
        //
        // THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
        // WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
        // MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
        // ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
        // WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
        // ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
        // OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
        let mut rcp_table = [0xFFFFu16; 512];
        for i in 1..512 {
            rcp_table[i] = ((((1u64 << 34) / ((i as u64) + 512)) + 1) >> 8) as u16;
        }

        // this algorithm comes from n64-systemtest, so: Copyright (c) 2021 lemmy-64 under the MIT license
        let mut rsq_table = [0u16; 512];
        let alg_start_time = std::time::Instant::now();
        for i in 0..512 {
            let a = (if i < 256 { i + 256 } else { ((i - 256) << 1) + 512 }) as u64;
            let mut b = 1u64 << 17;
            rsq_table[i] = {
                // find the largest b where b < 1.0 / sqrt(a)
                let mut inc = 512;
                while inc != 0 {
                    while (a * (b + inc) * (b + inc)) < (1u64 << 44) { b += inc; }
                    inc >>= 1;
                }
                (b >> 1) as u16
            };
        }
        let alg_time = alg_start_time.elapsed();
        info!(target: "RSP", "RSQ table generation took {:.2?}", alg_time);

        let mut core = RspCpuCore {
            comms: comms,

            num_steps: 0,
            pc: 0,
            current_instruction_pc: 0,
            next_instruction: 0,
            next_instruction_pc: 0,
            is_delay_slot: false,
            next_is_delay_slot: false,

            gpr: [0u32; 32],
            ccr: [0u32; 4],

            rcp_high: false,
            rcp_input: 0,
            rcp_table: rcp_table,
            rsq_table: rsq_table,
            div_result: 0u32,

            v: [unsafe { _mm_setzero_si128() }; 32],
            vacc: _wmm512_setzero_si512(),
            vacc_mask: _wmm512_set1_epi64(0xFFFF_FFFF_FFFF_FFFFu64 as i64), 
            v256_ones: unsafe { _mm256_set1_epi32(1) },
            rotate_base: unsafe { _mm_set_epi8(31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16) },

            inst: InstructionDecode::default(),
            inst_e: 0,
            inst_vt: 0,
            inst_vs: 0,
            inst_vd: 0,

            mem: mem,
            shared_state: shared_state,
            broke_tx: None,
            dma_completed_tx: dma_completed_tx,

            halted: true,
            halted_self: false,
            broke: false,

            rdp: rdp,
            process_task: false,

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
   /* 000_ */   Cpu::cop2_vmulf   , Cpu::cop2_vmulu   , Cpu::cop2_vrndp   , Cpu::cop2_vmulq   , Cpu::cop2_vmudl   , Cpu::cop2_vmudm   , Cpu::cop2_vmudn   , Cpu::cop2_vmudh   ,
   /* 001_ */   Cpu::cop2_vmacf   , Cpu::cop2_vmacu   , Cpu::cop2_vrndn   , Cpu::cop2_vmacq   , Cpu::cop2_vmadl   , Cpu::cop2_vmadm   , Cpu::cop2_vmadn   , Cpu::cop2_vmadh   ,
   /* 010_ */   Cpu::cop2_vadd    , Cpu::cop2_vsub    , Cpu::cop2_vweird  , Cpu::cop2_vabs    , Cpu::cop2_vaddc   , Cpu::cop2_vsubc   , Cpu::cop2_vweird  , Cpu::cop2_vweird  ,
   /* 011_ */   Cpu::cop2_vweird  , Cpu::cop2_vweird  , Cpu::cop2_vweird  , Cpu::cop2_vweird  , Cpu::cop2_vweird  , Cpu::cop2_vsar    , Cpu::cop2_vweird  , Cpu::cop2_vweird  ,
   /* 100_ */   Cpu::cop2_vlt     , Cpu::cop2_veq     , Cpu::cop2_vne     , Cpu::cop2_vge     , Cpu::cop2_vcl     , Cpu::cop2_vch     , Cpu::cop2_vcr     , Cpu::cop2_vmrg    ,
   /* 101_ */   Cpu::cop2_vand    , Cpu::cop2_vnand   , Cpu::cop2_vor     , Cpu::cop2_vnor    , Cpu::cop2_vxor    , Cpu::cop2_vnxor   , Cpu::cop2_vweird  , Cpu::cop2_vweird  ,
   /* 110_ */   Cpu::cop2_vrcp    , Cpu::cop2_vrcpl   , Cpu::cop2_vrcph   , Cpu::cop2_vmov    , Cpu::cop2_vrsq    , Cpu::cop2_vrsql   , Cpu::cop2_vrsqh   , Cpu::cop2_vnop    ,
   /* 111_ */   Cpu::cop2_vweird  , Cpu::cop2_vweird  , Cpu::cop2_vweird  , Cpu::cop2_vweird  , Cpu::cop2_vweird  , Cpu::cop2_vweird  , Cpu::cop2_vweird  , Cpu::cop2_vnop    
            ],
        };

        let _ = core.reset();
        core
    }

    fn reset(&mut self) -> Result<(), ReadWriteFault> {
        self.pc                 = 0;
        self.halted             = true;
        self.halted_self        = false;
        self.is_delay_slot      = false;
        self.next_is_delay_slot = false;
        self.gpr                = [0u32; 32];
        self.ccr                = [0u32; 4];
        self.rcp_high           = false;
        self.rcp_input          = 0;
        self.div_result         = 0;
        self.num_steps          = 0;
        self.process_task       = false;

        self.v    = [unsafe { _mm_setzero_si128() }; 32];
        self.vacc = _wmm512_setzero_si512();

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
        self.num_steps += 1;
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

        //info!(target: "RSP", "${:08X}: inst ${:08X} op=0b{:06b}", self.current_instruction_pc | 0x1000, inst, self.inst.op);

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
        error!(target: "RSP", "reserved instruction ${:03b}_{:03b} at pc=${:08X}", self.inst.op >> 3, self.inst.op & 0x07, self.current_instruction_pc);
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
        let imm = self.inst.signed_imm as u32;
        self.gpr[self.inst.rt] = rs.wrapping_add(imm);
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
        //info!(target: "RSP", "ori r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] | (self.inst.imm as u32);
        Ok(())
    }

    fn inst_xori(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] ^ (self.inst.imm as u32);
        Ok(())
    }

    fn inst_lui(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "lui r{}, ${:04X}", self.inst.rt, self.inst.imm);
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

        // trigger interrupt
        let shared_state = self.shared_state.read().unwrap();
        if shared_state.intbreak {
            self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_SP, InterruptUpdateMode::SetInterrupt)).unwrap();
            self.comms.break_cpu();
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
                    Cop0_DmaCache => {
                        let shared_state = self.shared_state.read().unwrap();
                        shared_state.dma_current_cache
                    },

                    Cop0_DmaDram => {
                        let shared_state = self.shared_state.read().unwrap();
                        shared_state.dma_current_dram
                    },

                    Cop0_DmaReadLength => {
                        let shared_state = self.shared_state.read().unwrap();
                        shared_state.dma_current_read_length
                    },

                    Cop0_DmaWriteLength => {
                        let shared_state = self.shared_state.read().unwrap();
                        shared_state.dma_current_write_length
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

                    Cop0_CmdStart => {
                        self.rdp.lock().unwrap().read_u32(0x0010_0000)?
                    },

                    Cop0_CmdEnd => {
                        self.rdp.lock().unwrap().read_u32(0x0010_0004)?
                    },

                    Cop0_CmdCurrent => {
                        self.rdp.lock().unwrap().read_u32(0x0010_0008)?
                    },

                    Cop0_CmdStatus => {
                        self.rdp.lock().unwrap().read_u32(0x0010_000C)?
                    },

                    Cop0_CmdClock => {
                        self.rdp.lock().unwrap().read_u32(0x0010_0010)?
                    },

                    _ => todo!("unhandled cop0 register read $c{}", self.inst.rd),
                };

                Ok(())
            },

            0b00_100 => { // MTC
                let mut val = self.gpr[self.inst.rt];

                // fix bits of the various registers
                // NOTE: if you change these, you should change write_register as well
                match self.inst.rd {
                    Cop0_DmaCache => {
                        //debug!(target: "RSP", "DMA RSP address set to ${:04X}", val & 0x1FFF);
                        let mut shared_state = self.shared_state.write().unwrap();
                        shared_state.dma_cache = val & 0x1FFF;
                        Ok(())
                    },

                    Cop0_DmaDram => {
                        //debug!(target: "RSP", "DMA DRAM address set to ${:04X}", val & 0x00FF_FFFF);
                        let mut shared_state = self.shared_state.write().unwrap();
                        shared_state.dma_dram = val & 0x00FF_FFFF;
                        Ok(())
                    },

                    Cop0_DmaReadLength => { // RDRAM -> I/DRAM
                        //debug!(target: "RSP", "DMA read length set to ${:04X}", val);
                        let mut shared_state = self.shared_state.write().unwrap();
                        shared_state.dma_read_length = val;

                        let length = ((val & 0x0FFF) | 0x07) + 1;
                        let count  = ((val >> 12) & 0xFF) + 1;
                        let skip   = (val >> 20) & 0xFFF;
                        shared_state.dma_total_size = count * length + (count - 1) * skip;

                        let dma_info = DmaInfo {
                            initiator     : "RSP-READ",
                            source_address: (shared_state.dma_dram) & !0x07,
                            dest_address  : (shared_state.dma_cache | 0x0400_0000) & !0x07,
                            count         : count,
                            length        : length,
                            source_stride : skip,
                            dest_mask     : 0x0000_0FFF,
                            dest_bits     : 0x0400_0000 | (shared_state.dma_cache & 0x1000),
                            completed     : Some(self.dma_completed_tx.clone()),
                            ..Default::default()
                        };

                        if shared_state.dma_busy {
                            if shared_state.dma_full.is_none() {
                                shared_state.dma_full = Some(dma_info);
                            }
                        } else {
                            // update the addresses as if the dma has been completed
                            shared_state.dma_current_cache        = (shared_state.dma_cache & 0x1000) | (((shared_state.dma_cache & !0x07) + shared_state.dma_total_size) & 0x0FFF);
                            shared_state.dma_current_dram         = (shared_state.dma_dram & !0x07) + shared_state.dma_total_size;
                            shared_state.dma_current_read_length  = 0xFF8;
                            shared_state.dma_current_write_length = 0xFF8;
                            shared_state.dma_busy = true;
                            self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                            self.comms.break_cpu();
                        }

                        Ok(())
                    },

                    Cop0_DmaWriteLength => {
                        //debug!(target: "RSP", "DMA write length set to ${:04X}", val);
                        let mut shared_state = self.shared_state.write().unwrap();
                        shared_state.dma_write_length = val;

                        let length = ((val & 0x0FFF) | 0x07) + 1;
                        let count  = ((val >> 12) & 0xFF) + 1;
                        let skip   = (val >> 20) & 0xFFF;
                        shared_state.dma_total_size = count * length + (count - 1) * skip;

                        let dma_info = DmaInfo {
                            initiator     : "RSP-WRITE",
                            source_address: (shared_state.dma_cache | 0x0400_0000) & !0x07,
                            dest_address  : (shared_state.dma_dram) & !0x07,
                            count         : count,
                            length        : length,
                            dest_stride   : skip,
                            source_mask   : 0x0000_0FFF,
                            source_bits   : 0x0400_0000 | (shared_state.dma_cache & 0x1000),
                            completed     : Some(self.dma_completed_tx.clone()),
                            ..Default::default()
                        };

                        if shared_state.dma_busy {
                            if shared_state.dma_full.is_none() {
                                shared_state.dma_full = Some(dma_info);
                            }
                        } else {
                            // update the addresses as if the dma has been completed
                            shared_state.dma_current_cache        = (shared_state.dma_cache & 0x1000) | (((shared_state.dma_cache & !0x07) + shared_state.dma_total_size) & 0x0FFF);
                            shared_state.dma_current_dram         = (shared_state.dma_dram & !0x07) + shared_state.dma_total_size;
                            shared_state.dma_current_read_length  = 0xFF8;
                            shared_state.dma_current_write_length = 0xFF8;
                            shared_state.dma_busy = true;
                            self.comms.start_dma_tx.as_ref().unwrap().send(dma_info).unwrap();
                            self.comms.break_cpu();
                        }

                        Ok(())
                    },

                    Cop0_Status => {
                        //info!(target: "RSP", "Cop0_Status write value ${:08X}", val);

                        // CLR_BROKE
                        if (val & 0x04) != 0 {
                            self.broke = false;
                            if let Some(broke_tx) = &self.broke_tx {
                                broke_tx.send(false).unwrap();
                            }

                            val &= !0x04;
                        }

                        // SET_HALT: halt the RSP
                        if (val & 0x02) != 0 {
                            info!(target: "RSP", "RSP halted self!");
                            self.halted = true;
                            self.halted_self = true;
                            let mut shared_state = self.shared_state.write().unwrap();
                            shared_state.halted_self = true;
                            val &= !0x02;
                        }

                        // SET_SSTEP: wtf
                        if (val & 0x40) != 0 {
                            warn!(target: "RSP", "Single step mode enabled");
                            val &= !0x40;
                        }

                        // SET_INTR
                        if (val & 0x10) != 0 {
                            self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_SP, InterruptUpdateMode::SetInterrupt)).unwrap();
                            self.comms.break_cpu();
                            val &= !0x10;
                        }

                        if (val & 0x1FF) != 0 { // TODO
                            todo!("wrote status bits ${val:08X}");
                        }

                        // CLR_INTR: ack SP interrupt
                        if (val & 0x08) != 0 {
                            todo!();
                        }

                        // loop over the signals
                        val >>= 9; // signals start at bit 9
                        if val != 0 {
                            let mut shared_state = self.shared_state.write().unwrap();
                            for i in 0..16 {
                                // i loops over 8 signals each CLEAR then SET bits
                                // so signal number is i/2, and SET when i is odd, otherwise clear
                                if (val & (1 << i)) == 0 { continue; }

                                // toggle the odd bit and if it is also set, do nothing
                                // (you cannot clear and set a signal at the same time)
                                if (val & (1 << (i ^ 0x01))) != 0 { continue; }

                                let signo = i >> 1;

                                if (i & 0x01) == 0 { // CLEAR signal
                                    //info!(target: "RSP", "clearing signal {}", signo);
                                    shared_state.signals &= !(1 << signo);
                                } else { // SET signal
                                    //info!(target: "RSP", "setting signal {}", signo);
                                    shared_state.signals |= 1 << signo;
                                }
                            }
                        }

                        Ok(())
                    },

                    Cop0_Semaphore => {
                        let mut shared_state = self.shared_state.write().unwrap();
                        shared_state.semaphore = false;
                        Ok(())
                    },

                    Cop0_CmdStart => {
                        self.rdp.lock().unwrap().write_u32(val, 0x0010_0000)?;
                        Ok(())
                    },

                    Cop0_CmdEnd => {
                        self.rdp.lock().unwrap().write_u32(val, 0x0010_0004)?;
                        Ok(())
                    },

                    Cop0_CmdCurrent => {
                        self.rdp.lock().unwrap().write_u32(val, 0x0010_0008)?;
                        Ok(())
                    },

                    Cop0_CmdStatus => {
                        self.rdp.lock().unwrap().write_u32(val, 0x0010_000C)?;
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
    fn v_insert_byte(src: &__m128i, byte: u8, element: u8) -> __m128i {
        unsafe { 
            // sure do which we had non-const versions...
            match element {
                 0 => _mm_insert_epi8(*src, byte as i32, 15),
                 1 => _mm_insert_epi8(*src, byte as i32, 14),
                 2 => _mm_insert_epi8(*src, byte as i32, 13),
                 3 => _mm_insert_epi8(*src, byte as i32, 12),
                 4 => _mm_insert_epi8(*src, byte as i32, 11),
                 5 => _mm_insert_epi8(*src, byte as i32, 10),
                 6 => _mm_insert_epi8(*src, byte as i32,  9),
                 7 => _mm_insert_epi8(*src, byte as i32,  8),
                 8 => _mm_insert_epi8(*src, byte as i32,  7),
                 9 => _mm_insert_epi8(*src, byte as i32,  6),
                10 => _mm_insert_epi8(*src, byte as i32,  5),
                11 => _mm_insert_epi8(*src, byte as i32,  4),
                12 => _mm_insert_epi8(*src, byte as i32,  3),
                13 => _mm_insert_epi8(*src, byte as i32,  2),
                14 => _mm_insert_epi8(*src, byte as i32,  1),
                15 => _mm_insert_epi8(*src, byte as i32,  0),
                 _ => _mm_setzero_si128(),
            }
        }
    }

    #[inline(always)]
    fn v_insert_short(src: &__m128i, short: u16, element: u8) -> __m128i {
        if element != 15 {
            Self::v_insert_byte(&Self::v_insert_byte(src, (short >> 8) as u8, element), short as u8, element + 1)
        } else {
            Self::v_insert_byte(&src, (short >> 8) as u8, element)
        }
    }

    #[inline(always)]
    fn v_insert_long(src: &__m128i, long: u32, element: u8) -> __m128i {
        let high_short = Self::v_insert_short(src, (long >> 16) as u16, element);
        match element {
            13 => Self::v_insert_byte(&high_short, (long >> 8) as u8, element + 2),
            14 => high_short,
            15 => high_short,
             _ => Self::v_insert_short(&high_short, long as u16, element + 2),
        }
    }

    #[inline(always)]
    fn v_insert_double(src: &__m128i, double: u64, element: u8) -> __m128i {
        let high_long = Self::v_insert_long(src, (double >> 32) as u32, element);
        match element {
             9 => Self::v_insert_byte(&Self::v_insert_short(&high_long, (double >> 16) as u16, element + 4),
                                      (double >> 8) as u8, element + 6),
            10 => Self::v_insert_short(&high_long, (double >> 16) as u16, element + 4),
            11 => Self::v_insert_byte(&high_long, (double >> 24) as u8, element + 4),
            12 => high_long,
            13 => high_long,
            14 => high_long,
            15 => high_long,
             _ => Self::v_insert_long(&high_long, double as u32, element + 4),
        }
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
    #[allow(dead_code)]
    fn v_as_u128(src: &__m128i) -> u128 {
        unsafe {
            (((_mm_extract_epi64(*src, 1) as u64) as u128) << 64)
              | ((_mm_extract_epi64(*src, 0) as u64) as u128)
        }
    }

    // get two u128 from __m128i (hi,lo)
    #[inline(always)]
    #[allow(dead_code)]
    fn v_as_u256(src: &__m256i) -> (u128, u128) {
        unsafe {
            let hi = (((_mm256_extract_epi64(*src, 3) as u64) as u128) << 64)
                      | ((_mm256_extract_epi64(*src, 2) as u64) as u128);
            let lo = (((_mm256_extract_epi64(*src, 1) as u64) as u128) << 64)
                      | ((_mm256_extract_epi64(*src, 0) as u64) as u128);
            (hi, lo)
        }
    }

    #[allow(dead_code)]
    fn v_print_vacc(&self) {
        unsafe {
            let v128 = _wmm512_cvtepi64_epi16(_wmm512_srli_epi64::<32>(self.vacc));
            let v256 = _wmm512_cvtepi64_epi32(self.vacc);

            let v = (((_mm_extract_epi16(v128, 7) as u16) as u64) << 32) | ((_mm256_extract_epi32(v256, 7) as u32) as u64);
            println!("vacc[{}]=${:04X}_${:04X}_${:04X}", 0, (v >> 32) as u16, (v >> 16) as u16, v as u16);
            let v = (((_mm_extract_epi16(v128, 6) as u16) as u64) << 32) | ((_mm256_extract_epi32(v256, 6) as u32) as u64);
            println!("vacc[{}]=${:04X}_${:04X}_${:04X}", 1, (v >> 32) as u16, (v >> 16) as u16, v as u16);
            let v = (((_mm_extract_epi16(v128, 5) as u16) as u64) << 32) | ((_mm256_extract_epi32(v256, 5) as u32) as u64);
            println!("vacc[{}]=${:04X}_${:04X}_${:04X}", 2, (v >> 32) as u16, (v >> 16) as u16, v as u16);
            let v = (((_mm_extract_epi16(v128, 4) as u16) as u64) << 32) | ((_mm256_extract_epi32(v256, 4) as u32) as u64);
            println!("vacc[{}]=${:04X}_${:04X}_${:04X}", 3, (v >> 32) as u16, (v >> 16) as u16, v as u16);
            let v = (((_mm_extract_epi16(v128, 3) as u16) as u64) << 32) | ((_mm256_extract_epi32(v256, 3) as u32) as u64);
            println!("vacc[{}]=${:04X}_${:04X}_${:04X}", 4, (v >> 32) as u16, (v >> 16) as u16, v as u16);
            let v = (((_mm_extract_epi16(v128, 2) as u16) as u64) << 32) | ((_mm256_extract_epi32(v256, 2) as u32) as u64);
            println!("vacc[{}]=${:04X}_${:04X}_${:04X}", 5, (v >> 32) as u16, (v >> 16) as u16, v as u16);
            let v = (((_mm_extract_epi16(v128, 1) as u16) as u64) << 32) | ((_mm256_extract_epi32(v256, 1) as u32) as u64);
            println!("vacc[{}]=${:04X}_${:04X}_${:04X}", 6, (v >> 32) as u16, (v >> 16) as u16, v as u16);
            let v = (((_mm_extract_epi16(v128, 0) as u16) as u64) << 32) | ((_mm256_extract_epi32(v256, 0) as u32) as u64);
            println!("vacc[{}]=${:04X}_${:04X}_${:04X}", 7, (v >> 32) as u16, (v >> 16) as u16, v as u16);
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

    fn v_left_shifted(&self, v: &__m128i, byte_shift: u8) -> __m128i {
        // left rotate and then mask out lowest bits to 0
        let rotated = self.v_left_rotated(v, byte_shift);

        // vector filled with a mask and return lower bytes zeroed
        unsafe { 
            let i = u128::MAX << (byte_shift * 8);
            _mm_and_si128(rotated, _mm_set_epi64x((i >> 64) as i64, i as i64))
        }
    }

    fn inst_lwc2(&mut self) -> Result<(), InstructionFault> {
        let lwc_op = self.inst.rd; // the Rd field contains a function code for which store operation is being done
        let offset = (((self.inst.v & 0x7F) | ((self.inst.v & 0x40) << 1)) as i8) as u32; // signed 7-bit offset
        let element = ((self.inst.v >> 7) & 0x0F) as u8;

        match lwc_op {
            0b00_000 => { // LBV vt[element], offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset) as usize;
                let byte = self.read_u8(address & 0x0FFF)?;
                let copy = self.v[self.inst.rt];
                self.v[self.inst.rt] = Self::v_insert_byte(&copy, byte, element);
            },

            0b00_001 => { // LSV vt[element], offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 1) as usize;
                let short = self.read_u16_wrapped(address & 0x0FFF)?;
                let copy = self.v[self.inst.rt];
                self.v[self.inst.rt] = Self::v_insert_short(&copy, short, element);
            },

            0b00_010 => { // LLV vt[element], offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 2) as usize;
                let long = self.read_u32_wrapped(address & 0x0FFF)?;
                let copy = self.v[self.inst.rt];
                self.v[self.inst.rt] = Self::v_insert_long(&copy, long, element);
            },

            0b00_011 => { // LDV vt[element], offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 3) as usize;
                let double = ((self.read_u32_wrapped(address & 0x0FFF)? as u64) << 32)
                                | (self.read_u32_wrapped((address + 4) & 0x0FFF)? as u64);
                let copy = self.v[self.inst.rt];
                self.v[self.inst.rt] = Self::v_insert_double(&copy, double, element);
            },

            0b00_100 => { // LQV vt, offset(base)
                //info!(target: "RSP", "lqv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                let size = 16 - (address & 0x0F);

                let b0 = self.read_u32_wrapped((address + 0) & 0x0FFF)? as u128;
                let b1 = if size <=  4 { 0 } else { self.read_u32_wrapped((address +  4) & 0x0FFF)? as u128 };
                let b2 = if size <=  8 { 0 } else { self.read_u32_wrapped((address +  8) & 0x0FFF)? as u128 };
                let b3 = if size <= 12 { 0 } else { self.read_u32_wrapped((address + 12) & 0x0FFF)? as u128 };
                let mut b = (b0 << 96) | (b1 << 64) | (b2 << 32) | b3;

                let v = &mut self.v[self.inst.rt];
                for i in 0..size {
                    let x = (b >> 120) as u8;
                    b <<= 8;
                    let n = element + (i as u8);
                    if n > 15 { break; }
                    *v = Self::v_insert_byte(v, x, n as u8);
                }

                //self.v[self.inst.rt as usize] = unsafe { _mm_set_epi32(b0, b1, b2, b3) };
                //.let rt = self.inst.rt;
                //.let rs = self.inst.rs;
                //.info!(target: "RSP", "LQV v{}, ${:04X}(r{}) // v{} = ${:04X}_{:04X}_{:04X}_{:04X}_{:04X}_{:04X}_{:04X}_{:04X}", rt, offset, rs, rt,
                //.    self.v_short(rt as usize, 0), self.v_short(rt as usize, 1), self.v_short(rt as usize, 2), self.v_short(rt as usize, 3),
                //.    self.v_short(rt as usize, 4), self.v_short(rt as usize, 5), self.v_short(rt as usize, 6), self.v_short(rt as usize, 7));
            },

            0b00_101 => { // LRV vt, offset(base)
                //info!(target: "RSP", "lrv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                let addr_base = address & !0x0F;
                let size = 16 - (address & 0x0F);

                let b0 = self.read_u32_wrapped((addr_base +  0) & 0x0FFF)? as u128;
                let b1 = self.read_u32_wrapped((addr_base +  4) & 0x0FFF)? as u128;
                let b2 = self.read_u32_wrapped((addr_base +  8) & 0x0FFF)? as u128;
                let b3 = self.read_u32_wrapped((addr_base + 12) & 0x0FFF)? as u128;
                let mut b = (b0 << 96) | (b1 << 64) | (b2 << 32) | b3;

                let v = &mut self.v[self.inst.rt];
                for i in size..16 {
                    let x = (b >> 120) as u8;
                    b <<= 8;
                    let n = element + (i as u8);
                    if n > 15 { break; }
                    *v = Self::v_insert_byte(v, x, n as u8);
                }
            },

            0b00_110 => { // LPV vt, offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 3) as usize;
                //info!(target: "RSP", "lpv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}, address = ${address:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let addr_base = address & !0x07;
                let addr_offset = address & 0x07;

                // load 8 consecutive bytes from memory to the upper byte of each element

                // we have to read individual bytes because elements have to be read and
                // wrap within a 128-aligned block of data
                // with the element as part of the address read, this also performs the
                // rotate into the register (v ends up having the data already rotated)
                let mut v: u64 = 0;
                for i in 0..8 {
                    v <<= 8;
                    let element_offset = (16 - element + i + addr_offset as u8) & 0x0F;
                    let src_addr = element_offset as usize + addr_base;
                    v |= self.read_u8(src_addr & 0x0FFF)? as u64;
                }
                
                self.v[self.inst.rt] = unsafe { 
                    // place 8 bytes in low i64
                    let m = _mm_set_epi64x(0, v as i64);

                    // mask into upper bytes (0x80 means output is set to 0)
                    _mm_shuffle_epi8(m, _mm_set_epi8(7, i8::MIN, 6, i8::MIN, 5, i8::MIN, 4, i8::MIN, 3, i8::MIN, 2, i8::MIN, 1, i8::MIN, 0, i8::MIN))
                };
            },

            0b00_111 => { // LUV vt, offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 3) as usize;
                //info!(target: "RSP", "luv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}, address = ${address:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let addr_base = address & !0x07;
                let addr_offset = address & 0x07;

                // load 8 consecutive bytes from memory to the upper byte of each element, and then shift all elements right 1
                // i.e., bytes are loaded into bit position 14

                // we have to read individual bytes because elements have to be read and
                // wrap within a 128-aligned block of data
                // with the element as part of the address read, this also performs the
                // rotate into the register (v ends up having the data already rotated)
                let mut v: u64 = 0;
                for i in 0..8 {
                    v <<= 8;
                    let element_offset = (16 - element + i + addr_offset as u8) & 0x0F;
                    let src_addr = element_offset as usize + addr_base;
                    v |= self.read_u8(src_addr & 0x0FFF)? as u64;
                }
                
                self.v[self.inst.rt] = unsafe { 
                    // place 8 bytes in low i64
                    let m = _mm_set_epi64x(0, v as i64);

                    // mask into upper bytes (0x80 means output is set to 0)
                    let m = _mm_shuffle_epi8(m, _mm_set_epi8(7, i8::MIN, 6, i8::MIN, 5, i8::MIN, 4, i8::MIN, 3, i8::MIN, 2, i8::MIN, 1, i8::MIN, 0, i8::MIN));

                    // shift everything right 1
                    _mm_srli_epi16(m, 1)
                };
            },

            0b01_000 => { // LHV vt, offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                //info!(target: "RSP", "lhv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}, address = ${address:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let addr_base = address & !0x07;
                let addr_offset = address & 0x07;

                // load 8 (every second-)bytes from memory to the upper byte of each element, and then shift all elements right 1
                // i.e., bytes are loaded into bit position 14

                // we have to read individual bytes because elements have to be read and
                // wrap within a 128-aligned block of data
                // with the element as part of the address read, this also performs the
                // rotate into the register (v ends up having the data already rotated)
                let mut v: u64 = 0;
                for i in 0..8 {
                    v <<= 8;
                    let element_offset = (16 - element + (i << 1) + addr_offset as u8) & 0x0F;
                    let src_addr = element_offset as usize + addr_base;
                    v |= self.read_u8(src_addr & 0x0FFF)? as u64;
                }
                
                self.v[self.inst.rt] = unsafe { 
                    // place 8 bytes in low i64
                    let m = _mm_set_epi64x(0, v as i64);

                    // mask into upper bytes (0x80 means output is set to 0)
                    let m = _mm_shuffle_epi8(m, _mm_set_epi8(7, i8::MIN, 6, i8::MIN, 5, i8::MIN, 4, i8::MIN, 3, i8::MIN, 2, i8::MIN, 1, i8::MIN, 0, i8::MIN));

                    // shift everything right 1
                    _mm_srli_epi16(m, 1)
                };
            },

            0b01_001 => { // LFV vt, offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                //info!(target: "RSP", "lfv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}, address = ${address:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let addr_base = address & !0x07;
                let addr_offset = address & 0x07;

                const fixed_offsets: [i8; 8] = [ 0, 4, 8, 12, 8, 12, 0, 4 ];
                const element_mod: [i8; 8] = [ 1, -1, -1, -1, -1, -1, -1, -1 ];

                // load 8 (every second-)bytes from memory to the upper byte of each element, and then shift all elements right 1
                // i.e., bytes are loaded into bit position 14

                // we have to read individual bytes because elements have to be read and
                // wrap within a 128-aligned block of data
                // with the element as part of the address read, this also performs the
                // rotate into the register (v ends up having the data already rotated)
                let mut v: u64 = 0;
                for i in 0..8 {
                    v <<= 8;
                    let element_offset = (addr_offset as i8 + fixed_offsets[i] + element_mod[i] * element as i8) & 0x0F;
                    let src_addr = element_offset as usize + addr_base;
                    v |= self.read_u8(src_addr & 0x0FFF)? as u64;
                }
                
                let tmp = unsafe { 
                    // place 8 bytes in low i64
                    let m = _mm_set_epi64x(0, v as i64);

                    // mask into upper bytes (0x80 means output is set to 0)
                    let m = _mm_shuffle_epi8(m, _mm_set_epi8(7, i8::MIN, 6, i8::MIN, 5, i8::MIN, 4, i8::MIN, 3, i8::MIN, 2, i8::MIN, 1, i8::MIN, 0, i8::MIN));

                    // shift everything right 1
                    _mm_srli_epi16(m, 1)
                };

                let size = std::cmp::min(8, 16 - element);

                // so tmp has the 4 shorts we want at byte offset `element`, and we can only write a maximum of 4 shorts
                // the lower bytes are zeroed out and we end up with
                // 0xAABB_CCDD_EEFF_GGHH_0000_0000_0000 or less, like 
                // 0xAABB_CC00_0000_0000_0000_0000_0000
                let tmp = if size < 8 {
                    // for less than 8 bytes, we can just shift filling in zeros
                    self.v_left_shifted(&tmp, element)
                } else {
                    // for 8 or more, we have to clear bits by rotating the entire vec and shifting in zeroes
                    self.v_left_shifted(&self.v_left_rotated(&tmp, 8 + element), 8)
                };

                // now we need the opposite bytes masked in from the source vector
                // clear `size` bytes starting at byte offset element, and then rotate into position 16 - `size`
                let vt = self.v_left_rotated(&self.v_left_shifted(&self.v_left_rotated(&self.v[self.inst.rt], element), size), 16 - size);

                // now we OR them together, and perform a final shift into the correct location
                let tmp = unsafe { _mm_or_si128(tmp, vt) };
                self.v[self.inst.rt] = self.v_left_rotated(&tmp, 16 - element);
            },

            0b01_010 => { // LWV vt, offset(base) -- NOP
            },

            0b01_011 => { // LTV vt, offset(base)
                let address = self.gpr[self.inst.rs].wrapping_add(offset << 4) as usize;
                //info!(target: "RSP", "ltv v{}[{}], ${:04X}(r{}) // r{} = ${:08X}, address = ${address:08X}", self.inst.rt, element, offset, self.inst.rs, self.inst.rs, self.gpr[self.inst.rs]);
                let addr_base = address & !0x07;
                let v_base = self.inst.rt & !0x07;

                // every other vector in memory is rotated by 8 bytes rather than use the true address offset
                let addr_offset = if (addr_base & 0x08) != 0 { 8 } else { 0 };

                for i in 0..8 {
                    // can't use read_u16_wrapped because we need to wrap within a 16 byte block, 
                    // so read upper and lower bytes individually
                    let offset = (addr_offset + (i * 2) as usize + element as usize) & 0x0F;
                    let b0 = self.read_u8((addr_base + ((offset    )       )) & 0x0FFF)?;
                    let b1 = self.read_u8((addr_base + ((offset + 1) & 0x0F)) & 0x0FFF)?;

                    // dest register is current slice (i) shifted by selected element, wrapping within group of 8 VU regs
                    let register_index = v_base | ((i + (element >> 1)) & 0x07) as usize;

                    // insert the short into element i position
                    self.v[register_index] = Self::v_insert_short(&self.v[register_index], ((b0 as u16) << 8) | (b1 as u16), (i * 2) as u8);
                }
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
            self.inst_e  = ((self.inst.v >> 21) & 0x0F) as u8;
            self.inst_vt = ((self.inst.v >> 16) & 0x1F) as usize;
            self.inst_vs = ((self.inst.v >> 11) & 0x1F) as usize;
            self.inst_vd = ((self.inst.v >>  6) & 0x1F) as usize;
            self.cop2_table[func as usize](self)
        } else {
            let cop2_op = (self.inst.v >> 21) & 0x1F;
            match cop2_op {
                0b00_000 => { // MFC2 rt, vd[e]
                    let src = &self.v[self.inst.rd];
                    let rt = &mut self.gpr[self.inst.rt];
                    let element = (self.inst.v >> 7) & 0x0F;
                    *rt = (((Self::v_byte(src, element as u8) as i16) << 8) 
                            | (Self::v_byte(src, ((element + 1) & 0x0F) as u8) as i16)) as u32;
                },

                0b00_100 => { // MTC2 rt, vd[e]
                    let rt = self.gpr[self.inst.rt] as u16;
                    let copy = self.v[self.inst.rd];
                    let element = (self.inst.v >> 7) & 0x0F;
                    self.v[self.inst.rd] = Self::v_insert_short(&copy, rt, element as u8);
                },

                0b00_010 => { // CFC2
                    let mut rd = self.inst.rd & 0x03;
                    if rd >= Cop2_VCE { 
                        rd = Cop2_VCE; 
                    }
                    let s = self.ccr[rd];
                    self.gpr[self.inst.rt] = s;
                },

                0b00_110 => { // CTC2
                    let mut s = self.gpr[self.inst.rt] as i16;    // truncate to 16 bits
                    let mut rd = self.inst.rd & 0x03;
                    if rd >= Cop2_VCE { 
                        rd = Cop2_VCE; 
                        s &= 0xFF; // truncate to 8 bits
                    }
                    self.ccr[rd] = s as u32; // sign-extend
                },

                _ => {
                    error!(target: "RSP", "unimplemented RSP COP2 instruction 0b{:02b}_{:03b}", cop2_op >> 3, cop2_op & 0x07);
                    todo!();
                    //Ok(())
                }
            };

            Ok(())
        }
    }

    fn v_math_elements(src: &__m128i, e: u8) -> __m128i {
        if e == 0 || e == 1 { 
            // return src as-is
            *src 
        } else if (e & 0x0E) == 0x02 {
            // src {A,B,C,D,E,F,G,H} goes to either {A,A,C,C,E,E,G,G} or {B,B,D,D,F,F,H,H}
            if (e & 0x01) == 0 {
                unsafe {
                    _mm_shuffle_epi8(*src, _mm_set_epi8(15, 14, 15, 14,
                                                        11, 10, 11, 10,
                                                         7,  6,  7,  6,
                                                         3,  2,  3,  2))
                }
            } else {
                unsafe {
                    _mm_shuffle_epi8(*src, _mm_set_epi8(13, 12, 13, 12,
                                                         9,  8,  9,  8,
                                                         5,  4,  5,  4,
                                                         1,  0,  1,  0))
                }
            }
        } else if (e & 0x0C) == 0x04 {
            // src {A,B,C,D,E,F,G,H} goes to one of 
            //  {A,A,A,A,E,E,E,E} 
            //  {B,B,B,B,F,F,F,F} 
            //  {C,C,C,C,G,G,G,G} 
            //  {D,D,D,D,H,H,H,H} 
            match e & 0x03 {
                0 => unsafe {
                    _mm_shuffle_epi8(*src, _mm_set_epi8(15, 14, 15, 14, 15, 14, 15, 14,
                                                         7,  6,  7,  6,  7,  6,  7,  6))
                },
                1 => unsafe {
                    _mm_shuffle_epi8(*src, _mm_set_epi8(13, 12, 13, 12, 13, 12, 13, 12,
                                                         5,  4,  5,  4,  5,  4,  5,  4))
                },
                2 => unsafe {
                    _mm_shuffle_epi8(*src, _mm_set_epi8(11, 10, 11, 10, 11, 10, 11, 10,
                                                         3,  2,  3,  2,  3,  2,  3,  2))
                },
                3 => unsafe {
                    _mm_shuffle_epi8(*src, _mm_set_epi8( 9,  8,  9,  8,  9,  8,  9,  8,
                                                         1,  0,  1,  0,  1,  0,  1,  0))
                },
                _ => unsafe { _mm_setzero_si128() },
            }
        } else if (e & 0x08) == 0x08 {
            // broadcast element of src to all lanes
            let i = 14 - (((e & 0x07) as i16) << 1); // i = 14,12,10,8,6,4,2,0
            let j = ((i + 1) << 8) | i;              // 0x0k0i where k is i + 1
            unsafe { _mm_shuffle_epi8(*src, _mm_set1_epi16(j)) }
        } else {
            todo!(); // TODO not in datasheet but probably needs to be implemented
        }
    }

    #[inline(always)]
    fn v_splat_vco256(&self) -> __m256i {
        let zeroes = unsafe { _mm256_setzero_si256() };

        // create 8x32-bit value where carry bit in VCC selects a 1 in the output lane
        _wmm256_mask_blend_epi32(self.ccr[Cop2_VCO] as __mmask8, zeroes, self.v256_ones)
    }

    #[inline(always)]
    fn v_set_accumulator_lo(&mut self, v: &__m128i) {
        self.vacc = {
            // _mm512_slli_epi64: shift bitmask left 16 to get 0xFFFF_FFFF_FFFF_0000 in each channel
            let shifted_mask = _wmm512_slli_epi64::<16>(self.vacc_mask);

            // _mm512_and_epi64: clear lower 16 bits of acc and store result in acc
            let cleared_vacc = _wmm512_and_epi64(self.vacc, shifted_mask);

            // _mm512_cvtepi16_epi64: zero extend epi16 to epi64 
            let v            = _wmm512_cvtepu16_epi64(*v);

            // _mm512_or_epi64: OR in lower 16 bits into accumulataor
            _wmm512_or_epi64(cleared_vacc, v)
        };
    }

    // Set a 32-bit value into the high and mid values of the accumulator
    #[inline(always)]
    fn v_set_accumulator_highmid(&mut self, v: &__m256i) {
        self.vacc = {
            // _mm512_srli_epi64: shift bitmask right 48 to get 0x0000_0000_0000_FFFF
            let shifted_mask = _wmm512_srli_epi64::<48>(self.vacc_mask);

            // _mm512_and_epi64: clear upper high and mid (upper 48, actually) bits of acc and store result in acc
            let cleared_vacc = _wmm512_and_epi64(self.vacc, shifted_mask);

            // _mm512_cvtepu32_epi64: zero-extend epi32 to epi64
            // _mm512_slli_epi64: shift input left 16 bits to have 0xSSSS_XXXX_XXXX_0000, where S is the sign bit
            let v = _wmm512_slli_epi64::<16>(_wmm512_cvtepu32_epi64(*v));

            // _mm512_or_epi64: OR in high mid bits into accumulator
            _wmm512_or_epi64(cleared_vacc, v)
        };
    }

    // shift vacc right 16 (placing high/mid in the lower 32-bits), and then truncate to __m256i 8x32-bit
    #[inline(always)]
    fn v_get_accumulator_highmid(&mut self) -> __m256i {
        _wmm512_cvtepi64_epi32(_wmm512_srli_epi64::<16>(self.vacc))
    }

    // truncate vacc to __m256i 8x32-bit
    #[inline(always)]
    #[allow(dead_code)]
    fn v_get_accumulator_midlow(&mut self) -> __m256i {
        _wmm512_cvtepi64_epi32(self.vacc)
    }

    // truncate vacc to __m128 8x16-bit
    #[inline(always)]
    fn v_get_accumulator_low(&mut self) -> __m128i {
        _wmm512_cvtepi64_epi16(self.vacc)
    }

    // saturate 48-bit accumulator down to 16-bits using the low short as a result when truncating
    #[inline(always)]
    fn v_accumulator_saturate_low(&mut self) -> __m128i {
        // _mm512_cvtsepi64_epi16 doesn't do what we want
        let highmid = self.v_get_accumulator_highmid();
        let low     = self.v_get_accumulator_low();

        // saturate the low 16-bit value of the accumulator, but we need to use the sign of the high lane
        unsafe { 
            let high      = _wmm256_cvtepi32_epi16(_mm256_srli_epi32(highmid, 16)); // extract high lane
            let high_neg  = _wmm_cmplt_epi16_mask(high, _mm_setzero_si128());   // set to 1 if vacc high is negative
            let high_neg1 = _wmm_cmpeq_epi16_mask(high, _mm_set1_epi16(-1i16)); // set to 1 if vacc high is 0xFFFF
            let high_zero = _wmm_cmpeq_epi16_mask(high, _mm_setzero_si128());   // set to 1 if vacc high is 0

            let mid       = _wmm256_cvtepi32_epi16(highmid); // extract mid lane
            let mid_neg   = _wmm_cmplt_epi16_mask(mid, _mm_setzero_si128()); // set to 1 if mid is negative
            
            // if high is negative but highmid is 0xFFFF8000 the result is saturated to zero, otherwise truncate
            let hm_neg = (!high_neg1) | (!mid_neg);
            let hm_neg_result = _wmm_mask_blend_epi16(!hm_neg, _mm_setzero_si128(), low);
            
            // if highmid is less than 0x00008000 saturate to -1
            let hm_small = (!high_zero) | mid_neg;
            let hm_small_result = _wmm_mask_blend_epi16(!hm_small, _mm_set1_epi16(-1i16), low);

            // pick hm_neg or hm_small, otherwise truncate
            _wmm_mask_blend_epi16(!high_neg, hm_neg_result, hm_small_result)
        }
    }

    fn cop2_vmov(&mut self) -> Result<(), InstructionFault> {
        let de = ((self.inst.v >> 11) & 0x1F) as u8;
        //info!(target: "RSP", "vmov v{}[0b{:05b}],  v{}[0b{:04b}]", self.inst_vd, de, self.inst_vt, self.inst_e);
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);

        // set ACC lo to vt[e]
        self.v_set_accumulator_lo(&right);

        // move lane [de] from vt[e] to vs
        let de = de & 7;
        self.v[self.inst_vd] = Self::v_insert_short(&self.v[self.inst_vd], Self::v_short(&right, de), de * 2);

        Ok(())
    }

    fn cop2_vsar(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vsar v{}[0b{:04b}]", self.inst_vd, self.inst_e);
        self.v[self.inst_vd] = match self.inst_e {
            8 => { // move high 16 bits of vacc to vd
                // _mm512_srli_epi64: shift acc to the right 32 bits
                // _mm512_cvtepi64_epi16: truncate epi64 to epi16
                _wmm512_cvtepi64_epi16(_wmm512_srli_epi64::<32>(self.vacc))
            },

            9 => { // move mid 16 bits of vacc to 
                // _mm512_srli_epi64: shift acc to the right 16 bits
                // _mm512_cvtepi64_epi16: truncate epi64 to epi16
                _wmm512_cvtepi64_epi16(_wmm512_srli_epi64::<16>(self.vacc))
            },

            10 => { // move low 16 bits of vacc to 
                // _mm512_cvtepi64_epi16: truncate epi64 to epi16
                _wmm512_cvtepi64_epi16(self.vacc)
            },

            // all others set vd to 0
            _ => unsafe { _mm_setzero_si128() },
        };
        Ok(())
    }

    // add the parameters and only store the accumulator, setting the result to 0
    fn cop2_vweird(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vweird v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);

        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        let dst = &mut self.v[self.inst_vd];

        unsafe { 
            *dst = _mm_setzero_si128();

            // sign-extend to eight 32-bits ints (so we can correctly calculate the accumulator and saturate with one vector op)
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);
            let result256 = _mm256_add_epi32(left256, right256); // the actual op: rs + rt(e)

            // for the accumulator, truncate to 16-bits and then covert to __m512i and store it into acc low

            // truncate result256 to 16bit and zero-extend to 64-bit
            let result512 = _wmm512_cvtepu16_epi64(_wmm256_cvtepi32_epi16(result256));

            // _mm512_slli_epi64: shift bitmask left 16 to get 0xFFFF_FFFF_FFFF_0000 in each channel
            let shifted_mask = _wmm512_slli_epi64::<16>(self.vacc_mask);

            // _mm512_and_epi64: clear lower 16 bits of acc
            let cleared_vacc = _wmm512_and_epi64(self.vacc, shifted_mask);

            // _mm512_or_epi64: OR in lower 16 bits into acc
            self.vacc = _wmm512_or_epi64(cleared_vacc, result512);
        };
        Ok(())
    }

    fn cop2_vadd(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vadd v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);

        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        let carry_in = self.v_splat_vco256();
        let dst = &mut self.v[self.inst_vd];

        let result128 = unsafe { 
            // sign-extend to eight 32-bits ints (so we can correctly calculate the accumulator and saturate with one vector op)
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);
            let result256 = _mm256_add_epi32(_mm256_add_epi32(left256, right256), carry_in); // the actual op: rs + rt(e) + vcc

            // saturate 8 32-bit ints using _mm256_cvtsepi32_epi16 (nightly api, not yet available) into 8 16-bit ints
            // uses vpmovsdw until _mm256_cvtsepi32_epi16 makes it out of nightly
            *dst = _wmm256_cvtsepi32_epi16(result256); // _mm256_cvtsepi32_epi16: saturate down to epi16

            // for the accumulator, truncate result to 16-bits and store in low 16-bits of acc
            _wmm256_cvtepi32_epi16(result256)
        };

        self.v_set_accumulator_lo(&result128);

        self.ccr[Cop2_VCO] &= !0xFFFF; // clear VCO
        Ok(())
    }

    fn cop2_vaddc(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vaddc v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);

        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));
        let carry_bits: u32;

        self.v[self.inst_vd] = unsafe { 
            // zero-extend to eight 32-bits ints and perform add
            let left256  = _mm256_cvtepu16_epi32(left);
            let right256 = _mm256_cvtepu16_epi32(right);
            let result256 = _mm256_add_epi32(left256, right256); // the actual op: rs + rt(e)

            // the result and accumulator are the same result with VADDC
            // truncate to 16bit
            let result128 = _wmm256_cvtepi32_epi16(result256);

            // To compute VCO we need bit 16 after the add, so we want bit 0x00010000 of each lane into VCO
            let result256 = _mm256_srli_epi32(_mm256_and_si256(result256, _mm256_set1_epi32(0x0001_0000i32)), 1);

            carry_bits = _pext_u32(_mm256_movemask_epi8(result256) as u32, 0x2222_2222) & 0xFF; // extract carry bits into lower concatenated bits

            result128
        };

        let copy = self.v[self.inst_vd];
        self.v_set_accumulator_lo(&copy);

        // bits in carry_bits need to be reversed
        self.ccr[Cop2_VCO] = (carry_bits as u8).reverse_bits() as u32; // clear high bits
        Ok(())
    }

    fn cop2_vsub(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vsub v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);

        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        let carry_in = self.v_splat_vco256();
        let dst = &mut self.v[self.inst_vd];

        let result128 = unsafe { 
            // sign-extend to eight 32-bits ints (so we can correctly calculate the accumulator and saturate with one vector op)
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);
            let result256 = _mm256_sub_epi32(_mm256_sub_epi32(left256, right256), carry_in); // the actual op: rs - rt(e) - vcc

            // saturate 8 32-bit ints using _mm256_cvtsepi32_epi16 into 8 16-bit ints
            *dst = _wmm256_cvtsepi32_epi16(result256);

            // truncate the result for the accumulator
            _wmm256_cvtepi32_epi16(result256)
        };

        // the accumulator low lane gets the truncated result
        self.v_set_accumulator_lo(&result128);

        self.ccr[Cop2_VCO] &= !0xFFFF; // clear VCO
        Ok(())
    }

    fn cop2_vsubc(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vsubc v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);

        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));
        let dst = &mut self.v[self.inst_vd];
        let carry_bits: u32;
        let zero_bits: u32;

        unsafe { 
            // zero-extend to eight 32-bits ints and perform add
            let left256  = _mm256_cvtepu16_epi32(left);
            let right256 = _mm256_cvtepu16_epi32(right);
            let result256 = _mm256_sub_epi32(left256, right256); // the actual op: rs - rt(e)

            // truncate for the result
            *dst = _wmm256_cvtepi32_epi16(result256);

            // VCO low bit depends on if the result is negative (bit 0x8000_0000 set)
            // we can use a movemask and just take that bit into carry low
            carry_bits = _pext_u32(_mm256_movemask_epi8(result256) as u32, 0x8888_8888) & 0xFF; // extract carry bits into lower concatenated bits

            // VCO high on the other hand depends on the 32-bit result being nonzero
            let zero_cmp = _mm256_cmpeq_epi32(result256, _mm256_setzero_si256()); // -1 if 0, 0 if not 0

            // if high bit is set in zero_cmp, then value is zero and vco high should be clear
            zero_bits = _pext_u32(!(_mm256_movemask_epi8(zero_cmp) as u32), 0x8888_8888) & 0xFF; // extract zero bits into lower concatenated bits
        };

        // the accumulator low gets a copy of result
        let copy = self.v[self.inst_vd];
        self.v_set_accumulator_lo(&copy);

        // both carry_bits and zero_bits need to be reversed
        self.ccr[Cop2_VCO] = (((zero_bits as u8).reverse_bits() as u32) << 8) | (carry_bits as u8).reverse_bits() as u32;
        Ok(())
    }

    fn cop2_vabs(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vabs v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);

        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        let dst = &mut self.v[self.inst_vd];

        let result = unsafe { 
            // sign performs almost exactly the same operation as VABS, but we have to correct one special case...
            let result = _mm_sign_epi16(right, left);

            // if a lane in left is negative and the corresponding right value is 0x8000 exactly, set the result to 0x7FFF
            // (left & 0x8000) = 0x8000 means left is negative
            let left_cmp = _mm_and_si128(left, _mm_set1_epi16(0x8000u16 as i16));
            // (right == 0x8000 ? 0xFFFF : 0) gives us 0xFFFF if right is exactly 0x8000
            let right_cmp = _mm_cmpeq_epi16(right, _mm_set1_epi16(0x8000u16 as i16));
            // AND together gives us 0x8000 or 0
            let cmp = _mm_and_si128(left_cmp, right_cmp); // 0x8000 or 0x0000
            // shifting and oring gives us 0x8080 or 0
            let cmp = _mm_or_si128(cmp, _mm_srli_epi16(cmp, 8)); // 0x8080 or 0x0000
            // and then blending between the result and 0x7FFF selects 0x7FFF when that particular condition is satisfied
            *dst = _mm_blendv_epi8(result, _mm_set1_epi16(0x7FFF), cmp); // replace with 0x7FFF

            result
        };

        self.v_set_accumulator_lo(&result);

        Ok(())
    }


    fn v_compare<T: FnOnce(__m128i, __m128i, u8, u8) -> (__m128i, u8)>(&mut self, cmp: T) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vlt v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);

        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let (result, result_mask) = cmp(left, right, (self.ccr[Cop2_VCO] >> 8) as u8, self.ccr[Cop2_VCO] as u8);

        // store the result
        self.v[self.inst_vd] = result;

        // ACC LO gets a copy of the result
        self.v_set_accumulator_lo(&result);

        // bits in result_mask need to be reversed
        self.ccr[Cop2_VCC] = result_mask.reverse_bits() as u32;
        self.ccr[Cop2_VCO] = 0;

        Ok(())
    }

    fn v_logical<T: FnOnce(__m128i, __m128i) -> __m128i>(&mut self, op: T) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vlt v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);

        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let result = op(left, right);

        // store the result
        self.v[self.inst_vd] = result;

        // ACC LO gets a copy of the result
        self.v_set_accumulator_lo(&result);

        Ok(())
    }

    fn cop2_vlt(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vlt v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);
        self.v_compare(|left, right, vco_high, vco_low| {
            let use_eq = vco_high & vco_low;
            let result_mask: u8;

            let result = { 
                // compare less than on all fields, and leq on only the fields that have both VCO bits set
                let lt_mask = _wmm_cmplt_epi16_mask(left, right);
                let eq_mask = _wmm_cmpeq_epi16_mask(left, right);
                result_mask = lt_mask | (eq_mask & use_eq);
                _wmm_mask_blend_epi16(result_mask, right, left)
            };

            (result, result_mask)
        })
    }

    fn cop2_veq(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "veq v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);
        self.v_compare(|left, right, vco_high, _| {
            let result_mask: u8;
            let result = { 
                let eq_mask = _wmm_cmpeq_epi16_mask(left, right);
                result_mask = eq_mask & !vco_high;
                right
            };
            (result, result_mask)
        })
    }

    fn cop2_vne(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vne v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);
        self.v_compare(|left, right, vco_high, _| {
            let result_mask: u8;
            let result = { 
                let eq_mask = _wmm_cmpneq_epi16_mask(left, right);
                result_mask = eq_mask | vco_high;
                left
            };
            (result, result_mask)
        })
    }

    fn cop2_vge(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vge v{}, v{}, v{}[0b{:04b}] // VCO=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO]);
        self.v_compare(|left, right, vco_high, vco_low| {
            let use_eq = !(vco_high & vco_low);
            let result_mask: u8;
            let result = { 
                let gt_mask = _wmm_cmpgt_epi16_mask(left, right);
                let eq_mask = _wmm_cmpeq_epi16_mask(left, right);
                result_mask = gt_mask | (eq_mask & use_eq);
                _wmm_mask_blend_epi16(result_mask, right, left)
            };
            (result, result_mask)
        })
    }

    fn cop2_vmrg(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmrg v{}, v{}, v{}[0b{:04b}] // VCC=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCC]);

        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        // bits in merge_mask need to be reversed and inverted, but we swap right/left in the blend
        let merge_mask = (self.ccr[Cop2_VCC] as u8).reverse_bits();
        let result = _wmm_mask_blend_epi16(merge_mask, right, left);

        // store the result
        self.v[self.inst_vd] = result;

        // ACC LO gets a copy of the result
        self.v_set_accumulator_lo(&result);

        // VCO is cleared
        self.ccr[Cop2_VCO] = 0;

        Ok(())
    }

    fn cop2_vand(&mut self) -> Result<(), InstructionFault> {
        self.v_logical(|left, right| { unsafe { _mm_and_si128(left, right) } })
    }

    fn cop2_vnand(&mut self) -> Result<(), InstructionFault> {
        self.v_logical(|left, right| { unsafe { _mm_xor_si128(_mm_set1_epi8(0xFFu8 as i8), _mm_and_si128(left, right)) } })
    }

    fn cop2_vor(&mut self) -> Result<(), InstructionFault> {
        self.v_logical(|left, right| { unsafe { _mm_or_si128(left, right) } })
    }

    fn cop2_vnor(&mut self) -> Result<(), InstructionFault> {
        self.v_logical(|left, right| { unsafe { _mm_xor_si128(_mm_set1_epi8(0xFFu8 as i8), _mm_or_si128(left, right)) } })
    }

    fn cop2_vxor(&mut self) -> Result<(), InstructionFault> {
        self.v_logical(|left, right| { unsafe { _mm_xor_si128(left, right) } })
    }

    fn cop2_vnxor(&mut self) -> Result<(), InstructionFault> {
        self.v_logical(|left, right| { unsafe { _mm_xor_si128(_mm_set1_epi8(0xFFu8 as i8), _mm_xor_si128(left, right)) } })
    }

    fn cop2_vnop(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn cop2_vcl(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vcl v{}, v{}, v{}[0b{:04b}] // VCO=${:04X} VCC=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO], self.ccr[Cop2_VCC]);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let vcc_low  = self.ccr[Cop2_VCC] as u8;
        let vcc_high = (self.ccr[Cop2_VCC] >> 8) as u8;
        let vco_low  = self.ccr[Cop2_VCO] as u8;
        let vco_high = (self.ccr[Cop2_VCO] >> 8) as u8;
        let vce      = self.ccr[Cop2_VCE] as u8;

        let (result, vcc_low, vcc_high) = unsafe { 
            //...if a vco_low bit is set...
            //add up the elements to see if we get zero or overflow
            let left256   = _mm256_cvtepu16_epi32(left); //zero-extend to 32-bits and add (all values are positive now)
            let right256  = _mm256_cvtepu16_epi32(right);
            let result256 = _mm256_add_epi32(left256, right256); // the actual op: rs + rt(e)
            let sum       = _wmm256_cvtepi32_epi16(result256); // truncate to 16-bit for the sum
            let zero_sum  = (_wmm_cmpeq_epi16_mask(sum, _mm_setzero_si128()) as u8).reverse_bits(); // compare sum to zero, create a mask
            let carry     = (_wmm256_cmpge_epi32_mask(result256, _mm256_set1_epi32(0x10000u32 as i32)) as u8).reverse_bits(); // bitmask where 1 indicates carry occurred
            //println!("sum=${:032X} zero_sum=0b{:08b} carry=0b{:08b}", Self::v_as_u128(&sum), zero_sum, carry);

            // new vcc low contains the equal/less-equal sign
            let mut new_vcc_low = (zero_sum & !carry) | (vce & (zero_sum | !carry));
            // new vcc low output is only set when vco_high of corresponding bit is clear, so we mask out those bits
            // and keep the old vcc_low bits
            new_vcc_low = (new_vcc_low & !vco_high) | (vcc_low & vco_high);

            // the output of when original vco_low is set (sign bit set) yields either -vt or vs depending on the result of le (new_vcc_low)
            let vco_low_set_result = _wmm_mask_blend_epi16(new_vcc_low.reverse_bits(), left, _mm_sub_epi16(_mm_setzero_si128(), right));

            //...if a vco_low bit is clear...

            // new vcc high contains the greater-than-or-equal result between vs and vt
            let mut new_vcc_high = (_wmm_cmpge_epu16_mask(left, right) as u8).reverse_bits();
            // but only gets compared when vco_high is clear, so mask out those bits and keep the old vcc_high bits
            new_vcc_high = (new_vcc_high & !vco_high) | (vcc_high & vco_high);

            // the output of when original vco_low is clear (sign bit clear) yields either vs or vt depending on the ge flag (new_vcc_high)
            let vco_low_clear_result = _wmm_mask_blend_epi16(new_vcc_high.reverse_bits(), left, right);

            // the final result is picking the elements based on vco_low (sign bit)
            let result = _wmm_mask_blend_epi16(vco_low, vco_low_clear_result, vco_low_set_result);
            new_vcc_low  = (new_vcc_low  &  vco_low) | (vcc_low  & !vco_low);   // select new_vcc_low when vco_low is set, otherwise keep old vcc_low
            new_vcc_high = (new_vcc_high & !vco_low) | (vcc_high &  vco_low);   // select new_vcc_high when vco_low is clear, otherwise keep old vcc_high

            (result, new_vcc_low, new_vcc_high)
        };

        // low bits of accumulator get a copy of the result
        self.v[self.inst_vd] = result;
        self.v_set_accumulator_lo(&result);

        // VCO and VCE are cleared
        self.ccr[Cop2_VCO] = 0;
        self.ccr[Cop2_VCE] = 0;

        // set VCC to the result of the comparisons
        self.ccr[Cop2_VCC] = ((vcc_high as u32) << 8) | (vcc_low as u32);

        Ok(())
    }

    fn cop2_vch(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vch v{}, v{}, v{}[0b{:04b}] // VCO=${:04X} VCC=${:04X} VCE=${:02X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, 
        //                                                                                           self.ccr[Cop2_VCO], self.ccr[Cop2_VCC], self.ccr[Cop2_VCE]);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let (new_vco_low, vt_lt_zero, vt_ne_not_vs, sum_le_zero, diff_ge_zero, sum_ne_zero, diff_ne_zero, sum_eq_neg1);

        let result = unsafe {
            // first compare vs^vt < 0
            new_vco_low = (_wmm_cmplt_epi16_mask(_mm_xor_si128(left, right), _mm_setzero_si128()) as u8).reverse_bits();

            // comparisons between vs and vt
            vt_lt_zero = (_wmm_cmplt_epi16_mask(right, _mm_setzero_si128()) as u8).reverse_bits();
            vt_ne_not_vs = (_wmm_cmpneq_epi16_mask(right, _mm_xor_si128(left, _mm_set1_epi8(0xFFu8 as i8))) as u8).reverse_bits();

            // calculate sum and diff
            let sum  = _mm_add_epi16(left, right);
            let diff = _mm_sub_epi16(left, right);

            // sum and diff comparisons
            sum_le_zero  = (_wmm_cmple_epi16_mask(sum, _mm_setzero_si128()) as u8).reverse_bits();
            diff_ge_zero = (_wmm_cmpge_epi16_mask(diff, _mm_setzero_si128()) as u8).reverse_bits();
            sum_ne_zero  = (_wmm_cmpneq_epi16_mask(sum, _mm_setzero_si128()) as u8).reverse_bits();
            diff_ne_zero = (_wmm_cmpneq_epi16_mask(diff, _mm_setzero_si128()) as u8).reverse_bits();
            sum_eq_neg1  = (_wmm_cmpeq_epi16_mask(sum, _mm_set1_epi16(-1i16)) as u8).reverse_bits();

            // results...
            // if vco_low bit is set, we select either vs or -vt based on whether the sum of the two is negative
            let vco_low_set_result = _wmm_mask_blend_epi16(sum_le_zero.reverse_bits(), left, _mm_sub_epi16(_mm_setzero_si128(), right));

            // if vco_low bit is clear, we select either vs or vt based on whether the diff is >=0 or not
            let vco_low_clear_result = _wmm_mask_blend_epi16(diff_ge_zero.reverse_bits(), left, right);

            // pick which result based on new vco_low
            _wmm_mask_blend_epi16(new_vco_low.reverse_bits(), vco_low_clear_result, vco_low_set_result)
        };

        // low bits of accumulator get a copy of the result
        self.v[self.inst_vd] = result;
        self.v_set_accumulator_lo(&result);

        // set VCC/VCE/VCO to the result of the comparisons
        // vcc_low set to vt_lt_zero if vco_low is clear, otherwise set to sum_le_zero
        let new_vcc_low = (vt_lt_zero & !new_vco_low) | (sum_le_zero & new_vco_low);

        // vcc_high set to vt_lt_zero only if vco_low is set, otherwise set to diff_ge_zero
        let new_vcc_high = (vt_lt_zero & new_vco_low) | (diff_ge_zero & !new_vco_low);
        
        // vce set to sum_eq_neg1 only if vco_low is set, otherwise cleared
        let new_vce = sum_eq_neg1 & new_vco_low;

        // new vco_high set to AND of sum_ne_zero and vt_ne_not_vs if vco_low set, otherwise set to diff_ne_zero
        let new_vco_high = ((sum_ne_zero & vt_ne_not_vs) & new_vco_low) | (diff_ne_zero & !new_vco_low);

        self.ccr[Cop2_VCC] = ((new_vcc_high as u32) << 8) | (new_vcc_low as u32);
        self.ccr[Cop2_VCE] = new_vce as u32;
        self.ccr[Cop2_VCO] = ((new_vco_high as u32) << 8) | (new_vco_low as u32);

        Ok(())
    }

    fn cop2_vcr(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vcr v{}, v{}, v{}[0b{:04b}] // VCO=${:04X} VCC=${:04X}", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e, self.ccr[Cop2_VCO], self.ccr[Cop2_VCC]);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let (high_bit, vt_lt_zero, sum_lt_zero, diff_ge_zero);

        let result = unsafe {
            // first compare vs^vt < 0, gives us whether the high bit after an op is set (number is negative)
            high_bit = (_wmm_cmplt_epi16_mask(_mm_xor_si128(left, right), _mm_setzero_si128()) as u8).reverse_bits();

            // calculate sum and diff
            let sum  = _mm_add_epi16(left, right);
            let diff = _mm_sub_epi16(left, right);

            // comparisons between vs and vt
            vt_lt_zero = (_wmm_cmplt_epi16_mask(right, _mm_setzero_si128()) as u8).reverse_bits();

            // sum and diff comparisons
            sum_lt_zero  = (_wmm_cmplt_epi16_mask(sum, _mm_setzero_si128()) as u8).reverse_bits();
            diff_ge_zero = (_wmm_cmpge_epi16_mask(diff, _mm_setzero_si128()) as u8).reverse_bits();

            // results...
            // if high_bit bit is set, we select either vs or !vt based on whether the sum of the two is negative
            let high_bit_set_result = _wmm_mask_blend_epi16(sum_lt_zero.reverse_bits(), left, _mm_xor_si128(right, _mm_set1_epi8(0xFFu8 as i8)));

            // if high_bit bit is clear, we select either vs or vt based on whether the diff is >=0 or not
            let high_bit_clear_result = _wmm_mask_blend_epi16(diff_ge_zero.reverse_bits(), left, right);

            // pick which result based on new vco_low
            _wmm_mask_blend_epi16(high_bit.reverse_bits(), high_bit_clear_result, high_bit_set_result)
        };

        // low bits of accumulator get a copy of the result
        self.v[self.inst_vd] = result;
        self.v_set_accumulator_lo(&result);

        // set VCC to the result of the comparisons
        // vcc_low set to vt_lt_zero if vco_low is clear, otherwise set to sum_lt_zero
        let new_vcc_low  = (sum_lt_zero & high_bit) | (vt_lt_zero   & !high_bit);

        // vcc_high set to vt_lt_zero if vco_low is set, otherwise set to diff_ge_zero
        let new_vcc_high = (vt_lt_zero  & high_bit) | (diff_ge_zero & !high_bit);

        // set only VCC, clear VCE and VCO
        self.ccr[Cop2_VCC] = ((new_vcc_high as u32) << 8) | (new_vcc_low as u32);
        self.ccr[Cop2_VCE] = 0;
        self.ccr[Cop2_VCO] = 0;

        Ok(())
    }

    // VMUDL - passing tests
    fn cop2_vmudl(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmudl v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let result256: __m256i;

        // set only the low lane of the accumulator
        self.vacc = unsafe {
            // zero-extend the inputs to 32-bit
            let left256  = _mm256_cvtepu16_epi32(left);
            let right256 = _mm256_cvtepu16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            // then shift the high 16 bits into the low position
            result256 = _mm256_srli_epi32(_mm256_mullo_epi32(left256, right256), 16);

            // set vacc low only
            _wmm512_cvtepu32_epi64(result256)
        };

        // truncate to epi16
        self.v[self.inst_vd] = _wmm256_cvtepi32_epi16(result256);

        Ok(())
    }

    // VMADL - passing tests
    fn cop2_vmadl(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmadl v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let result256: __m256i;

        // add only the high short of the result to the accumulator
        self.vacc = unsafe {
            // zero-extend the inputs to 32-bit
            let left256  = _mm256_cvtepu16_epi32(left);
            let right256 = _mm256_cvtepu16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            // then shift the high 16 bits into the low position
            result256 = _mm256_srli_epi32(_mm256_mullo_epi32(left256, right256), 16);

            // add result to the accumulator
            _wmm512_add_epi64(self.vacc, _wmm512_cvtepu32_epi64(result256))
        };

        // saturate the accumulator to epi16 using the low lane for truncation
        self.v[self.inst_vd] = self.v_accumulator_saturate_low();

        Ok(())
    }

    // VMUDM - passing tests
    fn cop2_vmudm(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmudl v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let result256: __m256i;

        // set the mid and low lanes of the accumulator
        self.vacc = unsafe {
            // sign-extende vs and zero-extend vt
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepu16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            result256 = _mm256_mullo_epi32(left256, right256);

            // set vacc mid and low only with sign extension
            _wmm512_cvtepi32_epi64(result256)
        };

        // set the result to the mid lane of the accumulator result
        self.v[self.inst_vd] = unsafe { _wmm256_cvtepi32_epi16(_mm256_srli_epi32(result256, 16)) };

        Ok(())
    }

    // VMADM - passing tests
    fn cop2_vmadm(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmadm v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let result256: __m256i;

        // set the mid and low lanes of the accumulator
        self.vacc = unsafe {
            // sign-extende vs and zero-extend vt
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepu16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            result256 = _mm256_mullo_epi32(left256, right256);

            // add result to vacc
            _wmm512_add_epi64(self.vacc, _wmm512_cvtepi32_epi64(result256))
        };

        // saturate the highmid value of the accumulator
        let highmid = self.v_get_accumulator_highmid();
        self.v[self.inst_vd] = _wmm256_cvtsepi32_epi16(highmid);

        Ok(())
    }

    // VMUDN - passing tests
    // multiply a fraction (vs) times an integer (vt)
    fn cop2_vmudn(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmudn v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let result256: __m256i;

        // multiply and set set the accumulator to the result
        self.vacc = unsafe {
            // zero-extend left and sign-extend right to 32-bits
            let left256  = _mm256_cvtepu16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            result256 = _mm256_mullo_epi32(left256, right256);

            // set vacc mid and low, with sign extension
            _wmm512_cvtepi32_epi64(result256)
        };

        // set vd to truncated low 16 bits
        self.v[self.inst_vd] = _wmm256_cvtepi32_epi16(result256);

        Ok(())
    }

    // VMADN - passing tests
    fn cop2_vmadn(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmadn v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let result256: __m256i;

        // multiply and set set the accumulator to the result
        self.vacc = unsafe {
            // zero-extend left and sign-extend right to 32-bits
            let left256  = _mm256_cvtepu16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            result256 = _mm256_mullo_epi32(left256, right256);

            // set vacc mid and low, with sign extension
            let mask = _wmm512_set1_epi64(0x0000_FFFF_FFFF_FFFFu64 as i64);
            _wmm512_add_epi64(_wmm512_and_epi64(self.vacc, mask), _wmm512_and_epi64(_wmm512_cvtepi32_epi64(result256), mask))
        };

        // saturate the low 16-bit value of the accumulator
        self.v[self.inst_vd] = self.v_accumulator_saturate_low();

        Ok(())
    }


    // VMUDH - passing tests
    // The datasheet bit indices seem completely way off.  
    // Multiply vs and vt[e], placing the result in ACC high+mid, clearing ACC lo, 
    // with the result being Saturate(ACC high+mid)
    fn cop2_vmudh(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmudh v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let result256: __m256i;

        // multiply and set set the accumulator to the result
        self.vacc = unsafe {
            // sign-extend to 32-bits
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            result256 = _mm256_mullo_epi32(left256, right256);

            // setting result256 into VACC high and mid, while clearing the low lane
            // convert to 512-bit and left shift 16
            _wmm512_slli_epi64::<16>(_wmm512_cvtepu32_epi64(result256))
        };

        // saturate the high-mid 32-bit value of the accumulator to 16-bit for the result
        self.v[self.inst_vd] = _wmm256_cvtsepi32_epi16(result256);

        Ok(())
    }

    // VMADH - passing tests
    // Multiply vs and vt[e] adding the result to ACC highmid
    // Result is Saturate(ACC highmid)
    fn cop2_vmadh(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmadh v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let highmid = self.v_get_accumulator_highmid();

        // multiply and set set the accumulator to the result
        let result256 = unsafe {
            // sign-extend to 32-bits
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            // and add to ACC highmid
            _mm256_add_epi32(_mm256_mullo_epi32(left256, right256), highmid)
        };

        // set the result into highmid, preserving low
        self.v_set_accumulator_highmid(&result256);

        // saturate the high-mid 32-bit value of the accumulator to 16-bit for the result
        self.v[self.inst_vd] = _wmm256_cvtsepi32_epi16(result256);

        Ok(())
    }

    // VMULF - passing tests
    fn cop2_vmulf(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmulf v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        // multiply and set set the accumulator to the result
        self.vacc = unsafe {
            // sign-extend to eight 32-bits ints and multiply
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            let result256 = _mm256_mullo_epi32(left256, right256);

            // shift the 64-bit result left 1 bit (multiply of fractions) and add 0x8000
            let result512 = _wmm512_cvtepi32_epi64(result256); // sign-extend to 64-bit
            _wmm512_add_epi64(_wmm512_slli_epi64::<1>(result512), _wmm512_set1_epi64(0x8000u64 as i64))
        };

        //self.v_print_vacc();

        // saturate the high-mid 32-bit value of the accumulator to 16-bit for the result
        self.v[self.inst_vd] = {
            let highmid = self.v_get_accumulator_highmid();
            _wmm256_cvtsepi32_epi16(highmid)
        };

        Ok(())
    }

    // VMULU - passing tests
    fn cop2_vmulu(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmulu v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        // multiply and set set the accumulator to the result
        self.vacc = unsafe {
            // sign-extend to eight 32-bits ints and multiply
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            let result256 = _mm256_mullo_epi32(left256, right256);

            // shift the 64-bit result left 1 bit (multiply of fractions) and add 0x8000
            let result512 = _wmm512_cvtepi32_epi64(result256); // sign-extend to 64-bit
            _wmm512_add_epi64(_wmm512_slli_epi64::<1>(result512), _wmm512_set1_epi64(0x8000u64 as i64))
        };

        // unsigned saturate the high-mid 32-bit value of the accumulator to 16-bit for the result
        self.v[self.inst_vd] = unsafe {
            let highmid = self.v_get_accumulator_highmid();
            let high = _wmm256_cvtepi32_epi16(_mm256_srli_epi32(highmid, 16));
            let mid  = _wmm256_cvtepi32_epi16(highmid);

            // _mm_packus_epi32 and _mm256_cvtusepi32_epi16 don't work the way we want -- it won't saturate 0x0000_8000 to 0x0000_FFFF (and why should it?)
            // so we do it ourselves!
            let uppermask = _wmm_cmplt_epi16_mask(high, _mm_setzero_si128()); // bit 0x8000_0000 is set - 32-bit number is negative, goes to 0
            let halfmask  = _wmm_cmplt_epi16_mask(_mm_xor_si128(high, mid), _mm_setzero_si128()); // when high bit is clear but bit 0x0000_8000 is set, saturate to 0xFFFF
            // if uppermask is set, return 0
            _wmm_mask_blend_epi16(!uppermask, _mm_setzero_si128(),
                                              // if halfmask is set, return 0xFFFF
                                              _wmm_mask_blend_epi16(!halfmask, _mm_set1_epi16(0xFFFFu16 as i16),
                                                                               // otherwise, ACC mid is used directly
                                                                               mid))
        };

        Ok(())
    }

    // VMULQ - passing tests
    fn cop2_vmulq(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmulq v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        let mut result256: __m256i;

        // set the high and mid lanes of the accumulator, with zero in the low
        self.vacc = unsafe {
            // sign-extende vs and zero-extend vt
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);

            // multiply into 64-bit intermedaite and use the low 32-bits of the result
            result256 = _mm256_mullo_epi32(left256, right256);
            
            // add 31 to all lanes that have a negative result
            let rplus31 = _mm256_add_epi32(result256, _mm256_set1_epi32(31));
            let rneg    = _wmm256_cmplt_epi32_mask(result256, _mm256_setzero_si256());
            result256   = _wmm256_mask_blend_epi32(rneg, result256, rplus31);

            // set vacc mid and low only with sign extension
            _wmm512_slli_epi64::<16>(_wmm512_cvtepi32_epi64(result256))
        };

        // shift right 1, clamp down to 16 bits, then mask out lower 4 bits
        self.v[self.inst_vd] = unsafe { 
            // _mm256_cvtsepi32_epi16 seems to work like the below ops
            _mm_and_si128(_wmm256_cvtsepi32_epi16(_mm256_srai_epi32(result256, 1)), _mm_set1_epi16(0xFFF0u16 as i16)) 

            //let shifted = _mm256_srai_epi32(result256, 1);
            //let neg = _mm256_cmplt_epi32_mask(shifted, _mm256_set1_epi32(0xFFFF8000u32 as i32));
            //let result = _mm256_mask_blend_epi32(!neg, _mm256_set1_epi32(0x8000i32), shifted);
            //let pos = _mm256_cmpge_epi32_mask(shifted, _mm256_set1_epi32(0x8000i32));
            //let result = _mm256_mask_blend_epi32(!pos, _mm256_set1_epi32(0x7FFFi32), result);
            //_mm_and_si128(_wmm256_cvtepi32_epi16(result), _mm_set1_epi16(0xFFF0u16 as i16)) 
        };

        Ok(())
    }


    // VMACU - passing tests
    fn cop2_vmacu(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmacu v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        // multiply and add the accumulator to the result
        self.vacc = unsafe {
            // sign-extend to eight 32-bits ints and multiply
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);

            // multiply and sign extend to 64 bits
            let result256 = _mm256_mullo_epi32(left256, right256);
            let result512 = _wmm512_cvtepi32_epi64(result256);

            // shift the result left 1 bit (multiply of fractions) and add to the accumulator
            _wmm512_add_epi64(self.vacc, _wmm512_slli_epi64::<1>(result512))
        };

        // unsigned saturate the high-mid 32-bit value of the accumulator to 16-bit for the result
        self.v[self.inst_vd] = unsafe {
            let highmid = self.v_get_accumulator_highmid();
            let high = _wmm256_cvtepi32_epi16(_mm256_srli_epi32(highmid, 16));
            let mid  = _wmm256_cvtepi32_epi16(highmid);

            // _mm_packus_epi32 and _mm256_cvtusepi32_epi16 don't work the way we want -- it won't saturate 0x0000_8000 to 0x0000_FFFF (and why should it?)
            // so we do it ourselves!
            let uppermask = _wmm_cmplt_epi16_mask(high, _mm_setzero_si128());  // bit 0x8000_0000 is set - 32-bit number is negative, goes to 0
            let high_nz   = _wmm_cmpneq_epi16_mask(high, _mm_setzero_si128()); // compare high != 0
            let mid_neg   = _wmm_cmplt_epi16_mask(mid, _mm_setzero_si128());   // compare mid < 0 (bit 0x0000_8000 set)

            // if uppermask is set, return 0
            _wmm_mask_blend_epi16(!uppermask, _mm_setzero_si128(),
                                              // if (high_nz && mid_neg), return 0xFFFF
                                              _wmm_mask_blend_epi16(!(high_nz | mid_neg), _mm_set1_epi16(0xFFFFu16 as i16),
                                                                                          // otherwise, ACC mid is used directly
                                                                                          mid))
        };


        Ok(())
    }


    // VMACF - passing tests
    fn cop2_vmacf(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmulf v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let left  = self.v[self.inst_vs];
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);
        //println!("left=${:032X} right=${:032X}", Self::v_as_u128(&left), Self::v_as_u128(&right));

        // multiply and add the accumulator to the result
        self.vacc = unsafe {
            // sign-extend to eight 32-bits ints and multiply
            let left256  = _mm256_cvtepi16_epi32(left);
            let right256 = _mm256_cvtepi16_epi32(right);
            // multiply and zero extend to 64 bits
            let result256 = _mm256_mullo_epi32(left256, right256); // the actual op: rs * rt(e)
            let result512 = _wmm512_cvtepi32_epi64(result256); // sign-extend to 64-bit

            // shift the result left 1 bit (multiply of fractions) and add to the accumulator
            _wmm512_add_epi64(self.vacc, _wmm512_slli_epi64::<1>(result512))
        };

        // saturate the high-mid 32-bit value of the accumulator to 16-bit for the result
        self.v[self.inst_vd] = {
            let highmid = self.v_get_accumulator_highmid();
            _wmm256_cvtsepi32_epi16(highmid)
        };

        Ok(())
    }

    // VMACQ - passing tests
    // "Oddifies" the accumulator -- adds or subtracts 0x20 based on bit 0x20 and sign of the acc
    // ignores vs and vt entirely
    fn cop2_vmacq(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vmacq v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);

        let highmid = self.v_get_accumulator_highmid();

        // add 0 if bit 21 is clear
        // add 0x20<<16 if highmid is negative
        // otherwise subtract 0x20<<16 if highmid is positive
        let highmid = unsafe {
            // get a bitmask for all the lanes with bit 0x20_0000 clear
            let zero_bits = _wmm256_cmpeq_epi32_mask(_mm256_and_si256(highmid, _mm256_set1_epi32(0x20u32 as i32)), _mm256_setzero_si256());
            // mask for all lanes that are <-0x20 or >0x20
            // using the inverse mask for code readability
            let gen20 = _wmm256_cmpge_epi32_mask(highmid, _mm256_set1_epi32(0xFFFF_FFE0u32 as i32)); // want <-0x20 but comparing >=-0x20
            let lep20 = _wmm256_cmple_epi32_mask(highmid, _mm256_set1_epi32(0x0000_0020u32 as i32)); // want > 0x20 but comparing <= 0x20

            // if bit 21 is set, add 0. if bit 21 is clear, add either -0x20 or +0x20 depending on if the lane is <-0x20 or >0x20
            let addend = _wmm256_mask_blend_epi32(zero_bits, _mm256_setzero_si256(), 
                                                  // if result <-0x20 add 0x20, otherwise check lep20
                                                  _wmm256_mask_blend_epi32(gen20, _mm256_set1_epi32(0x0000_0020u32 as i32),
                                                                                  // if result <= 0x20, add 0
                                                                                  // otherwise subtract 0x20
                                                                                  _wmm256_mask_blend_epi32(lep20, _mm256_set1_epi32(0xFFFF_FFE0u32 as i32),
                                                                                                                  _mm256_setzero_si256())));
            _mm256_add_epi32(highmid, addend)
        };

        // set the highmid lanes of the accumulator
        self.v_set_accumulator_highmid(&highmid);

        // saturate the high-mid 32-bit value of the accumulator to 16-bit for the result
        self.v[self.inst_vd] = unsafe {
            // right shift with sign
            _mm_and_si128(_wmm256_cvtsepi32_epi16(_mm256_srai_epi32(highmid, 1)), _mm_set1_epi16(0xFFF0u16 as i16))
        };

        Ok(())
    }

    // common code for cop2_vrcp and cop2_vrcpl
    fn v_rcp(&self, src: u32) -> u32 {
        if src == 0 {
            0x7FFF_FFFF
        } else if src == 0xFFFF_8000 {
            0xFFFF_0000
        } else {
            // adjust src if it's value is in -1..-32,768
            let src = if src > 0xFFFF_8000u32 { src - 1 } else { src };

            // invert if negative
            let neg = (src & 0x8000_0000) != 0;
            let src = if neg { !src } else { src };

            // calculate shift
            let shift = src.leading_zeros() + 1; // leading zeroes plus the one 1 (must exist because src != 0)
            
            // compute index into reciprocal table (9 bits, 0..=511)
            let index = ((src << shift) & 0xFF80_0000) >> 23;

            // invert result for negative input
            let result = ((0x10000 | (self.rcp_table[index as usize] as u32)) << 14) >> (32 - shift);
            if neg { !result } else { result }
        }
    }

    // despite being called *V*RCP, it only works on a single scalar value from vt
    // woulda been cooler if it did all lanes in vt[e]
    fn cop2_vrcp(&mut self) -> Result<(), InstructionFault> {
        let de = ((self.inst.v >> 11) & 0x1F) as u8;
        //info!(target: "RSP", "vrcp v{}[0b{:05b}],  v{}[0b{:04b}]", self.inst_vd, de, self.inst_vt, self.inst_e);

        // E selects the element of vt
        let src = (Self::v_short(&self.v[self.inst_vt], self.inst_e & 0x07) as i16) as u32;
        let result = self.v_rcp(src);
        //println!("src=${:08X} result=${:08X}", src, result);

        // low lane of the accumulator gets vt[e]. set before changing vd
        self.v_set_accumulator_lo(&Self::v_math_elements(&self.v[self.inst_vt], self.inst_e));

        // set the lower 16 bits to the specified lane of vd
        self.v[self.inst_vd] = Self::v_insert_short(&self.v[self.inst_vd], result as u16, (de & 7) * 2);

        // save the result
        self.div_result = result;
        self.rcp_high   = false;
        
        Ok(())
    }

    // much the same as VRCP but with a different, possibly 32 bit, input
    fn cop2_vrcpl(&mut self) -> Result<(), InstructionFault> {
        let de = ((self.inst.v >> 11) & 0x1F) as u8;
        //info!(target: "RSP", "vrcpl v{}[0b{:05b}],  v{}[0b{:04b}]", self.inst_vd, de, self.inst_vt, self.inst_e);

        // E selects the element of vt
        let tmp = Self::v_short(&self.v[self.inst_vt], self.inst_e & 0x07);

        // if VRCPH was used, make use of rcp_input
        let src = if self.rcp_high {
            ((self.rcp_input as u32) << 16) | (tmp as u32)
        } else {
            (tmp as i16) as u32
        };

        let result = self.v_rcp(src);
        //println!("src=${:08X} result=${:08X}", src, result);

        // low lane of the accumulator gets vt[e]. set before changing vd
        self.v_set_accumulator_lo(&Self::v_math_elements(&self.v[self.inst_vt], self.inst_e));

        // set the lower 16 bits to the specified lane of vd
        self.v[self.inst_vd] = Self::v_insert_short(&self.v[self.inst_vd], result as u16, (de & 7) * 2);

        // save the result
        self.div_result = result;
        self.rcp_high   = false;
        
        Ok(())
    }

    fn cop2_vrcph(&mut self) -> Result<(), InstructionFault> {
        let de = ((self.inst.v >> 11) & 0x1F) as u8;
        //info!(target: "RSP", "vrcph v{}[0b{:05b}],  v{}[0b{:04b}]", self.inst_vd, de, self.inst_vt, self.inst_e);
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);

        // set the input to rcph into the hidden input register
        self.rcp_high = true;
        self.rcp_input = Self::v_short(&right, de & 7); // grab lane de from vt[e]

        // low lane of the accumulator gets vt[e]. set before changing vd
        self.v_set_accumulator_lo(&right);

        // set the lane of vd to the high short of div_result
        self.v[self.inst_vd] = Self::v_insert_short(&self.v[self.inst_vd], (self.div_result >> 16) as u16, (de & 7) * 2);

        Ok(())
    }

    // common code for cop2_vrsq and cop2_vrsql
    fn v_rsq(&self, src: u32) -> u32 {
        if src == 0 {
            0x7FFF_FFFF
        } else if src == 0xFFFF_8000 {
            0xFFFF_0000
        } else {
            // adjust src if it's value is in -1..-32,768
            let src = if src > 0xFFFF_8000u32 { src - 1 } else { src };

            // invert if negative
            let neg = (src & 0x8000_0000) != 0;
            let src = if neg { !src } else { src };

            // calculate shift
            let shift = src.leading_zeros() + 1; // leading zeroes plus the one 1 (must exist because src != 0)
            
            // compute index into reciprocal table (9 bits, 0..=511), but odd/even shifts pick 0..255 or 256..511
            let index = (((src << shift) & 0xFF80_0000) >> 24) | ((shift & 1) << 8);

            // invert result for negative input
            let result = ((0x10000 | (self.rsq_table[index as usize] as u32)) << 14) >> ((32 - shift) / 2);
            if neg { !result } else { result }
        }
    }

    fn cop2_vrsq(&mut self) -> Result<(), InstructionFault> {
        let de = ((self.inst.v >> 11) & 0x1F) as u8;
        //info!(target: "RSP", "vrsq v{}[0b{:05b}],  v{}[0b{:04b}]", self.inst_vd, de, self.inst_vt, self.inst_e);

        // E selects the element of vt
        let src = (Self::v_short(&self.v[self.inst_vt], self.inst_e & 0x07) as i16) as u32;
        let result = self.v_rsq(src);
        //println!("src=${:08X} result=${:08X}", src, result);

        // low lane of the accumulator gets vt[e]. set before changing vd
        self.v_set_accumulator_lo(&Self::v_math_elements(&self.v[self.inst_vt], self.inst_e));

        // set the lower 16 bits to the specified lane of vd
        self.v[self.inst_vd] = Self::v_insert_short(&self.v[self.inst_vd], result as u16, (de & 7) * 2);

        // save the result
        self.div_result = result;
        self.rcp_high   = false;
        
        Ok(())
    }

    // much the same as VRSQ but with a different, possibly 32 bit, input
    fn cop2_vrsql(&mut self) -> Result<(), InstructionFault> {
        let de = ((self.inst.v >> 11) & 0x1F) as u8;
        //info!(target: "RSP", "vrsql v{}[0b{:05b}],  v{}[0b{:04b}]", self.inst_vd, de, self.inst_vt, self.inst_e);

        // E selects the element of vt
        let tmp = Self::v_short(&self.v[self.inst_vt], self.inst_e & 0x07);

        // if VRSQH was used, make use of rcp_input
        let src = if self.rcp_high {
            ((self.rcp_input as u32) << 16) | (tmp as u32)
        } else {
            (tmp as i16) as u32
        };

        let result = self.v_rsq(src);
        //println!("src=${:08X} result=${:08X}", src, result);

        // low lane of the accumulator gets vt[e]. set before changing vd
        self.v_set_accumulator_lo(&Self::v_math_elements(&self.v[self.inst_vt], self.inst_e));

        // set the lower 16 bits to the specified lane of vd
        self.v[self.inst_vd] = Self::v_insert_short(&self.v[self.inst_vd], result as u16, (de & 7) * 2);

        // save the result
        self.div_result = result;
        self.rcp_high   = false;
        
        Ok(())
    }

    fn cop2_vrsqh(&mut self) -> Result<(), InstructionFault> {
        let de = ((self.inst.v >> 11) & 0x1F) as u8;
        //info!(target: "RSP", "vrsqh v{}[0b{:05b}],  v{}[0b{:04b}]", self.inst_vd, de, self.inst_vt, self.inst_e);
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);

        // set the input to rcph into the hidden input register
        self.rcp_high = true;
        self.rcp_input = Self::v_short(&right, de & 7); // grab lane de from vt[e]

        // low lane of the accumulator gets vt[e]. set before changing vd
        self.v_set_accumulator_lo(&right);

        // set the lane of vd to the high short of div_result
        self.v[self.inst_vd] = Self::v_insert_short(&self.v[self.inst_vd], (self.div_result >> 16) as u16, (de & 7) * 2);

        Ok(())
    }

    fn cop2_vrndn(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vrndn v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);

        self.vacc = unsafe {
            // compute "product", which is either vt[e] sign-extended or shifted up
            let product = if (self.inst_vs & 1) == 0 {
                _mm256_cvtepi16_epi32(right)
            } else {
                _mm256_slli_epi32(_mm256_cvtepi16_epi32(right), 16)
            };

            // check if ACC < 0
            let vacc    = _wmm512_srai_epi64::<16>(_wmm512_slli_epi64::<16>(self.vacc));
            let acc_neg = _wmm512_cmplt_epi64_mask(vacc, _wmm512_setzero_si512());

            // if ACC < 0, add sign-extended (to 48 bit) product
            let acc_sum = _wmm512_add_epi64(vacc, _wmm512_cvtepi32_epi64(product));

            // clear high 16 bits of epi64
            let shifted_mask = _wmm512_srli_epi64::<16>(self.vacc_mask);
            _wmm512_and_epi64(_wmm512_mask_blend_epi64(acc_neg, vacc, acc_sum), shifted_mask)
        };

        // saturate the high-mid 32-bit value of the accumulator to 16-bit for the result
        self.v[self.inst_vd] = {
            let highmid = self.v_get_accumulator_highmid();
            _wmm256_cvtsepi32_epi16(highmid)
        };

        Ok(())
    }
    fn cop2_vrndp(&mut self) -> Result<(), InstructionFault> {
        //info!(target: "RSP", "vrndp v{}, v{}, v{}[0b{:04b}]", self.inst_vd, self.inst_vs, self.inst_vt, self.inst_e);
        let right = Self::v_math_elements(&self.v[self.inst_vt], self.inst_e);

        self.vacc = unsafe {
            // compute "product", which is either vt[e] sign-extended or shifted up
            let product = if (self.inst_vs & 1) == 0 {
                _mm256_cvtepi16_epi32(right)
            } else {
                _mm256_slli_epi32(_mm256_cvtepi16_epi32(right), 16)
            };

            // check if bit 0x0000_8000_0000_0000 is clear (checking ACC >= 0)
            let vacc    = _wmm512_srai_epi64::<16>(_wmm512_slli_epi64::<16>(self.vacc));
            let acc_pos = _wmm512_cmpge_epi64_mask(vacc, _wmm512_setzero_si512());

            // if ACC >= 0, add sign-extended (to 48 bit) product
            let acc_sum = _wmm512_add_epi64(vacc, _wmm512_cvtepi32_epi64(product));

            // clear high 16 bits of epi64
            let shifted_mask = _wmm512_srli_epi64::<16>(self.vacc_mask);
            _wmm512_and_epi64(_wmm512_mask_blend_epi64(acc_pos, vacc, acc_sum), shifted_mask)
        };

        // saturate the high-mid 32-bit value of the accumulator to 16-bit for the result
        self.v[self.inst_vd] = {
            let highmid = self.v_get_accumulator_highmid();
            _wmm256_cvtsepi32_epi16(highmid)
        };

        Ok(())
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
