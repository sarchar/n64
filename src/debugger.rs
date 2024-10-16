use std::cell::{RefCell, RefMut};
use std::rc::Rc;

use std::collections::HashMap;
use std::sync::atomic::Ordering;

use crate::*;

use crossbeam::channel::{self, Receiver, Sender};
use mips::{InterruptUpdate, InterruptUpdateMode};
use tracing::{warn, info};

pub const BP_READ : u8 = 0x01;
pub const BP_WRITE: u8 = 0x02;
pub const BP_EXEC : u8 = 0x04;
pub const BP_RW   : u8 = BP_READ | BP_WRITE;

#[derive(Clone, Debug)]
pub enum MemoryChunk {
    /// (start address, memory)
    Valid(u64, Vec<u32>), 
    /// (start address, invalid size)
    Invalid(u64, usize),
}

#[derive(Clone, Debug, Default)]
pub struct CpuStateInfo {
    pub next_instruction_pc: u64,
    pub running: bool,
    pub instruction_memory: Vec<MemoryChunk>,
}

#[derive(Copy, Clone, Debug)]
pub enum DebuggerCommandRequest {
    /// get the PC and cpu state, along with some memory
    ///
    /// instead of having to wait for a response on where the PC is, and then request that memory
    /// for listing displays, this flag tells the debugger to return data at an address offset from the PC
    GetCpuState(Option<(i64, usize)>),     // (offset, size in words)
    /// get the main cpu registers
    GetCpuRegisters,
    /// stop a running Cpu
    StopCpu,
    /// start the Cpu
    RunCpu,
    /// Step Cpu
    StepCpu(u64),
    /// Step over two instructions - this is a basic that that adds $08 to the current PC and uses that as a breakpoint
    StepOver,
    /// Run to address
    RunTo(u64),
    /// Read memory
    ReadBlock(u64, u64, usize), // (id, address, size in words)
    /// Get list of breakpoints
    GetBreakpoints,
    /// Set a new breakpoint. The id field will be overwritten with a new ID
    SetBreakpoint(BreakpointInfo),
    /// Enable/disable the breakpoint.
    EnableBreakpoint(u64, bool), // (virtual address of breakpoint, enable)
    /// Change the mode of a breakpoint
    ChangeBreakpointMode(u64, u8),
    /// Remove an existing breakpoint
    RemoveBreakpoint(u64), // virtual address of a breakpoint
    /// Change a cpu register
    SetRegister(usize, u64),
    /// Get Cop0 control registers
    GetCop0State,
    /// Get Cop1 fpu registers
    GetCop1State,
    /// Set a Cop1 fgr value
    SetFgrFloat64(usize, f64),
    /// Set a Cop1 fgr value
    SetFgrFloat32(usize, f32),
    /// Set a Cop1 fgr value
    SetFgrInt64(usize, u64),
    /// Set a Cop1 fgr value
    SetFgrInt32(usize, u32),
    /// Request a copy of the Cop0 TLB
    GetTlb,
    /// Set the break on exception flags
    SetBreakOnException(u32, u8, u8), // (exceptions, interrupts, rcp)
 }

#[derive(Debug, Clone)]
pub enum DebuggerCommandResponse {
    /// Response to [DebuggerCommandRequest::GetCpuState] containing the current CpuStateInfo structure
    CpuState(CpuStateInfo),

    /// Full set of CPU gp registers
    CpuRegisters([u64; 32]),

    /// Equivelent of a DMA read
    ReadBlock(u64, Vec<MemoryChunk>), // id, data

    /// Response to [DebuggerCommandReqeust::GetBreakpoints] containing a list of [BreakpointInfo]s
    Breakpoints(HashMap<u64, BreakpointInfo>),

    /// When calling StepCpu, return how many cycles were actually stepped
    StepCpuDone(u64),

    /// Cop0 control registers
    Cop0State([u64; 32]),
    
    /// Fpu registers
    Cop1State { fgr: [cop1::Fgr; 32], fcr: u32, fr_bit: bool },

    /// TLB
    Tlb([cpu::TlbEntry; 32]),
}

#[derive(Clone, Debug)]
pub struct DebuggerCommand {
    pub command_request: DebuggerCommandRequest,
    pub response_channel: Option<Sender<DebuggerCommandResponse>>,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct BreakpointInfo {
    /// virtual address of the breakpoint
    /// uniquely defines the breakpoint instance as well
    pub address: u64,
    /// RWX flags
    pub mode   : u8,
    /// bp won't trigger if enable is clear
    pub enable : bool,
}

struct Breakpoints {
    global_enable: bool,
    table: HashMap<u64, BreakpointInfo>,
    /// table of only the BP_EXEC breakpoints, used by System::run_for()
    execution_breakpoints: HashSet<u64>,
    /// debugger intercepting memory bus
    debugger_bus: Option<Rc<RefCell<dyn Addressable>>>,
    using_debugger_bus: bool,
    rw_breakpoint_count: usize,
}

impl Breakpoints {
    fn new() -> Breakpoints {
        Breakpoints {
            global_enable: true,
            table: HashMap::new(),
            execution_breakpoints: HashSet::new(),
            debugger_bus: None,
            using_debugger_bus: false, 
            rw_breakpoint_count: 0,
        }
    }

    pub fn set_debugger_bus(&mut self, debugger_bus: Option<Rc<RefCell<dyn Addressable>>>) {
        self.debugger_bus = debugger_bus;
    }

    pub fn enable_debugger_bus(&mut self, cpu: &mut RefMut<'_, cpu::Cpu>) {
        assert!(!self.using_debugger_bus);
        std::mem::swap(&mut cpu.bus, &mut self.debugger_bus.as_mut().unwrap());
        self.using_debugger_bus = true;
    }

    pub fn disable_debugger_bus(&mut self, cpu: &mut RefMut<'_, cpu::Cpu>) {
        assert!(self.using_debugger_bus);
        std::mem::swap(&mut cpu.bus, &mut self.debugger_bus.as_mut().unwrap());
        self.using_debugger_bus = false;
    }

    fn check_breakpoint(&self, virtual_address: u64, mode: u8) -> Option<BreakpointInfo> {
        if !self.global_enable { return None; }

        if let Some(breakpoint) = self.table.get(&virtual_address) {
            if breakpoint.enable && ((breakpoint.mode & mode) != 0) {
                return Some(*breakpoint);
            }
        }

        None
    }

    fn add_breakpoint(&mut self, mut cpu: RefMut<'_, cpu::Cpu>, breakpoint_info: BreakpointInfo) {
        let enable = breakpoint_info.enable;
        let virtual_address = breakpoint_info.address;

        if self.table.contains_key(&virtual_address) {
            warn!(target: "DEBUGGER", "can't duplicate breakpoints");
            return;
        }
        
        if enable {
            if (breakpoint_info.mode & BP_EXEC) != 0 {
                // include in the execution set
                self.execution_breakpoints.insert(virtual_address);

                // the JIT will need to invalidate the block where this breakpoint is
                if let Ok(Some(address)) = cpu.translate_address(virtual_address, false, false) {
                    cpu.invalidate_physical_block_cache_reference(address.physical_address & !3, 1);
                }
            }

            if (breakpoint_info.mode & BP_RW) != 0 {
                self.rw_breakpoint_count += 1;
                if self.rw_breakpoint_count == 1 {
                    self.enable_debugger_bus(&mut cpu);
                }
            }
        }

        self.table.insert(breakpoint_info.address, breakpoint_info);
    }

    fn remove_breakpoint(&mut self, mut cpu: RefMut<'_, cpu::Cpu>, virtual_address: u64) -> Result<(), ()> {
        if let Some(breakpoint_info) = self.table.remove(&virtual_address) {
            // safe to just remove, BP_EXEC or not
            self.execution_breakpoints.remove(&virtual_address);

            // invalidate the block in the JIT for whatever reason
            if let Ok(Some(address)) = cpu.translate_address(virtual_address, false, false) {
                cpu.invalidate_physical_block_cache_reference(address.physical_address & !3, 1);
            }

            if breakpoint_info.enable && (breakpoint_info.mode & BP_RW) != 0 {
                self.rw_breakpoint_count -= 1;
                if self.rw_breakpoint_count == 0 {
                    self.disable_debugger_bus(&mut cpu);
                }
            }

            Ok(())
        } else {
            warn!(target: "DEBUGGER", "cannot remove invalid breakpoint ${:016X}", virtual_address);
            Err(())
        }
    }

    /// returns the old state
    fn enable_breakpoint(&mut self, mut cpu: RefMut<'_, cpu::Cpu>, virtual_address: u64, enable: bool) -> Result<bool, ()> {
        if let Some(mut info) = self.table.get_mut(&virtual_address).map(|v| *v) {
            if (info.mode & BP_EXEC) != 0 {
                if info.enable && !enable {
                    // return the memory at the address to the real instruction
                    self.execution_breakpoints.remove(&virtual_address);
                } else if !info.enable && enable {
                    // re-set the `break` instruction
                    self.execution_breakpoints.insert(virtual_address);
                }

                // invalidate the block in the JIT since the state changed
                if let Ok(Some(address)) = cpu.translate_address(virtual_address, false, false) {
                    cpu.invalidate_physical_block_cache_reference(address.physical_address & !3, 1);
                }
            }

            if (info.mode & BP_RW) != 0 {
                if info.enable && !enable {
                    self.rw_breakpoint_count -= 1;
                    if self.rw_breakpoint_count == 0 {
                        self.disable_debugger_bus(&mut cpu);
                    }
                } else if !info.enable && enable {
                    self.rw_breakpoint_count += 1;
                    if self.rw_breakpoint_count == 1 {
                        self.enable_debugger_bus(&mut cpu);
                    }
                }
            }

            let old_enable = std::mem::replace(&mut info.enable, enable);
            self.table.insert(virtual_address, info);
            Ok(old_enable)
        } else {
            Err(())
        }       
    }

    /// returns old mode
    fn change_breakpoint_mode(&mut self, mut cpu: RefMut<'_, cpu::Cpu>, virtual_address: u64, new_mode: u8) -> Result<u8, ()> {
        if let Some(mut info) = self.table.get_mut(&virtual_address).map(|v| *v) {
            if info.enable {
                // if we're changing exec flag (and the breakpoint is enabled), notify CPU
                if ((info.mode | new_mode) & BP_EXEC) != 0 {
                    if (info.mode & BP_EXEC) != 0 && (new_mode & BP_EXEC) == 0 {
                        self.execution_breakpoints.remove(&virtual_address);
                    } else if (info.mode & BP_EXEC) == 0 && (new_mode & BP_EXEC) != 0 {
                        self.execution_breakpoints.insert(virtual_address);
                    }

                    // invalidate the block in the JIT since the state changed may have changed
                    if let Ok(Some(address)) = cpu.translate_address(virtual_address, false, false) {
                        cpu.invalidate_physical_block_cache_reference(address.physical_address & !3, 1);
                    }
                }

                // if we're changing read/write modes we can enable/disable the debugger bus
                if (info.mode & BP_RW) == 0 && (new_mode & BP_RW) != 0 {
                    self.rw_breakpoint_count += 1;
                    if self.rw_breakpoint_count == 1 {
                        self.enable_debugger_bus(&mut cpu);
                    }
                } else if (info.mode & BP_RW) != 0 && (new_mode & BP_RW) == 0 {
                    self.rw_breakpoint_count -= 1;
                    if self.rw_breakpoint_count == 0 {
                        self.disable_debugger_bus(&mut cpu);
                    }
                }
            }

            let old_mode = std::mem::replace(&mut info.mode, new_mode);
            self.table.insert(virtual_address, info);
            Ok(old_mode)
        } else {
            Err(())
        }
    }
}

pub struct Debugger {
    exit_requested: bool,
    cpu_running: bool,
    next_cpu_run_for: u64,
    run_to_address: Option<u64>,

    // exception breakpoints
    break_on_exception: u32,
    break_on_interrupt: u8,
    break_on_rcp: u8,

    ctrlc_count: u32,

    breakpoints: Rc<RefCell<Breakpoints>>,

    system: System,

    /// debugging commands
    command_receiver: Receiver<DebuggerCommand>,
}

impl Debugger {
    pub fn new(system: System) -> Debugger {
        // let cpu_running = Arc::new(AtomicBool::new(false));
        // let r = cpu_running.clone();

        // create the debugging communication channel
        let (sender, command_receiver) = channel::unbounded();
        *system.comms.debugger.write().unwrap() = Some(sender);
        
        // ctrlc::set_handler(move || {
        //     println!("Break!");
        //     r.store(false, Ordering::SeqCst);
        // }).expect("Error setting ctrl-c handler");

        // create the breakpoints struct first 
        let breakpoints = Rc::new(RefCell::new(Breakpoints::new()));

        // create the debugger memory bus with reference to the RCP and breakpoints
        let debugger_bus = Rc::new(RefCell::new(DebuggerBus::new(system.cpu.borrow().bus.clone(), breakpoints.clone())));

        // and tell the breakpoints about the bus. this creates a reference cycle, but we take care of it in Drop
        breakpoints.borrow_mut().set_debugger_bus(Some(debugger_bus.clone()));

        Debugger {
            exit_requested    : false,
            cpu_running       : true,
            next_cpu_run_for  : 0,     // first call to run_for() won't tick the CPU, it'll just calculate how many cycles to run for
            run_to_address    : None,
            break_on_exception: 0,
            break_on_interrupt: 0,
            break_on_rcp      : 0,
            ctrlc_count       : 0,
            breakpoints,
            system,
            command_receiver,
        }
    }

    /// blocking call to transmit a command request on the provided communications channel
    pub fn send_command(user_comms: &SystemCommunication, cmd: DebuggerCommand) -> Result<(), ()> {
        if let Some(ref tx) = user_comms.debugger.read().unwrap().as_ref() {
            tx.send(cmd).unwrap();
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn run(&mut self) -> Result<(), ()> {
        // let cpu = self.system.cpu.borrow_mut();
        // self.breakpoints.borrow_mut().add_breakpoint(cpu, BreakpointInfo { address: 0xFFFFFFFF800D34BC, mode: BP_EXEC, enable: true });

        // self.cpu_running = false;
        while !self.exit_requested { 
            if self.cpu_running {
                self.next_cpu_run_for = self.run_for(self.next_cpu_run_for); 
            }

            self.update();
        }

        // let mut rl = DefaultEditor::new()?;

        // if rl.load_history("history.txt").is_err() {
        //     println!("No history.txt file");
        // }

        // let mut lastline = String::from("");
        // let mut last_printed_pc = 0;
        // while self.alive {
        //     let readline = { // context for dropping cpu
        //         let mut cpu = self.system.cpu.borrow_mut();
        //         let next_instruction_pc = cpu.next_instruction_pc();
        //         if last_printed_pc != next_instruction_pc {
        //             let inst = if cpu.next_instruction().is_some() {
        //                 cpu::Cpu::disassemble(next_instruction_pc, cpu.next_instruction().unwrap(), true)
        //             } else {
        //                 format!("<cannot fetch instruction>")
        //             };
        //             print!("${:08X}: {} (next instruction)", next_instruction_pc, inst);

        //             if cpu.next_is_delay_slot() {
        //                 print!(" (delay slot)");
        //             }
        //             println!("");
        //             last_printed_pc = next_instruction_pc;
        //         }

        //         let prompt = format!("<PC:${:08X}>@ ", next_instruction_pc);
        //         rl.readline(&prompt)
        //     };

        //     match readline {
        //         RustylineResult::Ok(line) => {
        //             let mut line_str = String::from(line.as_str().trim());

        //             if line_str.len() == 0 {
        //                 line_str = lastline.clone();
        //             }

        //             lastline = line_str.clone();

        //             if line_str.len() > 0 {
        //                 rl.add_history_entry(line_str.as_str())?;
        //                 if let Err(err) = self.handle_line(line_str.as_str()) {
        //                     println!("error: {}", err);
        //                 }
        //             }

        //             self.ctrlc_count = 0;
        //         },

        //         RustylineResult::Err(ReadlineError::Interrupted) => {
        //             self.ctrlc_count += 1;
        //             if self.ctrlc_count == 3 { 
        //                 println!("Exiting...");
        //                 break; 
        //             }
        //             else if self.ctrlc_count == 1 {
        //                 println!("Ctrl-C, press twice more to exit");
        //             }
        //         },

        //         RustylineResult::Err(ReadlineError::Eof) => {
        //             println!("Exiting...");
        //             break;
        //         },

        //         RustylineResult::Err(err) => {
        //             panic!("ReadlineError: {}", err);
        //         },
        //     };
        // };

        // rl.save_history("history.txt")?;

        Ok(())
    }

    #[inline]
    fn run_for(&mut self, num_cycles: u64) -> u64 {
        let result = self.system.run_for(num_cycles, &self.breakpoints.borrow().execution_breakpoints);
        match result {
            Ok(next_cycles) => next_cycles,

            // Ignore CPU faults for now -- TODO: breakpoint on certain exceptions?
            Err(cpu::InstructionFault::OtherException(exception_code)) => {
                if (self.break_on_exception & (1 << exception_code)) != 0 {
                    if exception_code == cpu::ExceptionCode_Int {
                        let interrupt_cause = (self.system.cpu.borrow().cause() >> 8) as u8;
                        if (self.break_on_interrupt & interrupt_cause & (cpu::InterruptCode_RCP as u8)) != 0 {
                            // read the MIPS state
                            let mi_interrupt = self.system.cpu.borrow_mut().read_u32_phys_direct(0x0430_0008).unwrap() as u8; // read MI_INTERRUPT (TODO *should* use PEEK?)

                            // check if it overlaps with what we want to break on
                            if (self.break_on_rcp & mi_interrupt) != 0 {
                                // Create a string listing the interrupts that were trigged
                                let mut names = String::new();
                                for i in 0..6 {
                                    if (self.break_on_rcp & mi_interrupt & (1 << i)) != 0 {
                                        if names.len() > 0 {
                                            names.push_str(",");
                                        }
                                        names.push_str(&mips::MI_INTERRUPT_NAMES[i]);
                                    }
                                }
                                info!(target: "DEBUGGER", "CPU break on RCP interrupt(s) 0x{:02X} ({})", self.break_on_rcp & mi_interrupt, names);
                                self.cpu_running = false;
                            }
                        } else if (self.break_on_interrupt & interrupt_cause) != 0 {
                            info!(target: "DEBUGGER", "CPU break on Timer interrupt");
                            self.cpu_running = false;
                        }
                    } else {
                        info!(target: "DEBUGGER", "CPU break on exception {}", exception_code);
                        self.cpu_running = false;
                    }
                }

                // exceptions move the PC into exception handlers, so if a breakpoint exists at that address,
                // we should also stop
                let pc = self.system.cpu.borrow().next_instruction_pc();
                if self.breakpoints.borrow().execution_breakpoints.contains(&pc) {
                    info!(target: "DEBUGGER", "CPU break (and exception) at virtual address ${:016X}", pc);
                    self.cpu_running = false;
                }

                self.system.rcp.borrow().calculate_free_cycles()
            },
            
            // Break generated by cpu instructions won't reach this code path -- only breaks from our breakpoints will
            Err(cpu::InstructionFault::Break) => {
                let pc = self.system.cpu.borrow().next_instruction_pc();
                info!(target: "DEBUGGER", "CPU break at virtual address ${:08X}", pc);

                // stop the CPU
                self.cpu_running = false;

                // on all breaks, clear the runto address
                if let Some(run_to_address) = self.run_to_address {
                    if self.breakpoints.borrow_mut().execution_breakpoints.remove(&run_to_address) {
                        let mut cpu = self.system.cpu.borrow_mut();
                        if let Ok(Some(address)) = cpu.translate_address(run_to_address, false, false) {
                            cpu.invalidate_physical_block_cache_reference(address.physical_address, 1);
                        }
                    }
                }
                
                // recalculate cycles that can be run
                self.system.rcp.borrow().calculate_free_cycles()
            },

            err @ _ => panic!("unhandled CPU exception ${:?}", err),
        }
    }

    // read system memory in pages
    // count in 32-bit words
    fn read_memory_pages(&mut self, mut virtual_address: u64, mut count: usize) -> Vec<MemoryChunk> {
        let mut cpu = self.system.cpu.borrow_mut();
        let mut result = Vec::new();
        while count > 0 {
            let words_left_in_page = ((0x1000 - (virtual_address & 0xFFC)) >> 2) as usize;
            let read_size = std::cmp::min(words_left_in_page, count);

            match cpu.translate_address(virtual_address, false, false).unwrap() {
                Some(address) => {
                    // println!("requesting {} bytes from virtual=${:016X} physical=${:08X}", read_size << 2, virtual_address, address.physical_address);
                    match self.system.rcp.borrow_mut().read_block(address.physical_address as usize, (read_size << 2) as u32) {
                        Ok(memory) => {
                            result.push(MemoryChunk::Valid(virtual_address, memory));
                        },

                        _ => {
                            result.push(MemoryChunk::Invalid(virtual_address, read_size));
                        }
                    }
                },

                _ => {
                    result.push(MemoryChunk::Invalid(virtual_address, read_size));
                }
            }

            virtual_address += (read_size << 2) as u64;
            count -= read_size;
        }

        result
    }

    fn update(&mut self) {
        // active when at least 1 debugging window is open
        if self.system.comms.debugger_windows.load(Ordering::Relaxed) == 0 { return; }

        'next_command: while let Ok(req) = self.command_receiver.try_recv() {
            match req.command_request {
                DebuggerCommandRequest::GetCpuState(read_instruction_memory) => {
                    let response_channel = if let Some(r) = req.response_channel { r } else { continue 'next_command; };

                    let next_instruction_pc = self.system.cpu.borrow().next_instruction_pc();
                    let instruction_memory = if let Some((instruction_offset, instruction_count)) = read_instruction_memory {
                        let virtual_address = (next_instruction_pc as i64).wrapping_add(instruction_offset * 4);
                        self.read_memory_pages(virtual_address as u64, instruction_count)
                    } else { 
                        Vec::new()
                    };

                    let running = self.cpu_running;
                    let cpu_state = CpuStateInfo { next_instruction_pc, running, instruction_memory };
                    response_channel.send(DebuggerCommandResponse::CpuState(cpu_state)).unwrap();
                },

                DebuggerCommandRequest::GetCpuRegisters => {
                    let response_channel = if let Some(r) = req.response_channel { r } else { continue 'next_command; };
                    let cpu = self.system.cpu.borrow_mut();
                    let cpu_registers: [u64; 32] = cpu.regs_copy();
                    response_channel.send(DebuggerCommandResponse::CpuRegisters(cpu_registers)).unwrap();
                }

                DebuggerCommandRequest::StopCpu => {
                    self.cpu_running = false;
                },

                DebuggerCommandRequest::RunCpu => {
                    self.start_cpu();
                },

                DebuggerCommandRequest::StepCpu(num_cycles) => {
                    if self.cpu_running || num_cycles == 0 { continue 'next_command; }

                    let mut cycles_ran_for = 0;
                    while cycles_ran_for < num_cycles {
                        let cycle_start = self.system.cpu.borrow().num_steps();
                        let max_cycles = std::cmp::min(num_cycles - cycles_ran_for, self.next_cpu_run_for);
                        self.next_cpu_run_for = self.run_for(max_cycles);
                        cycles_ran_for += self.system.cpu.borrow().num_steps() - cycle_start;
                    }

                    if let Some(response_channel) = req.response_channel {
                        let _ = response_channel.send(DebuggerCommandResponse::StepCpuDone(cycles_ran_for));
                    }
                },

                DebuggerCommandRequest::StepOver => {
                    if self.cpu_running { continue 'next_command; }
                    
                    let break_pc = {
                        let cpu = self.system.cpu.borrow();
                        cpu.next_instruction_pc() + if cpu.next_is_delay_slot() { 4 } else { 8 }
                    };

                    self.set_run_to_address(break_pc);
                    
                    // run the CPU
                    self.start_cpu();
                }

                DebuggerCommandRequest::RunTo(virtual_address) => {
                    if self.cpu_running { continue 'next_command; }

                    self.set_run_to_address(virtual_address);

                    // run the CPU
                    self.start_cpu();
                },

                DebuggerCommandRequest::ReadBlock(id, virtual_address, size_in_words) => {
                    let response_channel = if let Some(r) = req.response_channel { r } else { continue 'next_command; };

                    let memory = self.read_memory_pages(virtual_address, size_in_words);

                    let _ = response_channel.send(DebuggerCommandResponse::ReadBlock(id, memory));
                },

                DebuggerCommandRequest::GetBreakpoints => {
                    let response_channel = if let Some(r) = req.response_channel { r } else { continue 'next_command; };
                    let breakpoints: HashMap<u64, BreakpointInfo> = self.breakpoints.borrow().table.clone();
                    response_channel.send(DebuggerCommandResponse::Breakpoints(breakpoints)).unwrap();
                },

                DebuggerCommandRequest::SetBreakpoint(breakpoint_info) => {
                    let cpu = self.system.cpu.borrow_mut();
                    self.breakpoints.borrow_mut().add_breakpoint(cpu, breakpoint_info);
                },

                DebuggerCommandRequest::EnableBreakpoint(virtual_address, enable) => {
                    let cpu = self.system.cpu.borrow_mut();
                    let _ = self.breakpoints.borrow_mut().enable_breakpoint(cpu, virtual_address, enable);
                },

                DebuggerCommandRequest::ChangeBreakpointMode(virtual_address, new_mode) => {
                    let cpu = self.system.cpu.borrow_mut();
                    let _ = self.breakpoints.borrow_mut().change_breakpoint_mode(cpu, virtual_address, new_mode);
                },

                DebuggerCommandRequest::RemoveBreakpoint(virtual_address) => {
                    let cpu = self.system.cpu.borrow_mut();
                    let _ = self.breakpoints.borrow_mut().remove_breakpoint(cpu, virtual_address);
                },

                DebuggerCommandRequest::SetRegister(rnum, value) => {
                    self.system.cpu.borrow_mut().regs_mut()[rnum] = value;
                },

                DebuggerCommandRequest::GetCop0State => {
                    let response_channel = if let Some(r) = req.response_channel { r } else { continue 'next_command; };
                    let cpu = self.system.cpu.borrow();
                    let cp0regs = cpu.cp0gpr_clone();
                    response_channel.send(DebuggerCommandResponse::Cop0State(cp0regs)).unwrap();
                },

                DebuggerCommandRequest::GetCop1State => {
                    let response_channel = if let Some(r) = req.response_channel { r } else { continue 'next_command; };
                    let cpu = self.system.cpu.borrow_mut();
                    let fgr: [cop1::Fgr; 32] = cpu.cop1().fgr_clone();
                    let fr_bit = cpu.cop1().fr_bit();
                    let fcr = cpu.cop1().fcr_control_status();
                    response_channel.send(DebuggerCommandResponse::Cop1State { fgr, fcr, fr_bit }).unwrap();
                }

                DebuggerCommandRequest::SetFgrFloat64(fgrnum, value) => {
                    let mut cpu = self.system.cpu.borrow_mut();
                    cpu.cop1_mut().set_fgr_f64(fgrnum, value);
                },

                DebuggerCommandRequest::SetFgrFloat32(fgrnum, value) => {
                    let mut cpu = self.system.cpu.borrow_mut();
                    cpu.cop1_mut().set_fgr_f32(fgrnum, value);
                },

                DebuggerCommandRequest::SetFgrInt64(fgrnum, value) => {
                    let mut cpu = self.system.cpu.borrow_mut();
                    cpu.cop1_mut().set_fgr_u64(fgrnum, value);
                },

                DebuggerCommandRequest::SetFgrInt32(fgrnum, value) => {
                    let mut cpu = self.system.cpu.borrow_mut();
                    cpu.cop1_mut().set_fgr_u32(fgrnum, value);
                },

                DebuggerCommandRequest::GetTlb => {
                    let response_channel = if let Some(r) = req.response_channel { r } else { continue 'next_command; };
                    let tlb = self.system.cpu.borrow().tlb_clone();
                    response_channel.send(DebuggerCommandResponse::Tlb(tlb)).unwrap();
                },

                DebuggerCommandRequest::SetBreakOnException(break_on_exception, break_on_interrupt, break_on_rcp) => {
                    self.break_on_exception = break_on_exception;
                    self.break_on_interrupt = break_on_interrupt;
                    self.break_on_rcp       = break_on_rcp;
                },
            }
        }
    }    

    fn start_cpu(&mut self) {
        self.cpu_running = true;
        if self.system.comms.cpu_throttle.load(Ordering::Relaxed) == 1 { // if throttling is enabled, skip for a single call to avoid a speedup
            self.system.comms.cpu_throttle.store(2, Ordering::Relaxed);
        }
    }

    pub fn stop_cpu(&mut self) {
        self.cpu_running = false;
    }

    fn set_run_to_address(&mut self, virtual_address: u64) {
        // insert the runto address into execution_breakpoints
        if self.breakpoints.borrow_mut().execution_breakpoints.insert(virtual_address) { // if it didn't exist before,
            // invalidate the JIT at the address for the stepover
            let mut cpu = self.system.cpu.borrow_mut();
            if let Ok(Some(address)) = cpu.translate_address(virtual_address, false, false) {
                cpu.invalidate_physical_block_cache_reference(address.physical_address & !3, 1);
            }

            // and set runto address
            self.run_to_address = Some(virtual_address);
        }
    }

    // fn handle_line(&mut self, line: &str) -> Result<(), String> {
    //     let lines = line.split(";").collect::<Vec<&str>>();
    //     for line in lines {
    //         let parts = line.split_whitespace().collect::<Vec<&str>>();

    //         if parts.len() == 0 { return Ok(()); }

    //         let result = match parts[0] {
    //             "r" | "ru" | "run"          => { self.run_cpu(&parts) },
    //             "s" | "st" | "ste" | "step" => { self.step(&parts) }
    //             "res" | "rese" | "reset"    => { self.reset(&parts) },
    //             "regs" | "rd"               => { self.dump_regs(&parts) },
    //             "rw"                        => { self.dump_regs_as_words(&parts) },
    //             "b" | "br" | "bre" | "brea"
    //              | "break" | "bp"           => { self.breakpoint(&parts) },
    //             "db" | "del" | "dbr" 
    //              | "dbrea" | "dbreak"       => { self.delete_breakpoint(&parts) },
    //             "log"                       => { self.logging(&parts) },
    //             "l" | "li" | "lis" | "list" => { self.listing(&parts) },
    //             "int"                       => { self.interrupt(&parts) },

    //             _ => {
    //                 Err(format!("unsupported debugger command \"{}\"", parts[0]))
    //             },
    //         };

    //         if let Err(_) = result { return result; }
    //     }

    //     Ok(())
    // }

    // fn run_cpu(&mut self, parts: &Vec<&str>) -> Result<(), String> {
    //     self.cpu_run_til = None;
    //     self.cpu_running.store(true, Ordering::SeqCst);

    //     if parts.len() > 1 {
    //         self.cpu_run_til = match parse_int(&parts[1]) {
    //             Ok(v) => Some(v as u64),
    //             Err(err) => {
    //                 return Err(err);
    //             }
    //         };
    //     }

    //     let start_steps = self.system.cpu.borrow().num_steps();
    //     let now = std::time::Instant::now();

    //     while self.cpu_running.load(Ordering::SeqCst) {
    //         //let address = *cpu.next_instruction_pc();
    //         //let inst = cpu::Cpu::disassemble(address, *cpu.next_instruction(), true);
    //         //println!("${:08X}: {}", address, inst);

    //         // Break loop on any instruction error or memory access
    //         match self.system.run_for(1) {
    //             Ok(_) => {},

    //             Err(InstructionFault::OtherException(_exception_code)) => {
    //                 // TODO: CPU exceptions should only interrupt if desired
    //             },

    //             e @ Err(_) => {
    //                 // All other errors are probably bad
    //                 println!("breaking on error ${:?}", e);
    //                 break;
    //             },
    //         }

    //         // Check breakpoints
    //         if let Some(breakpoint) = self.breakpoints.borrow().check_breakpoint((self.system.cpu.borrow().next_instruction_pc() as i32) as u64, BP_EXEC) {
    //             println!("Breakpoint ${:016X} hit", breakpoint.address);
    //             self.cpu_running.store(false, Ordering::SeqCst);
    //         }

    //         // Check run until
    //         match self.cpu_run_til {
    //             Some(v) => {
    //                 if self.system.cpu.borrow().next_instruction_pc() == v {
    //                     self.cpu_running.store(false, Ordering::SeqCst);
    //                 }
    //             },
    //             None => {}
    //         };
    //     }

    //     let steps = self.system.cpu.borrow().num_steps() - start_steps;
    //     let elapsed = now.elapsed();
    //     let duration = (elapsed.as_secs() as f64) + (elapsed.subsec_micros() as f64) / 1000000.0;

    //     println!("Cpu steps: {}, duration = {}, steps/sec = {}", steps, duration, (steps as f64) / duration);

    //     Ok(())
    // }

    // fn reset(&mut self, _: &Vec<&str>) -> Result<(), String> {
    //     self.system.reset();
    //     Ok(())
    // }

    // fn dump_regs(&mut self, _: &Vec<&str>) -> Result<(), String> {
    //     let cpu = self.system.cpu.borrow();
    //     let regs = cpu.regs();
        
    //     for k in 0..8 {
    //         for j in 0..4 {
    //             print!("R{:02}(${}): ${:08X}_{:08X} ", k*4+j, cpu::Cpu::abi_name(k*4+j), regs[(k*4+j) as usize] >> 32, regs[(k*4+j) as usize] & 0xFFFF_FFFF);
    //         }
    //         println!("");
    //     }

    //     println!("PC: ${:08X}", cpu.next_instruction_pc());

    //     Ok(())
    // }

    // fn dump_regs_as_words(&mut self, _: &Vec<&str>) -> Result<(), String> {
    //     let cpu = self.system.cpu.borrow();
    //     let regs = cpu.regs();
        
    //     for k in 0..4 {
    //         for j in 0..8 {
    //             print!("R{:02}(${}): {:08X} ", k*8+j, cpu::Cpu::abi_name(k*8+j), regs[(k*8+j) as usize] & 0xFFFF_FFFF);
    //         }
    //         println!("");
    //     }

    //     println!("PC: ${:08X}", cpu.next_instruction_pc());

    //     Ok(())
    // }

    // fn logging(&mut self, parts: &Vec<&str>) -> Result<(), String> {
    //     if parts.len() < 2 || parts.len() > 3 {
    //         return Err(format!("usage: loglevel trace|debug|info|warn|error (module=level[,module=level])"));
    //     }

    //     let default_level = match parts[1].to_lowercase().as_str() {
    //         "info" => Level::INFO,
    //         "debug" => Level::DEBUG,
    //         "warn" => Level::WARN,
    //         "error" => Level::ERROR,
    //         "trace" => Level::TRACE,
    //         _ => { return Err(format!("invalid default level \"{}\"", parts[2])); }
    //     };

    //     let format_str = if parts.len() == 3 {
    //         parts[2]
    //     } else {
    //         ""
    //     };

    //     (self.change_logging)(format_str, default_level);

    //     Ok(())
    // }

    fn interrupt(&mut self, parts: &Vec<&str>) -> Result<(), String> {
        if parts.len() != 2 {
            return Err(format!("usage: int [sp|si|vi|dp|ai|pi]"));
        }

        let signal = match parts[1].to_lowercase().as_str() {
            "sp" => mips::IMask_SP,
            "si" => mips::IMask_SI,
            "ai" => mips::IMask_AI,
            "vi" => mips::IMask_VI,
            "dp" => mips::IMask_DP,
            "pi" => mips::IMask_PI,
            _ => { return Err(format!("invalid mips interrupt \"{}\"", parts[1])); }
        };

        let mut rcp = self.system.rcp.borrow_mut();
        let chan = rcp.mi.get_update_channel();
        chan.send(InterruptUpdate(signal, InterruptUpdateMode::SetInterrupt)).unwrap();
        Ok(())
    }
}

impl Drop for Debugger {
    fn drop(&mut self) {
        self.breakpoints.borrow_mut().set_debugger_bus(None);
    }
}

pub struct DebuggerBus {
    bus: Rc<RefCell<dyn Addressable>>,
    breakpoints: Rc<RefCell<Breakpoints>>,
    /// location of the most recent memory breakpoint, allows for running over it
    memory_trigger: Option<u64>,
}

impl DebuggerBus {
    fn new(bus: Rc<RefCell<dyn Addressable>>, breakpoints: Rc<RefCell<Breakpoints>>) -> DebuggerBus {
        DebuggerBus {
            bus,
            breakpoints,
            memory_trigger: None,
        }
    }

    fn check_read_breakpoint<U, T>(&mut self, offset_list: &[u64], f: T) -> Result<U, ReadWriteFault> 
    where
        T: FnOnce(&mut dyn Addressable) -> Result<U, ReadWriteFault>
    {
        for offset in offset_list {
            let breakpoint = { self.breakpoints.borrow().check_breakpoint(*offset, BP_READ) };
            if let Some(breakpoint) = breakpoint {
                if self.check_memory_trigger(breakpoint.address) {
                    info!(target: "DEBUGGER", "Memory breakpoint ${:016X} hit (R)", breakpoint.address);
                    return Err(ReadWriteFault::Break);
                }
            }
        }
        f(&mut *self.bus.borrow_mut())
    }

    fn check_write_breakpoint<T>(&mut self, offset_list: &[u64], f: T) -> Result<WriteReturnSignal, ReadWriteFault> 
    where
        T: FnOnce(&mut dyn Addressable) -> Result<WriteReturnSignal, ReadWriteFault>
    {
        for offset in offset_list {
            let breakpoint = { self.breakpoints.borrow().check_breakpoint(*offset, BP_WRITE) };
            if let Some(breakpoint) = breakpoint {
                if self.check_memory_trigger(breakpoint.address) {
                    info!(target: "DEBUGGER", "Memory breakpoint ${:016X} hit (W)", breakpoint.address);
                    return Err(ReadWriteFault::Break);
                }
            }
        }
        f(&mut *self.bus.borrow_mut())
    }

    /// returns true if the memory trigger is set to the specified address, and then clears it
    /// or sets it if it is not already set
    fn check_memory_trigger(&mut self, breakpoint_address: u64) -> bool {
        if self.memory_trigger.is_some_and(|address| breakpoint_address == address) {
            self.memory_trigger = None;
        } else {
            self.memory_trigger = Some(breakpoint_address);
        }
        self.memory_trigger.is_none()
    }
}

impl Addressable for DebuggerBus {
    fn read_u64(&mut self, offset: usize) -> Result<u64, ReadWriteFault> {
        self.check_read_breakpoint(&[offset as u64, (offset+4) as u64], |bus| {
            bus.read_u64(offset)
        })
    }

    fn write_u64(&mut self, value: u64, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.check_write_breakpoint(&[offset as u64], |bus| {
            bus.write_u64(value, offset)
        })
    }

    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        self.check_read_breakpoint(&[offset as u64], |bus| {
            bus.read_u32(offset)
        })
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.check_write_breakpoint(&[offset as u64], |bus| {
            bus.write_u32(value, offset)
        })
    }

    /// not every device needs to implement these, so defaults are provided
    fn read_u16(&mut self, offset: usize) -> Result<u16, ReadWriteFault> {
        self.check_read_breakpoint(&[offset as u64], |bus| {
            bus.read_u16(offset)
        })
    }

    fn write_u16(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.check_write_breakpoint(&[offset as u64], |bus| {
            bus.write_u16(value, offset)
        })
    }

    fn read_u8(&mut self, offset: usize) -> Result<u8, ReadWriteFault> {
        self.check_read_breakpoint(&[offset as u64], |bus| {
            bus.read_u8(offset)
        })
    }

    fn write_u8(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        self.check_write_breakpoint(&[offset as u64], |bus| {
            bus.write_u8(value, offset)
        })
    }
}
