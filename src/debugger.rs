use std::cell::{RefCell, RefMut};
use std::rc::Rc;

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use crate::*;

use crossbeam::channel::{self, Receiver, Sender};
use mips::{InterruptUpdate, InterruptUpdateMode};
use tracing::{error, warn};

use crate::cpu::InstructionFault;

pub const BP_READ : u8 = 0x01;
pub const BP_WRITE: u8 = 0x02;
pub const BP_EXEC : u8 = 0x04;

#[derive(Default)]
pub struct CpuStateInfo {
    pub next_instruction_pc: u64,
    pub running: bool,
    pub instruction_memory: Option<Vec<u32>>,
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
    /// Read memory
    ReadBlock(u64, u64, usize), // (id, address, size in words)
    /// Get list of breakpoints
    GetBreakpoints,
    /// Set a new breakpoint. The id field will be overwritten with a new ID
    SetBreakpoint(BreakpointInfo),
    /// Enable/disable the breakpoint.
    EnableBreakpoint(u64, bool), // (virtual address of breakpoint, enable)
    /// Remove an existing breakpoint
    RemoveBreakpoint(u64), // virtual address of a breakpoint
}

pub enum DebuggerCommandResponse {
    /// Response to [DebuggerCommandRequest::GetCpuState] containing the current CpuStateInfo structure
    CpuState(CpuStateInfo),

    CpuRegisters([u64; 32]),
    ReadBlock(u64, Option<Vec<u32>>), // id, data

    /// Response to [DebuggerCommandReqeust::GetBreakpoints] containing a list of [BreakpointInfo]s
    Breakpoints(HashMap<u64, BreakpointInfo>),
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
    /// physical address of the above
    pub physical_address: u64,
    /// RWX flags
    pub mode   : u8,
    /// bp won't trigger if enable is clear
    pub enable : bool,
    /// word at the location (if mode & BP_EXEC)
    pub memory : u32,
}

struct Breakpoints {
    global_enable: bool,
    table: HashMap<u64, BreakpointInfo>,
}

impl Breakpoints {
    fn new() -> Breakpoints {
        Breakpoints {
            global_enable: true,
            table: HashMap::new(),
        }
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

    fn lookup_breakpoint(&self, virtual_address: u64) -> Option<BreakpointInfo> {
        self.table.get(&virtual_address).map(|v| *v)
    }

    fn print_breakpoints(&self) {
        for (_key, v) in self.table.iter() {
            println!("${:016X} mode {}", v.address, format_breakpoint_mode(v.mode));
        }
    }

    fn add_breakpoint(&mut self, cpu: RefMut<'_, cpu::Cpu>, address: u64, mode: u8, enable: bool) {
        let breakpoint_info = BreakpointInfo { address, mode, enable, ..Default::default() };
        self.add_breakpoint_info(cpu, breakpoint_info);
    }

    fn add_breakpoint_info(&mut self, mut cpu: RefMut<'_, cpu::Cpu>, mut breakpoint_info: BreakpointInfo) {
        let enable = breakpoint_info.enable;
        let virtual_address = breakpoint_info.address;
        let is_execute = (breakpoint_info.mode & BP_EXEC) != 0;

        if self.table.contains_key(&virtual_address) {
            warn!(target: "DEBUGGER", "can't duplicate breakpoints");
            return;
        }
        
        if enable && is_execute {
            // no exception here, so do the translate and call read_u32_phys
            if let Some(address) = cpu.translate_address(virtual_address, false, true).expect("please dont' fail") {
                breakpoint_info.physical_address = address.physical_address;
                breakpoint_info.memory = cpu.read_u32_phys(address).expect("please don't fail");
                cpu.write_u32_phys(0x0000000D, address).expect("please don't fail"); // break instruction
            } else {
                error!(target: "DEBUGGER", "Unable to add breakpoint at virtual address ${:08X}", virtual_address);
            }
        }

        self.table.insert(breakpoint_info.address, breakpoint_info);
    }

    fn remove_breakpoint(&mut self, mut cpu: RefMut<'_, cpu::Cpu>, virtual_address: u64) -> Result<(), ()> {
        if let Some(info) = self.table.remove(&virtual_address) {
            if info.enable && (info.mode & BP_EXEC) != 0 {
                cpu.write_u32_phys_direct(info.memory, info.physical_address).expect("please don't fail");
            }
            Ok(())
        } else {
            warn!(target: "DEBUGGER", "cannot remove invalid breakpoint ${:016X}", virtual_address);
            Err(())
        }
    }

    /// returns the old state
    fn enable_breakpoint(&mut self, mut cpu: RefMut<'_, cpu::Cpu>, virtual_address: u64, enable: bool) -> Result<bool, ()> {
        if let Some(mut info) = self.table.get_mut(&virtual_address) {
            if info.enable && !enable {
                // return the memory at the address to the real instruction
                if (info.mode & BP_EXEC) != 0 {
                    cpu.write_u32_phys_direct(info.memory, info.physical_address).expect("please don't fail");
                }
            } else if !info.enable && enable {
                // re-set the `break` instruction
                if (info.mode & BP_EXEC) != 0 {
                    cpu.write_u32_phys_direct(0x0000000D, info.physical_address).expect("please don't fail");
                }
            }

            Ok(std::mem::replace(&mut info.enable, enable))
        } else {
            Err(())
        }       
    }
}

pub struct Debugger {
    exit_requested: bool,
    cpu_running: bool,
    cpu_broke: bool,
    cpu_run_for: u64,

    ctrlc_count: u32,

    cpu_run_til: Option<u64>,

    // Ctrl-C help
    // cpu_running: Arc<AtomicBool>,

    breakpoints: Rc<RefCell<Breakpoints>>,

    system: System,

    // debugging commands
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

        let breakpoints = Rc::new(RefCell::new(Breakpoints::new()));

        // // replace the bus the CPU is connected to to our debugger bus
        // let old_bus = system.cpu.borrow().bus.clone();

        // system.cpu.borrow_mut().bus = Rc::new(RefCell::new(DebuggerBus::new(old_bus, breakpoints.clone())));

        Debugger {
            exit_requested  : false,
            cpu_running     : true,
            cpu_broke       : false, // true when the CPU stops due to a debugger breakpoint
            cpu_run_for     : 0,     // first call to run_for() won't tick the CPU, it'll just calculate how many cycles to run for
            ctrlc_count     : 0,
            cpu_run_til     : None,
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
        // self.cpu_running = false;
        while !self.exit_requested { 
            if self.cpu_running {
                self.cpu_run_for = self.run_for(self.cpu_run_for); 
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
        match self.system.run_for(num_cycles) {
            Ok(next_cycles) => next_cycles,
            Err(cpu::InstructionFault::Break) => {
                let pc = self.system.cpu.borrow().next_instruction_pc();
                println!("CPU break at virtual address ${:08X}", pc);

                // if PC is one of our breakpoints, stop the CPU, otherwise call the exception handler and continue running
                let mut is_game_breakpoint = true;
                if let Some(breakpoint_info) = self.breakpoints.borrow().table.get(&pc) {
                    // we have a breakpoint here, but only if the breakpoint is disabled then it was actually
                    // a game breakpoint and we need to 
                    is_game_breakpoint = !breakpoint_info.enable;
                }

                if is_game_breakpoint {
                    // will return Err(InstructionFault::OtherExcerption(ExceptionCode_Bp))
                    let _ = self.system.cpu.borrow_mut().breakpoint_exception();
                } else {
                    // stop the CPU
                    self.cpu_running = false;

                    // when the cpu continues running via the RunCpu command, we must execute one instruction via the interpreter
                    self.cpu_broke = true;
                }

                // recalculate cycles that can be run
                self.system.rcp.borrow().calculate_free_cycles()
            },
            _ => panic!("unhandled CPU exception"),
        }
    }

    fn run_interpreter_with_instruction(&mut self, inst: u32, num_cycles: u64) -> u64 {
        let old_interpreter_flag = self.system.comms.settings.read().unwrap().cpu_interpreter_only;
        self.system.comms.settings.write().unwrap().cpu_interpreter_only = true;

        // set the next instruction to the breakpoint memory at the current address
        let pc = self.system.cpu.borrow().next_instruction_pc();
        let memory = self.breakpoints.borrow().table.get(&pc).unwrap().memory;
        self.system.cpu.borrow_mut().set_next_instruction(memory);

        // run for 1 instruction
        let next_cycles = self.run_for(num_cycles); 

        // restore the interpreter flag to continuing running JIT code
        self.system.comms.settings.write().unwrap().cpu_interpreter_only = old_interpreter_flag;

        next_cycles
    }

    fn update(&mut self) {
        // active when at least 1 debugging window is open
        if self.system.comms.debugger_windows.load(Ordering::Relaxed) == 0 { return; }

        'next_command: while let Ok(req) = self.command_receiver.try_recv() {
            match req.command_request {
                DebuggerCommandRequest::GetCpuState(read_instruction_memory) => {
                    let response_channel = if let Some(r) = req.response_channel { r } else { continue 'next_command; };

                    let mut cpu = self.system.cpu.borrow_mut();
                    let next_instruction_pc = cpu.next_instruction_pc();
                    let instruction_memory = if let Some((instruction_offset, instruction_count)) = read_instruction_memory {
                        let virtual_address = (next_instruction_pc as i64).wrapping_add(instruction_offset * 4);
                        if let Some(address) = cpu.translate_address(virtual_address as u64, false, false).unwrap() {
                            // println!("next instruction pc = ${:08X}, physical_address=${:08X}, instruction_offset={}", next_instruction_pc, address.physical_address, instruction_offset);
                            let memory = self.system.rcp.borrow_mut().read_block(address.physical_address as usize, (instruction_count * 4) as u32).unwrap();
                            Some(memory)
                        } else {
                            None
                        }
                    } else { 
                        None
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
                    let pc = self.system.cpu.borrow().next_instruction_pc();
                    let breakpoint_info = self.breakpoints.borrow().check_breakpoint(pc, BP_EXEC);
                    if let Some(breakpoint_info) = breakpoint_info {
                        self.cpu_run_for = self.run_interpreter_with_instruction(breakpoint_info.memory, 1);
                    } else {
                        self.cpu_run_for = 0;
                    }

                    self.cpu_running = true;
                    if self.system.comms.cpu_throttle.load(Ordering::Relaxed) == 1 { // if throttling is enabled, skip for a single call to avoid a speedup
                        self.system.comms.cpu_throttle.store(2, Ordering::Relaxed);
                    }
                },

                DebuggerCommandRequest::StepCpu(mut num_cycles) => {
                    if self.cpu_running || num_cycles == 0 { continue 'next_command; }

                    // if the current instruction is a breakpoint, we have to execute the instruction in BreakpointInfo::memory instead
                    let pc = self.system.cpu.borrow().next_instruction_pc();
                    let breakpoint_info = self.breakpoints.borrow().check_breakpoint(pc, BP_EXEC);
                    if let Some(breakpoint_info) = breakpoint_info {
                        self.cpu_run_for = self.run_interpreter_with_instruction(breakpoint_info.memory, 1);
                        num_cycles -= 1;
                    }

                    if num_cycles > 0 {
                        self.cpu_run_for = self.run_for(num_cycles);
                    }
                },

                DebuggerCommandRequest::ReadBlock(id, virtual_address, size_in_words) => {
                    let response_channel = if let Some(r) = req.response_channel { r } else { continue 'next_command; };

                    let memory = {
                        let mut cpu = self.system.cpu.borrow_mut();
                        if let Some(address) = cpu.translate_address(virtual_address as u64, false, false).unwrap() {
                            // println!("next instruction pc = ${:08X}, physical_address=${:08X}, instruction_offset={}", next_instruction_pc, address.physical_address, instruction_offset);
                            let memory = self.system.rcp.borrow_mut().read_block(address.physical_address as usize, (size_in_words * 4) as u32).unwrap();
                            Some(memory)
                        } else {
                            None
                        }
                    };

                    let _ = response_channel.send(DebuggerCommandResponse::ReadBlock(id, memory));
                },

                DebuggerCommandRequest::GetBreakpoints => {
                    let response_channel = if let Some(r) = req.response_channel { r } else { continue 'next_command; };
                    let breakpoints: HashMap<u64, BreakpointInfo> = self.breakpoints.borrow().table.clone();
                    response_channel.send(DebuggerCommandResponse::Breakpoints(breakpoints)).unwrap();
                },

                DebuggerCommandRequest::SetBreakpoint(breakpoint_info) => {
                    let cpu = self.system.cpu.borrow_mut();
                    self.breakpoints.borrow_mut().add_breakpoint_info(cpu, breakpoint_info);
                },

                DebuggerCommandRequest::EnableBreakpoint(virtual_address, enable) => {
                    let cpu = self.system.cpu.borrow_mut();
                    let _ = self.breakpoints.borrow_mut().enable_breakpoint(cpu, virtual_address, enable);
                }

                DebuggerCommandRequest::RemoveBreakpoint(virtual_address) => {
                    let cpu = self.system.cpu.borrow_mut();
                    let _ = self.breakpoints.borrow_mut().remove_breakpoint(cpu, virtual_address);
                },
            }
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

    // fn step(&mut self, parts: &Vec<&str>) -> Result<(), String> {
    //     let mut count = if parts.len() > 1 {
    //         match parse_int(&parts[1]) {
    //             Ok(v) => v as u64,
    //             Err(err) => {
    //                 return Err(err);
    //             }
    //         }
    //     } else {
    //         1
    //     };

    //     self.cpu_running.store(true, Ordering::SeqCst);
    //     while count > 0 && self.cpu_running.load(Ordering::SeqCst) {
    //         // Break loop on any instruction error
    //         if let Err(_) = self.system.run_for(1) {
    //             break;
    //         }

    //         // update step count
    //         count -= 1;

    //         // Check breakpoints
    //         if let Some(breakpoint) = self.breakpoints.borrow().check_breakpoint((self.system.cpu.borrow().next_instruction_pc() as i32) as u64, BP_EXEC) {
    //             println!("Breakpoint ${:016X} hit", breakpoint.address);
    //             self.cpu_running.store(false, Ordering::SeqCst);
    //         }
    //     }

    //     Ok(())
    // }

    fn reset(&mut self, _: &Vec<&str>) -> Result<(), String> {
        self.system.reset();
        Ok(())
    }

    fn dump_regs(&mut self, _: &Vec<&str>) -> Result<(), String> {
        let cpu = self.system.cpu.borrow();
        let regs = cpu.regs();
        
        for k in 0..8 {
            for j in 0..4 {
                print!("R{:02}(${}): ${:08X}_{:08X} ", k*4+j, cpu::Cpu::abi_name(k*4+j), regs[(k*4+j) as usize] >> 32, regs[(k*4+j) as usize] & 0xFFFF_FFFF);
            }
            println!("");
        }

        println!("PC: ${:08X}", cpu.next_instruction_pc());

        Ok(())
    }

    fn dump_regs_as_words(&mut self, _: &Vec<&str>) -> Result<(), String> {
        let cpu = self.system.cpu.borrow();
        let regs = cpu.regs();
        
        for k in 0..4 {
            for j in 0..8 {
                print!("R{:02}(${}): {:08X} ", k*8+j, cpu::Cpu::abi_name(k*8+j), regs[(k*8+j) as usize] & 0xFFFF_FFFF);
            }
            println!("");
        }

        println!("PC: ${:08X}", cpu.next_instruction_pc());

        Ok(())
    }

    // fn breakpoint(&mut self, parts: &Vec<&str>) -> Result<(), String> {
    //     if parts.len() == 1 {
    //         self.breakpoints.borrow().print_breakpoints();
    //         Ok(())
    //     } else {
    //         let breakpoint_address = match parse_int(&parts[1]) {
    //             Err(err) => { return Err(err); },
    //             Ok(v) => {
    //                 if (v as u64) < 0x1_0000_0000 { // sign extend 32-bit value
    //                     (v as i32) as u64
    //                 } else {
    //                     v as u64 // 64-bit
    //                 }
    //             },
    //         };

    //         let mode = if parts.len() > 2 {
    //             let mode_str = parts[2];
    //             let mut mode_result: u8 = 0;
    //             for c in mode_str.as_bytes() {
    //                 mode_result |= match c {
    //                     b'r' | b'R' => { BP_READ },
    //                     b'w' | b'W' => { BP_WRITE },
    //                     b'x' | b'X' => { BP_EXEC },
    //                     _ => { return Err(format!("invalid format option '{}' (only rwx are valid)", c)); },
    //                 };
    //             }
    //             mode_result
    //         } else {
    //             // default to 'x' only
    //             BP_EXEC
    //         };

    //         self.breakpoints.borrow_mut().add_breakpoint(breakpoint_address, mode, true);

    //         println!("breakpoint set at ${:016X} (mode {})", breakpoint_address, format_breakpoint_mode(mode));

    //         Ok(())
    //     }
    // }

    // fn delete_breakpoint(&mut self, parts: &Vec<&str>) -> Result<(), String> {
    //     if parts.len() != 2 {
    //         return Err(format!("usage: db [breakpoint id]"));
    //     }

    //     let search_id = match parse_int(&parts[1]) {
    //         Err(err) => { return Err(err); },
    //         Ok(v) => { v as u64 },
    //     };

    //     self.breakpoints.borrow_mut().delete_breakpoint(search_id)?;

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

    fn listing(&mut self, parts: &Vec<&str>) -> Result<(), String> {
        let mut cpu = self.system.cpu.borrow_mut();
        let start_pc = cpu.next_instruction_pc(); // TODO set from an argument
        let mut count = 10;

        if parts.len() >= 3 {
            return Err(format!("usage: l[ist] [count (default 10)]"));
        }

        if parts.len() == 2 {
            count = match parse_int(&parts[1]) {
                Err(err) => { return Err(err); },
                Ok(v) => { if v < 0 { return Err(format!("number must be positive")); } v as u32 },
            };
        }

        for i in 0..count {
            let addr = start_pc + (i as u64) * 4;
            print!("${:08X}", addr);

            if let Ok(Some(address)) = cpu.translate_address(addr, false, false) {
                if let Ok(op) = cpu.bus.borrow_mut().read_u32(address.physical_address as usize) {
                    // let inst = cpu::Cpu::disassemble(addr, op, true);
                    // print!("(${:08X}): {}", op, inst);
                } else {
                    print!(": <unaccessable>");
                }
            } else {
                print!(": <address not mapped>");
            }

            if addr == cpu.next_instruction_pc() {
                print!(" <-");
            }

            println!("");
        }

        Ok(())
    }

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

pub struct DebuggerBus {
    bus: Rc<RefCell<dyn Addressable>>,
    breakpoints: Rc<RefCell<Breakpoints>>,
}

impl DebuggerBus {
    fn new(bus: Rc<RefCell<dyn Addressable>>, breakpoints: Rc<RefCell<Breakpoints>> ) -> DebuggerBus {
        DebuggerBus {
            bus: bus,
            breakpoints: breakpoints,
        }
    }

    //pub fn set_read_u32<F>(&mut self, read_u32: F)
    //    where F: Fn(usize) -> u32 {
    //    println!("{}", read_u32(0xbfc00000));
    //}
}

impl Addressable for DebuggerBus {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        if let Some(breakpoint) = self.breakpoints.borrow().check_breakpoint(offset as u64, BP_READ) {
            println!("Breakpoint ${:016X} hit", breakpoint.address);
            return Err(ReadWriteFault::Break);
        }

        self.bus.borrow_mut().read_u32(offset)
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        if let Some(breakpoint) = self.breakpoints.borrow().check_breakpoint(offset as u64, BP_WRITE) {
            println!("Breakpoint ${:016X} hit", breakpoint.address);
            return Err(ReadWriteFault::Break);
        }

        self.bus.borrow_mut().write_u32(value, offset)
    }

    /// not every device needs to implement these, so defaults are provided
    fn read_u16(&mut self, offset: usize) -> Result<u16, ReadWriteFault> {
        if let Some(breakpoint) = self.breakpoints.borrow().check_breakpoint(offset as u64, BP_READ) {
            println!("Breakpoint ${:016X} hit", breakpoint.address);
            return Err(ReadWriteFault::Break);
        }

        self.bus.borrow_mut().read_u16(offset)
    }

    fn write_u16(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        if let Some(breakpoint) = self.breakpoints.borrow().check_breakpoint(offset as u64, BP_WRITE) {
            println!("Breakpoint ${:016X} hit", breakpoint.address);
            return Err(ReadWriteFault::Break);
        }

        self.bus.borrow_mut().write_u16(value, offset)
    }

    fn read_u8(&mut self, offset: usize) -> Result<u8, ReadWriteFault> {
        if let Some(breakpoint) = self.breakpoints.borrow().check_breakpoint(offset as u64, BP_READ) {
            println!("Breakpoint ${:016X} hit", breakpoint.address);
            return Err(ReadWriteFault::Break);
        }

        self.bus.borrow_mut().read_u8(offset)
    }

    fn write_u8(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        if let Some(breakpoint) = self.breakpoints.borrow().check_breakpoint(offset as u64, BP_WRITE) {
            println!("Breakpoint ${:016X} hit", breakpoint.address);
            return Err(ReadWriteFault::Break);
        }

        self.bus.borrow_mut().write_u8(value, offset)
    }
}

fn parse_int(s: &str) -> Result<i64, String> {
    if &s[0..1] == "$" {
        match i64::from_str_radix(s.trim_start_matches("$"), 16) {
            Ok(v) => { Ok(v) },
            Err(err) => {
                Err(err.to_string())
            },
        }
    } else {
        match i64::from_str_radix(s, 10) {
            Ok(v) => { Ok(v) },
            Err(err) => {
                Err(err.to_string())
            },
        }
    }
}

fn format_breakpoint_mode(mode: u8) -> String {
    let mut res = String::new();
    if (mode & BP_READ) != 0 { res.push_str("r"); }
    if (mode & BP_WRITE) != 0 { res.push_str("w"); }
    if (mode & BP_EXEC) != 0 { res.push_str("x"); }
    res
}

