use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use rustyline::error::ReadlineError;
use rustyline::DefaultEditor;
use rustyline::Result as RustylineResult;

use crate::*;
use crate::cpu::Cpu;

const BP_READ : u8 = 0x01;
const BP_WRITE: u8 = 0x02;
const BP_EXEC : u8 = 0x04;

struct Breakpoint {
    id     : u64,
    address: u64,
    mode   : u8,
    enable : bool,
}

pub struct Debugger<T: Addressable> {
    alive: bool,
    cpu: Cpu<T>,
    ctrlc_count: u32,

    cpu_run_til: Option<u32>,

    // Ctrl-C help
    cpu_running: Arc<AtomicBool>,

    breakpoint_id: u64,
    breakpoints: HashMap<u64, Breakpoint>,
}

impl<T: Addressable> Debugger<T> {
    pub fn new(cpu: Cpu<T>) -> Debugger<T> {
        let cpu_running = Arc::new(AtomicBool::new(false));
        let r = cpu_running.clone();

        ctrlc::set_handler(move || {
            println!("Break!");
            r.store(false, Ordering::SeqCst);
        }).expect("Error setting ctrl-c handler");

        Debugger {
            alive: true,
            cpu: cpu,
            ctrlc_count: 0,
            cpu_run_til: None,
            cpu_running: cpu_running,
            breakpoint_id: 0,
            breakpoints: HashMap::new(),
        }
    }

    pub fn run(&mut self) -> RustylineResult<()> {
        let mut rl = DefaultEditor::new()?;

        if rl.load_history("history.txt").is_err() {
            println!("No history.txt file");
        }

        let mut lastline = String::from("");
        while self.alive {
            let prompt = format!("<PC:${:08X}>@ ", self.cpu.next_instruction_pc());
            let readline = rl.readline(&prompt);

            match readline {
                RustylineResult::Ok(line) => {
                    let mut line_str = String::from(line.as_str().trim());

                    if line_str.len() == 0 {
                        line_str = lastline.clone();
                    }

                    lastline = line_str.clone();

                    if line_str.len() > 0 {
                        rl.add_history_entry(line_str.as_str())?;
                        if let Err(err) = self.handle_line(line_str.as_str()) {
                            println!("error: {}", err);
                        }
                    }

                    self.ctrlc_count = 0;
                },

                RustylineResult::Err(ReadlineError::Interrupted) => {
                    self.ctrlc_count += 1;
                    if self.ctrlc_count == 3 { 
                        println!("Exiting...");
                        break; 
                    }
                    else if self.ctrlc_count == 1 {
                        println!("Ctrl-C, press twice more to exit");
                    }
                },

                RustylineResult::Err(ReadlineError::Eof) => {
                    println!("Exiting...");
                    break;
                },

                RustylineResult::Err(err) => {
                    panic!("ReadlineError: {}", err);
                },
            };
        };

        rl.save_history("history.txt")?;

        Ok(())
    }

    fn handle_line(&mut self, line: &str) -> Result<(), String> {
        let lines = line.split(";").collect::<Vec<&str>>();
        for line in lines {
            let parts = line.split_whitespace().collect::<Vec<&str>>();

            if parts.len() == 0 { return Ok(()); }

            let result = match parts[0] {
                "r" | "ru" | "run"          => { self.run_cpu(&parts) },
                "s" | "st" | "ste" | "step" => { self.step(&parts) }
                "res" | "rese" | "reset"    => { self.reset(&parts) },
                "regs" | "rd"               => { self.dump_regs(&parts) },
                "rw"                        => { self.dump_regs_as_words(&parts) },
                "b" | "br" | "bre" | "brea"
                 | "break" | "bp"           => { self.breakpoint(&parts) },
                "db" | "del" | "dbr" 
                 | "dbrea" | "dbreak"       => { self.delete_breakpoint(&parts) },

                _ => {
                    Err(format!("unsupported debugger command \"{}\"", parts[0]))
                },
            };

            if let Err(_) = result { return result; }
        }

        Ok(())
    }

    fn run_cpu(&mut self, parts: &Vec<&str>) -> Result<(), String> {
        self.cpu_run_til = None;
        self.cpu_running.store(true, Ordering::SeqCst);

        if parts.len() > 1 {
            self.cpu_run_til = match parse_int(&parts[1]) {
                Ok(v) => Some(v as u32),
                Err(err) => {
                    return Err(err);
                }
            };
        }

        while self.cpu_running.load(Ordering::SeqCst) {
            self.cpu.step();

            // Check breakpoints
            if let Some(breakpoint) = self.check_breakpoint((*self.cpu.next_instruction_pc() as i32) as u64, BP_EXEC) {
                println!("Breakpoint ${:016X} hit", breakpoint.address);
                self.cpu_running.store(false, Ordering::SeqCst);
            }

            // Check run until
            match self.cpu_run_til {
                Some(v) => {
                    if *self.cpu.next_instruction_pc() == v {
                        self.cpu_running.store(false, Ordering::SeqCst);
                    }
                },
                None => {}
            };
        }

        Ok(())
    }

    fn step(&mut self, parts: &Vec<&str>) -> Result<(), String> {
        let mut count = if parts.len() > 1 {
            match parse_int(&parts[1]) {
                Ok(v) => v as u64,
                Err(err) => {
                    return Err(err);
                }
            }
        } else {
            1
        };

        self.cpu_running.store(true, Ordering::SeqCst);
        while count > 0 && self.cpu_running.load(Ordering::SeqCst) {
            self.cpu.step();

            // update step count
            count -= 1;

            // Check breakpoints
            if let Some(breakpoint) = self.check_breakpoint((*self.cpu.next_instruction_pc() as i32) as u64, BP_EXEC) {
                println!("Breakpoint ${:016X} hit", breakpoint.address);
                self.cpu_running.store(false, Ordering::SeqCst);
            }
        }

        Ok(())
    }

    fn reset(&mut self, _: &Vec<&str>) -> Result<(), String> {
        self.cpu.reset();
        Ok(())
    }

    fn dump_regs(&mut self, _: &Vec<&str>) -> Result<(), String> {
        let regs = self.cpu.regs();
        
        for k in 0..8 {
            for j in 0..4 {
                print!("R{:02}(${}): ${:08X}_{:08X} ", k*4+j, self.cpu.abi_name(k*4+j), regs[(k*4+j) as usize] >> 32, regs[(k*4+j) as usize] & 0xFFFF_FFFF);
            }
            println!("");
        }

        println!("PC: ${:08X}", self.cpu.next_instruction_pc());

        Ok(())
    }

    fn dump_regs_as_words(&mut self, _: &Vec<&str>) -> Result<(), String> {
        let regs = self.cpu.regs();
        
        for k in 0..4 {
            for j in 0..8 {
                print!("R{:02}(${}): {:08X} ", k*8+j, self.cpu.abi_name(k*8+j), regs[(k*8+j) as usize] & 0xFFFF_FFFF);
            }
            println!("");
        }

        println!("PC: ${:08X}", self.cpu.next_instruction_pc());

        Ok(())
    }

    fn breakpoint(&mut self, parts: &Vec<&str>) -> Result<(), String> {
        if parts.len() == 1 {
            for (_key, v) in self.breakpoints.iter() {
                println!("{}: ${:016X} mode {}", v.id, v.address, format_breakpoint_mode(v.mode));
            }
            Ok(())
        } else {
            let breakpoint_address = match parse_int(&parts[1]) {
                Err(err) => { return Err(err); },
                Ok(v) => {
                    if (v as u64) < 0x1_0000_0000 { // sign extend 32-bit value
                        (v as i32) as u64
                    } else {
                        v as u64 // 64-bit
                    }
                },
            };

            let mode = if parts.len() > 2 {
                let mode_str = parts[2];
                let mut mode_result: u8 = 0;
                for c in mode_str.as_bytes() {
                    mode_result |= match c {
                        b'r' | b'R' => { BP_READ },
                        b'w' | b'W' => { BP_WRITE },
                        b'x' | b'X' => { BP_EXEC },
                        _ => { return Err(format!("invalid format option '{}' (only rwx are valid)", c)); },
                    };
                }
                mode_result
            } else {
                // default to 'x' only
                BP_EXEC
            };

            let id = self.breakpoint_id;
            self.breakpoint_id += 1;

            self.breakpoints.insert(breakpoint_address, Breakpoint {
                id     : id,
                address: breakpoint_address,
                mode   : mode,
                enable : true,
            });

            println!("breakpoint set at ${:016X} (mode {})", breakpoint_address, format_breakpoint_mode(mode));

            Ok(())
        }
    }

    fn delete_breakpoint(&mut self, parts: &Vec<&str>) -> Result<(), String> {
        if parts.len() != 2 {
            return Err(format!("usage: db [breakpoint id]"));
        }

        let search_id = match parse_int(&parts[1]) {
            Err(err) => { return Err(err); },
            Ok(v) => { v as u64 },
        };

        let mut found_key: Option<u64> = None;
        for (key, v) in self.breakpoints.iter() {
            if v.id == search_id {
                found_key = Some(*key);
            }
        }

        if let Some(key) = found_key {
            self.breakpoints.remove(&key);
        } else {
            return Err(format!("breakpoint id {} not valid", search_id));
        }

        Ok(())
    }

    fn check_breakpoint(&mut self, address: u64, mode: u8) -> Option<&Breakpoint> {
        if let Some(breakpoint) = self.breakpoints.get(&address) {
            if breakpoint.enable && ((breakpoint.mode & mode) != 0) {
                return Some(breakpoint);
            }
        }

        None
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
