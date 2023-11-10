use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use rustyline::error::ReadlineError;
use rustyline::DefaultEditor;
use rustyline::Result as RustylineResult;

use crate::*;
use crate::cpu::Cpu;

pub struct Debugger<T: Addressable> {
    alive: bool,
    cpu: Cpu<T>,
    ctrlc_count: u32,

    cpu_run_til: Option<u32>,

    // Ctrl-C help
    cpu_running: Arc<AtomicBool>,
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
        }
    }

    pub fn run(&mut self) -> RustylineResult<()> {
        let mut rl = DefaultEditor::new()?;

        if rl.load_history("history.txt").is_err() {
            println!("No history.txt file");
        }

        while self.alive {
            let readline = rl.readline("-> ");
            match readline {
                RustylineResult::Ok(line) => {
                    let line_str = line.as_str();
                    if line_str.len() > 0 {
                        rl.add_history_entry(line_str)?;
                        if let Err(err) = self.handle_line(line_str) {
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
        let parts = line.split_whitespace().collect::<Vec<&str>>();

        if parts.len() == 0 { return Ok(()); }

        match parts[0] {
            "r" | "ru" | "run" => {
                self.run_cpu(&parts)
            },

            "res" | "rese" | "reset" => {
                self.reset(&parts)
            },

            _ => {
                Err(format!("unsupported debugger command \"{}\"", parts[0]))
            },
        }
    }

    fn run_cpu(&mut self, parts: &Vec<&str>) -> Result<(), String> {
        self.cpu_run_til = None;
        self.cpu_running.store(true, Ordering::SeqCst);

        if parts.len() > 1 {
            let until = &parts[1];
            if &until[0..1] == "$" {
                match u32::from_str_radix(until.trim_start_matches("$"), 16) {
                    Ok(v) => {
                        self.cpu_run_til = Some(v);
                    },
                    Err(err) => {
                        return Err(err.to_string());
                    },
                };
            }
        }

        while self.cpu_running.load(Ordering::SeqCst) {
            self.step();

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

    fn step(&mut self) {
        self.cpu.step();
    }

    fn reset(&mut self, _: &Vec<&str>) -> Result<(), String> {
        self.cpu.reset();
        Ok(())
    }
}
