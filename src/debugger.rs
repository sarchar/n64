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
    cpu_running: bool,

    // Ctrl-C help
    interrupted: Arc<AtomicBool>,
}

impl<T: Addressable> Debugger<T> {
    pub fn new(cpu: Cpu<T>) -> Debugger<T> {
        let interrupted = Arc::new(AtomicBool::new(false));
        let int = interrupted.clone();

        ctrlc::set_handler(move || {
            println!("Break!");
            int.store(true, Ordering::SeqCst);
        }).expect("Error setting ctrl-c handler");

        Debugger {
            alive: true,
            cpu: cpu,
            ctrlc_count: 0,
            cpu_running: false,
            interrupted: interrupted,
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
                self.run_cpu();
                Ok(())
            },

            _ => {
                Err(format!("unsupported debugger command \"{}\"", parts[0]))
            },
        }
    }

    fn run_cpu(&mut self) {
        self.interrupted.store(false, Ordering::SeqCst);

        while !self.interrupted.load(Ordering::SeqCst) {
            self.step();
        }
    }

    fn step(&mut self) {
        self.cpu.step();
    }
}
