#![allow(non_upper_case_globals)]
use std::mem;
use std::sync::mpsc;

#[allow(unused_imports)]
use tracing::{debug, error, info, warn, trace};

use crate::*;

pub const IMask_SP: u32 = 0;
pub const IMask_SI: u32 = 1;
pub const IMask_AI: u32 = 2;
pub const IMask_VI: u32 = 3;
pub const IMask_PI: u32 = 4;
pub const IMask_DP: u32 = 5;

pub const MI_INTERRUPT_NAMES: [&'static str; 6] = ["SP", "SI", "AI", "VI", "PI", "DP"];

#[derive(Debug)]
pub enum InterruptUpdateMode {
    EnableInterrupt,  // Set/clear the interrupt enable flag
    DisableInterrupt,
    SetInterrupt,     // Trigger/ack an previous interrupt
    ClearInterrupt,
    TriggerOnce,
}

#[derive(Debug)]
pub struct InterruptUpdate(pub u32, pub InterruptUpdateMode);

pub struct MipsInterface {
    comms: SystemCommunication,

    interrupt_mask: u32, // enabled interrupts
    interrupt     : u32, // current interrupt signals (does not require enabled interrupts)
    trigger_int   : u32, // interrupt signals that actually generate an external int (requires interrupt enable)

    repeat_count  : Option<u32>,

    interrupt_update_rx: mpsc::Receiver<InterruptUpdate>,
    interrupt_update_tx: mpsc::Sender<InterruptUpdate>,
}

impl MipsInterface {
    pub fn new(comms: SystemCommunication) -> MipsInterface {
        let (tx, rx) = mpsc::channel();

        MipsInterface { 
            comms,
            interrupt_mask: 0,
            interrupt     : 0,
            trigger_int   : 0,

            repeat_count  : None,

            interrupt_update_rx: rx,
            interrupt_update_tx: tx,
        }
    }

    pub fn reset(&mut self) {
        info!(target: "MI", "reset");
        self.interrupt_mask = 0;
        self.interrupt = 0;
        self.trigger_int = 0;
        self.repeat_count = None;
        
        while self.interrupt_update_rx.try_recv().is_ok() {}
    }

    pub fn get_update_channel(&mut self) -> mpsc::Sender<InterruptUpdate> {
        self.interrupt_update_tx.clone()
    }
    
    pub fn step(&mut self) {
        self.update_interrupts();
    }

    pub fn should_interrupt(&mut self) -> u32 {
        // mem::replace(&mut self.trigger_int, 0)
        self.comms.interrupt_register.load(Ordering::Relaxed) as u32
    }

    fn update_interrupts(&mut self) {
        while let Ok(update) = self.interrupt_update_rx.try_recv() {
            let bit = 1 << update.0;
            match update.1 {
                InterruptUpdateMode::EnableInterrupt => {
                    // println!("Enabling interrupt mask bit 0x{:02X}", bit);
                    self.interrupt_mask |= bit;
                },

                InterruptUpdateMode::DisableInterrupt => {
                    // println!("Disabling interrupt mask bit 0x{:02X}", bit);
                    self.interrupt_mask &= !bit;
                },

                InterruptUpdateMode::SetInterrupt => {
                    // println!("Setting interrupt bit 0x{:02X}", bit);
                    self.interrupt |= bit;
                },

                InterruptUpdateMode::ClearInterrupt => {
                    // println!("Clearing interrupt bit 0x{:02X}", bit);
                    self.interrupt &= !bit;
                },

                InterruptUpdateMode::TriggerOnce => {
                    todo!("need a new implementation");
                    // println!("Force triggering bit 0x{:02X}", bit);
                    // force trigger interrupt (used for debugging)
                    // self.trigger_int |= bit;
                },
            }
        }

        self.comms.interrupt_register.store((self.interrupt & self.interrupt_mask) as u8, Ordering::Relaxed);
    }

    pub fn get_repeat_count(&mut self) -> Option<u32> {
        mem::replace(&mut self.repeat_count, None)
    }
}

impl Addressable for MipsInterface {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "MI", "read32 offset=${:08X}", offset);

        let result = match offset {
            // MI_VERSION
            // https://n64brew.dev/wiki/MIPS_Interface#0x0430_0004_-_MI_VERSION
            0x0_0004 => {
                //trace!(target: "MI", "version read");
                0x0202_0102
            },

            // MI_INTERRUPT
            0x0_0008 => {
                // trace!(target: "MI", "read32 MI_INTERRUPT value=${:02X}", self.interrupt);
                self.update_interrupts();
                self.interrupt
            },

            // MI_MASK
            0x0_000C => {
                //trace!(target: "MI", "MI_MASK read (mask=${:08X})", self.interrupt_mask);
                self.update_interrupts();
                self.interrupt_mask
            }

            _ => {
                error!(target: "MI", "unimplemented read32 offset=${:08X}", offset);
                0
            },
        };

        Ok(result)
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "MI", "write32 value=${:08X} offset=${:08X}", value, offset);

        match offset {
            0x0_0000 => { 
                trace!(target: "MI", "write MI_MODE value=${:08X}", value);

                // RDP interrupt is cleared by writing to MI
                if (value & 0x800) != 0 {
                    self.comms.rdp_full_sync.store(0, Ordering::SeqCst);
                    self.interrupt &= !(1 << IMask_DP);
                }

                if (value & 0x100) != 0 {
                    self.repeat_count = Some((value & 0x7F) + 1);
                }

                if (value & 0x200) != 0 {
                    self.repeat_count = None;
                }

            },

            // MI_INTERRUPT
            0x0_0008 => {
                warn!(target: "MI", "ignoring write to MI_INTERRUPT");
            },

            0x0_000C => {
                trace!(target: "MI", "write MI_MASK value=${:08X}", value);

                for mask in [IMask_DP, IMask_PI, IMask_VI, IMask_AI, IMask_SI, IMask_SP] {
                    // Set mask
                    if (value & (1 << ((mask * 2) + 1))) != 0 {
                        self.interrupt_mask |= 1 << mask;
                    }

                    // Clear mask
                    if (value & (1 << (mask * 2))) != 0 {
                        self.interrupt_mask &= !(1 << mask);
                    }
                }
            },

            _ => panic!("MI: unhandled write32 ${:08X}", offset),
        };

        Ok(WriteReturnSignal::None)
    }
}


