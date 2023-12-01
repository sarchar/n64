use std::sync::{mpsc, Arc, Mutex};
use std::thread;

use tracing::{trace, debug, error, info, warn};

use crate::*;

/// N64 Reality Signal Processor
/// Resides on the die of the RCP.
pub struct Rsp {
    mem: Vec<u32>,

    pc: u32,

    dma_busy: bool,

    semaphore: bool,

    core: Arc<Mutex<RspCpuCore>>,

    wakeup_tx: Option<mpsc::Sender<()>>,
    broke_rx: Option<mpsc::Receiver<()>>,

    // these are copies of state in RspCpuCore so we don't need a lock
    halted: bool,
    broke: bool,
}

#[derive(Debug, Copy, Clone, Default)]
struct RspCpuCore {
    halted: bool,
    broke: bool,
}

impl Rsp {
    pub fn new() -> Rsp {
        let core = Arc::new(Mutex::new(RspCpuCore::default()));
        core.lock().unwrap().reset();

        Rsp {
            mem: vec![0u32; 2*1024], // 8KiB
            pc: 0x0000_0000,
            semaphore: false,
            core: core,

            dma_busy: false,

            wakeup_tx: None,
            broke_rx: None,

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

        thread::spawn(move || {
            loop {
                let mut c = core.lock().unwrap();

                // halted can only be set while we don't have a lock, so check here
                if c.halted || c.broke {
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
                        c.step();

                        if c.broke { 
                            c.halted = true; // signal the core is halted before dropping the lock

                            // write break signal to channel
                            broke_tx.send(()).unwrap();
                            break; 
                        }
                    }
                }
            }
        });
    }

    //pub fn step(&mut self) {
    //    // don't step if halted
    //    if (self.si_status & 0x01) != 0 { return; }

    //    if let Ok(inst) = self.read_u32(self.pc as usize | 0x1000) {
    //        //info!(target: "RSP", "${:08X}: ${:08X}", self.pc | 0x1000, inst);
    //        self.pc = (self.pc + 4) & 0x0FFC;
    //    }
    //}

    fn read_register(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RSP", "read32 register offset=${:08X}", offset);

        let value = match offset {
            // SP_STATUS
            0x4_0010 => {
                info!(target: "RSP", "read SP_STATUS");

                // check if BREAK happened
                if let Some(broke_rx) = &self.broke_rx {
                    if let Ok(_) = broke_rx.try_recv() {
                        self.broke = true;
                    }
                }

                ((self.halted as u32) << 0)
                    | ((self.broke as u32) << 1)
                    | ((self.dma_busy as u32) << 2)
            },

            // SP_DMA_BUSY
            0x4_0018 => {
                debug!(target: "RSP", "read SP_DMA_BUSY");

                // mirror of DMA_BUSY in self.si_status
                self.dma_busy as u32
            },

            // SP_SEMAPHORE
            0x4_001C => {
                if !self.semaphore {
                    self.semaphore = true;
                    0
                } else {
                    1
                }
            },

            // SP_PC
            0x8_0000 => {
                debug!(target: "RSP", "read SP_PC");

                self.pc
            },

            _ => {
                error!(target: "RSP", "unknown register read offset=${:08X}", offset);

                0
            },
        };

        Ok(value)
    }

    fn write_register(&mut self, value: u32, offset: usize) {
        match offset {
            // SP_STATUS 
            0x4_0010 => {
                debug!(target: "RSP", "write SP_STATUS value=${:08X}", value);

                // CLR_BROKE: clear the broke flag
                if (value & 0x04) != 0 {
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
                    info!(target: "RSP", "sum of IMEM at start: ${:08X} (if this value is 0x9E2, it is a bootcode program)", sum);

                    info!(target: "RSP", "starting RSP at PC=${:08X}", self.pc);

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
                    let mut c = self.core.lock().unwrap();
                    c.halted = true;
                    self.halted = true;
                }

            },

            // SP_SEMAPHORE
            0x4_001C => {
                if value == 0 {
                    self.semaphore = false;
                }
            },

            // SP_PC
            0x8_0000 => {
                debug!(target: "RSP", "write SP_PC value=${:08X}", value);

                self.pc = value & 0x0FFC;
            },

            _ => {
                warn!(target: "RSP", "unknown write32 register value=${:08X} offset=${:08X}", value, offset);
            }
        };
    }
}

impl Addressable for Rsp {
    fn print_debug_ipl2(&self) {
        let base = 2017;
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+0] , self.mem[base+1] , self.mem[base+2] , self.mem[base+3]);
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+4] , self.mem[base+5] , self.mem[base+6] , self.mem[base+7]);
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+8] , self.mem[base+9] , self.mem[base+10], self.mem[base+11]);
        println!("m ${:08X} ${:08X} ${:08X} ${:08X}", self.mem[base+12], self.mem[base+13], self.mem[base+14], self.mem[base+15]);
    }

    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "RSP", "read32 offset=${:08X}", offset);

        match offset & 0x000F_0000 {
            0x0000_0000..=0x0003_FFFF => {
                let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
                Ok(self.mem[mem_offset as usize])
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
                    const TASK_TYPE: usize = 0xFC0 >> 2;
                    const DL_START: usize = 0xFF0 >> 2;
                    const DL_LEN: usize = 0xFF4 >> 2;
                    match mem_offset {
                        DL_START => {
                            info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (DL start)", value, offset);
                        },
                        DL_LEN => {
                            info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (DL length)", value, offset);
                        },
                        TASK_TYPE => {
                            info!(target: "RSPMEM", "wrote value=${:08X} to offset ${:08X} (TaskType)", value, offset);
                        },
                        _ => {},
                    }
                } else {
                    //info!(target: "RSPMEM", "write value=${:08X} offset=${:08X}", value, offset);
                }
                self.mem[mem_offset as usize] = value;
            },

            0x0004_0000..=0x000B_FFFF => self.write_register(value, offset & 0x000F_FFFF),

            _ => panic!("invalid RSP write"),
        };

        Ok(WriteReturnSignal::None)
    }
}

impl RspCpuCore {
    fn reset(&mut self) {
        self.halted = true;
    }

    fn step(&mut self) {
        info!(target: "RSP", "core step");
    }
}

