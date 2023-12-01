use std::sync::{mpsc, Arc, Mutex, RwLock};
use std::thread;

use tracing::{trace, debug, error, info, warn};

use crate::*;
use crate::cpu::{InstructionDecode, InstructionFault};

/// N64 Reality Signal Processor
/// Resides on the die of the RCP.
pub struct Rsp {
    dma_busy: bool,

    semaphore: bool,

    core: Arc<Mutex<RspCpuCore>>,

    wakeup_tx: Option<mpsc::Sender<()>>,
    broke_rx: Option<mpsc::Receiver<()>>,

    // access to RspCpuCore memory
    mem: Arc<RwLock<Vec<u32>>>,

    // these are copies of state in RspCpuCore so we don't need a lock
    halted: bool,
    broke: bool,
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

    inst: InstructionDecode,

    instruction_table: [CpuInstruction; 64],
    special_table: [CpuInstruction; 64],
    regimm_table: [CpuInstruction; 32],

    // multiple reader single writer memory
    mem: Arc<RwLock<Vec<u32>>>,

    halted: bool,
    broke: bool,
}

pub type CpuInstruction = fn(&mut RspCpuCore) -> Result<(), InstructionFault>;

impl Rsp {
    pub fn new() -> Rsp {
        let mem = Arc::new(RwLock::new(vec![0u32; 2*1024]));

        let core = Arc::new(Mutex::new(RspCpuCore::new(mem.clone())));

        Rsp {
            semaphore: false,
            core: core,

            mem: mem,

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
                        let _ = c.step(); // TODO handle errors

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

                let core = self.core.lock().unwrap();
                core.pc
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

                    {
                        let core = self.core.lock().unwrap();
                        info!(target: "RSP", "starting RSP at PC=${:08X}", core.pc);
                    }

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

                let mut core = self.core.lock().unwrap();
                core.pc = value & 0x0FFC;
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

                let mut mem = self.mem.write().unwrap();
                mem[mem_offset as usize] = value;
            },

            0x0004_0000..=0x000B_FFFF => self.write_register(value, offset & 0x000F_FFFF),

            _ => panic!("invalid RSP write"),
        };

        Ok(WriteReturnSignal::None)
    }
}

use RspCpuCore as Cpu; // shorthand so I can copy code from cpu.rs :)
impl RspCpuCore {
    fn new(mem: Arc<RwLock<Vec<u32>>>) -> Self {
        let mut core = RspCpuCore {
            pc: 0,
            current_instruction_pc: 0,
            next_instruction: 0,
            next_instruction_pc: 0,
            is_delay_slot: false,
            next_is_delay_slot: false,

            gpr: [0u32; 32],

            inst: InstructionDecode::default(),

            mem: mem,

            halted: false,
            broke: false,

            instruction_table: [
                //  _000                _001                _010                _011                _100                _101                _110                _111
   /* 000_ */   Cpu::inst_special , Cpu::inst_regimm  , Cpu::inst_j       , Cpu::inst_jal     , Cpu::inst_beq     , Cpu::inst_bne     , Cpu::inst_blez    , Cpu::inst_bgtz    ,
   /* 001_ */   Cpu::inst_addi    , Cpu::inst_addiu   , Cpu::inst_slti    , Cpu::inst_sltiu   , Cpu::inst_andi    , Cpu::inst_ori     , Cpu::inst_xori    , Cpu::inst_lui     ,
   /* 010_ */   Cpu::inst_cop0    , Cpu::inst_reserved, Cpu::inst_cop2    , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 011_ */   Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 100_ */   Cpu::inst_lb      , Cpu::inst_lh      , Cpu::inst_reserved, Cpu::inst_lw      , Cpu::inst_lbu     , Cpu::inst_lhu     , Cpu::inst_reserved, Cpu::inst_reserved,
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
   /* 10_ */    Cpu::regimm_unknown, Cpu::regimm_bgezal , Cpu::regimm_unknown, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 11_ */    Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved
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

    fn prefetch(&mut self) -> Result<(), ReadWriteFault> {
        self.next_instruction = self.read_u32(self.pc as usize | 0x1000)?; 
        self.next_instruction_pc = self.pc;
        self.pc += 4;

        Ok(())
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
        self.pc += 4;

        // in delay slot flag
        self.is_delay_slot = self.next_is_delay_slot;
        self.next_is_delay_slot = false;

        info!(target: "RSP", "${:08X}: inst ${:08X} (op=0b{:06b}", self.current_instruction_pc | 0x1000, inst, self.inst.op);

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

    fn inst_cop0(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn inst_cop2(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn inst_lb(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.gpr[self.inst.rt] = (self.read_u8(address as usize)? as i8) as u32;
        Ok(())
    }

    fn inst_lh(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.gpr[self.inst.rt] = (self.read_u16(address as usize)? as i16) as u32;
        Ok(())
    }

    fn inst_lw(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.gpr[self.inst.rt] = self.read_u32(address as usize)?;
        Ok(())
    }

    fn inst_lbu(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.gpr[self.inst.rt] = self.read_u8(address as usize)? as u32;
        Ok(())
    }

    fn inst_lhu(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.gpr[self.inst.rt] = self.read_u16(address as usize)? as u32;
        Ok(())
    }

    fn inst_sb(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.write_u8(self.gpr[self.inst.rt] as u32, address as usize)?;
        Ok(())
    }

    fn inst_sh(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm as u32);
        self.write_u16(self.gpr[self.inst.rt], address as usize)?;
        Ok(())
    }

    fn inst_sw(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn inst_lwc2(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn inst_swc2(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_sll(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_srl(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_sra(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_sllv(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_srlv(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_srav(&mut self) -> Result<(), InstructionFault> {
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
        Ok(())
    }

    fn special_add(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_addu(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_sub(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_subu(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_and(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_or(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_xor(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_nor(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_slt(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn special_sltu(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn regimm_bltz(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn regimm_bgez(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn regimm_bgezal(&mut self) -> Result<(), InstructionFault> {
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
            panic!("TODO");
            Err(ReadWriteFault::Invalid)
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        if offset < 0x4_0000 {
            let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
            let mut mem = self.mem.write().unwrap();
            mem[mem_offset as usize] = value;
            Ok(WriteReturnSignal::None)
        } else {
            panic!("TODO");
            Err(ReadWriteFault::Invalid)
        }
    }
}
