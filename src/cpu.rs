#![allow(non_upper_case_globals)]

use std::cell::RefCell;
use std::rc::Rc;

#[allow(unused_imports)]
use tracing::{debug, error, warn, info};

use crate::*;

// Exception handling registers
const Cop0_Index   : usize = 0;
const Cop0_Random  : usize = 1;
const Cop0_EntryLo0: usize = 2;
const Cop0_EntryLo1: usize = 3;
const Cop0_Context : usize = 4;
const Cop0_PageMask: usize = 5;
const Cop0_Wired   : usize = 6;
const Cop0_BadVAddr: usize = 8; // Bad Virtual Address
const Cop0_Count   : usize = 9;
const Cop0_EntryHi : usize = 10;
const Cop0_Compare : usize = 11;
const Cop0_Status  : usize = 12;
const Cop0_Cause   : usize = 13;
const Cop0_EPC     : usize = 14; // Exception Program Counter
const Cop0_PRId    : usize = 15; // Processor Revision Identifier
const _COP0_WATCHLO : usize = 18;
const _COP0_WATCHHI : usize = 19;
const Cop0_XContext: usize = 20;
const Cop0_PErr    : usize = 26; // Parity Error
const Cop0_CacheErr: usize = 27;
const _COP0_ERROREPC: usize = 30; // Error Exception Program Counter

const Cop0_Config: usize = 16;
const Cop0_LLAddr: usize = 17;

const STATUS_IM_TIMER_INTERRUPT_ENABLE_FLAG: u64 = 7;

const ExceptionCode_Int  : u64 = 0;  // interrupt
const ExceptionCode_Mod  : u64 = 1;  // TLB modification exception
const ExceptionCode_TLBL : u64 = 2;  // TLB Miss exception (load or instruction fetch)
const ExceptionCode_TLBS : u64 = 3;  // TLB Miss exception (store)
const ExceptionCode_AdEL : u64 = 4;  // Address Error exception (load or instruction fetch)
const ExceptionCode_AdES : u64 = 5;  // Address Error exception (store)
const _ExceptionCode_IBE  : u64 = 6;  // Bus Error exception (instruction fetch)
const _ExceptionCode_DBE  : u64 = 7;  // Bus Error exception (data reference: load or store)
const ExceptionCode_Sys  : u64 = 8;  // Syscall exception
const ExceptionCode_Bp   : u64 = 9;  // Breakpoint exception
const ExceptionCode_RI   : u64 = 10; // Reserved Instruction exception
const ExceptionCode_CpU  : u64 = 11; // Coprocessor Unusable exception
const ExceptionCode_Ov   : u64 = 12; // Arithmetic Overflow exception
const ExceptionCode_Tr   : u64 = 13; // Trap instruction
const ExceptionCode_FPE  : u64 = 15; // Floating-Point exception
const _ExceptionCode_WATCH: u64 = 23; // Watch exception

const InterruptCode_Timer: u64 = 0x80;

struct InstructionDecode {
    v : u32,      // full 32-bit instruction
    op: u32,      // 6-bit opcode field
    regimm: u32,  // 5-bit regimm op
    special: u32, // 6-bit special op

    // rd, rs, and rt are mostly used to index into self.gpr, so they will be usize
    // but they are 5 bits in the instruction
    rd: usize,
    rs: usize,
    rt: usize,

    // 16-bit immediate and sign extended value
    // the signed_imm value needs to be casted to i64/i32 for signed comparisons
    imm: u64,
    signed_imm: u64,

    // 5 bit shift amount
    sa: u32,

    // 27-bit jump targets
    target: u32,
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum MemorySpace {
    User,
    Supervisor,
    XKPhys, // TODO
    Kernel
}

#[derive(Copy, Clone, Debug)]
struct Address {
    virtual_address: u64,
    physical_address: u64,
    cached: bool,
    mapped: bool,
    space: MemorySpace,
}

#[derive(Copy, Clone, Debug, Default)]
struct TlbEntry {
    page_mask: u64,
    entry_hi: u64,
    entry_lo1: u64,
    entry_lo0: u64
}

pub struct Cpu {
    pub bus: Rc<RefCell<dyn Addressable>>,
    pc: u64,                     // lookahead PC
    current_instruction_pc: u64, // actual PC of the currently executing instruction
                                 // only valid inside step()
    next_instruction: u32,       // emulates delay slot (prefetch, next instruction)
    next_instruction_pc: u64,    // for printing correct delay slot addresses
    is_delay_slot: bool,         // true if the currently executing instruction is in a delay slot
    next_is_delay_slot: bool,    // set to true on branching instructions

    gpr: [u64; 32],
    lo: u64,
    hi: u64,

    cp0gpr: [u64; 32],
    cp0gpr_latch: u64,
    tlb: [TlbEntry; 32],
    cp2gpr_latch: u64,

    half_clock: u32,
    llbit: bool,

    cop1: cop1::Cop1,

    instruction_table: [CpuInstruction; 64],
    special_table: [CpuInstruction; 64],
    regimm_table: [CpuInstruction; 32],

    // instruction decode values
    inst: InstructionDecode,

    num_steps: u64,
}

// It may be worth mentioning that cpu exceptions are not InstructionFaults 
// InstructionFault is used for passing messages or interrupting cpu execution 
// within the emulator
pub enum InstructionFault {
    Invalid,
    Unimplemented,
    Break,
    OtherException(u64),
    CoprocessorUnusable,
    FloatingPointException,
    ReadWrite(ReadWriteFault),
}

impl From<ReadWriteFault> for InstructionFault {
    fn from(value: ReadWriteFault) -> Self {
        InstructionFault::ReadWrite(value)
    }
}

type CpuInstruction = fn(&mut Cpu) -> Result<(), InstructionFault>;

impl Cpu {
    pub fn new(bus: Rc<RefCell<dyn Addressable>>) -> Cpu {
        let mut cpu = Cpu {
            num_steps: 0,

            bus : bus,
            pc  : 0,
            current_instruction_pc: 0,
            next_instruction: 0,
            next_instruction_pc: 0,
            is_delay_slot: false,
            next_is_delay_slot: false,

            gpr : [0u64; 32],
            lo  : 0,
            hi  : 0,

            cp0gpr: [0u64; 32],
            cp0gpr_latch: 0,
            tlb: [TlbEntry::default(); 32],
            cp2gpr_latch: 0,
            half_clock: 0,
            llbit: false,

            cop1: cop1::Cop1::new(),

            // Sorry for making these so wide, but it maps to the instruction decode table in the datasheet better!
            instruction_table: [
                //  _000               _001              _010               _011                _100                _101                _110                _111
   /* 000_ */   Cpu::inst_special, Cpu::inst_regimm, Cpu::inst_j      , Cpu::inst_jal     , Cpu::inst_beq     , Cpu::inst_bne     , Cpu::inst_blez    , Cpu::inst_bgtz    ,
   /* 001_ */   Cpu::inst_addi   , Cpu::inst_addiu , Cpu::inst_slti   , Cpu::inst_sltiu   , Cpu::inst_andi    , Cpu::inst_ori     , Cpu::inst_xori    , Cpu::inst_lui     ,
   /* 010_ */   Cpu::inst_cop0   , Cpu::inst_cop   , Cpu::inst_cop2   , Cpu::inst_reserved, Cpu::inst_beql    , Cpu::inst_bnel    , Cpu::inst_blezl   , Cpu::inst_bgtzl   ,
   /* 011_ */   Cpu::inst_daddi  , Cpu::inst_daddiu, Cpu::inst_ldl    , Cpu::inst_ldr     , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 100_ */   Cpu::inst_lb     , Cpu::inst_lh    , Cpu::inst_lwl    , Cpu::inst_lw      , Cpu::inst_lbu     , Cpu::inst_lhu     , Cpu::inst_lwr     , Cpu::inst_lwu     ,
   /* 101_ */   Cpu::inst_sb     , Cpu::inst_sh    , Cpu::inst_swl    , Cpu::inst_sw      , Cpu::inst_sdl     , Cpu::inst_sdr     , Cpu::inst_swr     , Cpu::inst_cache   ,
   /* 110_ */   Cpu::inst_ll     , Cpu::inst_lwc1  , Cpu::inst_unknown, Cpu::inst_reserved, Cpu::inst_unknown , Cpu::inst_ldc1    , Cpu::inst_unknown , Cpu::inst_ld      ,
   /* 111_ */   Cpu::inst_sc     , Cpu::inst_swc1  , Cpu::inst_unknown, Cpu::inst_reserved, Cpu::inst_unknown , Cpu::inst_sdc1    , Cpu::inst_unknown , Cpu::inst_sd
            ],

            special_table: [
               //   _000                  _001                  _010                  _011                  _100                  _101                  _110                  _111
   /* 000_ */   Cpu::special_sll    , Cpu::inst_reserved  , Cpu::special_srl    , Cpu::special_sra    , Cpu::special_sllv   , Cpu::inst_reserved  , Cpu::special_srlv   , Cpu::special_srav   ,
   /* 001_ */   Cpu::special_jr     , Cpu::special_jalr   , Cpu::inst_reserved  , Cpu::inst_reserved  , Cpu::special_syscall, Cpu::special_break  , Cpu::inst_reserved  , Cpu::special_sync   ,
   /* 010_ */   Cpu::special_mfhi   , Cpu::special_mthi   , Cpu::special_mflo   , Cpu::special_mtlo   , Cpu::special_dsllv  , Cpu::inst_reserved  , Cpu::special_dsrlv  , Cpu::special_dsrav  ,
   /* 011_ */   Cpu::special_mult   , Cpu::special_multu  , Cpu::special_div    , Cpu::special_divu   , Cpu::special_dmult  , Cpu::special_dmultu , Cpu::special_ddiv   , Cpu::special_ddivu  ,
   /* 100_ */   Cpu::special_add    , Cpu::special_addu   , Cpu::special_sub    , Cpu::special_subu   , Cpu::special_and    , Cpu::special_or     , Cpu::special_xor    , Cpu::special_nor    ,
   /* 101_ */   Cpu::inst_reserved  , Cpu::inst_reserved  , Cpu::special_slt    , Cpu::special_sltu   , Cpu::special_dadd   , Cpu::special_daddu  , Cpu::special_dsub   , Cpu::special_dsubu  ,
   /* 110_ */   Cpu::special_tge    , Cpu::special_tgeu   , Cpu::special_tlt    , Cpu::special_tltu   , Cpu::special_teq    , Cpu::inst_reserved  , Cpu::special_tne    , Cpu::inst_reserved  ,
   /* 111_ */   Cpu::special_dsll   , Cpu::inst_reserved  , Cpu::special_dsrl   , Cpu::special_dsra   , Cpu::special_dsll32 , Cpu::inst_reserved  , Cpu::special_dsrl32 , Cpu::special_dsra32 ,
            ],

            regimm_table: [
                    //   _000                      _001                      _010                      _011                      _100                      _101                      _110                      _111
   /* 00_ */    Cpu::regimm_bltz   , Cpu::regimm_bgez   , Cpu::regimm_unknown, Cpu::regimm_bgezl  , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved ,
   /* 01_ */    Cpu::regimm_tgei   , Cpu::regimm_tgeiu  , Cpu::regimm_tlti   , Cpu::regimm_tltiu  , Cpu::regimm_teqi   , Cpu::inst_reserved , Cpu::regimm_tnei   , Cpu::inst_reserved ,
   /* 10_ */    Cpu::regimm_unknown, Cpu::regimm_bgezal , Cpu::regimm_unknown, Cpu::regimm_bgezall, Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved ,
   /* 11_ */    Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved ,
            ],

            inst: InstructionDecode {
                v: 0, op: 0, regimm: 0, special: 0, rd: 0, rs: 0, rt: 0,
                imm: 0, signed_imm: 0, sa: 0, target: 0,
            }
        };
        
        let _ = cpu.reset();
        cpu
    }

    pub fn reset(&mut self) -> Result<(), InstructionFault> {
        self.pc = 0xFFFF_FFFF_BFC0_0000;

        // COP0
        // TODO see section 6.4.4 Cold Reset for a complete list of initial register values
        self.cp0gpr[Cop0_Wired] = 0;
        self.cp0gpr[Cop0_Random] = 0x1F; // set to the upper bound (31) on reset
        self.cp0gpr[Cop0_PRId] = 0x0B22;
        self.cp0gpr[Cop0_Config] = 0x7006E463; // EC=1:15, EP=0, BE=1 (big endian), CU=0 (RFU?), K0=3 (kseg0 cache enabled)
        self.cp0gpr[Cop0_Status] = 0x200004; // BEV=1, ERL=1, SR=0 (SR will need to be 1 on soft reset), IE=0

        // fetch next_instruction before starting the loop
        self.prefetch()?;
        self.next_is_delay_slot = false;

        Ok(())
    }

    pub fn num_steps(&self) -> &u64 {
        &self.num_steps
    }

    pub fn next_instruction(&self) -> &u32 {
        &self.next_instruction
    }

    pub fn next_instruction_pc(&self) -> &u64 {
        &self.next_instruction_pc
    }

    pub fn next_is_delay_slot(&self) -> &bool {
        &self.next_is_delay_slot
    }

    pub fn regs(&self) -> &[u64] {
        &self.gpr
    }

    pub fn abi_name(i: usize) -> &'static str {
        const NAMES: [&str; 32] = [
            "r0", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
            "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "t8", "t9", "k0", "k1", "gp", "sp", "s8", "ra"
        ];

        NAMES[i]
    }

    pub fn register_name(i: usize) -> &'static str {
        const NAMES: [&str; 32] = [
            "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",
            "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31"
        ];

        NAMES[i]
    }

    fn interrupts_enabled(&self, interrupt_number: u64) -> bool {
        // interrupts are enabled when IE is set and EXL (exception level) and ERL (error level) are 0
        // (and also when the IM bit is set for the specified interrupt)
        let check = (0x100 << interrupt_number) | 0x07;
        (self.cp0gpr[Cop0_Status] & check) == (check & !0x06)
    }

    #[inline(always)]
    fn read_u8(&mut self, address: usize) -> Result<u8, InstructionFault> {
        Ok(self.bus.borrow_mut().read_u8(address)?)
    }

    #[inline(always)]
    fn read_u16(&mut self, address: usize) -> Result<u16, InstructionFault> {
        Ok(self.bus.borrow_mut().read_u16(address)?)
    }

    #[inline(always)]
    fn read_u32(&mut self, virtual_address: usize) -> Result<u32, InstructionFault> {
        if let Some(address) = self.translate_address(virtual_address as u64, true, false)? {
            self.read_u32_phys(address)
        } else {
            // invalid TLB translation, no physical address present to read
            // and no exception
            Ok(0)
        }
    }

    #[inline(always)]
    fn read_u32_phys(&mut self, address: Address) -> Result<u32, InstructionFault> {
        Ok(self.bus.borrow_mut().read_u32(address.physical_address as usize)?)
    }

    // The VR4300 has to do two reads to get a doubleword
    #[inline(always)]
    fn read_u64(&mut self, virtual_address: usize) -> Result<u64, InstructionFault> {
        if let Some(address) = self.translate_address(virtual_address as u64, true, false)? {
            self.read_u64_phys(address)
        } else {
            Ok(0)
        }
    }

    #[inline(always)]
    fn read_u64_phys(&mut self, mut address: Address) -> Result<u64, InstructionFault> {
        match self.read_u32_phys(address) {
            Err(err) => Err(err),
            Ok(v) => {
                address.physical_address += 4; // Hmm, might this cause problems on virtual page boundaries?
                match self.read_u32_phys(address) {
                    Err(err) => Err(err),
                    Ok(v2) => {
                        Ok(((v as u64) << 32) | (v2 as u64))
                    }
                }
            }
        }
    }

    #[inline(always)]
    fn write_u8(&mut self, value: u32, address: usize) -> Result<WriteReturnSignal, InstructionFault> {
        Ok(self.bus.borrow_mut().write_u8(value, address)?)
    }

    #[inline(always)]
    fn write_u16(&mut self, value: u32, address: usize) -> Result<WriteReturnSignal, InstructionFault> {
        Ok(self.bus.borrow_mut().write_u16(value, address)?)
    }

    #[inline(always)]
    fn write_u32(&mut self, value: u32, virtual_address: usize) -> Result<WriteReturnSignal, InstructionFault> {
        if let Some(address) = self.translate_address(virtual_address as u64, true, true)? {
            self.write_u32_phys(value, address)
        } else {
            // invalid TLB translation, no physical address present to read
            Ok(WriteReturnSignal::None)
        }
    }

    #[inline(always)]
    fn write_u32_phys(&mut self, value: u32, address: Address) -> Result<WriteReturnSignal, InstructionFault> {
        Ok(self.bus.borrow_mut().write_u32(value, address.physical_address as usize)?)
    }

    #[inline(always)]
    fn write_u64(&mut self, value: u64, virtual_address: usize) -> Result<WriteReturnSignal, InstructionFault> {
        if let Some(address) = self.translate_address(virtual_address as u64, true, false)? {
            self.write_u64_phys(value, address)
        } else {
            Ok(WriteReturnSignal::None)
        }
    }

    #[inline(always)]
    fn write_u64_phys(&mut self, value: u64, mut address: Address) -> Result<WriteReturnSignal, InstructionFault> {
        self.write_u32_phys((value >> 32) as u32, address)?;
        address.physical_address += 4;
        Ok(self.write_u32_phys(value as u32, address)?)
    }

    // prefetch the next instruction
    fn prefetch(&mut self) -> Result<(), InstructionFault> {
        self.next_instruction = self.read_u32(self.pc as usize)?; 
        self.next_instruction_pc = self.pc;
        self.pc += 4;

        Ok(())
    }

    fn exception(&mut self, exception_code: u64) -> Result<(), InstructionFault> {
        // clear BD, exception code and bits that should be 0
        self.cp0gpr[Cop0_Cause] &= !0xC0FF_00FF;

        // set the exception code
        self.cp0gpr[Cop0_Cause] |= exception_code << 2;

        // set the EXL bit to prevent another exception
        let previous_exl = self.cp0gpr[Cop0_Status] & 0x02;
        self.cp0gpr[Cop0_Status] |= 0x02;

        // set the Exception Program Counter to the currently executing instruction
        self.cp0gpr[Cop0_EPC] = (if self.is_delay_slot {
            // also set the BD flag
            self.cp0gpr[Cop0_Cause] |= 0x8000_0000;
            self.current_instruction_pc - 4
        } else {
            self.current_instruction_pc
        } as i32) as u64;

        // set PC and discard the delay slot. PC is set to the vector base determined by the BEV bit in Cop0_Status.
        // TODO XTLB miss needs to branch to base plus offset 0x080
        let is_tlb = (exception_code == ExceptionCode_TLBL) || (exception_code == ExceptionCode_TLBS);
        self.pc = (if (self.cp0gpr[Cop0_Status] & 0x200000) == 0 {  // check BEV==0
            0xFFFF_FFFF_8000_0000
        } else { 
            0xFFFF_FFFF_BFC0_0200
        }) /*+ (if previous_exl == 0 && is_tlb { // check EXL==0 and tlb miss
            0x000
        } else {
            0x180
        });*/
        + 0x180;

        // we need to throw away next_instruction, so prefetch the new instruction now
        self.prefetch()?;

        // break out of all processing
        Err(InstructionFault::OtherException(exception_code))
    }

    fn address_exception(&mut self, virtual_address: u64, is_write: bool) -> Result<(), InstructionFault> {
        //println!("CPU: address exception!");

        self.cp0gpr[Cop0_BadVAddr] = virtual_address;
        let bad_vpn2 = virtual_address >> 13;
        self.cp0gpr[Cop0_Context] = (self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) | ((bad_vpn2 & 0x7FFFF) << 4);

        // TODO the XContext needs the page table
        self.cp0gpr[Cop0_XContext] = ((self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) << 10) 
                                     | ((virtual_address >> 31) & 0x1_8000_0000) 
                                     | ((bad_vpn2 & 0x7FF_FFFF) << 4);

        let exception_code = if is_write { ExceptionCode_AdES } else { ExceptionCode_AdEL };
        self.exception(exception_code)
    }

    fn tlb_miss(&mut self, virtual_address: u64, is_write: bool) -> Result<(), InstructionFault> {
        info!(target: "CPU", "TLB miss exception virtual_address=${:016X} is_write={} 32-bits=${:X}!", virtual_address, is_write, self.cp0gpr[Cop0_Config] & 0xE0);

        // set EntryHi to the page that caused the fault, preserving ASID
        self.cp0gpr[Cop0_EntryHi] = (virtual_address & 0xFF_FFFF_E000) | (self.cp0gpr[Cop0_EntryHi] & 0xFF);

        self.cp0gpr[Cop0_BadVAddr] = virtual_address;
        let bad_vpn2 = virtual_address >> 13;
        self.cp0gpr[Cop0_Context] = (self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) | ((bad_vpn2 & 0x7FFFF) << 4);

        // TODO the XContext needs the page table
        self.cp0gpr[Cop0_XContext] = ((self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) << 10) 
                                     | ((virtual_address >> 31) & 0x1_8000_0000) 
                                     | ((bad_vpn2 & 0x7FF_FFFF) << 4);

        let exception_code = if is_write { ExceptionCode_TLBS } else { ExceptionCode_TLBL };
        self.exception(exception_code)
    }

    fn tlb_mod(&mut self, virtual_address: u64) -> Result<(), InstructionFault> {
        info!(target: "CPU", "TLB mod exception virtual_address=${:016X}", virtual_address);

        // set EntryHi to the page that caused the fault, preserving ASID
        self.cp0gpr[Cop0_EntryHi] = (virtual_address & 0xFF_FFFF_E000) | (self.cp0gpr[Cop0_EntryHi] & 0xFF);

        self.cp0gpr[Cop0_BadVAddr] = virtual_address;
        let bad_vpn2 = virtual_address >> 13;
        self.cp0gpr[Cop0_Context] = (self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) | ((bad_vpn2 & 0x7FFFF) << 4);

        // TODO the XContext needs the page table
        self.cp0gpr[Cop0_XContext] = ((self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) << 10) 
                                     | ((virtual_address >> 31) & 0x1_8000_0000) 
                                     | ((bad_vpn2 & 0x7FF_FFFF) << 4);

        self.exception(ExceptionCode_Mod)
    }


    fn breakpoint_exception(&mut self) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] &= !0x3000_0000; // clear coprocessor number
        self.exception(ExceptionCode_Bp)
    }

    fn overflow_exception(&mut self) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] &= !0x3000_0000; // clear coprocessor number
        self.exception(ExceptionCode_Ov)
    }

    fn reserved_instruction_exception(&mut self, coprocessor_number: u64) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0x3000_0000) | (coprocessor_number << 28);
        self.exception(ExceptionCode_RI)
    }

    fn syscall_exception(&mut self) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] &= !0x3000_0000; // clear coprocessor number
        self.exception(ExceptionCode_Sys)
    }

    fn trap_exception(&mut self) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] &= !0x3000_0000; // clear coprocessor number
        self.exception(ExceptionCode_Tr)
    }

    fn floating_point_exception(&mut self) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] &= !0x3000_0000; // clear coprocessor number
        self.exception(ExceptionCode_FPE)
    }

    fn coprocessor_unusable_exception(&mut self, coprocessor_number: u64) -> Result<(), InstructionFault> {
        //info!("CPU: coprocessor unusable exception (Cop0_Status = ${:08X})!", self.cp0gpr[Cop0_Status]);
        self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0x3000_0000) | (coprocessor_number << 28);
        self.exception(ExceptionCode_CpU)
    }

    fn interrupt(&mut self, interrupt_signal: u64) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0xFF0) | (interrupt_signal << 8);

        //println!("CPU: interrupt!");
        self.exception(ExceptionCode_Int)
    }

    #[inline(always)]
    fn timer_interrupt(&mut self) -> Result<(), InstructionFault> {
        self.interrupt(InterruptCode_Timer)
    }

    #[inline(always)]
    fn branch(&mut self, condition: bool) {
        if condition {
            // target is the sum of the address of the delay slot instruction
            // plus the sign extended and left-shifted immediate offset
            self.pc = (self.pc - 4).wrapping_add((self.inst.signed_imm as u64) << 2);
        }

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
    }

    // branch likely instructions discards the delay slot when the branch is not taken
    #[inline(always)]
    fn branch_likely(&mut self, condition: bool) -> Result<(), InstructionFault> {
        if condition {
            self.pc = (self.pc - 4).wrapping_add((self.inst.signed_imm as u64) << 2);

            // note that the next instruction to execute is a delay slot instruction
            self.next_is_delay_slot = true;
        } else {
            // we need to throw away next_instruction when branch is not taken
            self.prefetch()?;
        }
        Ok(())
    }

    #[inline(always)]
    fn set_tlb_entry(&mut self, index: u64) {
        let g = (self.cp0gpr[Cop0_EntryLo0] & self.cp0gpr[Cop0_EntryLo1]) & 0x1; // Global flag

        // the page mask needs to have the bits changed. the datasheet says they're undefined
        // when not certain values, but to mimic the N64 there is certain expected behavior.
        // 1. if the upper bit of the pair is not set then the lower bit of the pair is cleared
        let mut page_mask = (self.cp0gpr[Cop0_PageMask] >> 13) & 0xAAA;
        page_mask |= page_mask >> 1;

        self.tlb[(index & 0x1F) as usize] = TlbEntry {
            page_mask: page_mask << 13,
            entry_hi : self.cp0gpr[Cop0_EntryHi] | (g << 12),
            entry_lo1: self.cp0gpr[Cop0_EntryLo1] & 0x03FF_FFFE, // mask out G flag
            entry_lo0: self.cp0gpr[Cop0_EntryLo0] & 0x03FF_FFFE, // mask out G flag
        };
    }

    fn classify_address(&self, virtual_address: u64, _generate_exceptions: bool, _is_write: bool) -> Result<Option<Address>, InstructionFault> {
        let space = virtual_address >> 62;
        let mut address = Address {
            virtual_address: virtual_address,
            physical_address: 0,
            cached: true,
            mapped: false,
            space: match space {
                0b00 => MemorySpace::User,
                0b01 => MemorySpace::Supervisor,
                0b10 => MemorySpace::XKPhys,
                0b11 => MemorySpace::Kernel,
                _ => panic!("not possible"),
            },
        };

        match address.space {
            MemorySpace::User => {
                if virtual_address < 0x0000_0100_0000_0000 { // xkuseg, user segment, TLB mapped, cached
                    address.mapped = true;
                } else {
                    panic!("TODO: invalid user segment address (address exception)");
                }
            },

            MemorySpace::Supervisor => {
                if virtual_address < 0x4000_0100_0000_0000 { // xksseg, supervisor segment, TLB mapped, cached
                    address.mapped = true;
                } else {
                    panic!("TODO: invalid supervisor segment address (address exception)");
                }
            },

            MemorySpace::XKPhys => { // xkphys
                panic!("TODO: xkphys segment");
            },

            MemorySpace::Kernel => {
                if virtual_address < 0xC000_00FF_8000_0000 { // xkseg, TLB mapped, cached
                    address.mapped = true;
                } else if virtual_address < 0xFFFF_FFFF_8000_0000 { // invalid
                    panic!("TODO: address error");
                // the address range from FFFF_FFFF_8000_0000-FFFF_FFFF_FFFF_FFFF is the 32-bit compatibility range
                } else if virtual_address < 0xFFFF_FFFF_A000_0000 { // ckseg0, segment 0, directly mapped, cached
                    address.physical_address = virtual_address & 0x1FFF_FFFF;
                } else if virtual_address < 0xFFFF_FFFF_C000_0000 { // ckseg1, segment 0, directly mapped, uncached
                    address.cached = false;
                    address.physical_address = virtual_address & 0x1FFF_FFFF;
                } else if virtual_address < 0xFFFF_FFFF_E000_0000 { // cksseg, supervisor segment (unused), TLB mapped
                    address.mapped = true;
                } else { // ckseg3, kernel segment 3, TLB mapped, cached
                    address.mapped = true;
                }
            }
        }

        return Ok(Some(address));
    }

    fn translate_address(&mut self, virtual_address: u64, generate_exceptions: bool, is_write: bool) -> Result<Option<Address>, InstructionFault> {
        // create the Address struct by first classifying the virtual address
        let mut address = match self.classify_address(virtual_address, generate_exceptions, is_write)? {
            Some(v) => v,
            None => {
                return Ok(None);
            }
        };

        // TODO check address.space and make sure current execution mode allows access to that space
        // i.e., if we're running in supervisor mode and try to access kernel address, we need to
        // generate an address_exception(). But as far as I can tell games are always running in
        // kernel mode.  We may need to support it in the future to support demos or other demo
        // programs, for example.

        // if no further address translation is needed, return the Address
        if !address.mapped { return Ok(Some(address)); }

        // since the address is mapped there must be a TLB entry for it to be a valid
        // virtual_address, otherwise TLB exceptions occur

        //info!(target: "CPU", "TLB: translating virtual address ${:016X}!", virtual_address);

        // virtual addresses are 40 bits wide
        let virtual_address = virtual_address & 0xFF_FFFF_FFFF;

        // get the current ASID from EntryHi
        let asid = self.cp0gpr[Cop0_EntryHi] & 0xFF;

        for tlb_index in 0..32 {
            let tlb = &self.tlb[tlb_index];

            // the TLB entry is valid if either the ASID matches or the G flag is set
            // check if G bit is set, and skip the entry if not
            if (tlb.entry_hi & 0xFF) != asid && (tlb.entry_hi & 0x1000) == 0 { continue; }

            //info!(target: "CPU", "checking TLB entry {}", tlb_index);

            // PageMask is (either 0, 3, 5, 7, F, 3F) at bit position 13
            // So offset_mask will be a mask that can mask out the offset into the page from
            // the virtual address
            let offset_mask = (tlb.page_mask >> 1) | 0xFFF; // page mask is a set of 1s at bit 13
                                                            // and if it isn't, it's undefined
                                                            // behavior
            //info!(target: "CPU", "TLB: offset_mask = ${:016X}, page_size = {}", offset_mask, (offset_mask + 1));

            // VPN2 is the virtual page number (divided by 2) this TLB entry covers
            let vpn2 = tlb.entry_hi & 0xFFFFFFE000;

            // If the virtual address is referencing one of the two pages, it's a hit
            // notice that we keep PageMask in bit 13 rather shift it down
            let va_vpn_mask = !(tlb.page_mask | 0x1FFF) & 0xFF_FFFF_FFFF;
            //info!(target: "CPU", "TLB: EntryHi vpn2 = ${:016X}, va_vpn_mask = ${:016X}", vpn2, va_vpn_mask);

            //info!(target: "CPU", "TLB: virtual_address & va_vpn_mask = ${:016X}", virtual_address & va_vpn_mask);
            if (virtual_address & va_vpn_mask) != vpn2 { continue; }

            //info!(target: "CPU", "TLB: address matches tlb index {tlb_index}!");

            // the odd bit is the 1 bit above the offset bits
            let odd_bit = virtual_address & (offset_mask + 1);
            //info!(target: "CPU", "TLB: matched {} page", if odd_bit != 0 { "odd" } else { "even" });

            // get the correct EntryLo field based on the odd page bit
            let entry_lo = if odd_bit != 0 { tlb.entry_lo1 } else { tlb.entry_lo0 };

            // if V (valid) bit is not set, a TLB miss occurs
            if (entry_lo & 0x02) == 0 { break; }

            // if D (dirty) is not set and this is a write, a Mod exception occurs
            if generate_exceptions && (entry_lo & 0x04) == 0 && is_write {
                self.tlb_mod(virtual_address)?;
                return Ok(None);
            }

            // get the page frame number of the corresponding EntryLo field
            let pfn = entry_lo & 0x03FF_FFC0;

            // the physical address is the pfn plus the offset (and the odd page bit)
            // pfn is at bit 6 in EntryLo, and needs to be at position 12 for the physical address
            address.physical_address = (pfn << 6) | ((virtual_address & !odd_bit) & (tlb.page_mask | 0x1FFF));

            //info!(target: "CPU", "TLB: translated address = ${:016X}", address.physical_address);

            // done
            return Ok(Some(address));
        }

        if generate_exceptions {
            // If the high bit of the address is set and the upper bits of the address
            // are in user space (i.e., zero), then we have an address exception error instead
            // TODO I wish I understood more why this is the case. It seems to me like
            // 0x0000_0000_8xxx_xxxx is a valid user-space address and should generate a TLB miss, not
            // an address exception
            if (address.space == MemorySpace::User) && ((virtual_address & 0x8000_0000) != 0) {
                self.address_exception(address.virtual_address, is_write)?;
            } else {
                self.tlb_miss(address.virtual_address, is_write)?;
            }
        }

        Ok(None)
    }

    pub fn step(&mut self) -> Result<(), InstructionFault> {
        self.num_steps += 1;

        //if self.pc == 0x8000_02B4 {
        //    trace!(target: "CPU", "Starting cartridge ROM!");
        //}

        // increment Cop0_Count at half PClock and trigger exception
        self.half_clock ^= 1;
        self.cp0gpr[Cop0_Count] = ((self.cp0gpr[Cop0_Count] as u32) + self.half_clock) as u64;
        if self.cp0gpr[Cop0_Compare] == self.cp0gpr[Cop0_Count] {
            // Timer interrupt enable, bit 7 of IM field 
            if self.interrupts_enabled(STATUS_IM_TIMER_INTERRUPT_ENABLE_FLAG) { // TODO move to self.timer_interrupt()
                //println!("COP0: timer interrupt");
                self.timer_interrupt()?;
            }
        }

        // decrement Random every cycle
        // when Wired bit 5 is set, Random runs 0..63 with no wired limit
        self.cp0gpr[Cop0_Random] = if (self.cp0gpr[Cop0_Wired] & 0x20) != 0 || ((self.cp0gpr[Cop0_Random] ^ self.cp0gpr[Cop0_Wired]) & 0x1F) != 0 {
            (self.cp0gpr[Cop0_Random] & 0x3F).wrapping_sub(1) & 0x3F
        } else {
            if (self.cp0gpr[Cop0_Wired] & 0x20) != 0 {
                0x3F
            } else {
                0x1F
            }
        };

        // For address and TLB exceptions, the current_instruction_pc needs to point to
        // the address being fetched. This lets Cop0_EPC be set to the correct PC
        self.current_instruction_pc = self.pc; // For Cop0_EPC in case of a TLB miss in a delay
                                               // slot after a jump to an invalid TLB page
        // check for invalid PC addresses
        if (self.pc & 0x03) != 0 {
            return self.address_exception(self.pc, false);
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

        // next instruction fetch. we need to catch TLB misses
        self.next_instruction = match self.read_u32(self.pc as usize) {
            Err(InstructionFault::OtherException(ExceptionCode_TLBL)) => {
                // its possible for a TLB miss to occur while prefetching a delay slot instruction
                // BEFORE the jump and link instruction is executed. However the link register
                // still needs to be set correctly. So here, we check if the next-to-execute
                // instruction is a jump and link and set RA. Quite a hack, but works when we're
                // not cycle-accurate
                match self.inst.v & 0xFC00_003F {
                    0x0000_0009 => { // JALR
                        // the TLB miss of the delay slot is in BadVAddr, so set RA to the instruction after it
                        self.gpr[self.inst.rd] = self.cp0gpr[Cop0_BadVAddr] + 4;
                    },
                    // There's no test in n64-systemtests for this but it should be the same
                    // situation as JALR
                    0x0C00_0000..=0x0C00_003F => { // JL
                        warn!(target: "CPU", "let me know when this happens");
                        self.gpr[self.inst.rd] = self.cp0gpr[Cop0_BadVAddr] + 4;
                    },
                    _ => { },
                };

                return Err(InstructionFault::OtherException(ExceptionCode_TLBL));
            }

            Err(x) => { return Err(x); },
            Ok(x) => x,
        };

        // update and increment PC
        self.current_instruction_pc = self.next_instruction_pc;
        self.next_instruction_pc = self.pc;
        self.pc += 4;

        // in delay slot flag
        self.is_delay_slot = self.next_is_delay_slot;
        self.next_is_delay_slot = false;

        let result = self.instruction_table[self.inst.op as usize](self);
        let result = match result {
            // Raise FPE on InstructionFault::FloatingPointException
            Err(InstructionFault::FloatingPointException) => { 
                let _ = self.floating_point_exception();
                Ok(())
            },

            // Other exceptions continue execution
            Err(InstructionFault::OtherException(_)) => {
                Ok(())
            }

            Err(InstructionFault::CoprocessorUnusable) => {
                panic!("Am I using this?");
            }

            // Other faults like Break, Unimplemented actually stop processing
            Err(_) => {
                // on error, restore the previous instruction since it didn't complete
                self.pc -= 4;
                self.next_instruction_pc = self.current_instruction_pc;
                self.next_instruction = inst;
                result
            },

            // Everything else
            Ok(_) => { result },
        };

        // r0 must always be zero
        self.gpr[0] = 0;

        result
    }

    fn inst_reserved(&mut self) -> Result<(), InstructionFault> {
        //warn!(target: "CPU", "reserved instruction ${:03b}_{:03b}", self.inst.op >> 3, self.inst.op & 0x07);
        self.reserved_instruction_exception(0)?;
        Ok(())
    }

    fn inst_unknown(&mut self) -> Result<(), InstructionFault> {
        error!(target: "CPU", "unimplemented function ${:03b}_{:03b}", self.inst.op >> 3, self.inst.op & 0x07);
        Err(InstructionFault::Unimplemented)
    }

    fn inst_addi(&mut self) -> Result<(), InstructionFault> {
        // integer overflow exception occurs with ADDI, unlike ADDIU
        let rs = self.gpr[self.inst.rs] as u32;
        let rt = self.inst.signed_imm as u32;
        let result = rs.wrapping_add(rt);
        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000) != 0 {
            //println!("CPU: addi overflow detected: rs=${:08X} imm=${:04X} result=${:08X}", rs, rt, result);
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rt] = (result as i32) as u64;
        }

        Ok(())
    }

    fn inst_addiu(&mut self) -> Result<(), InstructionFault> {
        // no integer overflow exception occurs with ADDIU
        self.gpr[self.inst.rt] = ((self.gpr[self.inst.rs] as u32).wrapping_add(self.inst.signed_imm as u32) as i32) as u64;
        Ok(())
    }

    fn inst_andi(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] & self.inst.imm;
        Ok(())
    }

    fn inst_cop(&mut self) -> Result<(), InstructionFault> {
        let copno = ((self.inst.v >> 26) & 0x03) as u64;
        let cop = match copno {
            0 => panic!("TODO"),
            1 => &mut self.cop1,
            _ => {
                self.coprocessor_unusable_exception(copno)?;
                return Ok(());
            },
        };

        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & (0x1000_0000 << copno)) == 0 {
            self.coprocessor_unusable_exception(copno)?;
            return Ok(());
        }

        if (self.inst.v & (1 << 25)) != 0 {
            cop.special(self.inst.v)
        } else {
            let func = (self.inst.v >> 21) & 0x0F;
            match func {
                0b00_000 => { // MFC - move control word from coprocessor
                    self.gpr[self.inst.rt] = (cop.mfc(self.inst.rd)? as i32) as u64;
                    Ok(())
                },

                0b00_001 => { // DMFC
                    self.gpr[self.inst.rt] = cop.dmfc(self.inst.rd)?;
                    Ok(())
                },

                0b00_010 => { // CFC
                    self.gpr[self.inst.rt] = (cop.cfc(self.inst.rd)? as i32) as u64;
                    Ok(())
                },

                0b00_011 | 0b00_111 => { // dcfc1, dctc1 are unimplemented
                    cop.unimplemented_instruction()
                },

                0b00_100 => { // MTC
                    cop.mtc(self.gpr[self.inst.rt] as u32, self.inst.rd)
                },

                0b00_101 => { // DMTC
                    cop.dmtc(self.gpr[self.inst.rt], self.inst.rd)
                },

                0b00_110 => { // CTC - move control word to coprocessor
                    cop.ctc(self.gpr[self.inst.rt], self.inst.rd)
                },

                0b01_000 => {
                    let branch = (self.inst.v >> 16) & 0x1F;
                    match branch {
                        0b00_000 => { // BCzF
                            let coc = cop.condition_signal();
                            self.branch(!coc);
                            Ok(())
                        },

                        0b00_001 => { // BCzT
                            let coc = cop.condition_signal();
                            self.branch(coc);
                            Ok(())
                        },

                        0b00_010 => { // BCzFL 
                            let coc = cop.condition_signal();
                            self.branch_likely(!coc)
                        },

                        0b00_011 => { // BCzTL
                            let coc = cop.condition_signal();
                            self.branch_likely(coc)
                        },

                        _ => {
                            warn!(target: "COP1", "unhandled branch mode ${:05b}", branch);
                            Err(InstructionFault::Unimplemented)
                        }
                    }
                },

                _ => panic!("CPU: unknown cop function 0b{:02b}_{:03b} (called on cop{})", func >> 3, func & 7, copno),
            }
        }
    }

    // given a register number and value (to be written to that register), return a value that 
    // that masks out the proper bits or prevents writing, etc
    fn mask_cp0_register(&mut self, register_number: usize, value: u64) -> u64 {
        match register_number {
            Cop0_Index => {
                value & 0x0000_0000_8000_003F // 32-bit register
            },

            Cop0_Context => {
                (self.cp0gpr[Cop0_Context] & 0x7F_FFFF) | (value & 0xFFFF_FFFF_FF80_0000) // 64-bit register
            },

            Cop0_Wired => {
                // Cop0_Random is set to the upper bound when Cop0_Wired is written
                self.cp0gpr[Cop0_Random] = 0x1F | (value & 0x20);
                value & 0x3F
            },

            Cop0_Random => {
                // only bit 5 is writeable
                (self.cp0gpr[Cop0_Random] & 0x1F) | (value & 0x20)
            },

            Cop0_Config => {
                let read_only_mask = 0xF0FF7FF0u64;
                ((value & 0xFFFF_FFFF) & !read_only_mask) | (self.cp0gpr[Cop0_Config] & read_only_mask)
            },

            Cop0_XContext => { // read-only register
                (self.cp0gpr[Cop0_XContext] & 0x1_FFFF_FFFF) | (value & 0xFFFF_FFFE_0000_0000)
            },

            Cop0_Compare => {
                // TODO clear timer interrupt see 6.3.4
                // truncate to 32-bits
                value & 0x0000_0000_FFFF_FFFF
            },

            Cop0_Cause => {
                // handle software interrupts 
                if (value & 0x300) != 0 {
                    panic!("CPU: cop0 software interrupt");
                }

                // Only bits IP1 and IP0 are writable
                (self.cp0gpr[Cop0_Cause] & !0x300) | (value & 0x300)
            },

            Cop0_LLAddr => {
                value & 0x0000_0000_FFFF_FFFF // 32-bit register
            },

            Cop0_Status => {
                // TODO testing if little endian mode or 32-bit mode is ever used
                if (value & 0x0200_0080) != 0x00 {
                    warn!("CPU: unsupported 64-bit kernel addressing or little endian mode ${value:08X}");
                }

                if (value & 0x18) != 0 {
                    panic!("CPU: unsupported user or supervisor mode");
                }

                if (value & 0x800000) != 0 {
                    panic!("CPU: unsupported instruction trace mode");
                }

                if (value & 0x200000) != 0 {
                    panic!("CPU: exception vector change");
                }
                ///////////////////////////////////////////////////////////

                self.cop1.set_fr(((value >> 26) & 0x01) != 0); // notify COP1 on the FR bit change
                value & 0x0000_0000_FFF7_FFFF // 32-bit register, bit 19 always 0
            },

            Cop0_PErr => {
                value & 0xFF
            },

            // read-only registers
            Cop0_BadVAddr | Cop0_PRId  => {
                self.cp0gpr[register_number]
            },

            // TLB registers
            Cop0_EntryLo0 | Cop0_EntryLo1 => {
                // The datasheet says PFN is 20 bits wide but it seems like everywhere else,
                // including n64-systemtest, is saying it's 24 bits, so it's 24 bits.
                value & 0x0000_0000_3FFF_FFFF
            },

            Cop0_EntryHi => {
                value & 0xC000_00FF_FFFF_E0FF
            },

            Cop0_PageMask => {
                value & 0x0000_0000_01FF_E000
            },

            // unused registers
            7 | 21 | 22 | 23 | 24 | 25 | 31 | Cop0_CacheErr => {
                0
            },

            _ => { value },
        }
    }

    // convert into inst_cop at some point
    // all the cop should implement a common cop trait (mfc/mtc/ctc/etc)
    fn inst_cop0(&mut self) -> Result<(), InstructionFault> {
        let cop0_op = (self.inst.v >> 21) & 0x1F;
        match cop0_op {
            0b00_000 => { // MFC
                self.gpr[self.inst.rt] = (match self.inst.rd {
                    7 | 21 | 22 | 23 | 24 | 25 | 31 => {
                        self.cp0gpr_latch
                    },

                    _ => self.cp0gpr[self.inst.rd],
                } as i32) as u64;
            },

            0b00_001 => { // DMFC
                self.gpr[self.inst.rt] = match self.inst.rd {
                    7 | 21 | 22 | 23 | 24 | 25 | 31 => {
                        self.cp0gpr_latch
                    },

                    _ => self.cp0gpr[self.inst.rd],
                };
            },

            0b00_100 => { // MTC
                let val = (self.gpr[self.inst.rt] as i32) as u64;

                // latch value for unused register reads
                self.cp0gpr_latch = val;

                // fix bits of the various registers
                self.cp0gpr[self.inst.rd] = self.mask_cp0_register(self.inst.rd, val);
            },

            0b00_101 => { // DMTC
                let val = self.gpr[self.inst.rt];

                // save write value for unused register reads
                self.cp0gpr_latch = val;

                self.cp0gpr[self.inst.rd] = self.mask_cp0_register(self.inst.rd, val);
            },


            0b10_000..=0b11_111 => {
                let special = self.inst.v & 0x3F;
                match special {
                    0b000_001 => { // tlbr
                        let tlb = &self.tlb[(self.cp0gpr[Cop0_Index] & 0x1F) as usize];
                        let g = (tlb.entry_hi >> 12) & 0x01;
                        self.cp0gpr[Cop0_PageMask] = tlb.page_mask;
                        self.cp0gpr[Cop0_EntryHi]  = tlb.entry_hi & !(0x1000 | tlb.page_mask);
                        self.cp0gpr[Cop0_EntryLo1] = tlb.entry_lo1 | g;
                        self.cp0gpr[Cop0_EntryLo0] = tlb.entry_lo0 | g;
                        //info!(target: "CPU", "COP0: tlbr, index={}, EntryHi=${:016X}, EntryLo0=${:016X}, EntryLo1=${:016X}, PageMask=${:016X}",
                        //            self.cp0gpr[Cop0_Index], self.cp0gpr[Cop0_EntryHi], self.cp0gpr[Cop0_EntryLo0], self.cp0gpr[Cop0_EntryLo1], self.cp0gpr[Cop0_PageMask]);
                    },

                    0b000_010 => { // tlbwi
                        //info!(target: "CPU", "COP0: tlbwi, index={}, EntryHi=${:016X}, EntryLo0=${:016X}, EntryLo1=${:016X}, PageMask=${:016X}",
                        //            self.cp0gpr[Cop0_Index], self.cp0gpr[Cop0_EntryHi], self.cp0gpr[Cop0_EntryLo0], self.cp0gpr[Cop0_EntryLo1], self.cp0gpr[Cop0_PageMask]);
                        self.set_tlb_entry(self.cp0gpr[Cop0_Index] & 0x1F);
                    },

                    0b000_110 => { // tlbwr
                        //info!(target: "CPU", "COP0: tlbwr, random={}, EntryHi=${:016X}, EntryLo0=${:016X}, EntryLo1=${:016X}, PageMask=${:016X}",
                        //            self.cp0gpr[Cop0_Random], self.cp0gpr[Cop0_EntryHi], self.cp0gpr[Cop0_EntryLo0], self.cp0gpr[Cop0_EntryLo1], self.cp0gpr[Cop0_PageMask]);
                        self.set_tlb_entry(self.cp0gpr[Cop0_Random] & 0x1F);
                    },

                    0b001_000 => { // tlbp
                        //info!(target: "CPU", "COP0: tlbp, EntryHi=${:016X}",
                        //            self.cp0gpr[Cop0_EntryHi]);
                        self.cp0gpr[Cop0_Index] = 0x8000_0000;
                        let entry_hi = self.cp0gpr[Cop0_EntryHi];
                        let asid = entry_hi & 0xFF;
                        for i in 0..32 {
                            let tlb = &self.tlb[i];

                            // if ASID matches or G is set, check this TLB entry
                            if (tlb.entry_hi & 0xFF) != asid && (tlb.entry_hi & 0x1000) == 0 { continue; }

                            // region must match
                            if ((tlb.entry_hi ^ entry_hi) & 0xC000_0000_0000_0000) != 0 { continue; }

                            // compare only VPN2 with the inverse of page mask
                            let tlb_vpn = (tlb.entry_hi & 0xFF_FFFF_E000) & !tlb.page_mask;
                            let ehi_vpn = (entry_hi & 0xFF_FFFF_E000) & !tlb.page_mask;

                            if tlb_vpn == ehi_vpn {
                                self.cp0gpr[Cop0_Index] = i as u64;
                                break;
                            }
                        }
                    },

                    0b011_000 => { // eret
                        assert!(!self.is_delay_slot); // ERET must not be in a delay slot TODO error gracefully

                        // If ERL bit is set, load the contents of ErrorEPC to the PC and clear the
                        // ERL bit. Otherwise, load the PC from EPC and clear the EXL bit.
                        if (self.cp0gpr[Cop0_Status] & 0x04) != 0 {
                            panic!("COP0: error bit set"); // TODO
                        } else {
                            self.pc = self.cp0gpr[Cop0_EPC];
                            self.cp0gpr[Cop0_Status] &= !0x02;
                        }

                        self.prefetch()?;

                        // clear LLbit so that SC writes fail
                        self.llbit = false;
                    },

                    _ => panic!("COP0: unknown cp0 function 0b{:03b}_{:03b}", special >> 3, special & 0x07),
                }
            },

            _ => panic!("CPU: unknown cop0 op: 0b{:02b}_{:03b} (0b{:032b})", cop0_op >> 3, cop0_op & 0x07, self.inst.v)
        }

        Ok(())
    }

    fn inst_cop2(&mut self) -> Result<(), InstructionFault> {
        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & 0x4000_0000) == 0 {
            self.coprocessor_unusable_exception(2)?;
            return Ok(());
        }

        if (self.inst.v & (1 << 25)) != 0 {
            // no special functions on COP2
            self.coprocessor_unusable_exception(2)?;
            Ok(())
        } else {
            let func = (self.inst.v >> 21) & 0x0F;
            match func {
                0b00_000 => { // MFC - move control word from coprocessor
                    self.gpr[self.inst.rt] = (self.cp2gpr_latch as i32) as u64;
                    Ok(())
                },

                0b00_001 => { // DMFC
                    self.gpr[self.inst.rt] = self.cp2gpr_latch;
                    Ok(())
                },

                0b00_010 => { // CFC
                    self.gpr[self.inst.rt] = (self.cp2gpr_latch as i32) as u64;
                    Ok(())
                },

                0b00_011 | 0b00_111 => { // dcfc2, dctc2 are unimplemented
                    self.reserved_instruction_exception(2)?;
                    // set the cop in the Cause register
                    self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0x3000_0000) | 0x2000_0000;
                    Ok(())
                },

                0b00_100 => { // MTC
                    self.cp2gpr_latch = self.gpr[self.inst.rt];
                    Ok(())
                },

                0b00_101 => { // DMTC
                    self.cp2gpr_latch = self.gpr[self.inst.rt];
                    Ok(())
                },

                0b00_110 => { // CTC - move control word to coprocessor
                    self.cp2gpr_latch = self.gpr[self.inst.rt];
                    Ok(())
                },

                0b01_000 => { // No branches on COP2
                    warn!(target: "COP2", "no branches on COP2");
                    Err(InstructionFault::Unimplemented)
                },

                _ => panic!("CPU: unknown cop2 function 0b{:02b}_{:03b}", func >> 3, func & 7),
            }
        }
    }

    fn inst_beq(&mut self) -> Result<(), InstructionFault> {
        let condition = self.gpr[self.inst.rs] == self.gpr[self.inst.rt];
        self.branch(condition);
        Ok(())
    }

    fn inst_beql(&mut self) -> Result<(), InstructionFault> {
        let condition = self.gpr[self.inst.rs] == self.gpr[self.inst.rt];
        self.branch_likely(condition)
    }

    fn inst_bgtz(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) > 0;
        self.branch(condition);
        Ok(())
    }

    fn inst_blez(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) <= 0;
        self.branch(condition);
        Ok(())
    }

    fn inst_blezl(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) <= 0;
        self.branch_likely(condition)
    }

    fn inst_bgtzl(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) > 0;
        self.branch_likely(condition)
    }

    fn inst_bne(&mut self) -> Result<(), InstructionFault> {
        let condition = self.gpr[self.inst.rs] != self.gpr[self.inst.rt];
        self.branch(condition);
        Ok(())
    }

    fn inst_bnel(&mut self) -> Result<(), InstructionFault> {
        let condition = self.gpr[self.inst.rs] != self.gpr[self.inst.rt];
        self.branch_likely(condition)
    }

    fn inst_cache(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

     fn inst_daddi(&mut self) -> Result<(), InstructionFault> {
        // integer overflow exception occurs with DADDI, unlike DADDIU
        let rs = self.gpr[self.inst.rs] as u64;
        let rt = self.inst.signed_imm;
        let result = rs.wrapping_add(rt);

        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000_0000_0000) != 0 {
            //println!("CPU: daddi overflow detected: rs=${:16} imm=${:04X} result=${:16}", rs, rt, result);
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rt] = result;
        }
        Ok(())
    }

   fn inst_daddiu(&mut self) -> Result<(), InstructionFault> {
        // no integer overflow exception occurs with DADDIU
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        Ok(())
    }

    fn inst_j(&mut self) -> Result<(), InstructionFault> {
        let dest = ((self.pc - 4) & 0xFFFF_FFFF_F000_0000) | ((self.inst.target << 2) as u64);

        self.pc = dest;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
        Ok(())
    }

    fn inst_jal(&mut self) -> Result<(), InstructionFault> {
        let dest = ((self.pc - 4) & 0xFFFF_FFFF_F000_0000) | ((self.inst.target << 2) as u64);

        self.gpr[31] = self.pc;
        self.pc = dest;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
        Ok(())
    }

    fn inst_lb(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.gpr[self.inst.rt] = (self.read_u8(address as usize)? as u8) as u64;
        Ok(())
    }

    fn inst_lbu(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.gpr[self.inst.rt] = self.read_u8(address as usize)? as u64;
        Ok(())
    }

    fn inst_ld(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x07) != 0 {
            self.address_exception(address, false)?;
        }

        self.gpr[self.inst.rt] = self.read_u64(address as usize)?;
        Ok(())
    }

    fn inst_ldl(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u64 at the specified address
        // (perhaps an optimization would be only read one word if address & 0x4 is set
        let mem = self.read_u64(address & !0x07)?;

        // combine register and mem
        let shift = (address & 0x07) << 3;
        let new = if shift == 0 {
            mem
        } else {
            (self.gpr[self.inst.rt] & (u64::MAX >> (64 - shift))) | (mem << shift)
        };

        // set value
        self.gpr[self.inst.rt] = new; 
        Ok(())
    }

    fn inst_ldr(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u64 at the specified address
        // (perhaps an optimization would be only read one word if address & 0x4 is set
        let mem = self.read_u64(address & !0x07)?;

        // combine register and mem
        let shift = (address & 0x07) << 3;

        let new = if shift == 56 { // handle case where no shift occurs
            mem 
        } else {
            (self.gpr[self.inst.rt] & (u64::MAX << (8 + shift))) | (mem >> (56 - shift))
        };

        // set value
        self.gpr[self.inst.rt] = new; 
        Ok(())
    }

    fn inst_lh(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);

        if (address & 0x01) != 0 {
            self.address_exception(address, false)?;
        }

        self.gpr[self.inst.rt] = (self.read_u16((address & !0x02) as usize)? as i16) as u64;
        Ok(())
    }

    fn inst_lhu(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);

        if (address & 0x01) != 0 {
            self.address_exception(address, false)?;
        }

        self.gpr[self.inst.rt] = self.read_u16(address as usize)? as u64;
        Ok(())
    }

    fn inst_ll(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, false)?;
        }
        self.gpr[self.inst.rt] = (self.read_u32(address as usize)? as i32) as u64;

        // the "linked part" sets the LLAddr register in cop0 to the physical address
        // of the read, and the LLbit to 1
        self.cp0gpr[Cop0_LLAddr] = address & 0x1FFF_FFFF; // TODO use proper physical address
        self.llbit = true;

        Ok(())
    }

    fn inst_lui(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.inst.signed_imm << 16;
        Ok(())
    }

    fn inst_lw(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, false)?;
        } else {
            self.gpr[self.inst.rt] = (self.read_u32(address as usize)? as i32) as u64;
        }
        Ok(())
    }

    fn inst_lwu(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, false)?;
        } else {
            self.gpr[self.inst.rt] = self.read_u32(address as usize)? as u64;
        }
        Ok(())
    }

    fn inst_lwl(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u32 at the specified address
        let mem = self.read_u32(address & !0x03)?;

        // combine register and mem
        let shift = (address & 0x03) << 3;
        let new = if shift == 0 {
            mem
        } else {
            ((self.gpr[self.inst.rt] as u32) & (u32::MAX >> (32 - shift))) | (mem << shift)
        };

        // set value
        self.gpr[self.inst.rt] = (new as i32) as u64; 
        Ok(())
    }

    fn inst_lwr(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u32 at the specified address
        let mem = self.read_u32(address & !0x03)?;

        // combine register and mem
        let shift = (address & 0x03) << 3;
        let new = if shift == 24 { // handle case where no shift occurs
            mem 
        } else {
            ((self.gpr[self.inst.rt] as u32) & (u32::MAX << (8 + shift))) | (mem >> (24 - shift))
        };

        // set value
        self.gpr[self.inst.rt] = (new as i32) as u64; 
        Ok(())
    }

    fn inst_ldc1(&mut self) -> Result<(), InstructionFault> {
        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & 0x2000_0000) == 0 {
            self.coprocessor_unusable_exception(1)?;
            return Ok(());
        }

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        
        if (address & 0x07) != 0 {
            self.address_exception(address, false)?;
        }

        let value = self.read_u64(address as usize)?;
        self.cop1.ldc(self.inst.rt, value)
    }

    fn inst_lwc1(&mut self) -> Result<(), InstructionFault> {
        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & 0x2000_0000) == 0 {
            self.coprocessor_unusable_exception(1)?;
            return Ok(());
        }

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        
        if (address & 0x03) != 0 {
            self.address_exception(address, false)?;
        }

        let value = self.read_u32(address as usize)?;
        self.cop1.lwc(self.inst.rt, value)
    }

    fn inst_ori(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] | self.inst.imm;
        Ok(())
    }

    fn inst_regimm(&mut self) -> Result<(), InstructionFault> {
        self.inst.regimm = (self.inst.v >> 16) & 0x1F;
        self.regimm_table[self.inst.regimm as usize](self)
    }

    fn inst_sb(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.write_u8(self.gpr[self.inst.rt] as u32, address as usize)?;
        Ok(())
    }
    
    fn inst_sh(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.write_u16(self.gpr[self.inst.rt] as u32, address as usize)?;
        Ok(())
    }

    fn inst_sc(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, true)?;
        }

        if self.llbit && ((address & 0x1FFF_FFFF) == self.cp0gpr[Cop0_LLAddr]) {
            self.write_u32(self.gpr[self.inst.rt] as u32, address as usize)?;
            self.gpr[self.inst.rt] = 1;
        } else {
            self.gpr[self.inst.rt] = 0;
        }
        Ok(())
    }

    fn inst_sd(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x07) != 0 {
            self.address_exception(address, true)?;
        }

        self.write_u64(self.gpr[self.inst.rt], address as usize)?;
        Ok(())
    }

    fn inst_sdc1(&mut self) -> Result<(), InstructionFault> {
        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & 0x2000_0000) == 0 {
            self.coprocessor_unusable_exception(1)?;
            return Ok(());
        }

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x07) != 0 {
            self.address_exception(address, true)?;
        }

        // TODO: need to catch invalid sitatuations (see datasheet)

        let value = self.cop1.sdc(self.inst.rt)?;
        self.write_u64(value, address as usize)?;
        Ok(())
    }

    fn inst_swc1(&mut self) -> Result<(), InstructionFault> {
        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & 0x2000_0000) == 0 {
            self.coprocessor_unusable_exception(1)?;
            return Ok(());
        }

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, true)?;
        }

        // TODO: need to catch invalid sitatuations (see datasheet)

        let value = self.cop1.swc(self.inst.rt)?;
        self.write_u32(value, address as usize)?;
        Ok(())
    }


    fn inst_sdl(&mut self) -> Result<(), InstructionFault> {
        let virtual_address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // translate the address with the offset so that TLB errors produce the correct BadVAddr
        let mut address = match self.translate_address(virtual_address as u64, true, true)? {
            Some(address) => address,
            None => {
                // this shouldn't happen, but if it does we act like a NOP
                warn!(target: "CPU", "translate_address for ${:016X} returned None", virtual_address);
                return Ok(());
            }
        };

        // extract the dword offset
        let offset = address.physical_address & 0x07;
        address.physical_address &= !0x07;

        // need to fetch data on cache misses but not uncachable addresses
        let mem = if (virtual_address & 0xF000_0000) != 0xA000_0000 {
            self.read_u64_phys(address)?  // this read "simulates" the cache miss and fetch and 
                                          // doesn't happen for uncached addresses
        } else { panic!("test"); /*0*/ };

        // combine register and mem
        let shift = offset << 3;
        let new = if shift == 0 {
            self.gpr[self.inst.rt]
        } else {
            (self.gpr[self.inst.rt] >> shift) | (mem & (u64::MAX << (64 - shift)))
        };

        // write new value
        self.write_u64_phys(new, address)?;
        Ok(())
    }

    fn inst_sdr(&mut self) -> Result<(), InstructionFault> {
        let virtual_address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // translate the address with the offset so that TLB errors produce the correct BadVAddr
        let mut address = match self.translate_address(virtual_address as u64, true, true)? {
            Some(address) => address,
            None => {
                // this shouldn't happen, but if it does we act like a NOP
                warn!(target: "CPU", "translate_address for ${:016X} returned None", virtual_address);
                return Ok(());
            }
        };

        // extract the word offset
        let offset = address.physical_address & 0x07;
        address.physical_address &= !0x07;

        // need to fetch data on cache misses but not uncachable addresses
        let mem = if (virtual_address & 0xF000_0000) != 0xA000_0000 {
            self.read_u64_phys(address)? // this read "simulates" the cache miss and fetch and 
                                         // doesn't happen for uncached addresses
        } else { panic!("test"); /*0*/ };

        // combine register and mem
        let shift = 56 - (offset << 3);
        let new = if shift == 0 {
            self.gpr[self.inst.rt]
        } else {
            (self.gpr[self.inst.rt] << shift) | (mem & (u64::MAX >> (64 - shift)))
        };

        // write new value
        self.write_u64_phys(new, address)?;
        Ok(())
    }

    fn inst_slti(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i64) < (self.inst.signed_imm as i64) {
            self.gpr[self.inst.rt] = 1;
        } else {
            self.gpr[self.inst.rt] = 0;
        }

        Ok(())
    }

    fn inst_sltiu(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = (self.gpr[self.inst.rs] < self.inst.signed_imm) as u64;

        Ok(())
    }

    fn inst_special(&mut self) -> Result<(), InstructionFault> {
        self.inst.special = self.inst.v & 0x3F;
        self.special_table[self.inst.special as usize](self)
    }

    fn inst_sw(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, true)?;
        } else {
            self.write_u32(self.gpr[self.inst.rt] as u32, address as usize)?;
        }
        Ok(())
    }

    fn inst_swl(&mut self) -> Result<(), InstructionFault> {
        let virtual_address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // translate the address with the offset so that TLB errors produce the correct BadVAddr
        let mut address = match self.translate_address(virtual_address as u64, true, true)? {
            Some(address) => address,
            None => {
                // this shouldn't happen, but if it does we act like a NOP
                warn!(target: "CPU", "translate_address for ${:016X} returned None", virtual_address);
                return Ok(());
            }
        };

        // extract the word offset
        let offset = address.physical_address & 0x03;
        address.physical_address &= !0x03;

        // need to fetch data on cache misses but not uncachable addresses
        let mem = if (virtual_address & 0xF000_0000) != 0xA000_0000 {
            self.read_u32_phys(address)? // this read "simulates" the cache miss and fetch and 
                                         // doesn't happen for uncached addresses
        } else { panic!("test"); /*0*/ };

        // combine register and mem
        let shift = offset << 3;
        let new = if shift == 0 {
            self.gpr[self.inst.rt] as u32
        } else {
            ((self.gpr[self.inst.rt] as u32) >> shift) | (mem & (u32::MAX << (32 - shift)))
        };

        // write new value
        self.write_u32_phys(new, address)?;
        Ok(())
    }

    fn inst_swr(&mut self) -> Result<(), InstructionFault> {
        let virtual_address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // translate the address with the offset so that TLB errors produce the correct BadVAddr
        let mut address = match self.translate_address(virtual_address as u64, true, true)? {
            Some(address) => address,
            None => {
                // this shouldn't happen, but if it does we act like a NOP
                warn!(target: "CPU", "translate_address for ${:016X} returned None", virtual_address);
                return Ok(());
            }
        };

        // extract the word offset
        let offset = address.physical_address & 0x03;
        address.physical_address &= !0x03;

        // need to fetch data on cache misses but not uncachable addresses
        let mem = if (virtual_address & 0xF000_0000) != 0xA000_0000 {
            self.read_u32_phys(address)? // this read "simulates" the cache miss and fetch and 
                                         // doesn't happen for uncached addresses
        } else { panic!("test"); /*0*/ };

        // combine register and mem
        let shift = 24 - (offset << 3);
        let new = if shift == 0 {
            self.gpr[self.inst.rt] as u32
        } else {
            ((self.gpr[self.inst.rt] as u32) << shift) | (mem & (u32::MAX >> (32 - shift)))
        };

        // write new value
        self.write_u32_phys(new, address)?;
        Ok(())
    }

    fn inst_xori(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] ^ self.inst.imm;
        Ok(())
    }

    fn regimm_unknown(&mut self) -> Result<(), InstructionFault> {
        panic!("CPU: unimplemented regimm op: 0b{:02b}_{:03b}", self.inst.regimm >> 3, self.inst.regimm & 0x07);
    }

    fn regimm_bgezal(&mut self) -> Result<(), InstructionFault> {
        // compute condition before changing gpr31 rs can be 31...
        let condition = (self.gpr[self.inst.rs] as i64) >= 0;

        // addresses are sign extended
        self.gpr[31] = self.pc; // unconditionally, the address after the delay slot is stored in the link register

        self.branch(condition);

        Ok(())
    }

    fn regimm_bgezall(&mut self) -> Result<(), InstructionFault> {
        // addresses are sign extended
        self.gpr[31] = self.pc; // unconditionally, the address after the delay slot is stored in the link register

        let condition = (self.gpr[self.inst.rs] as i64) >= 0;
        self.branch_likely(condition)?;

        Ok(())
    }

    fn regimm_bgez(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) >= 0;
        self.branch(condition);

        Ok(())
    }

    fn regimm_bgezl(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) >= 0;
        self.branch_likely(condition)
    }

    fn regimm_bltz(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) < 0;
        self.branch(condition);

        Ok(())
    }

    fn special_add(&mut self) -> Result<(), InstructionFault> {
        // add does cause an overflow exception
        let rs = self.gpr[self.inst.rs] as u32;
        let rt = self.gpr[self.inst.rt] as u32;

        let result = rs.wrapping_add(rt);
        // if addends had the same sign but the result differs, overflow occurred
        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000) != 0 {
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rd] = (result as i32) as u64;
        }

        Ok(())
    }

    fn special_addu(&mut self) -> Result<(), InstructionFault> {
        // addu does not cause an overflow exception
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rs] as u32).wrapping_add(self.gpr[self.inst.rt] as u32) as i32) as u64;

        Ok(())
    }

    fn special_and(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] & self.gpr[self.inst.rt];

        Ok(())
    }

    fn special_break(&mut self) -> Result<(), InstructionFault> {
        self.breakpoint_exception()?;
        Ok(())
    }

    fn special_dadd(&mut self) -> Result<(), InstructionFault> {
        // add does cause an overflow exception
        let rs = self.gpr[self.inst.rs];
        let rt = self.gpr[self.inst.rt];
        let result = rs.wrapping_add(rt);

        // if addends had the same sign but the result differs, overflow occurred
        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000_0000_0000u64) != 0 {
            //println!("CPU: dadd overflow exception occurred");
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rd] = result;
        }

        Ok(())
    }

    fn special_daddu(&mut self) -> Result<(), InstructionFault> {
        // daddu does not cause an overflow exception
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs].wrapping_add(self.gpr[self.inst.rt]);

        Ok(())
    }

    fn divide(&mut self, dividend: i64, divisor: i64) {
        if divisor != 0 {
            if dividend == i64::MIN && divisor == -1 {
                (self.lo, self.hi) = (i64::MIN as u64, 0)
            } else {
                // ideally the compiler will optmize this to a divmod operation
                (self.lo, self.hi) = ((dividend / divisor) as u64, (dividend % divisor) as u64)
            }
        } else {
            // with divisor == 0, set self.lo (the quotient) to 1 or -1 basd on the sign
            // of the dividend
            self.lo = if dividend < 0 { 1 } else { u64::MAX };
            self.hi = dividend as u64;
        }
    }

    fn divide_unsigned(&mut self, dividend: u64, divisor: u64) {
        if divisor != 0 {
            // ideally the compiler will optmize this to a divmod operation
            (self.lo, self.hi) = (dividend / divisor, dividend % divisor)
        } else {
            self.lo = u64::MAX;
            self.hi = dividend as u64;
        }
    }

    fn special_ddiv(&mut self) -> Result<(), InstructionFault> {
        let rs = self.gpr[self.inst.rs] as i64;
        let rt = self.gpr[self.inst.rt] as i64;
        self.divide(rs, rt);

        Ok(())
    }

    fn special_ddivu(&mut self) -> Result<(), InstructionFault> {
        // unsigned 64 bit values
        self.divide_unsigned(self.gpr[self.inst.rs], self.gpr[self.inst.rt]);

        Ok(())
    }

    fn special_div(&mut self) -> Result<(), InstructionFault> {
        // must be 32-bit sign extended values
        let rs = (self.gpr[self.inst.rs] as i32) as i64;
        let rt = (self.gpr[self.inst.rt] as i32) as i64;
        self.divide(rs, rt);

        // with the 32 bit divide the sign needs to come from bit 31
        self.lo = (self.lo as i32) as u64;
        self.hi = (self.hi as i32) as u64;

        Ok(())
    }

    fn special_divu(&mut self) -> Result<(), InstructionFault> {
        // unsigned 32 bit values
        let rs = self.gpr[self.inst.rs] & 0xFFFF_FFFF;
        let rt = self.gpr[self.inst.rt] & 0xFFFF_FFFF;
        self.divide_unsigned(rs, rt);

        // with the 32 bit divide the sign needs to come from bit 31
        self.lo = (self.lo as i32) as u64;
        self.hi = (self.hi as i32) as u64;

        Ok(())
    }

    fn special_dmult(&mut self) -> Result<(), InstructionFault> {
        // use 64-bit signed integers and get 128-bit result
        let result = ((self.gpr[self.inst.rs] as i64) as i128) * ((self.gpr[self.inst.rt] as i64) as i128);

        // multu results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result as u64;
        self.hi = (result >> 64) as u64;

        Ok(())
    }

    fn special_dmultu(&mut self) -> Result<(), InstructionFault> {
        // must be 32-bit unsigned numbers
        let result = (self.gpr[self.inst.rs] as u128) * (self.gpr[self.inst.rt] as u128);

        // multu results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result as u64;
        self.hi = (result >> 64) as u64;

        Ok(())
    }


    fn special_dsll(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << self.inst.sa;
        Ok(())
    }

    fn special_dsllv(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << (self.gpr[self.inst.rs] & 0x3F);
        Ok(())
    }

    fn special_dsll32(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << (32 + self.inst.sa);
        Ok(())
    }

    fn special_dsra(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i64) >> self.inst.sa) as u64;
        Ok(())
    }

    fn special_dsrav(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i64) >> (self.gpr[self.inst.rs] & 0x3F)) as u64;
        Ok(())
    }

    fn special_dsra32(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i64) >> (32 + self.inst.sa)) as u64;
        Ok(())
    }

    fn special_dsrl32(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> (32 + self.inst.sa);
        Ok(())
    }

    fn special_dsrl(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> self.inst.sa;
        Ok(())
    }

    fn special_dsrlv(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> (self.gpr[self.inst.rs] & 0x3F);
        Ok(())
    }

    fn special_dsub(&mut self) -> Result<(), InstructionFault> {
        // sub causes an overflow exception
        let rs = self.gpr[self.inst.rs];
        let rt = self.gpr[self.inst.rt];

        let result = rs.wrapping_sub(rt);
        // if addends had the same sign but the result differs, overflow occurred
        if (((rs ^ rt) & (rs ^ result)) & 0x8000_0000_0000_0000u64) != 0 {
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rd] = result;
        }

        Ok(())
    }

    fn special_dsubu(&mut self) -> Result<(), InstructionFault> {
        // dsubu does not cause an overflow exception
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs].wrapping_sub(self.gpr[self.inst.rt]);

        Ok(())
    }

    fn special_jalr(&mut self) -> Result<(), InstructionFault> {
        let dest = self.gpr[self.inst.rs]; // get dest before changing RD, as RS could be == RD
        self.gpr[self.inst.rd] = self.pc; // pc pointing to after the delay slot already
        self.pc = dest;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;

        Ok(())
    }

    fn special_jr(&mut self) -> Result<(), InstructionFault> {
        self.pc = self.gpr[self.inst.rs];

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;

        Ok(())
    }

    fn special_mfhi(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.hi;
        Ok(())
    }

    fn special_mflo(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.lo;
        Ok(())
    }

    fn special_mthi(&mut self) -> Result<(), InstructionFault> {
        self.hi = self.gpr[self.inst.rd];
        Ok(())
    }

    fn special_mtlo(&mut self) -> Result<(), InstructionFault> {
        self.lo = self.gpr[self.inst.rd];
        Ok(())
    }

    fn special_mult(&mut self) -> Result<(), InstructionFault> {
        // must be 32-bit sign extended values
        let result = ((self.gpr[self.inst.rs] as i32) as u64).wrapping_mul((self.gpr[self.inst.rt] as i32) as u64);

        // multu results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result & 0xFFFF_FFFF;
        self.hi = result >> 32;

        Ok(())
    }

    fn special_multu(&mut self) -> Result<(), InstructionFault> {
        // must be 32-bit unsigned numbers
        let result = (self.gpr[self.inst.rs] & 0xFFFF_FFFF).wrapping_mul(self.gpr[self.inst.rt] & 0xFFFF_FFFF);

        // multu results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result & 0xFFFF_FFFF;
        self.hi = result >> 32;

        Ok(())
    }

    fn special_nor(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = !(self.gpr[self.inst.rs] | self.gpr[self.inst.rt]);
        Ok(())
    }

    fn special_or(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] | self.gpr[self.inst.rt];
        Ok(())
    }

    fn special_teq(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] == self.gpr[self.inst.rt] {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn regimm_teqi(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] == (self.inst.signed_imm as u64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn special_tge(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i64) >= (self.gpr[self.inst.rt] as i64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn regimm_tgei(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i64) >= (self.inst.signed_imm as i64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn special_tgeu(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] >= self.gpr[self.inst.rt] {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn regimm_tgeiu(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] >= (self.inst.signed_imm as u64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn special_tlt(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i64) < (self.gpr[self.inst.rt] as i64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn regimm_tlti(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i64) < (self.inst.signed_imm as i64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn special_tltu(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] < self.gpr[self.inst.rt] {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn regimm_tltiu(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] < (self.inst.signed_imm as u64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn special_tne(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] != self.gpr[self.inst.rt] {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn regimm_tnei(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] != (self.inst.signed_imm as u64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn special_sll(&mut self) -> Result<(), InstructionFault> {
        // 32-bit shift and sign extended into 64 bits
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) << self.inst.sa) as i32) as u64;
        Ok(())
    }

    fn special_sllv(&mut self) -> Result<(), InstructionFault> {
        // 32-bit shift and sign extended into 64 bits
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) << (self.gpr[self.inst.rs] & 0x1F)) as i32) as u64;
        Ok(())
    }

    fn special_slt(&mut self) -> Result<(), InstructionFault> {
        // set rd to 1 if rs < rt, otherwise 0
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rs] as i64) < (self.gpr[self.inst.rt] as i64)) as u64;
        Ok(())
    }

    fn special_sltu(&mut self) -> Result<(), InstructionFault> {
        // set rd to 1 if rs < rt, otherwise 0
        self.gpr[self.inst.rd] = (self.gpr[self.inst.rs] < self.gpr[self.inst.rt]) as u64;
        Ok(())
    }

    fn special_sra(&mut self) -> Result<(), InstructionFault> {
        // TODO I'm very confused here. The VR4300 datasheet is very clear that a 32-bit signed
        // integer is right shifted, and at the end 64-bit sign extended, but n64-systemtest
        // seems to check that it's just a 64 shift and then truncated to 32 bits and then sign
        // extended.  I've left the "working" one in place but the one I think is correct (at least
        // until someone explains to my why it isn't!) below.
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] >> self.inst.sa) as i32) as u64;

        // truncate to u32, convert to signed, shift (fills 1s in the upper bits) and sign extend to u64
        //self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) as i32) >> self.inst.sa) as u64;
        Ok(())
    }

    fn special_srav(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] >> (self.gpr[self.inst.rs] & 0x1F)) as i32) as u64;
        Ok(())
    }

    fn special_srl(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) >> self.inst.sa) as i32) as u64;
        Ok(())
    }

    fn special_srlv(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) >> (self.gpr[self.inst.rs] & 0x1F)) as i32) as u64;
        Ok(())
    }

    fn special_sub(&mut self) -> Result<(), InstructionFault> {
        // sub causes an overflow exception
        let rs = self.gpr[self.inst.rs] as u32;
        let rt = self.gpr[self.inst.rt] as u32;

        let result = rs.wrapping_sub(rt);
        // if addends had the same sign but the result differs, overflow occurred
        if (((rs ^ rt) & (rs ^ result)) & 0x8000_0000) != 0 {
            //println!("CPU: sub overflow exception occurred");
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rd] = (result as i32) as u64;
        }
        Ok(())
    }

    fn special_subu(&mut self) -> Result<(), InstructionFault> {
        // subu does not cause an overflow exception
        self.gpr[self.inst.rd] = (self.gpr[self.inst.rs].wrapping_sub(self.gpr[self.inst.rt]) as i32) as u64;
        Ok(())
    }

    fn special_syscall(&mut self) -> Result<(), InstructionFault> {
        self.syscall_exception()?;
        Ok(())
    }

    fn special_sync(&mut self) -> Result<(), InstructionFault> {
        // NOP on VR4300
        Ok(())
    }

    fn special_xor(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] ^ self.gpr[self.inst.rt];
        Ok(())
    }

    pub fn disassemble(address: u64, inst: u32, use_abi_names: bool) -> String {
        let op = inst >> 26;
        let rs = (inst >> 21) & 0x1F;
        let rt = (inst >> 16) & 0x1F;
        let rd = (inst >> 11) & 0x1F;
        let sa = (inst >> 6) & 0x1F;
        let func = inst & 0x3F;
        let imm = inst & 0xFFFF;
        let target = inst & 0x00FFFFFF;

        let rname = |r| {
            if use_abi_names {
                Cpu::abi_name(r as usize)
            } else {
                Cpu::register_name(r as usize)
            }
        };

        let no_type = |mn| {
            format!("{}", mn)
        };

        let j_type = |mn| {
            format!("{} ${:08X}", mn, (((address as u32) + 4) & 0xF000_0000) | (target << 2))
        };

        let i_type_rt = |mn| {
            format!("{} {}, ${:04X}", mn, rname(rt), imm)
        };

        let i_type_rt_base = |mn| {
            format!("{} {}, ${:04X}({})", mn, rname(rt), imm, rname(rs))
        };

        let i_type_ft_base = |mn| {
            format!("{} f{}, ${:04X}({})", mn, rt, imm, rname(rs))
        };

        let i_type1_rs = |mn| {
            format!("{} {}, ${:04X}", mn, rname(rs), imm)
        };

        let i_type_rs_rt = |mn| {
            format!("{} {}, {}, ${:04X}", mn, rname(rs), rname(rt), imm)
        };

        let i_type_rt_rs = |mn| {
            format!("{} {}, {}, ${:04X}", mn, rname(rt), rname(rs), imm)
        };

        let r_type_rd = |mn| {
            format!("{} {}", mn, rname(rd))
        };

        let r_type_rs = |mn| {
            format!("{} {}", mn, rname(rs))
        };

        let r_type_rd_rs = |mn| {
            format!("{} {}, {}", mn, rname(rd), rname(rs))
        };

        let r_type_rs_rt = |mn| {
            format!("{} {}, {}", mn, rname(rs), rname(rt))
        };

        let r_type_rt_rd = |mn| {
            format!("{} {}, {}", mn, rname(rt), rname(rd))
        };

        let r_type_rd_rt_rs = |mn| {
            format!("{} {}, {}, {}", mn, rname(rd), rname(rt), rname(rs))
        };

        let r_type_rd_rs_rt = |mn| {
            format!("{} {}, {}, {}", mn, rname(rd), rname(rs), rname(rt))
        };

        let r_type_rd_rt_sa = |mn| {
            format!("{} {}, {}, {}", mn, rname(rd), rname(rt), sa)
        };

        match op {
            0b000_000 => {
                match func {
                    0b000_000 => r_type_rd_rt_sa("sll"),
                    0b000_010 => r_type_rd_rt_sa("srl"),
                    0b000_011 => r_type_rd_rt_sa("sra"),
                    0b000_100 => r_type_rd_rt_rs("sllv"),
                    0b000_110 => r_type_rd_rt_rs("srlv"),
                    0b000_111 => r_type_rd_rt_rs("srav"),

                    0b001_000 => r_type_rs("jr"),
                    0b001_001 => r_type_rd_rs("jalr"),
                    0b001_100 => no_type("syscall"),
                    0b001_101 => no_type("break"),
                    0b001_111 => no_type("sync"),

                    0b010_000 => r_type_rd("mfhi"),
                    0b010_001 => r_type_rs("mthi"),
                    0b010_010 => r_type_rd("mflo"),
                    0b010_011 => r_type_rs("mtlo"),
                    0b010_100 => r_type_rd_rt_rs("dsllv"),
                    0b010_110 => r_type_rd_rt_rs("dsrlv"),
                    0b010_111 => r_type_rd_rt_rs("dsrav"),

                    0b011_000 => r_type_rs_rt("mult"),
                    0b011_001 => r_type_rs_rt("multu"),
                    0b011_010 => r_type_rs_rt("div"),
                    0b011_011 => r_type_rs_rt("divu"),
                    0b011_100 => r_type_rs_rt("dmult"),
                    0b011_101 => r_type_rs_rt("dmultu"),
                    0b011_110 => r_type_rs_rt("ddiv"),
                    0b011_111 => r_type_rs_rt("ddivu"),

                    0b100_000 => r_type_rd_rs_rt("add"),
                    0b100_001 => r_type_rd_rs_rt("addu"),
                    0b100_010 => r_type_rd_rs_rt("sub"),
                    0b100_011 => r_type_rd_rs_rt("subu"),
                    0b100_100 => r_type_rd_rs_rt("and"),
                    0b100_101 => r_type_rd_rs_rt("or"),
                    0b100_110 => r_type_rd_rs_rt("xor"),
                    0b100_111 => r_type_rd_rs_rt("nor"),

                    0b101_010 => r_type_rd_rs_rt("slt"),
                    0b101_011 => r_type_rd_rs_rt("sltu"),
                    0b101_100 => r_type_rd_rs_rt("dadd"),
                    0b101_101 => r_type_rd_rs_rt("daddu"),
                    0b101_110 => r_type_rd_rs_rt("dsub"),
                    0b101_111 => r_type_rd_rs_rt("dsubu"),

                    0b110_000 => r_type_rs_rt("tge"),
                    0b110_001 => r_type_rs_rt("tgeu"),
                    0b110_010 => r_type_rs_rt("tlt"),
                    0b110_011 => r_type_rs_rt("tltu"),
                    0b110_100 => r_type_rs_rt("teq"),
                    0b110_110 => r_type_rs_rt("tne"),

                    0b111_000 => r_type_rd_rt_sa("dsll"),
                    0b111_010 => r_type_rd_rt_sa("dsrl"),
                    0b111_011 => r_type_rd_rt_sa("dsra"),
                    0b111_100 => r_type_rd_rt_sa("dsll32"),
                    0b111_110 => r_type_rd_rt_sa("dsrl32"),
                    0b111_111 => r_type_rd_rt_sa("dsra32"),

                    _ => panic!("unknown special rs=${:03b}_{:03b}", func >> 3, func & 0x07),
                }
            },

            0b000_001 => {
                // RT field contains the instruction code
                const REGIMM: [&str; 32] = [
                    "bltz", "bgez", "bltzl", "bgezl", "?", "?", "?", "?",
                    "tgei", "tgeiu", "tlti", "tltiu", "teqi", "?", "tnei", "?",
                    "bltzal", "bgezal", "bltzall", "bgezall", "?", "?", "?", "?",
                    "?", "?", "?", "?", "?", "?", "?", "?",
                ];
                if REGIMM[rt as usize].starts_with("?") {
                    panic!("unknown regimm rs=${:02b}_{:03b}", rs >> 3, rs & 0x07);
                }
                i_type1_rs(REGIMM[rt as usize])
            },

            0b000_010 => j_type("j"),
            0b000_011 => j_type("jal"),

            0b000_100 => i_type_rs_rt("beq"),
            0b000_101 => i_type_rs_rt("bne"),
            0b000_110 => i_type_rs_rt("blez"),
            0b000_111 => i_type_rs_rt("bgtz"),

            0b001_000 => i_type_rt_rs("addi"),
            0b001_001 => i_type_rt_rs("addiu"),
            0b001_010 => i_type_rt_rs("slti"),
            0b001_011 => i_type_rt_rs("sltiu"),

            0b001_100 => i_type_rt_rs("andi"),
            0b001_101 => i_type_rt_rs("ori"),
            0b001_110 => i_type_rt_rs("xori"),

            0b001_111 => i_type_rt("lui"),

            0b010_000 | 0b010_001 | 0b010_010 => {
                // RS field contains the instruction code
                const COP_FN: [&str; 32] = [
                    "mfc", "dmfc", "cfc", "?", "mtc", "dmtc", "ctc", "?",
                    "?", "?", "?", "?", "?", "?", "?", "?",
                    "?", "?", "?", "?", "?", "?", "?", "?",
                    "?", "?", "?", "?", "?", "?", "?", "?"
                ];
                let copno = op & 0x03;
                if rs < 0b10_000 {
                    if COP_FN[rs as usize].starts_with("?") {
                        error!("unknown cop function rs=${:02b}_{:03b}", rs >> 3, rs & 0x07);
                    }
                    r_type_rt_rd(format!("{}{}", COP_FN[rs as usize], copno))
                } else {
                    match copno {
                        1 => {
                            const CP1_FN: [&str; 64] = [
                                "add", "sub", "mul", "div", "sqrt", "abs", "mov", "neg",
                                "round.l", "trunc.l", "ceil.l", "floor.l", "round.w", "trunc.w", "ceil.w", "floor.w",
                                "?", "?", "?", "?", "?", "?", "?", "?",
                                "?", "?", "?", "?", "?", "?", "?", "?",
                                "cvt.s", "cvt.d", "?", "?", "cvt.w", "cvt.l", "?", "?",
                                "?", "?", "?", "?", "?", "?", "?", "?",
                                "c.f", "c.un", "c.eq", "c.ueq", "c.olt", "c.ult", "c.ole", "c.ule",
                                "c.sf", "c.ngle", "c.seq", "c.ngl", "c.lt", "c.nge", "c.le", "c.ngt"
                            ];
                            if CP1_FN[func as usize].starts_with("?") {
                                error!("unknown cp1 function func=${:03b}_{:03b}", rs >> 3, rs & 0x07);
                            }
                            r_type_rt_rd(format!("{}", CP1_FN[func as usize]))
                        },
                        _ => {
                            format!("unhandled cop{} instruction ${:X}", copno, func)
                        },
                    }
                }
            },

            0b010_100 => i_type_rs_rt("beql"),
            0b010_101 => i_type_rs_rt("bnel"),
            0b010_110 => i_type_rs_rt("blezl"),
            0b010_111 => i_type_rs_rt("bgtzl"),

            0b011_000 => i_type_rt_rs("daddi"),
            0b011_001 => i_type_rt_rs("daddiu"),
            0b011_010 => i_type_rt_base("ldl"),
            0b011_011 => i_type_rt_base("ldr"),

            0b011_111 => no_type("<invalid>"),

            0b100_000 => i_type_rt_base("lb"),
            0b100_001 => i_type_rt_base("lh"),
            0b100_010 => i_type_rt_base("lwl"),
            0b100_011 => i_type_rt_base("lw"),
            0b100_100 => i_type_rt_base("lbu"),
            0b100_101 => i_type_rt_base("lhu"),
            0b100_110 => i_type_rt_base("lwr"),
            0b100_111 => i_type_rt_base("lwu"),
            0b101_000 => i_type_rt_base("sb"),
            0b101_001 => i_type_rt_base("sh"),
            0b101_010 => i_type_rt_base("swl"),
            0b101_011 => i_type_rt_base("sw"),
            0b101_100 => i_type_rt_base("sdl"),
            0b101_101 => i_type_rt_base("sdr"),
            0b101_110 => i_type_rt_base("swr"),
            0b101_111 => no_type("cache"),

            0b110_000 => i_type_rt("ll"),
            0b110_001 => i_type_ft_base("lwc1"),
            0b110_101 => i_type_ft_base("ldc1"),
            0b110_111 => i_type_rt("ld"),

            0b111_000 => i_type_rt("sc"),
            0b111_001 => i_type_rt("swc1"),
            0b111_101 => i_type_ft_base("sdc1"),
            0b111_111 => i_type_rt("sd"),

            _ => panic!("cannot disassemble 0b{:03b}_{:03b}", op >> 3, op & 0x07),
        }
    }
}

